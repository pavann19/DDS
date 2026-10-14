import asyncio
from dataclasses import replace
from types import SimpleNamespace
import pytest
import app.services.scene_camera_ingress as module


def setup():
    session=SimpleNamespace(session_id='session',epoch=1,
        executor=SimpleNamespace(clock=SimpleNamespace(sim_time_s=1.)),
        world=SimpleNamespace(physics=SimpleNamespace(scene_camera_controls=True)))
    identity=module.CaptureIdentity('session',1,10,1.,1,'browser')
    return session,identity,module.SceneCameraIngress(),bytes(module.FRAME_BYTES)


async def test_acceptance_stores_only_immutable_frame_and_pixel_detections():
    session,identity,ingress,rgb=setup()
    pending=await ingress.accept(session,identity,rgb)
    assert pending.frame.rgb==rgb and pending.detections==()
    assert pending.frame.calibration.hfov_deg==90.
    assert session.world.physics.scene_camera_pending is pending
    assert not hasattr(session,'latest_command')


@pytest.mark.parametrize('patch,reason',[
    ({'simulation_id':'old'},'session_or_epoch_changed'),({'epoch':0},'session_or_epoch_changed'),
    ({'capture_time_s':.1,'tick':1},'stale_capture'),
    ({'capture_time_s':1.1,'tick':11},'future_capture'),
    ({'tick':9},'tick_time_mismatch'),({'seq':-1},'invalid_identity')])
async def test_stale_future_identity_and_tick_rejected(patch,reason):
    session,identity,ingress,rgb=setup()
    with pytest.raises(module.IngressRejected) as error:
        await ingress.accept(session,replace(identity,**patch),rgb)
    assert error.value.reason==reason
    assert not hasattr(session.world.physics,'scene_camera_pending')


async def test_disabled_and_size_guards():
    session,identity,ingress,rgb=setup()
    with pytest.raises(module.IngressRejected):await ingress.accept(session,identity,b'')
    session.world.physics.scene_camera_controls=False
    with pytest.raises(module.IngressRejected) as error:await ingress.accept(session,identity,rgb)
    assert error.value.reason=='scene_camera_disabled'


async def test_duplicate_paused_capture_publisher_and_rate_limits(monkeypatch):
    session,identity,ingress,rgb=setup()
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:100.))
    await ingress.accept(session,identity,rgb)
    with pytest.raises(module.IngressRejected) as error:
        await ingress.accept(session,replace(identity,seq=2),rgb)
    assert error.value.reason=='duplicate_or_out_of_order'
    session.executor.clock.sim_time_s=1.1
    newer=replace(identity,tick=11,capture_time_s=1.1,seq=2)
    with pytest.raises(module.IngressRejected) as error:await ingress.accept(session,replace(newer,publisher='other'),rgb)
    assert error.value.reason=='publisher_conflict'
    with pytest.raises(module.IngressRejected) as error:await ingress.accept(session,newer,rgb)
    assert error.value.status==429
    ingress.last_wall-=1
    await ingress.accept(session,newer,rgb)


async def test_next_telemetry_tick_accepts_bounded_wall_arrival_jitter(monkeypatch):
    session,identity,ingress,rgb=setup()
    wall=[100.]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:wall[0]))
    await ingress.accept(session,identity,rgb)
    session.executor.clock.sim_time_s=1.1
    wall[0]+=.095
    await ingress.accept(session,replace(identity,tick=11,capture_time_s=1.1,seq=2),rgb)
    assert ingress.seq==2


async def test_expired_publisher_can_be_replaced_without_replaying_capture(monkeypatch):
    session,identity,ingress,rgb=setup()
    wall=[100.]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:wall[0]))
    await ingress.accept(session,identity,rgb)
    wall[0]+=2.1
    with pytest.raises(module.IngressRejected,match='duplicate_or_out_of_order'):
        await ingress.accept(session,replace(identity,publisher='replacement'),rgb)
    session.executor.clock.sim_time_s=1.1
    newer=replace(identity,tick=11,capture_time_s=1.1,seq=0,publisher='replacement')
    await ingress.accept(session,newer,rgb)
    assert ingress.publisher=='replacement' and ingress.seq==0
    session.executor.clock.sim_time_s=1.2
    with pytest.raises(module.IngressRejected,match='publisher_conflict'):
        await ingress.accept(session,replace(identity,tick=12,capture_time_s=1.2,seq=99),rgb)


async def test_reset_during_cv_discards_frame_and_bounds_inflight(monkeypatch):
    import threading
    session,identity,ingress,rgb=setup()
    entered,release=threading.Event(),threading.Event()
    def detect(frame):
        entered.set();release.wait(1.);return ()
    monkeypatch.setattr(module,'detect_controls',detect)
    task=asyncio.create_task(ingress.accept(session,identity,rgb))
    await asyncio.to_thread(entered.wait,1.)
    try:
        with pytest.raises(module.IngressRejected) as error:await ingress.accept(session,identity,rgb)
        assert error.value.status==429
        session.epoch=2
    finally:release.set()
    with pytest.raises(module.IngressRejected) as error:await task
    assert error.value.reason=='session_or_epoch_changed'
    assert not hasattr(session.world.physics,'scene_camera_pending') and not ingress.busy


async def test_api_rejects_oversize_and_accepts_binary(monkeypatch):
    from fastapi import FastAPI
    from httpx import ASGITransport,AsyncClient
    import app.api.scene_camera as api
    session,identity,ingress,rgb=setup()
    monkeypatch.setattr(api,'get_simulation_session',lambda:session)
    monkeypatch.setattr(api,'ingress',ingress)
    app=FastAPI();app.include_router(api.router)
    headers={'content-type':'application/octet-stream','x-simulation-id':'session',
        'x-camera-epoch':'1','x-camera-tick':'10','x-camera-capture-time':'1',
        'x-camera-seq':'1','x-camera-publisher':'browser'}
    async with AsyncClient(transport=ASGITransport(app=app),base_url='http://test') as client:
        assert (await client.post('/camera/scene',headers=headers,content=rgb+b'x')).status_code==413
        response=await client.post('/camera/scene',headers=headers,content=rgb)
        assert response.status_code==200
        payload=response.json()
        assert {k:payload[k] for k in ('accepted','seq','stale_reason')}=={'accepted':True,'seq':1,'stale_reason':None}
        assert 0<=payload['capture_age_s']<=.35
        assert payload['detection_ms']>=0


async def test_request_cancellation_keeps_worker_inflight_until_finished(monkeypatch):
    import threading
    session,identity,ingress,rgb=setup()
    entered,release=threading.Event(),threading.Event()
    def detect(frame):entered.set();release.wait(1.);return ()
    monkeypatch.setattr(module,'detect_controls',detect)
    task=asyncio.create_task(ingress.accept(session,identity,rgb))
    await asyncio.to_thread(entered.wait,1.)
    task.cancel()
    await asyncio.sleep(0)
    assert ingress.busy
    release.set()
    with pytest.raises(asyncio.CancelledError):await task
    assert not ingress.busy and not hasattr(session.world.physics,'scene_camera_pending')


async def test_frame_aging_during_cv_is_rejected(monkeypatch):
    session,identity,ingress,rgb=setup()
    def detect(frame):session.executor.clock.sim_time_s=1.5;return ()
    monkeypatch.setattr(module,'detect_controls',detect)
    with pytest.raises(module.IngressRejected) as error:await ingress.accept(session,identity,rgb)
    assert error.value.reason=='stale_capture'
    assert not hasattr(session.world.physics,'scene_camera_pending')
