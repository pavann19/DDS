from dataclasses import replace
import pytest
from tests.test_scene_camera_ingress import setup
from app.services.scene_camera_ingress import RIG_BYTES,CAMERA_IDS,FRAME_BYTES,AUX_FRAME_BYTES,IngressRejected
from app.services.driver.sensor_freshness import SensorFreshnessGuard
from app.services.interfaces import SensorHealth


async def test_rig_preserves_eight_distinct_rgb_buffers_and_calibrations():
    session,identity,ingress,_=setup()
    rgb=bytes([1])*FRAME_BYTES+b''.join(bytes([i+2])*AUX_FRAME_BYTES for i in range(7))
    assert len(rgb)==RIG_BYTES
    pending=await ingress.accept(session,identity,rgb,rig=True)
    assert pending.frame.rgb[0]==1
    assert tuple(name for name,_ in pending.auxiliary_frames)==CAMERA_IDS[1:]
    for i,(_,frame) in enumerate(pending.auxiliary_frames):
        assert frame.rgb==bytes([i+2])*AUX_FRAME_BYTES
        assert frame.capture_time_s==identity.capture_time_s
        assert (frame.calibration.width,frame.calibration.height,frame.calibration.hfov_deg)==(320,180,60. if i<2 else 100.)


async def test_rig_missing_bytes_and_replayed_capture_rejected():
    session,identity,ingress,_=setup()
    with pytest.raises(IngressRejected):await ingress.accept(session,identity,bytes(RIG_BYTES-1),rig=True)
    await ingress.accept(session,identity,bytes(RIG_BYTES),rig=True)
    with pytest.raises(IngressRejected):await ingress.accept(session,replace(identity,seq=2),bytes(RIG_BYTES),rig=True)


def health(t,missing=None):
    return SensorHealth(t,t,True,t,CAMERA_IDS,tuple((name,t if name!=missing else None) for name in CAMERA_IDS))


def test_all_eight_views_required_and_distinct_capture_recovery():
    g=SensorFreshnessGuard()
    initial=g.evaluate(0.,health(0.))
    assert len(initial.camera_view_health)==8
    assert all(row[1]==0. and row[2]==1 and not row[3] for row in initial.camera_view_health)
    for t in (0.,.1,.2):g.evaluate(t,health(t))
    assert g.status.ready and len(g.status.camera_view_health)==8
    assert not g.evaluate(.3,health(.3,'rear')).ready
    assert g.status.reason=='sensor_camera_missing'
    for t in (.4,.41,.42):
        sample=replace(health(t),camera_capture_times_s=tuple((name,.4) for name in CAMERA_IDS))
        assert not g.evaluate(t,sample).ready
    assert g.status.camera_recovery_samples==1
    assert not g.evaluate(.5,health(.5)).ready
    assert g.evaluate(.6,health(.6)).ready


def test_one_stale_view_holds_fresh_other_cameras_and_range_feeds():
    g=SensorFreshnessGuard()
    for t in (0.,.1,.2):g.evaluate(t,health(t))
    sample=replace(health(.56),camera_capture_times_s=tuple((name,.2 if name=='left' else .56) for name in CAMERA_IDS))
    assert not g.evaluate(.56,sample).ready
    assert g.status.reason=='sensor_camera_stale'
    assert not next(row for row in g.status.camera_view_health if row[0]=='left')[3]


def test_invalid_clock_clears_each_view_readiness():
    g=SensorFreshnessGuard()
    for t in (0.,.1,.2): g.evaluate(t,health(t))
    status=g.evaluate(float('nan'),health(.2))
    assert not status.ready
    assert len(status.camera_view_health)==8
    assert all(row[1] is None and row[2]==0 and not row[3] for row in status.camera_view_health)


async def test_each_auxiliary_view_runs_pixel_detector_not_labels(monkeypatch):
    import app.services.scene_camera_ingress as module
    seen=[]
    def detect(frame):
        seen.append((frame.rgb[0],frame.calibration.width))
        return ()
    monkeypatch.setattr(module,'detect_controls',detect)
    session,identity,ingress,_=setup()
    rgb=bytes([1])*FRAME_BYTES+b''.join(bytes([i+2])*AUX_FRAME_BYTES for i in range(7))
    pending=await ingress.accept(session,identity,rgb,rig=True)
    assert sorted(seen)==[(1,640)]+[(i,320) for i in range(2,9)]
    assert len(pending.auxiliary_detections)==7


async def test_failed_view_keeps_bundle_busy_until_other_workers_finish(monkeypatch):
    import asyncio, threading
    import app.services.scene_camera_ingress as module
    entered,release=threading.Event(),threading.Event()
    def detect(frame):
        if frame.rgb[0]==1: raise ValueError('failed_front')
        entered.set()
        release.wait(2.)
        return ()
    monkeypatch.setattr(module,'detect_controls',detect)
    session,identity,ingress,_=setup()
    rgb=bytes([1])*FRAME_BYTES+b''.join(bytes([i+2])*AUX_FRAME_BYTES for i in range(7))
    task=asyncio.create_task(ingress.accept(session,identity,rgb,rig=True))
    try:
        assert await asyncio.to_thread(entered.wait,1.)
        assert ingress.busy and not task.done()
    finally: release.set()
    with pytest.raises(ValueError,match='failed_front'): await task
    assert not ingress.busy
    assert not hasattr(session.world.physics,'scene_camera_pending')


async def test_rig_binary_endpoint_and_readonly_preview(monkeypatch):
    from fastapi import FastAPI
    from httpx import ASGITransport,AsyncClient
    from types import SimpleNamespace
    import app.api.scene_camera as api
    session,identity,ingress,_=setup()
    monkeypatch.setattr(api,'get_simulation_session',lambda:session)
    monkeypatch.setattr(api,'ingress',ingress)
    app=FastAPI();app.include_router(api.router)
    headers={'content-type':'application/octet-stream','x-simulation-id':'session',
        'x-camera-epoch':'1','x-camera-tick':'10','x-camera-capture-time':'1',
        'x-camera-seq':'1','x-camera-publisher':'browser'}
    async with AsyncClient(transport=ASGITransport(app=app),base_url='http://test') as client:
        assert (await client.post('/camera/scene/rig',headers=headers,content=bytes(RIG_BYTES+1))).status_code==413
        assert (await client.post('/camera/scene/rig',headers=headers,content=bytes(RIG_BYTES))).status_code==200
        pending=session.world.physics.scene_camera_pending
        session.world.physics.control_camera=SimpleNamespace(camera_frames=dict(pending.auxiliary_frames),frame=pending.frame)
        response=await client.get('/camera/scene/rear.png')
        assert response.status_code==200 and response.headers['x-camera-id']=='rear'
        assert response.headers['x-capture-time-s']=='1.0'
        assert (await client.get('/camera/scene/not_camera.png')).status_code==404
        assert session.world.physics.scene_camera_pending is pending
