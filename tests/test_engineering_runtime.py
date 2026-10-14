import asyncio
from unittest.mock import Mock

import pytest

from app.services.simulation_session import SimulationSession, SubscriberQueue


def session():
    return SimulationSession(seed=42, real_time=False)


def test_session_analytics_cannot_change_driving_commands():
    traces = []
    for label in ('Accelerate', 'Decelerate', 'Malformed model output'):
        s = session()
        s._handle_command({'type':'load_scenario','scenario_id':'emergency_cut_in'})
        trace = []
        for _ in range(50):
            s.latest_analytics = {'decision':label, 'confidence':1.0, 'driver_score':{'score':-999}}
            s.step_pipeline()
            trace.append((s.latest_command, s.world.physics.speed_kmh, s.world.physics.heading))
        traces.append(trace)
    assert traces[0] == traces[1] == traces[2]


def test_heartbeat_does_not_resample_stateful_analytics(monkeypatch):
    s = session()
    s.analytics_queue = asyncio.Queue(maxsize=10)
    features = Mock(wraps=s.world.physics.get_ml_features)
    monkeypatch.setattr(s.world.physics, 'get_ml_features', features)
    for _ in range(10): s._emit_telemetry_snapshot()
    assert features.call_count == 1
    assert s.analytics_queue.qsize() == 1
    s.executor.step()
    s._emit_telemetry_snapshot()
    assert features.call_count == 2


def test_scenario_geometry_change_is_published_with_new_version():
    s = session()
    _, q = s.subscribe()
    previous_version = s.route_version
    s._handle_command({'type':'load_scenario','scenario_id':'low_friction_curve'})
    messages = []
    while not q.empty(): messages.append(q.get_nowait())
    routes = [p for p in messages if p['type']=='route']
    assert s.route_version == previous_version + 1
    assert routes[-1]['route_version'] == s.route_version
    assert routes[-1]['epoch'] == s.epoch
    assert routes[-1]['waypoints'] == [list(p) for p in s.world.physics.route]


def test_actual_stage_calls_and_one_clock(monkeypatch):
    s = session()
    p = s.world.physics
    counters = {}
    for obj, name in [(p, 'step_perception'), (p, 'step_prediction'),
                      (s.driver, 'decide_lateral'), (s.driver, 'control')]:
        original = getattr(obj, name)
        counter = Mock(wraps=original)
        monkeypatch.setattr(obj, name, counter)
        counters[name] = counter
    for _ in range(100):
        s.executor.step()
        assert p.clock == s.executor.clock
        assert s.scenario_engine.elapsed_time_s == s.executor.clock.sim_time_s
    assert counters['step_perception'].call_count == 20
    assert counters['step_prediction'].call_count == 10
    assert counters['decide_lateral'].call_count == 10
    assert counters['control'].call_count == 50
    assert s.executor.timing_summary()['control_safety_physics']['count'] == 50


def test_command_ack_applied_and_rejected():
    s = session()
    sub, q = s.subscribe()
    s.enqueue_command({'type':'pause_simulation','command_id':'pause'}, subscriber_id=sub)
    s.process_pending_commands()
    s.enqueue_command({'type':'load_scenario','scenario_id':'missing','command_id':'bad'}, subscriber_id=sub)
    s.process_pending_commands()
    replies = []
    while not q.empty():
        p=q.get_nowait()
        if p['type']=='command_result': replies.append(p)
    assert [(p['command_id'],p['status']) for p in replies] == [('pause','applied'),('bad','rejected')]


def test_reliable_queue_survives_state_flood_and_reports_overflow():
    q = SubscriberQueue(maxsize=3)
    q.put_nowait({'type':'command_result','command_id':'x'})
    q.put_nowait({'type':'route'})
    for tick in range(20): q.put_nowait({'type':'state','tick':tick})
    assert q.get_nowait()['command_id']=='x'
    assert q.get_nowait()['type']=='route'
    assert q.get_nowait()['tick']==19
    for i in range(4): q.put_nowait({'type':'command_result','command_id':i})
    assert q.get_nowait()['type']=='subscriber_overflow'


def test_route_install_preserves_pose_and_failed_install_is_atomic():
    s=session()
    p=s.world.physics
    before=(p.lat,p.lng,p.heading,p.speed_kmh)
    old_version=s.route_version
    s.apply_route(p.route, [])
    assert (s.world.physics.lat,s.world.physics.lng,s.world.physics.heading,s.world.physics.speed_kmh)==before
    assert s.route_version==old_version+1
    current=s.world.physics
    with pytest.raises(ValueError): s.apply_route([(0.,0.),(.1,0.)],[])
    assert s.world.physics is current
    assert s.route_version==old_version+1


@pytest.mark.asyncio
async def test_route_a_b_a_race_and_scenario_invalidates_fetch(monkeypatch):
    import app.services.simulation_session as module
    s=session()
    pending=[]
    async def fetch(*args):
        f=asyncio.get_running_loop().create_future()
        pending.append(f)
        return await f
    monkeypatch.setattr(module,'get_route',fetch)
    route=list(s.world.physics.route)
    for lat in [37.7,37.8,37.7]:
        s._handle_command({'type':'set_destination','lat':lat,'lng':-122.4})
        await asyncio.sleep(0)
    original=s.world.physics
    pending[0].set_result((route,[]))
    pending[1].set_result((route,[]))
    await asyncio.sleep(0)
    s.process_pending_commands()
    assert s.world.physics is original
    pending[2].set_result((route,[]))
    await asyncio.sleep(0)
    s.process_pending_commands()
    assert s.route_version==2
    assert s.is_paused
    s._handle_command({'type':'set_destination','lat':37.7,'lng':-122.4})
    await asyncio.sleep(0)
    s._handle_command({'type':'load_scenario','scenario_id':'normal_cruising'})
    active=s.world.physics
    # Scenario restart restores the recorded geometry rather than the
    # re-smoothed destination route; it must publish that geometry change.
    scenario_version = s.route_version
    assert scenario_version == 3
    pending[3].set_result((route,[]))
    await asyncio.sleep(0)
    s.process_pending_commands()
    assert s.world.physics is active
    assert s.route_version==scenario_version


def test_hmi_uses_controller_target_and_committed_plan():
    s=session()
    s.step_pipeline()
    p=s.build_telemetry_payload()
    assert p['channels']['pose']['ego']['target_velocity']==s.driver.target_speed_mps
    assert p['channels']['semantic']['planner']['trajectory']==[]
    assert p['channels']['semantic']['planner']['trajectory_source']=='unavailable'
    s._handle_command({'type':'load_scenario','scenario_id':'traffic_overtake'})
    for _ in range(200):
        s.step_pipeline()
        if s.driver.spatiotemporal_traj is not None: break
    assert s.driver.spatiotemporal_traj is not None
    samples=s.build_telemetry_payload()['channels']['semantic']['planner']['trajectory']
    assert samples
    assert samples[0]['x']==pytest.approx(s.driver.spatiotemporal_traj.lat_poly.pos(s.driver._maneuver_t))
    assert samples[0]['z']==pytest.approx(-s.driver.spatiotemporal_traj.lon_poly.pos(s.driver._maneuver_t))


@pytest.mark.asyncio
@pytest.mark.parametrize('peer_closes_during_send',[False,True])
async def test_websocket_cleans_both_tasks_without_stopping_run(monkeypatch, peer_closes_during_send):
    from app.api import websockets
    from fastapi import WebSocketDisconnect
    from starlette.websockets import WebSocketState
    s=session()
    s.start=Mock()
    s.stop=Mock()
    monkeypatch.setattr(websockets,'get_simulation_session',lambda:s)
    class Socket:
        headers={}
        client_state=WebSocketState.CONNECTED
        application_state=WebSocketState.CONNECTED
        async def accept(self): pass
        async def receive_text(self):
            await asyncio.sleep(.01)
            self.client_state=WebSocketState.DISCONNECTED
            raise WebSocketDisconnect()
        async def send_json(self,p):
            if peer_closes_during_send:
                raise RuntimeError("Unexpected ASGI message 'websocket.send', after sending 'websocket.close'.")
        async def close(self): self.application_state=WebSocketState.DISCONNECTED
    ws=Socket()
    await asyncio.wait_for(websockets.websocket_endpoint(ws),.5)
    assert not s._subscribers
    assert ws not in websockets.manager.active_connections
    s.stop.assert_not_called()
