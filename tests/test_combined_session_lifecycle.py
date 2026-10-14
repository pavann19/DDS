"""Combined-drive configuration survives session boundaries without blind motion."""
from app.services.simulation_session import SimulationSession


def test_combined_load_pauses_and_missing_camera_holds_after_resume():
    session = SimulationSession(seed=11, real_time=False)
    session._handle_command({'type': 'load_scenario', 'scenario_id': 'driving_environment'})
    assert session.is_paused and session.scenario_engine.is_paused
    p = session.world.physics
    assert p.camera_required_for_drive and p.scene_camera_controls
    assert p.traffic.finite_world
    session._handle_command({'type': 'resume_simulation'})
    session.step_pipeline(num_substeps=100)
    assert p.speed_kmh == 0.
    assert not session.driver.sensor_guard.status.ready
    assert session.driver.sensor_guard.status.reason == 'sensor_camera_missing'


def test_combined_reconfiguration_keeps_reduced_grip_and_requested_density():
    session = SimulationSession(seed=11, real_time=False)
    session._handle_command({'type': 'load_scenario', 'scenario_id': 'driving_environment',
                            'traffic_density': 'high', 'params': {'friction_mu': .5}})
    epoch = session.epoch
    session.reset()
    assert session.epoch > epoch
    assert session.world.physics.friction_mu == .5
    assert session.scenario_engine.active_density == 'high'
    assert session.world.physics.camera_required_for_drive
    assert session.world.physics.control_camera is not None
    assert session.world.physics.control_camera.frame is None
    assert session.is_paused and session.scenario_engine.is_paused


def test_combined_destination_keeps_finite_world_and_requires_new_route_camera():
    from app.services.control_camera import CameraFrame, CameraCalibration
    session = SimulationSession(seed=11, real_time=False)
    session._handle_command({'type': 'load_scenario', 'scenario_id': 'driving_environment',
                            'traffic_density': 'high'})
    old = session.world.physics
    old.control_camera.frame = CameraFrame(bytes(640*360*3), 0., CameraCalibration(640,360,140.))
    old.scene_camera_pending = object()
    route = list(old.route)
    epoch = session.epoch
    session.apply_route(route, [])
    p = session.world.physics
    assert session.epoch > epoch
    assert p.traffic.finite_world and len(p.traffic.npcs) == 14
    assert p.camera_required_for_drive and p.scene_camera_controls
    assert p.control_camera.frame is None and p.scene_camera_pending is None
    assert p.traffic.world_controls == p.route_controls
    assert p.parking_mission is None  # Arbitrary destination is not a verified lot.
    assert session.scenario_engine.get_state()['environment']['traffic_extent_end_m'] == p.traffic.total_length_m
