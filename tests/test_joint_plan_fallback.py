"""Rejected joint maneuvers may not bypass the physics filter."""
from dataclasses import replace
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services.interfaces import SensorObservation, EgoProprioception, LeadObservation
from app.services.planner.state_machine import LaneChangeState
from app.services.driver import driver as module


def setup_driver():
    p = PhysicsEngine(seed=42)
    p.set_destination(37.002, -122.)
    p.set_route([(37., -122.), (37.001, -122.), (37.002, -122.)])
    p.reset_state(speed_kmh=18., lateral_offset_m=1.75)
    p.step_world_sensors(.02, perceive=False, predict=False)
    ctx = replace(p.build_driver_context(.1), waypoint_dist_m=100.,
                  dist_to_dest_m=1000., lane_clear_fn=lambda *_: True)
    obs = SensorObservation(clock=p.clock,
        ego=EgoProprioception(5., p.heading, 0., 0., p.current_station_m, 1.75),
        frenet_frame=p.frenet_frame, safety_detections=())
    return p, obs, ctx


def test_rejected_joint_maneuver_holds_offset_and_requests_stop(monkeypatch):
    p, obs, ctx = setup_driver()
    p._driver._lc_sm.state = LaneChangeState.EXECUTE_LANE_CHANGE
    monkeypatch.setattr(module, 'plan_spatiotemporal', lambda *_a, **_k: None)
    def forbidden(*_a, **_k):
        raise AssertionError('Rejected joint maneuver reached unchecked passing planner')
    monkeypatch.setattr(module, 'plan_lateral_offset', forbidden)
    p._driver.decide_lateral(obs, ctx)
    assert p._driver._joint_plan_unavailable
    assert p._driver.lateral_target_d_m == ctx.lateral_offset_m
    assert p._driver.spatiotemporal_traj is None
    assert not p._driver._changing_lane_now
    result = p._driver.decide_longitudinal(obs, ctx)
    assert p._driver.target_speed_mps == 0.
    assert result.accel_mps2 < 0.
    assert result.accel_mps2 >= -p.A_MAX_BRAKE_MPS2
    assert result.speed_limit_reason == 'joint_trajectory_infeasible'


def test_prepare_phase_cannot_request_unvalidated_passing_candidate(monkeypatch):
    p, obs, ctx = setup_driver()
    obs = replace(obs, lead=LeadObservation(12., 0.))
    original = module.plan_lateral_offset
    calls = []
    def checked(**kwargs):
        calls.append(kwargs['adjacent_lane_clear'])
        return original(**kwargs)
    monkeypatch.setattr(module, 'plan_lateral_offset', checked)
    p._driver.decide_lateral(obs, ctx)
    assert p._driver._lc_sm.state == LaneChangeState.PREPARE_LANE_CHANGE
    assert calls == [False]
    assert not p._driver._joint_plan_unavailable


def test_route_reset_clears_joint_hold_state():
    p, _, _ = setup_driver()
    p._driver._joint_plan_unavailable = True
    p._driver._maneuver_friction_mu = .8
    p._driver.on_route_reset()
    assert not p._driver._joint_plan_unavailable
    assert p._driver._maneuver_friction_mu is None


def test_hmi_keeps_committed_lateral_tail_after_shorter_longitudinal_horizon():
    from types import SimpleNamespace
    from app.services.simulation_session import SimulationSession
    from app.services.planner.polynomials import QuinticPolynomial, QuarticPolynomial
    session = SimulationSession(seed=42, real_time=False)
    lat = QuinticPolynomial(1.75, 0., 0., 5.25, 0., 0., 6.)
    lon = QuarticPolynomial(100., 10., 0., 10., 0., 2.)
    session.driver.spatiotemporal_traj = SimpleNamespace(lat_poly=lat, lon_poly=lon, t_lat_s=6., t_lon_s=2.)
    session.driver._maneuver_t = 3.
    planner = session.build_telemetry_payload()['channels']['semantic']['planner']
    samples = planner['trajectory']
    assert len(samples) == 21
    assert samples[0]['x'] == pytest.approx(lat.pos(3.))
    assert samples[0]['z'] == pytest.approx(-130.)
    assert samples[-1]['z'] == pytest.approx(-150.)
    assert planner['joint_plan_status'] == 'committed'


def test_joint_planner_vehicle_limits_follow_the_actual_driver_configuration():
    from app.services.driver.driver import Driver
    p, _, _ = setup_driver()
    actual = replace(p._driver.cfg, WHEELBASE_M=3.1, MAX_STEER_RAD=.5,
                     STEER_RATE_MAX_RADPS=.4, A_LAT_MAX_MPS2=1.2, A_MAX_BRAKE_MPS2=3.)
    driver = Driver(cfg=actual)
    assert driver.planner_cfg.wheelbase_m == 3.1
    assert driver.planner_cfg.max_steer_rad == .5
    assert driver.planner_cfg.max_steer_rate_radps == .4
    assert driver.planner_cfg.max_lat_accel_mps2 == 1.2
    assert driver.planner_cfg.min_long_accel_mps2 == -3.
