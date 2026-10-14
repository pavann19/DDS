"""Synthetic boundary observations exercise the actual driver integration."""
from dataclasses import replace
import math

import pytest

from app.services.driver import driver as module
from app.services.interfaces import SafetyDetection
from app.services.physics_engine import PhysicsEngine
from app.services.simulation_session import SimulationSession
from app.services.vehicle_kinematics import sideslip


def test_collision_sweep_uses_center_motion_and_covers_steered_tires(monkeypatch):
    p = PhysicsEngine(seed=42)
    p.set_route([(37., -122.), (37.01, -122.)])
    p.reset_state(speed_kmh=18., lateral_offset_m=1.75)
    p.step_world_sensors(.02)
    ctx = replace(p.build_driver_context(.02), steering_angle_rad=.3)
    obs = replace(p.observe(), safety_detections=(SafetyDetection(10., 0., 0., 0., 4.8, 1.8),))
    captured = []
    def record(ego, actor, *_a, **_kw):
        captured.append(ego)
        return False
    monkeypatch.setattr(module, 'intersects_during', record)
    p._driver.decide_longitudinal(obs, ctx)
    assert captured
    ego = captured[0]
    expected_motion = math.radians(ctx.heading_deg) + sideslip(.3)
    assert ego.vx == pytest.approx(5*math.sin(expected_motion))
    assert ego.vz == pytest.approx(-5*math.cos(expected_motion))
    assert ego.width_m > 2.18
    assert ego.length_m == 4.8


def test_telemetry_separates_model_proof_from_unproved_cartesian_sweep():
    from app.services.planner.spatiotemporal import PlannerStart, PlannerContext, plan as plan_spatiotemporal
    session = SimulationSession(seed=42, real_time=False)
    traj = plan_spatiotemporal(PlannerStart(0., 5., 0., 1.75, 0., 0.),
        PlannerContext(5., 1.75, 10., .1, actual_steer_rad=0.))
    assert traj is not None
    session.driver.spatiotemporal_traj = traj
    planner = session.build_telemetry_payload()['channels']['semantic']['planner']
    assert planner['continuous_modeled_corridor_verified']
    assert planner['sampled_modeled_vehicle_envelope_verified']
    assert planner['initial_actual_steering_checked']
    assert not planner['full_cartesian_swept_corridor_verified']
    assert not planner['continuous_route_station_verified']  # no mapped frame in this unit fixture
    assert planner['modeled_straight_vehicle_width_m'] == pytest.approx(2.18)
