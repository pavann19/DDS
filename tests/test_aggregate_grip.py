"""Modeled μg envelope tests; no surveyed tire/road properties implied."""
import math
from dataclasses import replace
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services import safety_shield
from app.services.vehicle_kinematics import sideslip
from app.services.planner.spatiotemporal import _feasible, PlannerConfig, PlannerContext
from app.services.planner.polynomials import QuinticPolynomial, QuarticPolynomial


def emergency(engine):
    engine.shield_verdict = safety_shield.ShieldVerdict(approved=False,
        risk_level=safety_shield.RISK_CRITICAL,
        override_action=safety_shield.OVERRIDE_EMERGENCY_BRAKE)


def test_icy_emergency_braking_does_not_exceed_straight_grip():
    p = PhysicsEngine(seed=42)
    p.speed_kmh = 36.
    p.friction_mu = .2
    emergency(p)
    p.step_dynamics(steer_rad=0., accel_mps2=3., dt=.02)
    assert p.acceleration_mps2 == pytest.approx(-.2*9.81)
    assert p.speed_kmh < 36.


def test_planner_rejects_individually_valid_demands_outside_aggregate_grip():
    lat = QuinticPolynomial(0.,0.,.15,0.,0.,0.,6.)
    lon = QuarticPolynomial(20.,10.,-.975,6.,0.,6.)
    cfg = replace(PlannerConfig(), max_lat_jerk_mps3=100.)
    ctx = PlannerContext(10.,0.,20.,.1,friction_mu=.1)
    # Restrict horizon to initial sample, isolating shared-envelope failure.
    assert _feasible(lat,lon,0.,0.,1,cfg,max_lat_accel=.2,context=ctx) is None


@pytest.mark.parametrize("direction",[-1.,1.])
def test_full_grip_emergency_unwinds_at_rate_and_does_not_trap_vehicle(direction):
    from app.services.grip_budget import allocate_grip
    steer=direction*.15
    v=math.sqrt(.2*9.81/(math.cos(sideslip(steer))*abs(math.tan(steer))/2.8))
    result=allocate_grip(speed_mps=v,previous_steer_rad=steer,
        requested_steer_rad=steer,requested_acceleration_mps2=3.,friction_mu=.2,
        dt_s=.02,wheelbase_m=2.8,max_steer_rad=math.radians(35),
        steer_rate_radps=.6,emergency=True,max_braking_mps2=4.5)
    assert result.acceleration_mps2<-.1
    assert abs(result.steer_rad)<abs(steer)
    assert abs(result.steer_rad-steer)<=.6*.02+1e-12
    assert math.hypot(result.acceleration_mps2,result.lateral_bound_mps2)<=result.limit_mps2+1e-9


def test_ordinary_near_stop_relaxation_does_not_double_comfort_jerk():
    p=PhysicsEngine(seed=42)
    p.speed_kmh=.001
    p.acceleration_mps2=-1.
    p.step_dynamics(steer_rad=0.,accel_mps2=0.,dt=.02)
    assert abs(p.acceleration_mps2+1.)<=p.JERK_MAX_MPS3*.02+1e-9


def test_invalid_previous_actuator_domain_rejected():
    from app.services.grip_budget import allocate_grip
    with pytest.raises(ValueError):
        allocate_grip(speed_mps=5.,previous_steer_rad=2.,requested_steer_rad=0.,
            requested_acceleration_mps2=0.,friction_mu=1.,dt_s=.02,
            wheelbase_m=2.8,max_steer_rad=math.radians(35),steer_rate_radps=.6,
            emergency=False,max_braking_mps2=4.5)


@pytest.mark.parametrize("direction",[-1.,1.])
def test_turning_and_braking_share_budget_in_actual_dynamics(direction):
    p=PhysicsEngine(seed=42)
    p.friction_mu=.2
    p.speed_kmh=36.
    p.steering_angle_rad=direction*.02
    p.kinematic_sideslip_rad=sideslip(p.steering_angle_rad)
    p.acceleration_mps2=-4.5
    p.step_dynamics(steer_rad=p.steering_angle_rad,accel_mps2=-4.5,dt=.02)
    budget=p.get_navigation_state()["grip_budget"]
    assert budget["acceleration_limited"] and budget["within_modeled_budget"]
    assert abs(p.acceleration_mps2)<.2*9.81
    assert math.hypot(p.acceleration_mps2,p.lateral_accel_mps2)<=.2*9.81+1e-9
    assert budget["total_bound_mps2"]<=budget["limit_mps2"]+1e-9


@pytest.mark.parametrize("direction",[-1.,1.])
def test_transient_wheel_slew_reservation_prevents_hidden_motion_acceleration(direction):
    p=PhysicsEngine(seed=42)
    p.friction_mu=.2
    p.speed_kmh=36.
    p.step_dynamics(steer_rad=direction*.2,accel_mps2=0.,dt=.02)
    budget=p.grip_budget_state
    assert budget["steering_limited"] and budget["within_modeled_budget"]
    assert abs(p.steering_angle_rad)/.02<=p.STEER_RATE_MAX_RADPS+1e-9
    assert abs(budget["motion_lateral_acceleration_mps2"])<=budget["lateral_bound_mps2"]+1e-9
    assert budget["total_bound_mps2"]<=budget["limit_mps2"]+1e-9


def test_positive_acceleration_checks_faster_end_speed():
    from app.services.grip_budget import allocate_grip
    result=allocate_grip(speed_mps=10.,previous_steer_rad=.03,
        requested_steer_rad=.03,requested_acceleration_mps2=1.8,friction_mu=.2,
        dt_s=.1,wheelbase_m=2.8,max_steer_rad=math.radians(35),
        steer_rate_radps=.6,emergency=False,max_braking_mps2=4.5)
    end_speed=10.+result.acceleration_mps2*.1
    end_lateral=end_speed**2*math.cos(sideslip(result.steer_rad))*math.tan(result.steer_rad)/2.8
    assert result.acceleration_limited
    assert math.hypot(result.acceleration_mps2,end_lateral)<=.2*9.81+1e-9


def test_icy_turning_emergency_stops_and_preserves_heading_bicycle_model():
    p=PhysicsEngine(seed=42)
    p.friction_mu=.2
    p.speed_kmh=36.
    p.steering_angle_rad=.02
    p.kinematic_sideslip_rad=sideslip(.02)
    emergency(p)
    for _ in range(400):
        old_speed=p.speed_kmh/3.6
        old_heading=p.heading
        old_steer=p.steering_angle_rad
        p.step_dynamics(steer_rad=.2,accel_mps2=3.,dt=.02)
        expected_yaw=old_speed*math.cos(sideslip(p.steering_angle_rad))*math.tan(p.steering_angle_rad)/p.WHEELBASE_M
        heading_delta=(p.heading-old_heading+180)%360-180
        assert math.radians(heading_delta)/.02==pytest.approx(expected_yaw,abs=1e-10)
        assert abs(p.steering_angle_rad-old_steer)<=p.STEER_RATE_MAX_RADPS*.02+1e-9
        assert p.grip_budget_state["within_modeled_budget"]
    assert p.speed_kmh==0.


def test_abrupt_friction_loss_reports_initial_violation_and_eventually_recovers():
    p=PhysicsEngine(seed=42)
    p.speed_kmh=54.
    p.steering_angle_rad=.1
    p.kinematic_sideslip_rad=sideslip(.1)
    p.friction_mu=.1
    violations=[]
    for _ in range(100):
        old_steer=p.steering_angle_rad
        old_speed=p.speed_kmh
        p.step_dynamics(steer_rad=.3,accel_mps2=3.,dt=.02)
        violations.append(p.grip_budget_state["already_infeasible"])
        if p.grip_budget_state["already_infeasible"]:
            assert not p.grip_budget_state["within_modeled_budget"]
            assert p.speed_kmh<old_speed
            assert abs(p.steering_angle_rad)<=abs(old_steer)
        assert abs(p.steering_angle_rad-old_steer)<=p.STEER_RATE_MAX_RADPS*.02+1e-9
    assert violations[0] and not violations[-1]
    assert p.grip_budget_state["within_modeled_budget"]


def test_reset_clears_prior_sideslip_and_grip_recovery_state():
    p=PhysicsEngine(seed=42)
    p.steering_angle_rad=.2
    p.kinematic_sideslip_rad=sideslip(.2)
    p.grip_budget_state={"already_infeasible":True,"recovery_active":True}
    p.reset_state(speed_kmh=36.)
    assert p.steering_angle_rad==0.
    assert p.kinematic_sideslip_rad==0.
    assert not p.grip_budget_state["already_infeasible"]
    p.step_dynamics(steer_rad=0.,accel_mps2=0.,dt=.02)
    assert p.steering_angle_rad==0. and p.yaw_rate_degps==0.
    assert p.grip_budget_state["within_modeled_budget"]
