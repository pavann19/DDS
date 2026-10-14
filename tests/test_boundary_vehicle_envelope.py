"""Independent road boundary shield uses actual modeled body/tire dimensions."""
import math
from dataclasses import replace
import pytest
from app.services.safety_shield import evaluate,OVERRIDE_RECOVER_LOW_SPEED,OVERRIDE_EMERGENCY_BRAKE
from app.services.vehicle_geometry import lateral_half_extent
from app.services.vehicle_kinematics import sideslip


def test_stationary_steered_tires_cross_boundary_even_when_old_body_did_not():
    width=2*lateral_half_extent(0.,math.radians(35.))
    common=dict(ego_speed_mps=0.,lateral_offset_m=6.,lateral_accel_mps2=0.,sensed_lead_gap_m=None,sensed_lead_speed_mps=None)
    assert evaluate(**common).approved  # explicit legacy default compatibility
    verdict=evaluate(**common,vehicle_width_m=width,vehicle_length_m=4.8)
    assert not verdict.approved and not verdict.body_contained
    assert verdict.override_action==OVERRIDE_RECOVER_LOW_SPEED


@pytest.mark.parametrize('sign',[-1.,1.])
def test_boundary_sweep_uses_motion_heading_but_body_orientation_stays_separate(sign):
    delta=-sign*math.radians(35.)
    beta=sideslip(delta)
    width=2*lateral_half_extent(0.,delta)
    args=dict(ego_speed_mps=5.,lateral_offset_m=sign*5.7,lateral_accel_mps2=0.,
              sensed_lead_gap_m=None,sensed_lead_speed_mps=None,heading_error_rad=0.,vehicle_width_m=width)
    old=evaluate(**args)
    assert old.approved and old.body_contained
    verdict=evaluate(**args,motion_heading_error_rad=beta)
    assert verdict.body_contained  # beta must not rotate the body rectangle
    assert not verdict.swept_body_contained and not verdict.approved


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_invalid_motion_heading_fails_closed(bad):
    verdict=evaluate(0.,0.,0.,None,None,motion_heading_error_rad=bad)
    assert not verdict.approved and verdict.override_action==OVERRIDE_EMERGENCY_BRAKE


def test_driver_monitor_receives_actual_steered_vehicle_dimensions():
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorObservation,EgoProprioception
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.001,-122.)])
    p.reset_state(station_m=10.,speed_kmh=0.,lateral_offset_m=6.)
    p.step_world_sensors(0.,perceive=False,predict=False)
    p.steering_angle_rad=math.radians(35.)
    ctx=p.build_driver_context(.02)
    obs=SensorObservation(clock=p.clock,ego=EgoProprioception(0.,p.heading,0.,p.steering_angle_rad,p.current_station_m,6.),frenet_frame=p.frenet_frame,safety_detections=())
    p._driver.decide_longitudinal(obs,ctx)
    assert not p._driver.shield_verdict.body_contained
    assert not p._driver.shield_verdict.approved
