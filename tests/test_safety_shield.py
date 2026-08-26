"""
Unit tests for app/services/safety_shield.py -- the independent runtime
check that can override the planner/IDM's decision.
"""
import pytest

from app.services.safety_shield import (
    evaluate,
    compute_ttc_s,
    ROAD_BOUNDARY_HARD_LIMIT_M,
    HARD_LATERAL_ACCEL_LIMIT_MPS2,
    TTC_CRITICAL_S,
    TTC_WARNING_S,
    RISK_NONE,
    RISK_MEDIUM,
    RISK_HIGH,
    RISK_CRITICAL,
    OVERRIDE_EMERGENCY_BRAKE,
    OVERRIDE_RECOVER_LOW_SPEED,
)


# --- compute_ttc_s ----------------------------------------------------------

def test_ttc_none_with_no_lead_vehicle():
    assert compute_ttc_s(gap_m=None, ego_speed_mps=20.0, lead_speed_mps=None) is None


def test_ttc_none_when_not_closing():
    """Matching speed or pulling away is not a collision trajectory,
    however small the gap already is."""
    assert compute_ttc_s(gap_m=2.0, ego_speed_mps=20.0, lead_speed_mps=20.0) is None
    assert compute_ttc_s(gap_m=2.0, ego_speed_mps=15.0, lead_speed_mps=20.0) is None


def test_ttc_computed_correctly_when_closing():
    ttc = compute_ttc_s(gap_m=40.0, ego_speed_mps=20.0, lead_speed_mps=0.0)
    assert ttc == pytest.approx(2.0)


def test_ttc_treats_missing_lead_speed_as_matching_ego_speed():
    """A conservative default: unknown lead speed assumes matching speed
    (zero closing speed -> None), not stationary -- an unknown speed
    should not itself manufacture a phantom collision alarm."""
    ttc = compute_ttc_s(gap_m=20.0, ego_speed_mps=10.0, lead_speed_mps=None)
    assert ttc is None


# --- evaluate(): TTC ----------------------------------------------------------

def test_approved_with_no_lead_vehicle_and_normal_state():
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=1.75, lateral_accel_mps2=0.5,
                       sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is True
    assert verdict.risk_level == RISK_NONE
    assert verdict.override_action is None


def test_critical_ttc_overrides_with_emergency_brake():
    # gap=10m, ego 20 m/s, lead stopped -> ttc = 0.5s, well under critical.
    verdict = evaluate(ego_speed_mps=20.0, lateral_offset_m=1.75, lateral_accel_mps2=0.5,
                       sensed_lead_gap_m=10.0, sensed_lead_speed_mps=0.0)
    assert verdict.approved is False
    assert verdict.risk_level == RISK_CRITICAL
    assert verdict.override_action == OVERRIDE_EMERGENCY_BRAKE
    assert verdict.ttc_s < TTC_CRITICAL_S


def test_warning_ttc_flags_but_does_not_override():
    """Between warning and critical, IDM is already the primary response
    to a closing lead vehicle -- the shield surfaces the risk without
    double-braking a car that's already correctly braking."""
    gap = 20.0 * (TTC_CRITICAL_S + TTC_WARNING_S) / 2  # lands squarely between the two thresholds
    verdict = evaluate(ego_speed_mps=20.0, lateral_offset_m=1.75, lateral_accel_mps2=0.5,
                       sensed_lead_gap_m=gap, sensed_lead_speed_mps=0.0)
    assert verdict.risk_level == RISK_MEDIUM
    assert verdict.approved is True
    assert verdict.override_action is None


def test_ample_ttc_is_not_flagged():
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=1.75, lateral_accel_mps2=0.5,
                       sensed_lead_gap_m=500.0, sensed_lead_speed_mps=0.0)
    assert verdict.risk_level == RISK_NONE
    assert verdict.approved is True


# --- evaluate(): road boundary ----------------------------------------------

def test_within_road_boundary_is_not_flagged():
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=ROAD_BOUNDARY_HARD_LIMIT_M - 0.5,
                       lateral_accel_mps2=0.5, sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is True


def test_beyond_road_boundary_overrides_with_recover_low_speed_not_full_stop():
    """Regression: an earlier version used EMERGENCY_BRAKE (full stop) here,
    which created a livelock -- yaw_rate needs forward speed, so a car
    braked to zero off-road can never steer back onto it, and re-triggers
    the same override forever. Must be RECOVER_LOW_SPEED, not a full stop."""
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=ROAD_BOUNDARY_HARD_LIMIT_M + 2.0,
                       lateral_accel_mps2=0.5, sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is False
    assert verdict.risk_level == RISK_CRITICAL
    assert verdict.override_action == OVERRIDE_RECOVER_LOW_SPEED
    assert any("left the road" in r for r in verdict.reasons)


def test_road_boundary_check_is_symmetric():
    """A large NEGATIVE lateral offset (off the other side of the road)
    must be caught too -- this is an abs() check, not a one-sided one."""
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=-(ROAD_BOUNDARY_HARD_LIMIT_M + 2.0),
                       lateral_accel_mps2=0.5, sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is False


# --- evaluate(): vehicle dynamics -------------------------------------------

def test_within_hard_lateral_accel_limit_is_not_flagged():
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=1.75,
                       lateral_accel_mps2=HARD_LATERAL_ACCEL_LIMIT_MPS2 - 0.5,
                       sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is True


def test_beyond_hard_lateral_accel_limit_overrides_with_recover_low_speed():
    verdict = evaluate(ego_speed_mps=15.0, lateral_offset_m=1.75,
                       lateral_accel_mps2=HARD_LATERAL_ACCEL_LIMIT_MPS2 + 1.0,
                       sensed_lead_gap_m=None, sensed_lead_speed_mps=None)
    assert verdict.approved is False
    assert verdict.risk_level == RISK_HIGH
    assert verdict.override_action == OVERRIDE_RECOVER_LOW_SPEED


# --- evaluate(): combined / severity ordering -------------------------------

def test_emergency_brake_takes_precedence_over_recover_low_speed():
    """An imminent collision (TTC critical) must win over a simultaneous
    road-boundary/lateral-accel violation -- collision risk trumps
    recovery-speed preservation, never the other way round."""
    verdict = evaluate(
        ego_speed_mps=20.0,
        lateral_offset_m=ROAD_BOUNDARY_HARD_LIMIT_M + 2.0,  # would request RECOVER_LOW_SPEED
        lateral_accel_mps2=HARD_LATERAL_ACCEL_LIMIT_MPS2 + 1.0,  # would request RECOVER_LOW_SPEED
        sensed_lead_gap_m=10.0, sensed_lead_speed_mps=0.0,  # CRITICAL (ttc) -> EMERGENCY_BRAKE
    )
    assert verdict.risk_level == RISK_CRITICAL
    assert len(verdict.reasons) == 3
    assert verdict.override_action == OVERRIDE_EMERGENCY_BRAKE


def test_recover_low_speed_alone_without_a_collision_risk():
    verdict = evaluate(
        ego_speed_mps=20.0,
        lateral_offset_m=ROAD_BOUNDARY_HARD_LIMIT_M + 2.0,
        lateral_accel_mps2=0.5,
        sensed_lead_gap_m=None, sensed_lead_speed_mps=None,
    )
    assert verdict.override_action == OVERRIDE_RECOVER_LOW_SPEED
