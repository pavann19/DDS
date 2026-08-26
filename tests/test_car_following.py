"""
Unit tests for app/services/car_following.py -- the IDM longitudinal
car-following model.
"""
import pytest

from app.services.car_following import idm_acceleration, MIN_GAP_M


def test_no_lead_vehicle_returns_none():
    assert idm_acceleration(v_mps=15.0, v0_mps=20.0, gap_m=None, lead_speed_mps=None, a_max_mps2=3.0) is None


def test_far_lead_vehicle_at_same_speed_gives_near_zero_interaction():
    """A lead vehicle far away at matching speed shouldn't meaningfully
    constrain acceleration -- the interaction term should be small."""
    a_free = idm_acceleration(v_mps=15.0, v0_mps=20.0, gap_m=None, lead_speed_mps=None, a_max_mps2=3.0)
    a_far_lead = idm_acceleration(v_mps=15.0, v0_mps=20.0, gap_m=500.0, lead_speed_mps=15.0, a_max_mps2=3.0)
    assert a_far_lead is not None
    # Free-road term alone (a_free is None here since gap=None short-circuits;
    # recompute the free-road component directly for comparison).
    free_road_term = 1.0 - (15.0 / 20.0) ** 4
    expected_far = 3.0 * free_road_term  # interaction term ~0 at 500m
    assert a_far_lead == pytest.approx(expected_far, abs=0.05)


def test_closing_on_a_stopped_lead_vehicle_produces_strong_braking():
    a = idm_acceleration(v_mps=20.0, v0_mps=20.0, gap_m=15.0, lead_speed_mps=0.0, a_max_mps2=3.0)
    assert a is not None
    assert a < -2.0, "must brake hard when closing fast on a stopped lead vehicle at a short gap"


def test_tight_gap_at_matching_speed_still_brakes_below_min_gap():
    """Even at zero relative speed, a gap below the desired time-gap
    headway must produce braking (not zero/positive accel), because the
    desired gap includes a nonzero minimum (s0) and time-headway term."""
    a = idm_acceleration(v_mps=20.0, v0_mps=20.0, gap_m=3.0, lead_speed_mps=20.0, a_max_mps2=3.0)
    assert a is not None
    assert a < 0.0


def test_ample_gap_and_below_cruise_speed_gives_positive_acceleration():
    a = idm_acceleration(v_mps=5.0, v0_mps=20.0, gap_m=200.0, lead_speed_mps=20.0, a_max_mps2=3.0)
    assert a is not None
    assert a > 0.0


def test_at_cruise_speed_with_ample_gap_acceleration_is_near_zero():
    a = idm_acceleration(v_mps=20.0, v0_mps=20.0, gap_m=200.0, lead_speed_mps=20.0, a_max_mps2=3.0)
    assert a is not None
    assert abs(a) < 0.1


def test_zero_or_negative_desired_speed_does_not_divide_by_zero():
    a = idm_acceleration(v_mps=5.0, v0_mps=0.0, gap_m=50.0, lead_speed_mps=5.0, a_max_mps2=3.0)
    assert a is not None  # must not raise ZeroDivisionError
    assert a <= 0.0  # no free-road drive to accelerate toward with v0=0


def test_gap_below_min_gap_still_returns_a_finite_large_braking_value():
    """Guards against a near-zero gap producing a division blow-up: the
    interaction term denominator is floored, so this must stay finite."""
    a = idm_acceleration(v_mps=10.0, v0_mps=20.0, gap_m=0.01, lead_speed_mps=0.0, a_max_mps2=3.0)
    assert a is not None
    assert math_isfinite(a)
    assert a < -3.0


def math_isfinite(x):
    import math
    return math.isfinite(x)


def test_faster_lead_vehicle_relaxes_braking_vs_stopped_lead():
    a_fast_lead = idm_acceleration(v_mps=20.0, v0_mps=20.0, gap_m=15.0, lead_speed_mps=18.0, a_max_mps2=3.0)
    a_stopped_lead = idm_acceleration(v_mps=20.0, v0_mps=20.0, gap_m=15.0, lead_speed_mps=0.0, a_max_mps2=3.0)
    assert a_fast_lead > a_stopped_lead
