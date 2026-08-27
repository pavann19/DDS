"""Unit tests for app/services/driver/lateral_planner.py (ADR-001 item 3).

Locks the extracted lateral-planning stage against the underlying
planner.py primitives it composes. Behaviour-preservation vs. the former
inline code is covered by tests/test_physics_engine.py staying green.
"""
import math

import pytest

from app.services.driver import plan_lateral_offset
from app.services.frenet import build_frenet_frame
from app.services.planner import (
    LANE_CENTER_D_M,
    generate_candidates,
    select_best_candidate,
)

# A ~1 km straight route due north, ~5 m spacing (like smoothed OSRM output).
_ROUTE = [(37.7749 + i * 0.000045, -122.4194) for i in range(200)]
_FRAME = build_frenet_frame(_ROUTE)

_CFG = dict(
    lateral_target_rate_mps=1.0,
    pp_lookahead_k=1.5,
    pp_lookahead_min_m=10.0,
    wheelbase_m=2.8,
)


def _call(**overrides):
    kwargs = dict(
        current_lateral_offset_m=LANE_CENTER_D_M,
        lead_gap_m=None,
        adjacent_lane_clear=False,
        lateral_target_d_m=LANE_CENTER_D_M,
        frenet_frame=_FRAME,
        current_station_m=100.0,
        ego_lat=_ROUTE[20][0],
        ego_lng=_ROUTE[20][1],
        heading_deg=0.0,
        v_mps=13.9,
        dt=0.1,
        steer_limit_rad=math.radians(35.0),
        **_CFG,
    )
    kwargs.update(overrides)
    return plan_lateral_offset(**kwargs)


def test_candidates_match_planner_primitive():
    plan = _call()
    expected = generate_candidates(
        current_d=LANE_CENTER_D_M, lead_gap_m=None, adjacent_lane_clear=False,
    )
    assert [c.d_target for c in plan.candidates] == [c.d_target for c in expected]
    assert plan.chosen_d_m == pytest.approx(select_best_candidate(expected).d_target)


def test_lateral_target_is_rate_limited():
    # Chosen target far from the current tracked target: the move per tick is
    # capped at lateral_target_rate_mps * dt = 0.1 m.
    plan = _call(
        lead_gap_m=10.0, adjacent_lane_clear=True,  # forces a lane-change candidate
        lateral_target_d_m=LANE_CENTER_D_M,
    )
    assert abs(plan.lateral_target_d_m - LANE_CENTER_D_M) <= 0.1 + 1e-9


def test_desired_steer_is_clamped_to_limit():
    tight = math.radians(2.0)
    plan = _call(steer_limit_rad=tight, heading_deg=90.0)  # big heading error
    assert abs(plan.desired_steer_rad) <= tight + 1e-9


def test_straight_route_on_lane_centre_needs_near_zero_steer():
    plan = _call(heading_deg=0.0, current_lateral_offset_m=LANE_CENTER_D_M)
    assert abs(plan.desired_steer_rad) < math.radians(5.0)
