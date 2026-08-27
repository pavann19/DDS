"""
Unit tests for app/services/planner.py -- the candidate lateral-offset
scoring and pure-pursuit steering geometry.
"""
import math
import pytest

from app.services.planner import (
    LANE_CENTER_D_M,
    ADJACENT_LANE_D_M,
    ROAD_HALF_WIDTH_M,
    LANE_CHANGE_TRIGGER_GAP_M,
    generate_candidates,
    select_best_candidate,
    pure_pursuit_steering,
)


def test_lane_center_wins_with_no_lead_vehicle_and_already_centred():
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=None)
    best = select_best_candidate(candidates)
    assert best.d_target == pytest.approx(LANE_CENTER_D_M)


def test_candidates_near_the_road_edge_are_penalised():
    # LANE_CENTER_D_M (the real near-lane centre, 1.75m) sits comfortably
    # inside the 7m road edge, so exercise the edge-clearance penalty
    # explicitly near the edge instead of relying on the ego's own default
    # lane happening to be close to it.
    near_edge_lane = ROAD_HALF_WIDTH_M - 1.0
    candidates = generate_candidates(current_d=near_edge_lane, lead_gap_m=None, lane_center_d=near_edge_lane)
    by_offset = {round(c.d_target - near_edge_lane, 3): c for c in candidates}
    # +2m candidate (closer to the 7m edge) must cost strictly more on safety
    # than the centred (0m) candidate.
    assert by_offset[2.0].safety_cost > by_offset[0.0].safety_cost


def test_a_candidate_beyond_the_road_half_width_is_heavily_penalised():
    candidates = generate_candidates(current_d=0.0, lead_gap_m=None,
                                      lane_center_d=ROAD_HALF_WIDTH_M - 0.5)
    off_road = max(candidates, key=lambda c: c.d_target)
    assert off_road.safety_cost > 0.0


def test_tight_lead_gap_makes_staying_exactly_in_lane_relatively_costlier():
    loose = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=80.0)
    tight = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=3.0)
    loose_center = next(c for c in loose if c.d_target == LANE_CENTER_D_M)
    tight_center = next(c for c in tight if c.d_target == LANE_CENTER_D_M)
    assert tight_center.safety_cost > loose_center.safety_cost


def test_far_lead_gap_does_not_affect_candidate_scoring():
    far = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=90.0)
    none_ = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=None)
    for a, b in zip(sorted(far, key=lambda c: c.d_target), sorted(none_, key=lambda c: c.d_target)):
        assert a.cost == pytest.approx(b.cost)


def test_a_large_lateral_jump_costs_more_than_a_small_one():
    """Comfort cost must reward staying near the current lateral position --
    otherwise the planner would happily snap between candidates every tick."""
    candidates = generate_candidates(current_d=0.0, lead_gap_m=None, lane_center_d=0.0)
    by_offset = {c.d_target: c for c in candidates}
    assert by_offset[2.0].comfort_cost > by_offset[1.0].comfort_cost > by_offset[0.0].comfort_cost


def test_pure_pursuit_zero_steer_for_a_point_directly_ahead():
    """Heading due east (90deg); a lookahead point purely east of the car
    (dx>0, dz=0 in the local x=East,z=South frame) must require zero steer."""
    delta = pure_pursuit_steering(
        heading_deg=90.0, lookahead_dx=10.0, lookahead_dz=0.0,
        lookahead_dist_m=10.0, wheelbase_m=2.8,
    )
    assert delta == pytest.approx(0.0, abs=1e-9)


def test_pure_pursuit_steers_left_for_a_target_to_the_left():
    """Heading due north (0deg); a lookahead point to the west (dx<0) is to
    the driver's left and must produce a negative (left) steer angle."""
    delta = pure_pursuit_steering(
        heading_deg=0.0, lookahead_dx=-5.0, lookahead_dz=-10.0,
        lookahead_dist_m=math.hypot(5.0, 10.0), wheelbase_m=2.8,
    )
    assert delta < 0.0


def test_pure_pursuit_steers_right_for_a_target_to_the_right():
    delta = pure_pursuit_steering(
        heading_deg=0.0, lookahead_dx=5.0, lookahead_dz=-10.0,
        lookahead_dist_m=math.hypot(5.0, 10.0), wheelbase_m=2.8,
    )
    assert delta > 0.0


def test_pure_pursuit_steer_grows_with_sharper_offset_angle():
    shallow = pure_pursuit_steering(0.0, 2.0, -10.0, math.hypot(2.0, 10.0), 2.8)
    sharp = pure_pursuit_steering(0.0, 8.0, -10.0, math.hypot(8.0, 10.0), 2.8)
    assert abs(sharp) > abs(shallow)


def test_pure_pursuit_zero_lookahead_distance_returns_zero_steer():
    assert pure_pursuit_steering(0.0, 1.0, 1.0, 0.0, 2.8) == 0.0


# ---------------------------------------------------------------------------
# Real lane-change candidate: a genuine decision (not a fictional in-lane
# wobble) that only exists once the lane is blocked AND the adjacent lane
# is verified clear -- see BLOCKED_LANE_PENALTY's docstring for why.
# ---------------------------------------------------------------------------

def test_no_lane_change_candidate_when_lane_is_not_blocked():
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=None, adjacent_lane_clear=True)
    assert not any(c.is_lane_change for c in candidates)


def test_no_lane_change_candidate_when_gap_is_ample():
    ample_gap = LANE_CHANGE_TRIGGER_GAP_M + 50.0
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=ample_gap, adjacent_lane_clear=True)
    assert not any(c.is_lane_change for c in candidates)


def test_no_lane_change_candidate_when_blocked_but_adjacent_lane_not_clear():
    tight_gap = LANE_CHANGE_TRIGGER_GAP_M - 5.0
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=tight_gap, adjacent_lane_clear=False)
    assert not any(c.is_lane_change for c in candidates)


def test_lane_change_candidate_exists_when_blocked_and_adjacent_lane_clear():
    tight_gap = LANE_CHANGE_TRIGGER_GAP_M - 5.0
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=tight_gap, adjacent_lane_clear=True)
    lane_change = [c for c in candidates if c.is_lane_change]
    assert len(lane_change) == 1
    assert lane_change[0].d_target == pytest.approx(ADJACENT_LANE_D_M)


def test_lane_change_candidate_wins_when_blocked_and_clear():
    """The headline behaviour: once genuinely blocked with a verified-clear
    adjacent lane, the planner must actually PREFER changing lanes, not
    just generate the option and still pick an in-lane candidate."""
    tight_gap = LANE_CHANGE_TRIGGER_GAP_M - 15.0
    candidates = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=tight_gap, adjacent_lane_clear=True)
    best = select_best_candidate(candidates)
    assert best.is_lane_change is True


def test_blocked_lane_penalty_applies_to_every_in_lane_candidate():
    tight_gap = LANE_CHANGE_TRIGGER_GAP_M - 5.0
    blocked = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=tight_gap, adjacent_lane_clear=False)
    unblocked = generate_candidates(current_d=LANE_CENTER_D_M, lead_gap_m=None, adjacent_lane_clear=False)
    for b, u in zip(sorted(blocked, key=lambda c: c.d_target), sorted(unblocked, key=lambda c: c.d_target)):
        assert b.safety_cost > u.safety_cost
