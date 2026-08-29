"""
: local lateral planner (quintic-polynomial-cost candidates in Frenet
`d`) + pure-pursuit lateral control, built on top of frenet.py's exact
station/lateral projection.

Replaces the previous proportional heading controller's steering law. That
controller chased a lookahead *waypoint* directly, which is why the car
tracked the raw route centreline rather than a lane centre -- there was
never a notion of "lane" in the steering law, only "the road". This module
adds that notion explicitly: candidates are lateral OFFSETS from the route
centreline (Frenet `d`), scored, and the winner is followed geometrically
via pure pursuit -- the standard AV-literature split between path planning
(what line to drive) and path tracking (how to steer to stay on it).
"""
import math
from dataclasses import dataclass
from typing import List, Optional

# Matches traffic.py's EGO_LANE_OFFSET_M / LANE_OFFSETS -- the near-side,
# same-direction lane centre under right-hand traffic. Centreline driving
# (d_target = 0) was P6-1's behaviour; a shared lane BOUNDARY (the old
# 3.5) was P6-2's -- both replaced by the real lane-aligned value so a lane
# change (below) has an actual second lane to go to, not a wobble around
# a line that split two lanes down the middle.
LANE_CENTER_D_M = 1.75
# The real far-side same-direction lane (traffic.py's ADJACENT_LANE_OFFSET_M)
# -- the only lane-change target modelled. Passing traffic uses this lane
# too, which is exactly why a lane change needs the sense_lane_clear() gate
# below rather than always being available.
ADJACENT_LANE_D_M = 5.25

# Matches RoadMesh's halfWidth (frontend/DriveScene.tsx) -- the real modelled
# road edge, 7 m either side of the route centreline.
ROAD_HALF_WIDTH_M = 7.0

# Candidate lateral targets, expressed as offsets from LANE_CENTER_D_M --
# small in-lane comfort/safety margins, not lane changes. The actual lane
# change (when triggered) is a SEPARATE candidate at ADJACENT_LANE_D_M,
# added conditionally in generate_candidates() below.
CANDIDATE_OFFSETS_M = (0.0, -1.0, 1.0, -2.0, 2.0)

PLANNING_HORIZON_S = 2.0

W_COMFORT = 1.0
W_SAFETY = 4.0
W_PROGRESS = 0.5

# Below this clearance to the road edge, safety cost starts climbing steeply.
MIN_EDGE_CLEARANCE_M = 2.0

# Below this following gap, the current lane is considered "blocked" --
# this is what makes a lane change a real decision (worth the manoeuvre)
# rather than cosmetic: every in-lane candidate pays BLOCKED_LANE_PENALTY,
# while a verified-clear adjacent-lane candidate does not, so the planner
# only actually prefers changing lanes once staying put has a real,
# quantified cost -- not merely because a lane change exists as an option.
LANE_CHANGE_TRIGGER_GAP_M = 25.0
BLOCKED_LANE_PENALTY = 8.0


@dataclass
class LateralCandidate:
    d_target: float
    cost: float
    comfort_cost: float
    safety_cost: float
    progress_cost: float
    is_lane_change: bool = False


def quintic_lateral_maneuver_cost(d0: float, d1: float, horizon_s: float) -> float:
    """Comfort cost proxy for a quintic-polynomial lateral maneuver from d0 to
    d1 completed over horizon_s (zero lateral velocity/acceleration at both
    ends, the standard boundary conditions for a comfortable lane-centring
    move). A full quintic's peak jerk scales as |delta_d| / T^3 -- this uses
    that scaling directly for candidate RANKING rather than solving the 6
    boundary-condition coefficients per candidate per tick, since only the
    relative ordering of candidates is needed to pick a target; the winning
    d_target is then tracked continuously by pure pursuit + the rate-limited
    blend in physics_engine.py, not by executing the polynomial itself."""
    if horizon_s <= 0:
        return abs(d1 - d0) * 1000.0
    return abs(d1 - d0) / (horizon_s ** 3)


def generate_candidates(
    current_d: float,
    lead_gap_m: Optional[float],
    lane_center_d: float = LANE_CENTER_D_M,
    adjacent_lane_d: float = ADJACENT_LANE_D_M,
    adjacent_lane_clear: bool = False,
    horizon_s: float = PLANNING_HORIZON_S,
) -> List[LateralCandidate]:
    is_blocked = lead_gap_m is not None and lead_gap_m < LANE_CHANGE_TRIGGER_GAP_M

    candidates = []
    for offset in CANDIDATE_OFFSETS_M:
        d_target = lane_center_d + offset
        comfort_cost = quintic_lateral_maneuver_cost(current_d, d_target, horizon_s)

        clearance_to_edge = ROAD_HALF_WIDTH_M - abs(d_target)
        safety_cost = max(0.0, MIN_EDGE_CLEARANCE_M - clearance_to_edge) ** 2
        if is_blocked:
            # Staying in a genuinely blocked lane always costs more once
            # blocked -- flat, not proportional to offset, since no small
            # in-lane wobble actually escapes a same-lane lead vehicle.
            safety_cost += BLOCKED_LANE_PENALTY

        progress_cost = abs(offset)

        cost = W_COMFORT * comfort_cost + W_SAFETY * safety_cost + W_PROGRESS * progress_cost
        candidates.append(LateralCandidate(d_target, cost, comfort_cost, safety_cost, progress_cost))

    if is_blocked and adjacent_lane_clear:
        # The real lane-change candidate: only exists when there is
        # something worth escaping (is_blocked) AND traffic.py's
        # sense_lane_clear() has verified the target lane is actually
        # empty around the ego -- never generated speculatively.
        comfort_cost = quintic_lateral_maneuver_cost(current_d, adjacent_lane_d, horizon_s)
        clearance_to_edge = ROAD_HALF_WIDTH_M - abs(adjacent_lane_d)
        safety_cost = max(0.0, MIN_EDGE_CLEARANCE_M - clearance_to_edge) ** 2  # no blocked penalty -- verified clear
        progress_cost = abs(adjacent_lane_d - lane_center_d)
        cost = W_COMFORT * comfort_cost + W_SAFETY * safety_cost + W_PROGRESS * progress_cost
        candidates.append(LateralCandidate(adjacent_lane_d, cost, comfort_cost, safety_cost, progress_cost,
                                           is_lane_change=True))

    return candidates


def select_best_candidate(candidates: List[LateralCandidate]) -> LateralCandidate:
    return min(candidates, key=lambda c: c.cost)


def pure_pursuit_steering(
    heading_deg: float,
    lookahead_dx: float,
    lookahead_dz: float,
    lookahead_dist_m: float,
    wheelbase_m: float,
) -> float:
    """Standard pure-pursuit steering law: delta = atan(2*L*sin(alpha)/Ld),
    where alpha is the angle between the vehicle's heading and the line to
    the lookahead point (Coulter, 1992).

    (lookahead_dx, lookahead_dz) is the lookahead point's offset from the
    vehicle in the SAME local (x, z) frame frenet.py projects into: x=East,
    z=South (negated latitude), matching frontend/DriveScene.tsx's
    `toLocalXZ`/`headingToForward` convention (forward at heading h is
    (sin(h), -cos(h))) -- required so this steering law and the Frenet frame
    agree on what "forward" means; a sign mismatch here would silently steer
    the wrong way exactly like the frontend's own heading-sign bug found and
    fixed 2026-07-20 (see DriveScene.tsx's headingToForward comment)."""
    if lookahead_dist_m < 1e-6:
        return 0.0
    heading_rad = math.radians(heading_deg)
    target_bearing_rad = math.atan2(lookahead_dx, -lookahead_dz)
    alpha = target_bearing_rad - heading_rad
    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
    return math.atan2(2.0 * wheelbase_m * math.sin(alpha), lookahead_dist_m)
