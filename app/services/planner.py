"""
P6-2: local lateral planner (quintic-polynomial-cost candidates in Frenet
`d`) + pure-pursuit lateral control, built on top of frenet.py's exact
station/lateral projection.

Replaces the P6-1 proportional heading controller's steering law. That
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
# (d_target = 0) was the P6-1 behaviour this task replaces.
LANE_CENTER_D_M = 3.5

# Matches RoadMesh's halfWidth (frontend/DriveScene.tsx) -- the real modelled
# road edge, 7 m either side of the route centreline.
ROAD_HALF_WIDTH_M = 7.0

# Candidate lateral targets, expressed as offsets from LANE_CENTER_D_M. Small
# and symmetric: this is a lane-KEEPING planner (comfort/safety margin within
# the lane), not a lane-change planner -- MOBIL-style lane changing is P6-4's
# stretch scope for NPCs, not the ego here.
CANDIDATE_OFFSETS_M = (0.0, -1.0, 1.0, -2.0, 2.0)

PLANNING_HORIZON_S = 2.0

W_COMFORT = 1.0
W_SAFETY = 4.0
W_PROGRESS = 0.5

# Below this clearance to the road edge, safety cost starts climbing steeply.
MIN_EDGE_CLEARANCE_M = 2.0
# Below this following gap, candidates that keep the ego in-lane accrue a
# small extra cost -- a *bias* toward finding room to the side, not a lane
# change decision by itself (P6-3's IDM still owns the longitudinal
# response to the same sensed gap).
TIGHT_GAP_M = 15.0


@dataclass
class LateralCandidate:
    d_target: float
    cost: float
    comfort_cost: float
    safety_cost: float
    progress_cost: float


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
    horizon_s: float = PLANNING_HORIZON_S,
) -> List[LateralCandidate]:
    candidates = []
    for offset in CANDIDATE_OFFSETS_M:
        d_target = lane_center_d + offset
        comfort_cost = quintic_lateral_maneuver_cost(current_d, d_target, horizon_s)

        clearance_to_edge = ROAD_HALF_WIDTH_M - abs(d_target)
        safety_cost = max(0.0, MIN_EDGE_CLEARANCE_M - clearance_to_edge) ** 2
        if lead_gap_m is not None and lead_gap_m < TIGHT_GAP_M and abs(offset) < 0.5:
            safety_cost += (TIGHT_GAP_M - lead_gap_m) * 0.3

        progress_cost = abs(offset)

        cost = W_COMFORT * comfort_cost + W_SAFETY * safety_cost + W_PROGRESS * progress_cost
        candidates.append(LateralCandidate(d_target, cost, comfort_cost, safety_cost, progress_cost))
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
