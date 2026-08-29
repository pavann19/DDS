"""Motion planning package.

- ``lateral``: the Phase 6.2 decoupled lateral-offset candidate planner
  (in-lane comfort offsets + a conditional lane-change candidate) plus the
  pure-pursuit steering law. Kept as the tracking/fallback layer.
- ``polynomials``: quintic / quartic Frenet trajectory primitives.
- ``spatiotemporal``: the Phase 8 joint ``(s, d, t)`` planner that replaces
  the decoupled lateral-then-longitudinal decision.
- ``state_machine``: the lane-change state machine (LANE_KEEP / PREPARE /
  EXECUTE / ABORT) that gates which end states the joint planner may target.

The public names of the former ``planner.py`` module are re-exported here
unchanged, so ``from app.services.planner import LANE_CENTER_D_M`` etc.
keep working.
"""
from app.services.planner.lateral import (  # noqa: F401
    ADJACENT_LANE_D_M,
    BLOCKED_LANE_PENALTY,
    CANDIDATE_OFFSETS_M,
    LANE_CENTER_D_M,
    LANE_CHANGE_TRIGGER_GAP_M,
    MIN_EDGE_CLEARANCE_M,
    PLANNING_HORIZON_S,
    ROAD_HALF_WIDTH_M,
    W_COMFORT,
    W_PROGRESS,
    W_SAFETY,
    LateralCandidate,
    generate_candidates,
    pure_pursuit_steering,
    quintic_lateral_maneuver_cost,
    select_best_candidate,
)

__all__ = [
    "ADJACENT_LANE_D_M",
    "BLOCKED_LANE_PENALTY",
    "CANDIDATE_OFFSETS_M",
    "LANE_CENTER_D_M",
    "LANE_CHANGE_TRIGGER_GAP_M",
    "MIN_EDGE_CLEARANCE_M",
    "PLANNING_HORIZON_S",
    "ROAD_HALF_WIDTH_M",
    "W_COMFORT",
    "W_PROGRESS",
    "W_SAFETY",
    "LateralCandidate",
    "generate_candidates",
    "pure_pursuit_steering",
    "quintic_lateral_maneuver_cost",
    "select_best_candidate",
]
