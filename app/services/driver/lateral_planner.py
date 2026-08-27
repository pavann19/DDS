"""Driver-side lateral planning + path tracking (ADR-001, Phase 6.5, item 3).

The Frenet local-planner block extracted **verbatim** from
``PhysicsEngine.update()``'s routed-bicycle branch:

1. generate lateral-offset candidates (``planner.generate_candidates``),
2. pick the lowest-cost one,
3. rate-limit the tracked lateral target toward it,
4. project a pure-pursuit lookahead point and compute the front-wheel angle
   (``planner.pure_pursuit_steering``), clamped to the grip-limited range.

Inputs are the scalar subset of a ``SensorObservation`` this stage needs
(ego proprioception + the forward-sensor gap + the lane-clear query result +
the route's Frenet frame). It is handed **no** ``TrafficModel`` and **no**
NPC list -- ``adjacent_lane_clear`` arrives already reduced to a bool by the
world, exactly as ``TrafficModel.sense_lane_clear`` returns it. Action Item 4
swaps this signature for the full ``SensorObservation`` object.

Behaviour-preserving: same calls, same order, same arithmetic as the former
inline code.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

from app.services.frenet import frenet_to_local_xz, latlng_to_local
from app.services.planner import (
    LateralCandidate,
    generate_candidates,
    pure_pursuit_steering,
    select_best_candidate,
)


@dataclass
class LateralPlan:
    """Result of one lateral-planning tick.

    ``lateral_target_d_m`` is the *new* rate-limited tracked target (the
    caller assigns it back). ``desired_steer_rad`` is already clamped to
    ``steer_limit_rad``; the caller still applies the steering-actuator rate
    limit and the post-rate re-clamp, unchanged.
    """

    candidates: List[LateralCandidate]
    chosen_d_m: float
    lateral_target_d_m: float
    desired_steer_rad: float


def plan_lateral_offset(
    *,
    current_lateral_offset_m: float,
    lead_gap_m: Optional[float],
    adjacent_lane_clear: bool,
    lateral_target_d_m: float,
    frenet_frame,
    current_station_m: float,
    ego_lat: float,
    ego_lng: float,
    heading_deg: float,
    v_mps: float,
    dt: float,
    steer_limit_rad: float,
    lateral_target_rate_mps: float,
    pp_lookahead_k: float,
    pp_lookahead_min_m: float,
    wheelbase_m: float,
) -> LateralPlan:
    candidates = generate_candidates(
        current_d=current_lateral_offset_m,
        lead_gap_m=lead_gap_m,
        adjacent_lane_clear=adjacent_lane_clear,
    )
    best = select_best_candidate(candidates)
    chosen_d_m = best.d_target

    max_d_step = lateral_target_rate_mps * dt
    d_error = chosen_d_m - lateral_target_d_m
    lateral_target_d_m = lateral_target_d_m + max(-max_d_step, min(max_d_step, d_error))

    lookahead_m = pp_lookahead_k * v_mps + pp_lookahead_min_m
    s_lookahead = current_station_m + lookahead_m
    look_x, look_z, _, _ = frenet_to_local_xz(frenet_frame, s_lookahead, lateral_target_d_m)
    ego_x, ego_z = latlng_to_local(
        ego_lat, ego_lng, frenet_frame.origin_lat, frenet_frame.origin_lng,
    )
    dx, dz = look_x - ego_x, look_z - ego_z
    lookahead_dist_m = math.hypot(dx, dz)
    desired_steer = pure_pursuit_steering(heading_deg, dx, dz, lookahead_dist_m, wheelbase_m)
    desired_steer = max(-steer_limit_rad, min(steer_limit_rad, desired_steer))

    return LateralPlan(
        candidates=candidates,
        chosen_d_m=chosen_d_m,
        lateral_target_d_m=lateral_target_d_m,
        desired_steer_rad=desired_steer,
    )
