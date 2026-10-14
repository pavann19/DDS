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

from app.services.frenet import frenet_to_local_xz, signed_curvature_at
from app.services.planner import (
    LateralCandidate,
    ff_stanley_steer,
    generate_candidates,
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
    lane_center_d_m: Optional[float] = None,
    adjacent_lane_d_m: Optional[float] = None,
    road_heading_deg: Optional[float] = None,
    road_curvature_signed_1pm: Optional[float] = None,
) -> LateralPlan:
    # After a completed lane change the "current lane" is the one the ego
    # moved into -- the caller (physics_engine, via the Phase 8 state
    # machine) passes it here so the decoupled planner keeps the car in
    # that lane instead of pulling back to the module-default centre.
    gc_kwargs = {}
    if lane_center_d_m is not None:
        gc_kwargs["lane_center_d"] = lane_center_d_m
    if adjacent_lane_d_m is not None:
        gc_kwargs["adjacent_lane_d"] = adjacent_lane_d_m
    candidates = generate_candidates(
        current_d=current_lateral_offset_m,
        lead_gap_m=lead_gap_m,
        adjacent_lane_clear=adjacent_lane_clear,
        **gc_kwargs,
    )
    best = select_best_candidate(candidates)
    chosen_d_m = best.d_target

    max_d_step = lateral_target_rate_mps * dt
    d_error = chosen_d_m - lateral_target_d_m
    lateral_target_d_m = lateral_target_d_m + max(-max_d_step, min(max_d_step, d_error))

    # --- feed-forward + Stanley feedback path tracking (shared helper) ---
    if road_curvature_signed_1pm is None:
        road_curvature_signed_1pm = signed_curvature_at(frenet_frame, current_station_m)
    if road_heading_deg is None:
        _, _, _t_x, _t_z = frenet_to_local_xz(frenet_frame, current_station_m, 0.0)
        road_heading_deg = math.degrees(math.atan2(_t_x, -_t_z))

    desired_steer = ff_stanley_steer(
        current_d_m=current_lateral_offset_m,
        target_d_m=lateral_target_d_m,
        ego_heading_deg=heading_deg,
        road_heading_deg=road_heading_deg,
        road_curvature_signed_1pm=road_curvature_signed_1pm,
        v_mps=v_mps,
        wheelbase_m=wheelbase_m,
        steer_limit_rad=steer_limit_rad,
    )

    return LateralPlan(
        candidates=candidates,
        chosen_d_m=chosen_d_m,
        lateral_target_d_m=lateral_target_d_m,
        desired_steer_rad=desired_steer,
    )
