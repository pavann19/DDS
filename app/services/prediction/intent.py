"""Maneuver-intent estimation (Phase 7).

Turns an agent's *lane-relative* kinematics into a probability distribution
over five intents:

    LANE_KEEP  MERGE_LEFT  MERGE_RIGHT  DECELERATING  STOPPING

and a single ``p_cut_in`` -- the probability the agent is merging toward the
ego's lane -- plus the estimated time until it crosses the lane divider
between them.

Deliberately a small, interpretable scoring model, not a learned one: every
term is a named physical quantity with a documented reference scale, so a
reviewer can see exactly why a given verdict came out the way it did (the
same auditability line the rest of this project holds).

The key input is ``agent_v_d`` -- lateral drift measured in the route's
Frenet frame (see ``forecaster.project_agent_frenet``). An agent tracking a
curve has ``agent_v_d ~= 0`` however hard its Cartesian heading is turning,
which is what keeps this from firing on every bend (Gate 7.3). A sustained
real drift toward the ego (Gate 7.1: 0.4 m/s) drives ``p_cut_in`` well
above the 0.65 action threshold.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Sequence

from app.services.frenet import FrenetFrame
from app.services.prediction.forecaster import (
    DEFAULT_LANE_CENTERS_M,
    project_agent_frenet,
)


class Intent(str, Enum):
    LANE_KEEP = "LANE_KEEP"
    MERGE_LEFT = "MERGE_LEFT"
    MERGE_RIGHT = "MERGE_RIGHT"
    DECELERATING = "DECELERATING"
    STOPPING = "STOPPING"


# Drift speed (m/s, Frenet lateral) that reads as "clearly merging". Set so
# Gate 7.1's 0.4 m/s sits comfortably past it.
MERGE_DRIFT_REF_MPS = 0.16
# Below this |drift| nothing counts as a merge at all -- kills tracker noise.
MERGE_DRIFT_DEADBAND_MPS = 0.06
MERGE_GAIN = 10.0
# Extra weight when the agent is also close to the lane boundary it is
# drifting toward (it is committing, not just wandering).
BOUNDARY_PROXIMITY_BOOST = 1.6
LANE_HALF_WIDTH_M = 1.75

# Longitudinal deceleration (m/s^2, positive number) scales.
DECEL_DEADBAND_MPS2 = 0.6
DECEL_REF_MPS2 = 2.5
DECEL_GAIN = 3.0
# A near-stopped, hard-braking agent is STOPPING, not merely DECELERATING.
STOPPING_SPEED_MPS = 3.0
STOPPING_DECEL_MPS2 = 1.2

LANE_KEEP_BASE = 1.0

# Action threshold from the roadmap: P(MERGE toward ego) above this triggers
# proactive clearance.
CUT_IN_ACTION_THRESHOLD = 0.65


@dataclass
class IntentEstimate:
    distribution: Dict[str, float]
    p_cut_in: float
    dominant: str
    time_to_cross_s: Optional[float]

    def is_cut_in(self, threshold: float = CUT_IN_ACTION_THRESHOLD) -> bool:
        return self.p_cut_in > threshold


def _sigmoid(z: float) -> float:
    if z < -60.0:
        return 0.0
    return 1.0 / (1.0 + math.exp(-z))


def estimate_intent(
    *,
    agent_d: float,
    agent_v_d: float,
    agent_a_long_mps2: float,
    agent_speed_mps: float,
    ego_d: float,
    lane_half_width_m: float = LANE_HALF_WIDTH_M,
    lane_centers: Sequence[float] = DEFAULT_LANE_CENTERS_M,
) -> IntentEstimate:
    """All inputs are Frenet/lane-relative. ``agent_v_d`` > 0 is drift to the
    right (matching frenet.py's sign convention); ``agent_a_long_mps2`` < 0
    is braking. ``lane_centers`` is the set of modelled lane-centre offsets
    the agent is snapped to (they are NOT on a zero-based grid).
    """
    # --- merge scores -------------------------------------------------
    drift = agent_v_d
    drift_mag = abs(drift)
    if drift_mag <= MERGE_DRIFT_DEADBAND_MPS:
        merge_mag = 0.0
    else:
        merge_mag = _sigmoid(MERGE_GAIN * (drift_mag - MERGE_DRIFT_REF_MPS))

    # Which lane boundary is the agent heading for, and is it close to it?
    lane_center = min(lane_centers, key=lambda c: abs(c - agent_d))
    d_in_lane = agent_d - lane_center
    heading_for_boundary = (drift > 0 and d_in_lane > 0) or (drift < 0 and d_in_lane < 0)
    boundary_gap = max(0.0, lane_half_width_m - abs(d_in_lane))
    proximity = (1.0 - boundary_gap / lane_half_width_m) if heading_for_boundary else 0.0
    merge_mag *= 1.0 + BOUNDARY_PROXIMITY_BOOST * max(0.0, proximity)

    s_merge_left = merge_mag if drift < 0 else 0.0
    s_merge_right = merge_mag if drift > 0 else 0.0

    # --- longitudinal scores ---------------------------------------------
    decel = max(0.0, -agent_a_long_mps2)
    if decel <= DECEL_DEADBAND_MPS2:
        s_decel = 0.0
    else:
        s_decel = DECEL_GAIN * _sigmoid(3.0 * (decel - DECEL_REF_MPS2) / DECEL_REF_MPS2)
    s_stop = 0.0
    if agent_speed_mps < STOPPING_SPEED_MPS and decel > STOPPING_DECEL_MPS2:
        s_stop = DECEL_GAIN * 1.5
        s_decel *= 0.3  # STOPPING absorbs most of the DECELERATING mass

    # --- lane keep: strong by default, eroded by any active maneuver ----
    s_keep = LANE_KEEP_BASE / (1.0 + 3.0 * merge_mag + 0.5 * s_decel + 1.0 * s_stop)

    scores = {
        Intent.LANE_KEEP.value: s_keep,
        Intent.MERGE_LEFT.value: s_merge_left,
        Intent.MERGE_RIGHT.value: s_merge_right,
        Intent.DECELERATING.value: s_decel,
        Intent.STOPPING.value: s_stop,
    }
    total = sum(scores.values()) or 1.0
    dist = {k: v / total for k, v in scores.items()}

    # --- cut-in: the merge component that heads toward the ego ----------
    if ego_d < agent_d:
        p_cut_in = dist[Intent.MERGE_LEFT.value]      # agent is right of ego, drifting left
    elif ego_d > agent_d:
        p_cut_in = dist[Intent.MERGE_RIGHT.value]
    else:
        p_cut_in = dist[Intent.MERGE_LEFT.value] + dist[Intent.MERGE_RIGHT.value]

    # --- time to cross the divider between agent and ego ----------------
    ttc: Optional[float] = None
    toward_ego = (ego_d - agent_d) * drift > 0
    if toward_ego and drift_mag > MERGE_DRIFT_DEADBAND_MPS:
        # Distance from the agent to the boundary of its current lane on the
        # ego's side.
        boundary_on_ego_side = lane_center + math.copysign(lane_half_width_m, ego_d - agent_d)
        ttc = abs(boundary_on_ego_side - agent_d) / drift_mag

    dominant = max(dist, key=dist.get)
    return IntentEstimate(distribution=dist, p_cut_in=p_cut_in, dominant=dominant, time_to_cross_s=ttc)


def estimate_intent_from_track(
    *,
    frame: Optional[FrenetFrame],
    x: float,
    z: float,
    vx: float,
    vz: float,
    a_long_mps2: float,
    ego_d: float,
    lane_half_width_m: float = LANE_HALF_WIDTH_M,
) -> IntentEstimate:
    """Convenience wrapper: project the agent's Cartesian state into the
    route frame first, then score. Without a frame, lateral drift is taken
    as the raw ``vx`` (ego-local East), which is only meaningful on a roughly
    straight heading -- callers with a route should always pass ``frame``.
    """
    if frame is not None:
        _, agent_d, _, agent_v_d, _ = project_agent_frenet(frame, x, z, vx, vz)
    else:
        agent_d, agent_v_d = x, vx
    speed = math.hypot(vx, vz)
    return estimate_intent(
        agent_d=agent_d,
        agent_v_d=agent_v_d,
        agent_a_long_mps2=a_long_mps2,
        agent_speed_mps=speed,
        ego_d=ego_d,
        lane_half_width_m=lane_half_width_m,
    )
