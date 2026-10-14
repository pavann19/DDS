"""The sensor-derived observation a learned driving component is allowed to see.

This module deliberately imports NOTHING that carries privileged simulator
state (no ``traffic``, no ``NpcVehicle``). ``build_observation`` accepts only
plain sensor/derived values and raises if handed a ground-truth traffic
object. The *content* of the observation (exact fields, encoding) is an open
I2 decision -- what is fixed here is the BOUNDARY: this is the single choke
point through which a learned component receives the world, and it is
sensor-only by construction and by test (tests/test_observation_boundary.py).

I2 revision (see docs/I2_EXPERIMENT_A_FINDINGS.md §4):
  * ``AgentSlot`` is now **role-typed** (ego/left/right lane × lead/follower +
    2 "other nearest") instead of nearest-8-by-raw-range -- removes slot
    saturation and role mixing.
  * per-agent ``d_rate_mps`` / ``d_accel_mps2`` (de-biased, from the
    prediction stage) + ``track_age_ticks`` + ``pos_sigma_m`` (EKF
    uncertainty proxy) -- the noise-robust temporal/uncertainty substrate a
    learned head needs so it does not have to consume the fragile heuristic
    ``p_cut_in``.
  * ``curvature_profile_1pm`` (curvature sampled at several look-ahead
    distances) + ``dist_to_next_maneuver_m`` -- road context ahead, not just
    at the current station.
The heuristic intent scalars (``p_cut_in`` etc.) are retained as ONE input
among many, no longer the sole temporal signal.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# Fixed-width masked agent set (permutation handled by the consumer).
MAX_AGENTS = 8

# Role-typed slot layout. Slots 0..5 are the six lane roles (masked if that
# role is empty this tick); slots 6..7 are the nearest remaining agents of any
# other type. Fixed order => a flat consumer needs no permutation handling.
ROLE_SLOTS: Tuple[str, ...] = (
    "ego_lead", "ego_follow",
    "left_lead", "left_follow",
    "right_lead", "right_follow",
)
_N_OTHER_SLOTS = MAX_AGENTS - len(ROLE_SLOTS)

# Lane geometry for role typing (highway prior; a real lane graph lands with
# the Phase-10 semantic map). dd > 0 == agent is to the ego's RIGHT
# (azimuth = atan2(right, forward), see perception/sensor_rig.py).
_LANE_HALF_WIDTH_M = 1.75
_ADJACENT_LANE_MAX_M = 5.25

# Type names that must never appear as build_observation arguments -- a
# defence in depth alongside the static-import test.
_FORBIDDEN_TYPE_NAMES = {"TrafficModel", "NpcVehicle", "PhysicsEngine"}


@dataclass(frozen=True)
class AgentSlot:
    """One sensor-tracked nearby agent, ego/road-relative. Every value is
    derivable from a SurroundTrack + the prediction stage + the route frame --
    never from ground truth."""
    valid: bool = False
    role: str = "other"           # one of ROLE_SLOTS, or "other"
    ds_m: float = 0.0             # longitudinal gap along the ego axis (m), + ahead
    closing_mps: float = 0.0      # closing speed (m/s), + = ego closing
    dd_m: float = 0.0             # lateral offset from the ego (m), + = right
    d_rate_mps: float = 0.0       # de-biased lane-relative lateral rate (prediction stage)
    d_accel_mps2: float = 0.0     # de-biased longitudinal accel (prediction stage)
    lateral_rate_mps: float = 0.0  # == d_rate_mps (kept for back-compat; was always 0)
    p_cut_in: float = 0.0         # heuristic intent -- ONE input, not the sole temporal signal
    p_decel: float = 0.0
    p_lane_keep: float = 1.0
    length_m: float = 4.3
    track_confidence: float = 0.0  # 1.0 fresh CONFIRMED, lower when coasted
    track_age_ticks: int = 0      # how long this track has been alive
    pos_sigma_m: float = 0.0      # EKF position-uncertainty proxy (m)


@dataclass(frozen=True)
class Observation:
    """Everything a learned component may condition on. Sensor-derived only."""
    # ego proprioception
    speed_mps: float
    accel_mps2: float
    lane_offset_m: float           # ego Frenet d (localisation, not privileged)
    lateral_rate_mps: float
    heading_err_rad: float         # ego heading minus road tangent
    road_curvature_1pm: float      # signed, low-passed, from the map
    friction_estimate: float       # a grip-SENSOR estimate (see NOTE below)
    dist_to_goal_bucket: int       # coarse route-progress bucket, not exact metres

    # forward collision corridor (sense_lead_vehicle) -- a dedicated 1-D
    # forward sensor, longer range than the surround rig. Retained alongside
    # the role-typed ``ego_lead`` slot; a learned head may use either.
    lead_gap_m: Optional[float]    # bumper-to-bumper; None = corridor clear
    lead_closing_mps: Optional[float]

    # lane structure (highway prior until the Phase 10 semantic map)
    n_same_direction_lanes: int
    ego_lane_index: int
    left_lane_clear: bool
    right_lane_clear: bool

    # which deterministic constraint is currently binding
    binding_constraint: str

    # masked, role-typed agent set (fixed width MAX_AGENTS, fixed slot order)
    agents: Tuple[AgentSlot, ...] = field(default_factory=tuple)

    # road context AHEAD (not just at the current station)
    curvature_profile_1pm: Tuple[float, ...] = ()        # signed curvature at each look-ahead
    curvature_profile_lookahead_m: Tuple[float, ...] = ()  # the look-ahead distances (m)
    dist_to_next_maneuver_m: float = 0.0                 # to the next significant bend, capped

    # NOTE on friction_estimate: a real vehicle estimates road grip from
    # wheel-slip / IMU / vision. DDS currently exposes friction_mu as a known
    # scalar; it is included here LABELLED AS AN ESTIMATE so a learned
    # component treats it as a (potentially wrong) sensor input, not an
    # oracle. A future grip-estimation model replaces this value with no
    # interface change. Tracked in docs/OBSERVATION_BOUNDARY.md.


def _reject_privileged(**kwargs) -> None:
    for name, val in kwargs.items():
        if type(val).__name__ in _FORBIDDEN_TYPE_NAMES:
            raise TypeError(
                f"build_observation received a privileged simulator object "
                f"({name}={type(val).__name__}). The learned pipeline may only "
                f"see sensor-derived values -- pass SurroundPerceptionEngine "
                f"output, not the world model."
            )
        if isinstance(val, (list, tuple)) and val and \
                type(val[0]).__name__ in _FORBIDDEN_TYPE_NAMES:
            raise TypeError(
                f"build_observation received a sequence of "
                f"{type(val[0]).__name__} ({name}) -- that is ground truth, "
                f"not perception."
            )


def _role_for(ds_m: float, dd_m: float) -> str:
    """Role type from ego-relative longitudinal/lateral offset. dd > 0 = right."""
    a = abs(dd_m)
    if a < _LANE_HALF_WIDTH_M:
        lane = "ego"
    elif a < _ADJACENT_LANE_MAX_M:
        lane = "right" if dd_m > 0.0 else "left"
    else:
        return "other"
    return f"{lane}_{'lead' if ds_m >= 0.0 else 'follow'}"


def build_observation(
    *,
    ego_speed_mps: float,
    ego_accel_mps2: float,
    ego_lane_offset_m: float,
    ego_lateral_rate_mps: float,
    ego_heading_err_rad: float,
    road_curvature_1pm: float,
    friction_estimate: float,
    dist_to_goal_m: float,
    lead_gap_m: Optional[float],
    lead_speed_mps: Optional[float],
    surround_tracks: Sequence,          # SurroundTrack list (sensor output)
    intent_by_track: dict,             # {track_id: IntentEstimate} (sensor-only)
    left_lane_clear: bool,
    right_lane_clear: bool,
    n_same_direction_lanes: int = 3,
    same_direction_lane_centres_m: Sequence[float] = (1.75, 5.25),
    binding_constraint: str = "cruise",
    route_tangent_heading_rad: float = 0.0,
    # --- I2 substrate (all optional; default to the pre-I2 behaviour) ---
    lateral_rate_by_track: Optional[Dict[int, float]] = None,
    long_accel_by_track: Optional[Dict[int, float]] = None,
    curvature_profile_1pm: Sequence[float] = (),
    curvature_profile_lookahead_m: Sequence[float] = (),
    dist_to_next_maneuver_m: float = 0.0,
) -> Observation:
    """Assemble the Observation from sensor-derived inputs only.

    ``surround_tracks`` MUST be SurroundPerceptionEngine output (each item
    exposes ``track_id, x, z, vx, vz, range_m, azimuth_deg, status,
    length_m, age_ticks, pos_sigma_m``), NOT ``traffic.npcs``. A ground-truth
    object here raises TypeError.
    """
    _reject_privileged(surround_tracks=surround_tracks)
    lateral_rate_by_track = lateral_rate_by_track or {}
    long_accel_by_track = long_accel_by_track or {}

    # ego lane index from its localised offset (was hardcoded 0)
    ego_lane_index = min(
        range(len(same_direction_lane_centres_m)),
        key=lambda i: abs(same_direction_lane_centres_m[i] - ego_lane_offset_m),
    ) if same_direction_lane_centres_m else 0

    lead_closing = None
    if lead_gap_m is not None and lead_speed_mps is not None:
        lead_closing = ego_speed_mps - lead_speed_mps

    # ---- build a candidate slot per track, then role-type it --------------
    @dataclass
    class _Cand:
        slot: AgentSlot
        role: str
        ds_abs: float
        range_m: float

    cands: List[_Cand] = []
    for t in surround_tracks:
        rng = float(getattr(t, "range_m", 0.0))
        az = math.radians(float(getattr(t, "azimuth_deg", 0.0)))
        ds = rng * math.cos(az)
        dd = rng * math.sin(az)
        vx, vz = float(getattr(t, "vx", 0.0)), float(getattr(t, "vz", 0.0))
        agent_fwd = (
            vx * math.cos(route_tangent_heading_rad)
            - vz * math.sin(route_tangent_heading_rad)
        )
        closing = ego_speed_mps - agent_fwd
        tid = int(getattr(t, "track_id", -1))
        est = intent_by_track.get(tid)
        p_cut = float(getattr(est, "p_cut_in", 0.0)) if est is not None else 0.0
        dist = getattr(est, "distribution", {}) if est is not None else {}
        p_decel = float(dist.get("DECELERATING", 0.0)) + float(dist.get("STOPPING", 0.0))
        p_keep = float(dist.get("LANE_KEEP", max(0.0, 1.0 - p_cut - p_decel)))
        conf = 1.0 if str(getattr(t, "status", "CONFIRMED")) == "CONFIRMED" else 0.4
        d_rate = float(lateral_rate_by_track.get(tid, 0.0))
        d_acc = float(long_accel_by_track.get(tid, 0.0))
        role = _role_for(ds, dd)
        cands.append(_Cand(
            slot=AgentSlot(
                valid=True, role=role, ds_m=ds, closing_mps=closing, dd_m=dd,
                d_rate_mps=d_rate, d_accel_mps2=d_acc, lateral_rate_mps=d_rate,
                p_cut_in=p_cut, p_decel=p_decel,
                p_lane_keep=max(0.0, min(1.0, p_keep)),
                length_m=float(getattr(t, "length_m", 4.3)), track_confidence=conf,
                track_age_ticks=int(getattr(t, "age_ticks", 0)),
                pos_sigma_m=float(getattr(t, "pos_sigma_m", 0.0)),
            ),
            role=role, ds_abs=abs(ds), range_m=rng,
        ))

    slots: List[AgentSlot] = []
    used = [False] * len(cands)
    for role in ROLE_SLOTS:
        best_i = -1
        best_ds = None
        for i, c in enumerate(cands):
            if used[i] or c.role != role:
                continue
            if best_ds is None or c.ds_abs < best_ds:
                best_ds, best_i = c.ds_abs, i
        if best_i >= 0:
            used[best_i] = True
            slots.append(cands[best_i].slot)
        else:
            slots.append(AgentSlot(role=role))
    others = sorted(
        (c for i, c in enumerate(cands) if not used[i]),
        key=lambda c: c.range_m,
    )
    for c in others[:_N_OTHER_SLOTS]:
        slots.append(c.slot)
    while len(slots) < MAX_AGENTS:
        slots.append(AgentSlot())

    return Observation(
        speed_mps=ego_speed_mps,
        accel_mps2=ego_accel_mps2,
        lane_offset_m=ego_lane_offset_m,
        lateral_rate_mps=ego_lateral_rate_mps,
        heading_err_rad=ego_heading_err_rad,
        road_curvature_1pm=road_curvature_1pm,
        friction_estimate=friction_estimate,
        dist_to_goal_bucket=int(min(20.0, max(0.0, dist_to_goal_m // 50.0))),
        lead_gap_m=lead_gap_m,
        lead_closing_mps=lead_closing,
        n_same_direction_lanes=n_same_direction_lanes,
        ego_lane_index=ego_lane_index,
        left_lane_clear=bool(left_lane_clear),
        right_lane_clear=bool(right_lane_clear),
        binding_constraint=binding_constraint,
        agents=tuple(slots),
        curvature_profile_1pm=tuple(float(k) for k in curvature_profile_1pm),
        curvature_profile_lookahead_m=tuple(float(m) for m in curvature_profile_lookahead_m),
        dist_to_next_maneuver_m=float(dist_to_next_maneuver_m),
    )
