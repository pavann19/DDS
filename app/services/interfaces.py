"""
Typed contracts for the World / Driver autonomy pipeline (ADR-001, Phase 6.5).

This module is deliberately **pure data**: dataclasses with type signatures and
docstrings, no behavioural logic. It exists so the boundary between the
simulated *world* and the autonomy *driver* becomes a set of type signatures
rather than a convention enforced only by comments (ADR-001, Problem 2).

The single most important contract here is ``SensorObservation``. The rule the
rest of the refactor enforces:

    The Driver may only ever read what ``SensorInterface`` returns, i.e. a
    ``SensorObservation``. It never receives a ``TrafficModel`` or an
    ``NpcVehicle`` -- those are ground truth owned by the World.

These types describe the data that *already* flows through
``PhysicsEngine.update()`` today (forward range sense, lane-clear query,
surround tracks, occupancy grid, Frenet projection, the commanded
steer/accel). Fields anticipated by later phases (per-agent predictions in
Phase 7, a spatiotemporal trajectory in Phase 8) are included as optional so
the contract does not need to be reshaped when those land; nothing is required
to populate them yet.

Nothing imports this module yet -- Action Items 2 and 3 of ADR-001 are what
wire ``world/`` and ``driver/`` onto these contracts. Adding the file is
strictly additive and changes no behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SimClock:
    """Authoritative, fixed-step simulation time.

    Wall-clock ``dt`` is what makes the current loop nondeterministic
    (ADR-001, Problem 3): identical inputs produce different trajectories
    depending on machine load. The multi-rate executor (Action Item 4)
    advances an instance of this instead of reading ``time.time()``.

    ``dt_s`` is the base substep of the executor (the fastest rate -- 50 Hz
    controller/safety => 0.02 s). ``tick`` counts base substeps since the run
    started; ``sim_time_s`` is ``tick * dt_s`` carried explicitly so a
    consumer never has to recompute it.

    Frozen: sim time is never mutated in place, only advanced into a new
    value via :meth:`advance` (keeps it safe to stash a clock on a message
    and know it will not change underneath you).
    """

    tick: int = 0
    dt_s: float = 0.02
    sim_time_s: float = 0.0

    def advance(self, substeps: int = 1) -> "SimClock":
        """Return the clock ``substeps`` base steps later. Pure; no mutation."""
        new_tick = self.tick + substeps
        return replace(self, tick=new_tick, sim_time_s=new_tick * self.dt_s)

    def is_rate_tick(self, hz: float) -> bool:
        """True on ticks where a ``hz``-rate stage should run.

        A convenience predicate, not control flow -- the executor decides
        stage scheduling. ``hz`` must divide the base rate (1 / ``dt_s``);
        e.g. base 50 Hz supports 50 / 25 / 10 Hz cleanly, 20 Hz needs a base
        that divides it.
        """
        base_hz = 1.0 / self.dt_s
        period = base_hz / hz
        rounded = round(period)
        if rounded <= 0:
            return True
        return self.tick % rounded == 0


# ---------------------------------------------------------------------------
# World -> Driver: the only legal boundary
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LeadObservation:
    """Forward range-sensor return: a gap and a relative speed, nothing else.

    Mirrors ``traffic.SensedLeadVehicle`` but expressed in SI (m, m/s) and
    carrying no identity or absolute position -- that omission is the
    perception/control boundary, not an oversight.
    """

    gap_m: float
    lead_speed_mps: float


@dataclass(frozen=True)
class EgoProprioception:
    """What the vehicle knows about *itself* -- IMU / wheel-odometry class
    signals. Always available to the Driver; contains no world state."""

    speed_mps: float
    heading_deg: float
    accel_mps2: float
    steer_rad: float
    # Frenet self-localisation against the active route geometry. The route
    # polyline is map data, not traffic ground truth, so exposing the ego's
    # own projection onto it does not breach the boundary.
    station_m: float
    lateral_offset_m: float


@dataclass(frozen=True)
class SensorObservation:
    """Everything -- and *only* what -- the Driver is allowed to read this tick.

    Produced by the World's ``SensorInterface``. If a field is not here, the
    Driver has no legal way to know it. In particular there is no NPC list,
    no NPC identity, and no NPC absolute pose beyond what the ego's own
    modelled sensors resolved into ``surround_tracks``.
    """

    clock: SimClock
    ego: EgoProprioception

    # Forward 1D range sensor (feeds IDM car-following).
    lead: Optional[LeadObservation] = None

    # Lane-change safety query result for the single modelled adjacent lane.
    # A bare bool answer, exactly like ``TrafficModel.sense_lane_clear``.
    adjacent_lane_clear: bool = False

    # 360 deg surround perception: tracker output, already reduced to the
    # ego's sensor-resolved picture (``perception_engine.SurroundTrack`` or a
    # dict shaped like it). Never raw NPCs.
    surround_tracks: Tuple[Any, ...] = ()

    # Log-odds occupancy grid snapshot (``occupancy_grid.OccupancyGrid`` or an
    # immutable view of one). Optional: only present when a route exists.
    occupancy: Optional[Any] = None

    # Active route geometry (``frenet.FrenetFrame``). Map data -- the Driver
    # needs it to project candidates; it carries no traffic state.
    frenet_frame: Optional[Any] = None


# ---------------------------------------------------------------------------
# Driver-internal stage outputs
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TrackEstimate:
    """One tracked actor as the Perception stage hands it downstream.

    SI units, ego-local ``(x, z)`` metres matching ``frenet``'s convention
    (x = East, z = South). This is the shape Prediction consumes.
    """

    track_id: int
    entity_class: str
    status: str
    x: float
    z: float
    vx: float
    vz: float
    length_m: float
    width_m: float
    height_m: float


@dataclass(frozen=True)
class PerceptionOutput:
    """Perception stage result (20 Hz)."""

    clock: SimClock
    tracks: Tuple[TrackEstimate, ...] = ()
    occupancy: Optional[Any] = None
    # Ego state estimate the stage localised (today this is passed through
    # from proprioception unchanged; a real estimator lands later).
    ego: Optional[EgoProprioception] = None


@dataclass(frozen=True)
class PredictedState:
    """A single forecasted pose for one agent at ``t_s`` ahead of now."""

    t_s: float
    x: float
    z: float
    vx: float
    vz: float


@dataclass(frozen=True)
class AgentPrediction:
    """Forecast for one tracked agent (Phase 7 populates this)."""

    track_id: int
    states: Tuple[PredictedState, ...] = ()
    # Intent distribution, e.g. {"LANE_KEEP": 0.8, "MERGE_LEFT": 0.2}.
    intent: Tuple[Tuple[str, float], ...] = ()


@dataclass(frozen=True)
class PredictionOutput:
    """Prediction stage result (10 Hz). Empty until Phase 7."""

    clock: SimClock
    agents: Tuple[AgentPrediction, ...] = ()
    # Spatiotemporal risk field handle (Phase 7). Opaque here.
    risk_field: Optional[Any] = None


@dataclass(frozen=True)
class TrajectoryPoint:
    """One point on a timed trajectory the Controller tracks."""

    t_s: float
    s_m: float
    d_m: float
    speed_mps: float
    curvature: float = 0.0


@dataclass(frozen=True)
class PlannedTrajectory:
    """Planner stage result (10 Hz): a *timed* trajectory, not a steer value.

    The Controller (50 Hz) consumes this and decides the actuator command;
    the Planner never does. That split is what decouples the rates
    (ADR-001, Decision rule 2).

    Today's planner emits a lateral target and a scored candidate set rather
    than a full ``(s, d, t)`` lattice; ``points`` may therefore be a short
    interpolation and ``candidates`` carries the Phase 3 lateral candidates
    verbatim. Phase 8 replaces the internals without changing this contract.
    """

    clock: SimClock
    points: Tuple[TrajectoryPoint, ...] = ()
    chosen_d_m: float = 0.0
    target_speed_mps: float = 0.0
    is_lane_change: bool = False
    # Scored alternatives, for the HMI / evaluation. Shaped like
    # ``planner.LateralCandidate`` (kept as opaque objects here).
    candidates: Tuple[Any, ...] = ()


# ---------------------------------------------------------------------------
# Driver -> World
# ---------------------------------------------------------------------------
# Provenance tags for ``ActuatorCommand.source`` -- which layer produced the
# final command. Mirrors the strings physics_engine already uses for
# ``speed_limit_reason`` so nothing downstream has to learn a new vocabulary.
SOURCE_PLANNER = "planner"
SOURCE_SAFETY_OVERRIDE = "safety_override"


@dataclass(frozen=True)
class ActuatorCommand:
    """The Driver's output: a front-wheel angle and a longitudinal
    acceleration, plus provenance so an override is auditable.

    ``accel_mps2`` is the *demanded* acceleration before the World applies
    its own jerk limit and integration -- same division of responsibility
    the bicycle controller already uses internally.
    """

    steer_rad: float
    accel_mps2: float
    source: str = SOURCE_PLANNER
    # Which constraint is currently binding, e.g. "cruise", "car_following",
    # "lateral_accel_limit", "safety_shield_override". Free-form, surfaced to
    # the HMI exactly as ``speed_limit_reason`` is today.
    speed_limit_reason: str = "cruise"


__all__ = [
    "SimClock",
    "LeadObservation",
    "EgoProprioception",
    "SensorObservation",
    "TrackEstimate",
    "PerceptionOutput",
    "PredictedState",
    "AgentPrediction",
    "PredictionOutput",
    "TrajectoryPoint",
    "PlannedTrajectory",
    "ActuatorCommand",
    "SOURCE_PLANNER",
    "SOURCE_SAFETY_OVERRIDE",
]
