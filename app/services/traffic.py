"""
Server-side NPC traffic simulation + a forward range-sensor model (P6-1b).

Context (2026-07-20 architecture decision, see PHASE_6_TASK_BOARD.md): the
project needed genuine sensor detection instead of a planner that reads
privileged ground-truth state. Unity/Unreal/ROS+Gazebo/real driving
recordings were considered and rejected -- they would discard the entire
tested Phases 1-6 architecture for a rewrite. Instead: real geometric
sensing built on the existing stack.

This module is the "world" half of that decision -- it owns NPC vehicle
state authoritatively (previously invented client-side only, in
frontend/src/app/components/SimulatedTraffic.tsx, which the backend planner
had no way to see at all). `PhysicsEngine.sense_lead_vehicle()` is the
"sensor" half: it queries this module the way a forward-facing range sensor
would -- gap and closing speed to the nearest vehicle ahead in the same
lane, within a maximum range -- and NEVER hands the planner the full NPC
list or any NPC's exact identity/position. That boundary is deliberate: it
is what makes P6-3's car-following a perception-driven controller instead
of one with oracle access.

Scoping note for the thesis report: this is a simplified 1D forward-cone
range sensor (gap + relative speed to the nearest same-lane vehicle ahead),
not a full 3D LIDAR point cloud or camera-based detector. That fidelity is
adequate for single-lane car-following (P6-3) and is honestly described as
such -- see PHASE_6_TASK_BOARD.md's P6-7 for the later, real-vision-model
addition ("Option 2" of the sensing decision).

Lane-offset and spawn conventions (4 lanes, offsets, oncoming-vs-same-
direction split, speed range) are kept consistent with the frontend's
pre-existing (client-only) traffic simulation so the two can be reconciled
by P6-1c without a redesign.
"""
import random
from dataclasses import dataclass, field
from typing import List, Optional

# 4 lanes total across a 14 m wide road (matches RoadMesh's halfWidth=7 and
# the lane-marking offsets already used in the frontend road geometry).
# Negative offsets = oncoming lanes (left side, opposite direction);
# positive = same-direction lanes (right side, right-hand traffic).
LANE_OFFSETS = (-5.25, -1.75, 1.75, 5.25)
EGO_LANE_OFFSET_M = 3.5  # matches the frontend's LANE_OFFSET_M / the near-side lane

NPC_COUNT = 8
MIN_NPC_SPEED_KMH = 30.0
MAX_NPC_SPEED_KMH = 60.0

# How far (route-station meters) an NPC may drift from the ego before being
# recycled to a fresh spot nearby. Matches SimulatedTraffic.tsx's
# NPC_VISIBILITY_WINDOW_M -- see that file's comment for why: on any route
# longer than ~1km, NPCs spread uniformly across the whole route would sit
# outside the visible/sensed window for the entire drive.
VISIBILITY_WINDOW_M = 150.0
RECYCLE_SPREAD_FRACTION = 0.8

# Forward range-sensor parameters (P6-1b). A real narrow-beam radar/LIDAR
# forward sensor, not omnidirectional and not full-scene.
SENSOR_MAX_RANGE_M = 100.0
SENSOR_LANE_TOLERANCE_M = 1.75  # half a lane width either side of the queried lane


@dataclass
class NpcVehicle:
    id: str
    lane_offset: float
    speed_kmh: float
    station_m: float


@dataclass
class SensedLeadVehicle:
    """What a forward range sensor actually reports: a gap and a relative
    speed. No identity, no exact position, no access to the rest of the
    scene -- this is the whole point of the perception/control boundary."""
    gap_m: float
    lead_speed_kmh: float


@dataclass
class TrafficModel:
    """Authoritative server-side NPC state for one active drive/route.

    Instantiated once route length is known (physics_engine.py creates one
    whenever set_route() supplies a real route with nonzero length), seeded
    for deterministic tests, and advanced every physics tick from the SAME
    ego station_m the physics engine already tracks (P5's station-latitude
    work) -- see PhysicsEngine.update()/sense_lead_vehicle().
    """
    total_length_m: float
    seed: Optional[int] = None
    npcs: List[NpcVehicle] = field(default_factory=list)
    _rng: random.Random = field(default=None, repr=False)

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        if self.total_length_m > 0:
            self.npcs = [self._spawn(i) for i in range(NPC_COUNT)]

    def _spawn(self, i: int) -> NpcVehicle:
        is_oncoming = i % 2 == 0
        if is_oncoming:
            lane_offset = LANE_OFFSETS[i % 2]
            speed_kmh = -(MIN_NPC_SPEED_KMH + self._rng.random() * (MAX_NPC_SPEED_KMH - MIN_NPC_SPEED_KMH))
        else:
            lane_offset = LANE_OFFSETS[2 + (i % 2)]
            speed_kmh = MIN_NPC_SPEED_KMH + self._rng.random() * (MAX_NPC_SPEED_KMH - MIN_NPC_SPEED_KMH)
        station_m = self._rng.random() * self.total_length_m
        return NpcVehicle(id=f"npc-{i}", lane_offset=lane_offset, speed_kmh=speed_kmh, station_m=station_m)

    def update(self, dt: float, ego_station_m: float) -> None:
        """Advance every NPC along the route by dt, recycling any that have
        drifted out of the visibility window around the ego."""
        if self.total_length_m <= 0:
            return
        for npc in self.npcs:
            npc.station_m += (npc.speed_kmh / 3.6) * dt

            if npc.station_m > self.total_length_m:
                npc.station_m = 0.0
            elif npc.station_m < 0:
                npc.station_m = self.total_length_m

            if abs(npc.station_m - ego_station_m) > VISIBILITY_WINDOW_M:
                spread = (self._rng.random() - 0.5) * 2 * VISIBILITY_WINDOW_M * RECYCLE_SPREAD_FRACTION
                npc.station_m = max(0.0, min(self.total_length_m, ego_station_m + spread))

    def sense_lead_vehicle(
        self,
        ego_station_m: float,
        ego_lane_offset: float = EGO_LANE_OFFSET_M,
        max_range_m: float = SENSOR_MAX_RANGE_M,
        lane_tolerance_m: float = SENSOR_LANE_TOLERANCE_M,
    ) -> Optional[SensedLeadVehicle]:
        """The forward range sensor. Returns the gap and speed of the
        nearest vehicle ahead of the ego, in the same lane, within range --
        or None if nothing qualifies. This is deliberately the ONLY way
        PhysicsEngine may learn about NPCs; it must never be given the
        `npcs` list directly."""
        best: Optional[SensedLeadVehicle] = None
        for npc in self.npcs:
            if abs(npc.lane_offset - ego_lane_offset) > lane_tolerance_m:
                continue
            gap = npc.station_m - ego_station_m
            if gap <= 0 or gap > max_range_m:
                continue
            if best is None or gap < best.gap_m:
                best = SensedLeadVehicle(gap_m=gap, lead_speed_kmh=npc.speed_kmh)
        return best

    def get_npc_states(self) -> List[dict]:
        """Full NPC state for streaming to the frontend renderer (P6-1c) --
        this is a rendering concern, separate from what the ego's sensor is
        allowed to perceive for control purposes."""
        return [
            {"id": npc.id, "lane_offset": npc.lane_offset, "speed_kmh": npc.speed_kmh, "station_m": npc.station_m}
            for npc in self.npcs
        ]
