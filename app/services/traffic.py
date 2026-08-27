"""
Server-side NPC traffic simulation + a forward range-sensor model .

Architecture decision: the
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
is what makes the car-following a perception-driven controller instead
of one with oracle access.

Scoping note for the thesis report: this is a simplified 1D forward-cone
range sensor (gap + relative speed to the nearest same-lane vehicle ahead),
not a full 3D LIDAR point cloud or camera-based detector. That fidelity is
adequate for single-lane car-following  and is honestly described as
such. A future improvement could add a real vision-model-based sensor.

Lane-offset and spawn conventions (4 lanes, offsets, oncoming-vs-same-
direction split, speed range) are kept consistent with the frontend's
pre-existing (client-only) traffic simulation so the two can be reconciled
by the previous without a redesign.
"""
import random
from dataclasses import dataclass, field
from typing import List, Optional

from app.services.car_following import idm_acceleration

# 4 lanes total across a 14 m wide road (matches RoadMesh's halfWidth=7 and
# the lane-marking offsets already used in the frontend road geometry).
# Negative offsets = oncoming lanes (left side, opposite direction);
# positive = same-direction lanes (right side, right-hand traffic).
LANE_OFFSETS = (-5.25, -1.75, 1.75, 5.25)

# EGO_LANE_OFFSET_M used to be 3.5 -- the LANE BOUNDARY between the two
# same-direction lanes (1.75 and 5.25), not either lane's centre. That
# quietly made every "same lane" sensor query (SENSOR_LANE_TOLERANCE_M=1.75
# either side of 3.5 spans [1.75, 5.25], i.e. BOTH same-direction lanes at
# once) and made a real lane-change meaningless, since the ego was never
# actually aligned with a specific lane to change out of. Fixed to the real
# near-side lane centre; ADJACENT_LANE_OFFSET_M is the real far-side lane
# the planner can now change into.
EGO_LANE_OFFSET_M = 1.75
ADJACENT_LANE_OFFSET_M = 5.25

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

# Forward range-sensor parameters . A real narrow-beam radar/LIDAR
# forward sensor, not omnidirectional and not full-scene.
SENSOR_MAX_RANGE_M = 100.0
SENSOR_LANE_TOLERANCE_M = 1.75  # half a lane width either side of the queried lane

# NPC-to-NPC IDM car-following (P6-4): same-direction NPCs now react to
# each other instead of driving at an independently-scripted constant
# speed forever. Scoped to same-direction traffic only (oncoming NPCs keep
# the old constant-speed behaviour) -- oncoming-lane interaction doesn't
# affect anything the ego can observe or the HMI shows prominently, so it
# is not worth the added complexity of handling the opposite direction of
# travel (station_m decreasing over time) in this pass. NPCs are allowed
# oracle access to each other's exact state (unlike the ego, which only
# ever gets the perception-boundary sense_lead_vehicle() below) -- there is
# no perception/control boundary between two simulated background vehicles.
NPC_A_MAX_MPS2 = 2.0
LANE_MATCH_TOLERANCE_M = 0.1  # lane_offset is one of 4 fixed spawn constants; never drifts

# Lane-change safety gate (used by app/services/planner.py): an adjacent
# lane is only a real candidate if nothing occupies a safety envelope
# around the ego's station in that lane, both ahead AND behind -- a lane
# change into a gap that's clear ahead but has a fast-closing car behind is
# not actually safe, matching MOBIL's safety criterion in spirit (a full
# MOBIL incentive/politeness calculation is out of scope; this is the
# minimum real safety check a lane-change candidate needs to not be
# fictional).
LANE_CHANGE_CLEARANCE_AHEAD_M = 20.0
LANE_CHANGE_CLEARANCE_BEHIND_M = 15.0


@dataclass
class NpcVehicle:
    id: str
    lane_offset: float
    speed_kmh: float
    station_m: float
    # The free-road/cruise speed this NPC targets when nothing is ahead of
    # it -- separate from speed_kmh, which now fluctuates under IDM
    # car-following. Fixed at spawn time, mirroring how the ego's own
    # cruise speed is a fixed target the IDM composition falls back to.
    desired_speed_kmh: float = 0.0
    prevent_recycle: bool = False
    # Perception-layer classification (app/services/perception/entities.py's
    # EntityClass, kept as a plain string here so traffic.py has no import
    # dependency on the perception package). Defaults to SEDAN, which is
    # every NPC's real behaviour before this field existed -- adding it
    # changes nothing for existing scenarios/tests.
    entity_class: str = "SEDAN"


@dataclass
class SensedLeadVehicle:
    """What a forward range sensor actually reports: a gap and a relative
    speed. No identity, no exact position, no access to the rest of the
    scene -- this is the whole point of the perception/control boundary."""
    gap_m: float
    lead_speed_kmh: float


DENSITY_NPC_COUNTS = {
    "low": 4,
    "medium": 8,
    "high": 14,
}


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
    density: str = "medium"
    _rng: random.Random = field(default=None, repr=False)

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        if self.total_length_m > 0 and not self.npcs:
            count = DENSITY_NPC_COUNTS.get(self.density.lower(), NPC_COUNT)
            self.npcs = [self._spawn(i) for i in range(count)]

    def spawn_scripted_npcs(self, npcs: List[NpcVehicle]) -> None:
        """Replace active NPCs with a deterministic, scripted list of scenario actors."""
        self.npcs = list(npcs)

    def _spawn(self, i: int) -> NpcVehicle:
        is_oncoming = i % 2 == 0
        if is_oncoming:
            lane_offset = LANE_OFFSETS[i % 2]
            speed_kmh = -(MIN_NPC_SPEED_KMH + self._rng.random() * (MAX_NPC_SPEED_KMH - MIN_NPC_SPEED_KMH))
        else:
            lane_offset = LANE_OFFSETS[2 + (i % 2)]
            speed_kmh = MIN_NPC_SPEED_KMH + self._rng.random() * (MAX_NPC_SPEED_KMH - MIN_NPC_SPEED_KMH)
        station_m = self._rng.random() * self.total_length_m
        return NpcVehicle(id=f"npc-{i}", lane_offset=lane_offset, speed_kmh=speed_kmh,
                          station_m=station_m, desired_speed_kmh=speed_kmh)

    def _find_lead_npc(self, npc: NpcVehicle) -> Optional[NpcVehicle]:
        """Nearest OTHER same-lane, same-direction NPC ahead of `npc` (i.e.
        with a larger station_m) -- oracle access is fine here, see the
        module-level comment on NPC-to-NPC IDM above."""
        best: Optional[NpcVehicle] = None
        best_gap = float("inf")
        for other in self.npcs:
            if other is npc:
                continue
            if abs(other.lane_offset - npc.lane_offset) > LANE_MATCH_TOLERANCE_M:
                continue
            gap = other.station_m - npc.station_m
            if gap <= 0:
                continue
            if gap < best_gap:
                best_gap, best = gap, other
        return best

    def update(self, dt: float, ego_station_m: float) -> None:
        """Advance every NPC along the route by dt, recycling any that have
        drifted out of the visibility window around the ego. Same-direction
        NPCs first adjust their speed via IDM against whichever same-lane
        NPC is ahead of them (P6-4) -- this is what makes traffic form
        realistic queues instead of independently-scripted motion; oncoming
        NPCs are unaffected (see the module-level comment on NPC-to-NPC IDM)."""
        if self.total_length_m <= 0:
            return
        for npc in self.npcs:
            is_same_direction = npc.desired_speed_kmh > 0
            if is_same_direction:
                lead = self._find_lead_npc(npc)
                gap_m = (lead.station_m - npc.station_m) if lead is not None else None
                lead_speed_mps = (lead.speed_kmh / 3.6) if lead is not None else None
                idm_accel = idm_acceleration(
                    v_mps=npc.speed_kmh / 3.6,
                    v0_mps=npc.desired_speed_kmh / 3.6,
                    gap_m=gap_m,
                    lead_speed_mps=lead_speed_mps,
                    a_max_mps2=NPC_A_MAX_MPS2,
                )
                if idm_accel is not None:
                    new_speed_mps = max(0.0, npc.speed_kmh / 3.6 + idm_accel * dt)
                    npc.speed_kmh = new_speed_mps * 3.6
                else:
                    # No lead vehicle -- ease back toward free-road cruise
                    # speed rather than snapping, so a queue that just
                    # cleared accelerates away smoothly.
                    speed_error_mps = (npc.desired_speed_kmh - npc.speed_kmh) / 3.6
                    npc.speed_kmh += max(-NPC_A_MAX_MPS2, min(NPC_A_MAX_MPS2, speed_error_mps)) * dt * 3.6

            npc.station_m += (npc.speed_kmh / 3.6) * dt

            if npc.station_m > self.total_length_m:
                npc.station_m = 0.0
            elif npc.station_m < 0:
                npc.station_m = self.total_length_m

            if not npc.prevent_recycle and abs(npc.station_m - ego_station_m) > VISIBILITY_WINDOW_M:
                spread = (self._rng.random() - 0.5) * 2 * VISIBILITY_WINDOW_M * RECYCLE_SPREAD_FRACTION
                npc.station_m = max(0.0, min(self.total_length_m, ego_station_m + spread))
                # A recycled NPC may have been mid-queue (slowed well below
                # its cruise speed) right before teleporting to a fresh
                # spot -- reset to free-road speed so it doesn't reappear
                # inexplicably crawling.
                npc.speed_kmh = npc.desired_speed_kmh

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

    def sense_lane_clear(
        self,
        ego_station_m: float,
        target_lane_offset: float,
        clearance_ahead_m: float = LANE_CHANGE_CLEARANCE_AHEAD_M,
        clearance_behind_m: float = LANE_CHANGE_CLEARANCE_BEHIND_M,
        lane_tolerance_m: float = SENSOR_LANE_TOLERANCE_M,
    ) -> bool:
        """A second, distinct sensor query (like sense_lead_vehicle) for a
        real lane-change safety check: is `target_lane_offset` clear of any
        vehicle within a safety envelope around the ego, both ahead AND
        behind? A lane with a gap that's clear ahead but has a fast car
        right behind is not a safe change. Returns a bare bool -- same
        perception-boundary principle as sense_lead_vehicle: the planner
        gets an answer, never the NPC list itself."""
        for npc in self.npcs:
            if abs(npc.lane_offset - target_lane_offset) > lane_tolerance_m:
                continue
            gap = npc.station_m - ego_station_m
            if -clearance_behind_m <= gap <= clearance_ahead_m:
                return False
        return True

    def get_npc_states(self) -> List[dict]:
        """Full NPC state for streaming to the frontend renderer  --
        this is a rendering concern, separate from what the ego's sensor is
        allowed to perceive for control purposes."""
        return [
            {"id": npc.id, "lane_offset": npc.lane_offset, "speed_kmh": npc.speed_kmh, "station_m": npc.station_m}
            for npc in self.npcs
        ]
