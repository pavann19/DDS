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
import math
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from bisect import bisect_left, bisect_right

from app.services.car_following import idm_acceleration
from app.services.safety_geometry import Footprint


@dataclass(frozen=True)
class WorldVehicleState:
    """World-only body state for background following, never Driver input."""
    station_m: float
    body: Footprint

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

# Vehicle geometry -- the forward sensor works in a physical COLLISION
# CORRIDOR, not a point-lane gate: a vehicle whose body would overlap the
# ego's if their stations crossed is a lead the ego must react to, even
# mid-lane-change when the ego's lateral offset has moved off the lead's
# lane. Half-length is subtracted from the station gap so the reported
# gap_m is bumper-to-bumper, which is what IDM's MIN_GAP_M / the Safety
# Shield's TTC assume.
VEHICLE_HALF_LENGTH_M = 2.15   # ~4.3 m car (matches the rendered ego model)
VEHICLE_HALF_WIDTH_M = 0.98    # ~1.95 m wide
FCW_LATERAL_MARGIN_M = 0.35    # extra lateral keep-clear beyond the two half-widths

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
    # dependency on the perception package). Defaults to SEDAN.
    entity_class: str = "SEDAN"
    # Parameterized driver personality & motion realism
    base_lane_offset: float = 0.0
    aggressiveness: float = 0.5
    wander_amplitude_m: float = 0.0
    wander_freq_rad: float = 0.15
    wander_phase_rad: float = 0.0
    lateral_speed_mps: float = 0.0
    relative_heading_deg: float = 0.0

    def __post_init__(self):
        if self.base_lane_offset == 0.0:
            self.base_lane_offset = self.lane_offset


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
    ego station_m the physics engine already tracks -- see PhysicsEngine.update().
    """
    total_length_m: float
    seed: Optional[int] = None
    npcs: List[NpcVehicle] = field(default_factory=list)
    density: str = "medium"
    sensor_noise_enabled: bool = False
    sensor_dropout_rate: float = 0.0
    elapsed_time_s: float = 0.0
    # Opt-in finite world; legacy fixture recycling/wrapping stays unchanged.
    finite_world: bool = False
    world_controls: tuple = ()
    speed_limit_at_station: Optional[Callable[[float], Optional[float]]] = field(default=None, repr=False)
    _control_stopped_since: dict = field(default_factory=dict, repr=False)
    _control_released: set = field(default_factory=set, repr=False)
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
            wander_amp = 0.0
        else:
            lane_offset = LANE_OFFSETS[2 + (i % 2)]
            speed_kmh = MIN_NPC_SPEED_KMH + self._rng.random() * (MAX_NPC_SPEED_KMH - MIN_NPC_SPEED_KMH)
            # Subtle lateral wandering: 5cm to 15cm (bounded <= 25cm)
            wander_amp = 0.04 + self._rng.random() * 0.10
        station_m = self._rng.random() * self.total_length_m
        aggressiveness = 0.3 + self._rng.random() * 0.5
        wander_freq = 0.10 + self._rng.random() * 0.10
        wander_phase = self._rng.random() * 2.0 * math.pi
        return NpcVehicle(
            id=f"npc-{i}",
            lane_offset=lane_offset,
            base_lane_offset=lane_offset,
            speed_kmh=speed_kmh,
            station_m=station_m,
            desired_speed_kmh=speed_kmh,
            aggressiveness=aggressiveness,
            wander_amplitude_m=wander_amp,
            wander_freq_rad=wander_freq,
            wander_phase_rad=wander_phase,
        )

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

    @staticmethod
    def _npc_dimensions(npc):
        from app.services.perception.entities import EntityClass,dimensions_for
        try:
            return dimensions_for(EntityClass(npc.entity_class))
        except ValueError:
            return dimensions_for(EntityClass.SEDAN)

    def _npc_lead_gap(self,npc,lead):
        return lead.station_m-npc.station_m-(self._npc_dimensions(npc)[0]+self._npc_dimensions(lead)[0])/2

    @staticmethod
    def _path_samples(frame,stations,offset):
        # Tiny leader queries do not amortize NumPy's array setup. Both
        # mappings use the same segment selection and offset convention.
        from app.services.frenet import frenet_to_local_xz,frenet_to_local_xz_batch
        if len(stations)<=16:
            return tuple(zip(*(frenet_to_local_xz(frame,s,offset) for s in stations)))
        return frenet_to_local_xz_batch(frame,stations,[offset]*len(stations))

    def _npc_world_vehicle(self,npc,frame):
        end=min(frame.total_length_m,npc.station_m+.1)
        start=max(0.,npc.station_m-.1) if end==npc.station_m else npc.station_m
        xs,zs,txs,tzs=self._path_samples(frame,[start,end,npc.station_m],npc.lane_offset)
        dt_station=max(1e-9,end-start)
        speed=npc.speed_kmh/3.6
        length,width,_=self._npc_dimensions(npc)
        return WorldVehicleState(npc.station_m,Footprint(float(xs[-1]),float(zs[-1]),
            math.atan2(float(txs[-1]),-float(tzs[-1]))+math.radians(npc.relative_heading_deg),
            length,width,float(xs[1]-xs[0])/dt_station*speed,float(zs[1]-zs[0])/dt_station*speed))

    def _body_following_constraint(self, npc, leader, frame):
        """Physical bumper gap along this NPC's mapped path to a leader body.

        Adjacent bodies are excluded by their oriented lateral support, not a
        nominal lane-ID match. Arc distance follows the actual piecewise mapped
        offset path; station distance alone overstates room on inner bends.
        """
        distance=leader.station_m-npc.station_m
        if not 0.<distance<=200.:
            return None
        length,width,_=self._npc_dimensions(npc)
        lo=bisect_right(frame.station,npc.station_m)
        hi=bisect_left(frame.station,leader.station_m)
        stations=[npc.station_m,*frame.station[lo:hi],leader.station_m]
        xs,zs,_,_=self._path_samples(frame,stations,npc.lane_offset)
        # The last offset-path segment defines the NPC's approach to the body.
        dx,dz=float(xs[-1]-xs[-2]),float(zs[-1]-zs[-2])
        segment=math.hypot(dx,dz)
        if segment<1e-9:
            return None
        tx,tz=dx/segment,dz/segment
        nx,nz=-tz,tx
        body=leader.body
        forward,left=body.axes()
        support=lambda ux,uz:(abs(ux*forward[0]+uz*forward[1])*body.length_m/2+
                              abs(ux*left[0]+uz*left[1])*body.width_m/2)
        ex,ez=body.x-float(xs[-1]),body.z-float(zs[-1])
        if abs(ex*nx+ez*nz)>support(nx,nz)+width/2+FCW_LATERAL_MARGIN_M:
            return None
        arc=sum(math.hypot(float(bx-ax),float(bz-az))
                for ax,az,bx,bz in zip(xs,zs,xs[1:],zs[1:]))
        gap=arc+ex*tx+ez*tz-support(tx,tz)-length/2
        # Stored NPC speed advances route station. Convert its local motion to
        # physical path speed before comparing with actual ego world velocity.
        first_distance=math.hypot(float(xs[1]-xs[0]),float(zs[1]-zs[0]))
        scale=first_distance/(stations[1]-stations[0])
        if not math.isfinite(scale) or scale<=1e-6:
            return None
        return gap,max(0.,body.vx*tx+body.vz*tz),scale

    def update(self, dt: float, ego_station_m: float, *, ego_vehicle=None, frenet_frame=None) -> None:
        """Advance every NPC along the route by dt, recycling any that have
        drifted out of the visibility window around the ego. Same-direction
        NPCs first adjust their speed via IDM against whichever same-lane
        NPC is ahead of them (P6-4) -- this is what makes traffic form
        realistic queues instead of independently-scripted motion; oncoming
        NPCs are unaffected (see the module-level comment on NPC-to-NPC IDM)."""
        if self.total_length_m <= 0:
            return
        self.elapsed_time_s += dt
        for npc in self.npcs:
            is_same_direction = npc.desired_speed_kmh > 0
            if is_same_direction:
                lead = self._find_lead_npc(npc)
                gap_m = self._npc_lead_gap(npc,lead) if lead is not None else None
                lead_speed_mps = (lead.speed_kmh / 3.6) if lead is not None else None
                # Background actors may use world truth; the ego never reads
                # this helper. A control becomes a stationary virtual lead.
                control_gap = self._world_control_gap(npc)
                if control_gap is not None and (gap_m is None or control_gap < gap_m):
                    gap_m, lead_speed_mps = control_gap, 0.
                free_speed_kmh = npc.desired_speed_kmh
                if self.speed_limit_at_station is not None:
                    limit = self.speed_limit_at_station(npc.station_m)
                    if limit is not None:
                        free_speed_kmh = min(free_speed_kmh, limit)
                agg = getattr(npc, 'aggressiveness', 0.5)
                a_max = 1.5 + agg * 1.5
                idm_accel = idm_acceleration(
                    v_mps=npc.speed_kmh / 3.6,
                    v0_mps=free_speed_kmh / 3.6,
                    gap_m=gap_m,
                    lead_speed_mps=lead_speed_mps,
                    a_max_mps2=a_max,
                )
                if frenet_frame is not None:
                    leaders=[]
                    if lead is not None:leaders.append(self._npc_world_vehicle(lead,frenet_frame))
                    if ego_vehicle is not None:leaders.append(ego_vehicle)
                    for leader in leaders:
                        constraint=self._body_following_constraint(npc,leader,frenet_frame)
                        if constraint is not None:
                            body_gap,body_speed,scale=constraint
                            body_accel=idm_acceleration(v_mps=npc.speed_kmh/3.6*scale,
                                v0_mps=free_speed_kmh/3.6*scale,gap_m=body_gap,
                                lead_speed_mps=body_speed,a_max_mps2=a_max)/scale
                            idm_accel=body_accel if idm_accel is None else min(idm_accel,body_accel)
                if idm_accel is not None:
                    new_speed_mps = max(0.0, npc.speed_kmh / 3.6 + idm_accel * dt)
                    npc.speed_kmh = new_speed_mps * 3.6
                else:
                    # No lead vehicle -- ease back toward free-road cruise
                    # speed rather than snapping, so a queue that just
                    # cleared accelerates away smoothly.
                    speed_error_mps = (free_speed_kmh - npc.speed_kmh) / 3.6
                    npc.speed_kmh += max(-a_max, min(a_max, speed_error_mps)) * dt * 3.6

                # Kinematic background actors use an exact standstill hold
                # near a controlling virtual lead; the ego plant is separate.
                if control_gap is not None and control_gap <= 2.5 and npc.speed_kmh < .5:
                    npc.speed_kmh = 0.

                # Subtle lateral lane wandering (bounded <= 25cm)
                w_amp = getattr(npc, 'wander_amplitude_m', 0.0)
                if w_amp > 0.0:
                    base_offset = getattr(npc, 'base_lane_offset', npc.lane_offset)
                    if abs(npc.lane_offset - base_offset) > 0.35:
                        base_offset = npc.lane_offset
                        npc.base_lane_offset = base_offset
                    w_freq = getattr(npc, 'wander_freq_rad', 0.15)
                    w_phase = getattr(npc, 'wander_phase_rad', 0.0)
                    t = self.elapsed_time_s
                    d_wander = w_amp * math.sin(w_freq * t + w_phase)
                    d_dot = w_amp * w_freq * math.cos(w_freq * t + w_phase)
                    npc.lane_offset = base_offset + d_wander
                    npc.lateral_speed_mps = d_dot
                    v_mps = max(1.0, abs(npc.speed_kmh) / 3.6)
                    npc.relative_heading_deg = math.degrees(math.atan2(d_dot, v_mps))

            npc.station_m += (npc.speed_kmh / 3.6) * dt

            if self.finite_world:
                continue  # No world wrapping or ego-relative teleporting.
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

        if self.finite_world:
            self.npcs = [n for n in self.npcs if 0 <= n.station_m <= self.total_length_m]
            alive = {n.id for n in self.npcs}
            self._control_stopped_since = {k:v for k,v in self._control_stopped_since.items() if k[0] in alive}
            self._control_released = {k for k in self._control_released if k[0] in alive}

    def _world_control_gap(self, npc):
        """Simplified background traffic law, explicitly world-owned.

        No intersection conflict clearance or ego state drives this response.
        STOP dwell and world signal phases are not a full traffic-law model.
        """
        from app.services.junctions import simulated_signal_phase
        nearest = None
        for control in self.world_controls:
            key = (npc.id, control.node_id)
            distance = control.station_m - 6. - npc.station_m
            if key in self._control_released or distance < 0 or distance > 180:
                continue
            if control.kind == 'traffic_signals':
                if simulated_signal_phase(control.node_id, self.elapsed_time_s) == 'GREEN':
                    continue
            elif control.kind == 'stop':
                if distance <= 2.5 and npc.speed_kmh == 0.:
                    self._control_stopped_since.setdefault(key, self.elapsed_time_s)
                else:
                    self._control_stopped_since.pop(key, None)
                if self.elapsed_time_s-self._control_stopped_since.get(key,self.elapsed_time_s) >= 2.:
                    self._control_released.add(key)
                    continue
            # Unknown/yield has no unsupported clearance claim: hold.
            # Account for the IDM standstill gap in the virtual lead geometry.
            gap = max(.1, distance+2.)
            nearest = gap if nearest is None else min(nearest, gap)
        return nearest

    def sense_lead_vehicle(
        self,
        ego_station_m: float,
        ego_lane_offset: float = EGO_LANE_OFFSET_M,
        max_range_m: float = SENSOR_MAX_RANGE_M,
        lane_tolerance_m: float = SENSOR_LANE_TOLERANCE_M,
        ego_target_lane_offset: Optional[float] = None,
    ) -> Optional[SensedLeadVehicle]:
        """The forward collision sensor. Returns the BUMPER-TO-BUMPER gap and
        speed of the nearest vehicle ahead whose body lies in the ego's
        collision corridor -- or None if nothing qualifies.

        The corridor spans the lateral band the ego currently occupies AND
        (when ``ego_target_lane_offset`` is given) the band it is sweeping
        through during a maneuver, widened by both vehicles' half-widths plus
        a margin. This is why a real forward sensor does not lose a stopped
        car the instant the ego's lateral offset drifts half a lane during a
        lane change -- the old point-lane gate did exactly that, and the ego
        then accelerated into the car it had been braking for.

        This is deliberately the ONLY way PhysicsEngine may learn about NPCs;
        it must never be given the `npcs` list directly."""
        cur = ego_lane_offset
        tgt = ego_target_lane_offset if ego_target_lane_offset is not None else ego_lane_offset
        band_lo = min(cur, tgt) - (VEHICLE_HALF_WIDTH_M * 2.0 + FCW_LATERAL_MARGIN_M)
        band_hi = max(cur, tgt) + (VEHICLE_HALF_WIDTH_M * 2.0 + FCW_LATERAL_MARGIN_M)
        # never wider than the legacy half-a-lane-either-side unless a maneuver
        # target genuinely widens it (keeps steady-state behaviour unchanged)
        band_lo = min(band_lo, cur - lane_tolerance_m)
        band_hi = max(band_hi, cur + lane_tolerance_m)

        best: Optional[SensedLeadVehicle] = None
        best_gap = None
        for npc in self.npcs:
            if not (band_lo <= npc.lane_offset <= band_hi):
                continue
            station_gap = npc.station_m - ego_station_m
            if station_gap <= 0 or station_gap > max_range_m:
                continue
            bumper_gap = station_gap - 2.0 * VEHICLE_HALF_LENGTH_M
            if best_gap is None or bumper_gap < best_gap:
                best_gap = bumper_gap
                best = SensedLeadVehicle(gap_m=bumper_gap, lead_speed_kmh=npc.speed_kmh)

        if best is not None:
            # Check sensor dropout
            if self.sensor_dropout_rate > 0.0 and self._rng.random() < self.sensor_dropout_rate:
                return None
            measured_gap = best.gap_m
            measured_speed = best.lead_speed_kmh
            if self.sensor_noise_enabled:
                noise_r = self._rng.gauss(0.0, 0.35)
                measured_gap = max(-2.0, measured_gap + noise_r)
                noise_v = self._rng.gauss(0.0, 0.5)
                measured_speed = max(0.0, measured_speed + noise_v)
            return SensedLeadVehicle(gap_m=measured_gap, lead_speed_kmh=measured_speed)
        return None

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
            {
                "id": npc.id,
                "lane_offset": npc.lane_offset,
                "speed_kmh": npc.speed_kmh,
                "station_m": npc.station_m,
                "relative_heading_deg": getattr(npc, 'relative_heading_deg', 0.0),
                "lateral_speed_mps": getattr(npc, 'lateral_speed_mps', 0.0),
                "entity_class": getattr(npc, 'entity_class', 'SEDAN'),
            }
            for npc in self.npcs
        ]
