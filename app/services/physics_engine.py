import math
from bisect import bisect_left
import logging
import random
import time
from enum import Enum
from typing import Optional, List, Dict, Any

from app.services.interfaces import (
    SimClock,
    SensorObservation,
    SensorHealth,
    EgoProprioception,
    LeadObservation,
)
from app.services.traffic import TrafficModel, WorldVehicleState, EGO_LANE_OFFSET_M, ADJACENT_LANE_OFFSET_M
from app.services.camera_rig import CAMERA_IDS
from app.services.path_smoothing import smooth_route
from app.services.frenet import (
    build_frenet_frame,
    project_to_frenet,
    frenet_to_local_xz,
    frenet_to_latlng,
    latlng_to_local,
    signed_curvature_at,
)
from app.services.car_following import idm_acceleration
from app.services import safety_shield
from app.services.world import advance_position, step_powertrain
from app.services.grip_budget import allocate_grip
from app.services.vehicle_kinematics import sideslip, wheel_geometry, TRACK_WIDTH_M, WHEEL_RADIUS_M
from app.services.driver import plan_lateral_offset, SafetyMonitor
from app.services.driver.driver import Driver, DriverConfig
from app.services.driver.context import DriverContext
from app.services.interfaces import SafetyDetection
from app.services.perception.entities import dimensions_for
from app.services.perception.perception_engine import (
    SurroundPerceptionEngine,
    PerceptionNoiseConfig,
)
from app.services.prediction import PredictionEngine
from app.services.planner import (
    LANE_CENTER_D_M,
    LANE_CHANGE_TRIGGER_GAP_M,
    ff_stanley_steer,
    generate_candidates,
    pure_pursuit_steering,
)
from app.services.planner.spatiotemporal import (
    PlannerContext,
    PlannerStart,
    plan as plan_spatiotemporal,
)
from app.services.planner.state_machine import LaneChangeState, LaneChangeStateMachine

# Fixed simulation substep (ADR-001 item 4): 50 Hz. update() quantises its
# dt onto this grid to advance a deterministic SimClock; the physics
# integration itself still uses the raw dt, so this changes reported sim
# time only, not the trajectory.
SIM_SUBSTEP_DT_S = 0.02


class DrivingState(str, Enum):
    IDLE = "IDLE"
    DEPARTING = "DEPARTING"
    CRUISING = "CRUISING"
    APPROACHING_TURN = "APPROACHING_TURN"
    TURNING = "TURNING"
    FINAL_APPROACH = "FINAL_APPROACH"
    ARRIVING = "ARRIVING"
    ARRIVED = "ARRIVED"

class PhysicsEngine:
    # --- Vehicle & control parameters  ---------------------------------
    # Kinematic bicycle model constants. These replace the previous point-mass
    # model, in which speed was lerped straight toward a target and heading was
    # nudged proportionally -- that model had no notion of acceleration, jerk,
    # or steering geometry, which is why motion read as "flickering"/unphysical
    # and why cornering speed had to be hand-tuned with an ad-hoc lookup.
    # Values are typical passenger-car figures; they are simulation parameters,
    # NOT measurements of any specific vehicle.
    WHEELBASE_M = 2.8            # front-to-rear axle distance
    A_MAX_ACCEL_MPS2 = 3.0       # comfortable forward acceleration
    A_MAX_BRAKE_MPS2 = 4.5       # comfortable braking (larger than accel, as in a real car)
    JERK_MAX_MPS3 = 2.5          # limit on d(acceleration)/dt -- this is what removes the visual "flicker"
    A_LAT_MAX_MPS2 = 3.0         # comfortable lateral acceleration; sets cornering speed physically
    MAX_STEER_RAD = math.radians(35.0)   # max front-wheel angle
    STEER_RATE_MAX_RADPS = 0.6   # how fast the wheel can be turned
    SPEED_KP = 0.6               # proportional gain: speed error (m/s) -> demanded accel (m/s^2)
    STEER_KP = 1.2               # proportional gain: heading error (rad) -> demanded steer (rad)
    # Steering lookahead distance: L_d = LOOKAHEAD_K * v + LOOKAHEAD_MIN_M.
    #
    #  this MUST be expressed as a distance in metres, never as a
    # waypoint count. Before route resampling, the lookahead formula
    # (max(8, 0.8*v) ~= 11 m at cruise) happened to step past one raw OSRM
    # waypoint, and since raw spacing averaged ~21 m the car got ~21 m of
    # effective lookahead by pure accident. Uniform 5 m resampling removed
    # that accident and left a genuinely 11 m lookahead, which at 50 km/h is
    # under a second of travel -- the controller over-steered, overshot, and
    # spiralled off-route (measured: cross-track error growing 6 m -> 60 m
    # while heading rotated continuously through 100+ degrees).
    # Values below were tuned by sweeping against the real SF OSRM route
    # (cross-track RMS: 23.2 m at the old effective lookahead -> 3.3 m here;
    # degrades again past ~5 s as the car starts cutting corners). Lmin is the
    # sensitive parameter: 18 m gives 9.2 m RMS, 20 m gives 3.3 m.
    #
    # CAVEAT: tuned on a single route, and 3 s is long for a lookahead --
    # it is compensating for the fact that this is still a crude proportional
    # HEADING controller, which needs the extra damping. the previous replaces this
    # steering law with proper pure-pursuit geometry and should re-tune (and
    # will likely want a shorter lookahead). Treat these as a stopgap, not a
    # result.
    LOOKAHEAD_K = 3.0            # seconds of travel to look ahead
    LOOKAHEAD_MIN_M = 20.0       # floor, so the target stays sane at low speed
    MIN_CORNER_SPEED_KMH = 8.0   # floor so a hairpin never brings the car to a dead stop
    PROACTIVE_CUT_IN_MAX_DIP_KMH = 20.0  # a proactive cut-in ease-off may not pull the
                                        # ego more than this below its target speed
    # Above this fraction of the current grip-limited steering range, the
    # tracking-error speed cap starts biting (see its call site for why).
    # 0.5 means the cap only engages once the wheel is already turned more
    # than halfway to its limit -- ordinary lane-keeping correction stays
    # completely unaffected; this is specifically for large heading errors.
    TRACKING_ERROR_STEER_FRACTION_THRESHOLD = 0.25

    # --- : Frenet local planner + pure-pursuit lateral control -----------
    # Pure pursuit is proper steering GEOMETRY (it reasons about a real
    # lookahead point on a real path via delta = atan(2*L*sin(alpha)/Ld)),
    # not a proportional controller on heading error -- so it needs less
    # lookahead distance to stay damped. Swept against the real dense corner
    # route (tests/test_physics_engine.py's _corner_route): the previous proportional
    # controller needed 3.0s/20m to avoid spiralling; pure pursuit tracks the
    # same route within the 15m regression bound at roughly half that lookahead.
    PP_LOOKAHEAD_K = 1.5           # seconds of travel to look ahead
    PP_LOOKAHEAD_MIN_M = 10.0      # floor, so the target stays sane at low speed
    # How fast the planner's TARGET lateral offset is allowed to move
    # (m/s) -- an approximation of a quintic profile's own smooth transition
    # (see planner.py's quintic_lateral_maneuver_cost docstring) so the
    # tracked target itself doesn't jump discretely between candidates.
    LATERAL_TARGET_RATE_MPS = 1.0

    def __init__(self, start_lat=37.7749, start_lng=-122.4194, controller="bicycle",
                 seed: Optional[int] = None):
        # controller: "bicycle" ( kinematic bicycle + jerk-limited
        # longitudinal control) or "legacy" (the pre-P6 point-mass lerp).
        # The legacy path is deliberately retained as the experimental CONTROL
        # condition for the previous A/B evaluation -- do not delete it.
        self.controller = controller

        self.lat = start_lat
        self.lng = start_lng
        self.target_lat = start_lat + 0.005
        self.target_lng = start_lng + 0.005
        self.heading = 45.0
        # Normalised steering in [-1, 1] (kept for the existing frontend body-roll
        # effect and the ML feature vector). Under the bicycle model this is
        # derived from the real front-wheel angle below.
        self.steering_angle = 0.0
        # Real physical control state .
        self.steering_angle_rad = 0.0
        self.wheel_rotation_rad = [0.0] * 4
        self.acceleration_mps2 = 0.0
        # Diagnostics surfaced for the HMI / evaluation .
        self.path_curvature = 0.0       # 1/m, max |curvature| in the lookahead window
        self.lateral_accel_mps2 = 0.0   # v * yaw_rate, the comfort-limiting quantity
        self.signed_lateral_accel_mps2 = 0.0  # signed lateral accel for chassis roll direction
        self.yaw_rate_degps = 0.0       # realized yaw rate in deg/s
        self.kinematic_sideslip_rad = 0.0  # geometric/kinematic sideslip estimate: beta = atan((lr/L)*tan(delta))
        self.grip_budget_state = {"scope": "modeled_acceleration_budget", "already_infeasible": False}
        self.friction_mu = 1.0          # road friction coefficient (1.0 = dry asphalt, 0.5 = wet, 0.2 = icy)
        # speed_limit_reason / friction_speed_derate_active / friction_curve_cap_kmh
        # / friction_derate_kmh now live on self._driver (created below) and are
        # surfaced here via forwarding properties -- the binding-constraint and
        # low-friction-derate diagnostics are a Driver output, not world state.

        # Real road-following route : a list of (lat, lng) waypoints
        # from app.services.routing.get_route(), fetched asynchronously by
        # the WebSocket handler and pushed in via set_route(). Empty until
        # a route is fetched or if the routing service is unavailable, in
        # which case navigation falls back to a straight bearing to
        # target_lat/target_lng (the pre-routing behavior).
        self.route = []
        self.route_index = 0
        self.station_distances = []
        self.current_station_m = 0.0
        #  exact Frenet (station, lateral) frame built from the smoothed
        # route in set_route(); None until a real route is set. current_d_m is
        # the ego's own exact signed lateral offset from the route centreline
        # (positive = right / same-direction lane, matching traffic.py's
        # LANE_OFFSETS convention). lateral_target_d_m is the planner's
        # currently-tracked target (rate-limited toward the winning
        # candidate); planner_candidates/chosen_d_m are the last tick's
        # scored candidate set, surfaced for the HMI  and .
        self.frenet_frame = None
        # Separate from route_index deliberately: route_index is a coarser
        # NEAREST-WAYPOINT heuristic (the coarser projection), which can advance
        # past the segment that is still the true closest-by-perpendicular-
        # distance match for exact projection (e.g. just past a waypoint's
        # midpoint but still short of the perpendicular foot on the segment
        # behind it). Anchoring project_to_frenet's search window on
        # route_index directly caused the search to sometimes miss the true
        # segment and clamp t=0, snapping current_station_m to an exact
        # waypoint station every few ticks (observed: station alternating
        # ...112.5, 115.0(snap), 116.0, 120.0(snap)... on a straight route
        # instead of advancing smoothly with real distance travelled).
        # Tracking the frenet frame's own last matched segment instead (with
        # a small backward-search allowance) keeps the two systems decoupled.
        self.frenet_search_idx = 0
        self.current_lateral_offset_m = 0.0
        # ADR-001 Phase 6.5 / CARLA_MIGRATION_PLAN.md Phase B: the decision
        # loop (lateral steering + longitudinal target + IDM + proactive
        # cut-in + Safety Shield) is now a simulator-independent Driver. It
        # owns the lane-change FSM, the committed-maneuver trajectory, the
        # lateral-motion smoothing, the independent SafetyMonitor, the
        # planner's HMI candidate bookkeeping, the shield verdict, and the
        # binding-constraint / friction-derate diagnostics. PhysicsEngine
        # keeps world integration only. The former public attributes
        # (shield_verdict, planner_candidates, lateral_target_d_m, _lc_sm,
        # ...) are re-exposed via forwarding @property so callers/tests are
        # unchanged.
        self._driver = Driver(cfg=DriverConfig(
            WHEELBASE_M=self.WHEELBASE_M,
            A_LAT_MAX_MPS2=self.A_LAT_MAX_MPS2,
            MAX_STEER_RAD=self.MAX_STEER_RAD,
            STEER_RATE_MAX_RADPS=self.STEER_RATE_MAX_RADPS,
            JERK_MAX_MPS3=self.JERK_MAX_MPS3,
            STEER_KP=self.STEER_KP,
            SPEED_KP=self.SPEED_KP,
            A_MAX_ACCEL_MPS2=self.A_MAX_ACCEL_MPS2,
            A_MAX_BRAKE_MPS2=self.A_MAX_BRAKE_MPS2,
            MIN_CORNER_SPEED_KMH=self.MIN_CORNER_SPEED_KMH,
            PROACTIVE_CUT_IN_MAX_DIP_KMH=self.PROACTIVE_CUT_IN_MAX_DIP_KMH,
            TRACKING_ERROR_STEER_FRACTION_THRESHOLD=self.TRACKING_ERROR_STEER_FRACTION_THRESHOLD,
            LATERAL_TARGET_RATE_MPS=self.LATERAL_TARGET_RATE_MPS,
            PP_LOOKAHEAD_K=self.PP_LOOKAHEAD_K,
            PP_LOOKAHEAD_MIN_M=self.PP_LOOKAHEAD_MIN_M,
            EGO_LANE_OFFSET_M=EGO_LANE_OFFSET_M,
            ADJACENT_LANE_OFFSET_M=ADJACENT_LANE_OFFSET_M,
        ))
        self.driving_state = DrivingState.IDLE
        self.mapped_road_rules = None
        self.route_controls = ()
        self.camera_controls = False
        self.scene_camera_controls = False
        self.camera_required_for_drive = False
        self.scene_camera_pending = None
        self.camera_occluded = False
        self.camera_dropout = False
        self.control_camera = None
        self.parking_mission = None
        # Build the shared read-only map index during initialization, never
        # lazily inside the first timed control observation.
        from app.services.road_semantics import recorded_map
        recorded_map()
        # Latched once the destination is reached, and cleared only by
        # set_destination(). Without a latch the target speed is a pure
        # function of the CURRENT distance to the destination, so a car that
        # coasts a few metres past it sees the distance grow, re-accelerates to
        # cruise, turns around, and orbits forever -- observed during development
        # development once the bicycle model made overshoot possible.
        self.has_arrived = False
        #  server-side NPC traffic + forward range sensor. Created in
        # set_route() once route length is known; None until then. Deliberately
        # NOT accessed directly anywhere except via sense_lead_vehicle()/
        # get_npc_states() below -- see traffic.py's module docstring for why
        # that boundary matters.
        self.traffic = None
        self.sensed_lead = None  # traffic.SensedLeadVehicle | None, refreshed every tick

        # Phase 6: 360-degree surround perception (sensor_rig + tracking +
        # occupancy grid) -- one engine per PhysicsEngine session, same
        # lifetime as self.traffic.
        # Measurement-boundary imperfection config for the surround sensor
        # (position/velocity noise, dropout, occlusion, latency, ghosts).
        # Default no-op -> the perception output is byte-identical to the
        # pre-noise pipeline. Set via scenario params or set_perception_noise().
        self.perception_noise = PerceptionNoiseConfig()
        self.surround_perception = SurroundPerceptionEngine(noise=self.perception_noise, seed=seed)
        self.sensor_faults = {"forward": None, "surround": None, "lane": None}
        self.sensor_outage_start_s = None
        self.sensor_outage_duration_s = 0.
        self.camera_controls = False
        self.scene_camera_controls = False
        self.camera_required_for_drive = False
        self.scene_camera_pending = None
        self.camera_occluded = False
        self.camera_dropout = False
        self.control_camera = None
        self.surround_tracks = []
        self._safety_detections = None
        self._safety_measurement_time_s = None
        self._forward_measurement_time_s = None

        # Phase 7: prediction stage (forecast + intent + risk field +
        # proactive cut-in response). Same session lifetime as the
        # perception engine; carries per-track history between ticks.
        self.prediction_engine = PredictionEngine()
        self.prediction_result = None

        self.speed_kmh = 0.0
        self.rpm = 800.0
        self.coolant_temp = 80.0
        self.fuel_rate = 0.0
        self.co2 = 0.0
        # Real training data's Altitude feature (processed_telemetry.csv) is
        # scaled 128-203 (whatever units/baseline the source OBD-II dataset
        # used), not "meters of elevation" -- this used to start at 10.0,
        # which is wildly out-of-distribution for every tick of every
        # simulated drive (found via robustness eval; the
        # classifier silently tolerated it since Altitude has low feature
        # importance, but it was never a physically-meaningful input).
        # Start at the training mean so simulated telemetry is actually
        # in-distribution.
        self.altitude = 162.5

        self.last_rpm = 800.0
        self.last_co2 = 0.0
        self.last_fuel = 0.0
        self.last_update_time = time.time()
        self.is_paused = False
        self.active_scenario = None

        # Deterministic simulation time (ADR-001 item 4). update() advances
        # this on a fixed 20 ms substep grid, quantising wall-clock jitter
        # on the legacy dt path. The physics integration still uses the raw
        # dt -- only sim_time_s / tick come from the clock -- so the ego
        # trajectory is unchanged.
        self.clock = SimClock(dt_s=SIM_SUBSTEP_DT_S)
        # Optional seeded RNG for step_powertrain's RPM/altitude noise.
        # None => the module-global random (behaviour unchanged for every
        # existing caller and test); an int makes that noise reproducible,
        # which is what the determinism gate (6.5.2) needs.
        self._rng = random.Random(seed) if seed is not None else None
        # Same seed also flows into the per-route TrafficModel spawned in
        # set_route(). NPC start stations / speeds are drawn from that model's
        # RNG, and the forward sensor feeds IDM longitudinal control, so an
        # unseeded (OS-entropy) TrafficModel makes the driven trajectory
        # non-reproducible run-to-run -- most visibly at the low-speed apex of
        # a curve, where whether an NPC happens to sit within sensing range is
        # the difference between the ego stalling and rolling through. seed
        # is None for every existing caller (production, and tests that don't
        # pass one), so TrafficModel(seed=None) keeps its current OS-seeded
        # behaviour unchanged; an int makes (route, dt, seed) bit-reproducible.
        self._seed = seed

    # --- Driver-owned state, re-exposed so callers/tests are unchanged ----
    # (ADR-001 Phase 6.5 / CARLA_MIGRATION_PLAN.md Phase B). Read+write
    # forwarding: a test that does ``engine.lateral_target_d_m = 1.75`` still
    # steers the next tick, and ``engine.shield_verdict`` still reads the
    # independent monitor's verdict.
    @property
    def shield_verdict(self):
        return self._driver.shield_verdict

    @shield_verdict.setter
    def shield_verdict(self, v):
        self._driver.shield_verdict = v

    @property
    def safety_monitor(self):
        return self._driver.safety_monitor

    @property
    def speed_limit_reason(self):
        return self._driver.speed_limit_reason

    @speed_limit_reason.setter
    def speed_limit_reason(self, v):
        self._driver.speed_limit_reason = v

    @property
    def planner_candidates(self):
        return self._driver.planner_candidates

    @planner_candidates.setter
    def planner_candidates(self, v):
        self._driver.planner_candidates = v

    @property
    def planner_chosen_d_m(self):
        return self._driver.planner_chosen_d_m

    @planner_chosen_d_m.setter
    def planner_chosen_d_m(self, v):
        self._driver.planner_chosen_d_m = v

    @property
    def lateral_target_d_m(self):
        return self._driver.lateral_target_d_m

    @lateral_target_d_m.setter
    def lateral_target_d_m(self, v):
        self._driver.lateral_target_d_m = v

    @property
    def spatiotemporal_traj(self):
        return self._driver.spatiotemporal_traj

    @spatiotemporal_traj.setter
    def spatiotemporal_traj(self, v):
        self._driver.spatiotemporal_traj = v

    @property
    def _lc_sm(self):
        return self._driver._lc_sm

    @property
    def _changing_lane_now(self):
        return self._driver._changing_lane_now

    @property
    def _planner_v_target_mps(self):
        return self._driver._planner_v_target_mps

    @property
    def _lat_speed_mps(self):
        return self._driver._lat_speed_mps

    @property
    def _lat_accel_mps2(self):
        return self._driver._lat_accel_mps2

    @property
    def friction_speed_derate_active(self):
        return self._driver.friction_speed_derate_active

    @friction_speed_derate_active.setter
    def friction_speed_derate_active(self, v):
        self._driver.friction_speed_derate_active = v

    @property
    def friction_curve_cap_kmh(self):
        return self._driver.friction_curve_cap_kmh

    @friction_curve_cap_kmh.setter
    def friction_curve_cap_kmh(self, v):
        self._driver.friction_curve_cap_kmh = v

    @property
    def friction_derate_kmh(self):
        return self._driver.friction_derate_kmh

    @friction_derate_kmh.setter
    def friction_derate_kmh(self, v):
        self._driver.friction_derate_kmh = v

    def set_destination(self, lat, lng):
        self.target_lat = lat
        self.target_lng = lng
        # Clear any previous route -- until an async set_route() call
        # supplies real road-following waypoints for this new destination,
        # fall back to a straight line so the car still drives toward it
        # immediately rather than waiting on the routing fetch.
        self.route = []
        self.route_index = 0
        # A new destination re-arms the vehicle after a previous arrival.
        self.has_arrived = False

    def set_route(self, waypoints):
        """waypoints: list of (lat, lng) tuples tracing a real road-following
        path to the current destination (from app.services.routing.get_route()).
        Pass an empty list/None if routing failed -- navigation then falls
        back to a straight bearing to target_lat/target_lng.

        : the raw OSRM polyline is spline-smoothed and resampled to
        uniform ~5 m arc length on ingestion, so the physics engine and the
        HMI share ONE smoothed source of truth ( showed how badly
        backend/frontend disagreement about the world behaves). Curvature
        computed downstream is consequently well-conditioned -- see
        path_smoothing.py for why raw OSRM spacing is not."""
        self.route = smooth_route(waypoints) if waypoints else []
        self.parking_mission = None
        self.route_index = 0
        self.station_distances = [0.0]
        if self.route:
            for i in range(1, len(self.route)):
                dist = self.calculate_distance(self.route[i-1][0], self.route[i-1][1], self.route[i][0], self.route[i][1])
                self.station_distances.append(self.station_distances[-1] + dist)

        #  exact Frenet frame for the new route (None if there is no
        # route). Built from the SAME smoothed waypoint list station_distances
        # above was computed from, so the two stay consistent.
        self.frenet_frame = build_frenet_frame(self.route) if self.route else None
        from app.services.road_semantics import recorded_map
        from app.services.junctions import associate_controls
        road_map = recorded_map()
        self.route_controls = associate_controls(
            {"features": road_map.features, "controls": road_map.controls}, self.frenet_frame)
        self.frenet_search_idx = 0
        self.current_lateral_offset_m = 0.0
        # Driver lateral-stack reset (FSM rebuild, smoothing zeroed, committed
        # maneuver + HMI candidates cleared, target lane -> centre, shield ->
        # NONE, SafetyMonitor reset). Was the inline block here.
        self._driver.on_route_reset()

        if self.controller != "legacy" and self.frenet_frame is not None:
            # OSRM often snaps the route origin to the nearest drivable road
            # rather than echoing the raw requested GPS coordinate. For a
            # software-in-the-loop prototype, the simulated ego should start
            # on that routed road, in its intended lane, with a heading that
            # matches the route tangent. Otherwise the first few ticks can
            # begin several metres off-lane and immediately trip traffic/TTC
            # safety logic before the controller has a physically possible
            # chance to recover.
            self.current_station_m = 0.0
            self.current_lateral_offset_m = LANE_CENTER_D_M
            self.lat, self.lng = frenet_to_latlng(self.frenet_frame, 0.0, LANE_CENTER_D_M)
            _, _, dir_x, dir_z = frenet_to_local_xz(self.frenet_frame, 0.0, LANE_CENTER_D_M)
            self.heading = (math.degrees(math.atan2(dir_x, -dir_z)) + 360.0) % 360.0

        #  (re)spawn traffic for the new route's length. A fresh
        # TrafficModel on every set_route() call means a new destination gets
        # a newly-seeded traffic pattern rather than NPCs left over from the
        # previous route's (now irrelevant) length.
        total_length_m = self.station_distances[-1] if self.route else 0.0
        self.traffic = (
            TrafficModel(total_length_m=total_length_m, seed=self._seed)
            if total_length_m > 0 else None
        )
        self.sensed_lead = None

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        initial_bearing = math.atan2(x, y)
        return (math.degrees(initial_bearing) + 360) % 360

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def set_perception_noise(self, cfg: PerceptionNoiseConfig) -> None:
        """Install a surround-sensor measurement-noise config and rebuild the
        perception engine around it (seeded for reproducibility)."""
        self.perception_noise = cfg
        self.surround_perception = SurroundPerceptionEngine(noise=cfg, seed=self._seed)
        self.surround_tracks = []
        self._safety_detections = None
        self._safety_measurement_time_s = None
        self._forward_measurement_time_s = None

    def update(self, ai_decision: str = "Maintain Speed", dt: Optional[float] = None):
        # ai_decision: retained for call-site/telemetry compatibility only.
        # Since ADR-001 item 5 it has NO effect on control -- the learned
        # model is scored as driver-behaviour analytics, not a driving
        # policy. Kept as a positional arg so existing callers/tests are
        # unchanged.
        now = time.time()
        if dt is None:
            dt = min(now - self.last_update_time, 0.5)
        self.last_update_time = now
        if self.is_paused:
            return
        # Deterministic sim-time advance: quantise dt onto the fixed substep
        # grid (>=1 substep per update). Reported time/tick are then
        # reproducible regardless of wall-clock jitter.
        self.clock = self.clock.advance(max(1, round(dt / self.clock.dt_s)))
        # Captured before any steering/heading integration so the realised yaw
        # rate (and hence lateral acceleration) can be measured at the end.
        heading_before_update = self.heading

        self.step_world_sensors(dt)

        # ---- Driver decision: lateral (extracted, simulator-independent) --
        # ADR-001 Phase 6.5 / CARLA_MIGRATION_PLAN.md Phase B. The ~250-line
        # inline block that used to be here now lives in
        # ``app/services/driver/driver.py`` (``Driver.decide_lateral``) and
        # runs the *identical* math: legacy point-mass steering OR the
        # bicycle model (grip-limited steer cap -> Frenet planner /
        # committed-maneuver FF+Stanley / no-route proportional fallback ->
        # wheel-rate limit -> yaw integration). The Driver owns the
        # lane-change FSM, the committed trajectory, and the lateral-motion
        # smoothing. PhysicsEngine only applies the result.
        _obs = self.observe()
        driver_ctx = self.build_driver_context(dt)

        _lat = self._driver.decide_lateral(_obs, driver_ctx)
        self.steering_angle_rad = _lat.steering_angle_rad
        self.steering_angle = _lat.steering_angle
        self.heading = _lat.heading_deg

        # State Machine Logic
        # Keyed off has_arrived rather than the raw distance so that a car which
        # coasts a few metres past the destination before stopping still reports
        # ARRIVED, instead of falling back to FINAL_APPROACH as though it were
        # still driving there.
        dist = self._last_dist
        corner_turn_deg = self._last_corner_turn_deg
        waypoint_dist = self._last_waypoint_dist
        if self.has_arrived:
            self.driving_state = (DrivingState.ARRIVED if self.speed_kmh < 1.0
                                  else DrivingState.ARRIVING)
        elif dist < 40.0:
            self.driving_state = DrivingState.FINAL_APPROACH
        elif self.speed_kmh < 10.0 and dist > 40.0:
            self.driving_state = DrivingState.DEPARTING
        elif corner_turn_deg > 15.0 and waypoint_dist < 60.0:
            self.driving_state = DrivingState.APPROACHING_TURN
        elif abs(self.steering_angle) > 0.15:
            self.driving_state = DrivingState.TURNING
        else:
            self.driving_state = DrivingState.CRUISING

        if self.speed_kmh < 1.0 and dist > 10.0 and not self.route:
            self.driving_state = DrivingState.IDLE

        parking = self.observe_parking()
        if (parking and parking['parked']) or dist < (.7 if self.parking_mission else 10.0):
            self.has_arrived = True

        # ---- Driver decision: longitudinal (extracted) ------------------
        _lon = self._driver.decide_longitudinal(_obs, driver_ctx)
        if self.controller == "legacy":
            self.speed_kmh = _lon.speed_kmh_override
            heading_delta_deg = (self.heading - heading_before_update + 180) % 360 - 180
            yaw_rate_radps = math.radians(heading_delta_deg) / dt if dt > 0 else 0.0
            self.yaw_rate_degps = math.degrees(yaw_rate_radps)
            self.lateral_accel_mps2 = abs((self.speed_kmh / 3.6) * yaw_rate_radps)
            self.signed_lateral_accel_mps2 = (self.speed_kmh / 3.6) * yaw_rate_radps
            self.kinematic_sideslip_rad = math.atan(0.5 * math.tan(self.steering_angle_rad))

            powertrain_decision = (
                "Accelerate" if self.acceleration_mps2 > 0.3
                else "Decelerate" if self.acceleration_mps2 < -0.3
                else "Maintain Speed"
            )
            pt = step_powertrain(
                speed_kmh=self.speed_kmh,
                ai_decision=powertrain_decision,
                rpm=self.rpm,
                coolant_temp=self.coolant_temp,
                fuel_rate=self.fuel_rate,
                co2=self.co2,
                altitude=self.altitude,
                dt=dt,
                rng=self._rng,
            )
            self.rpm = pt.rpm
            self.coolant_temp = pt.coolant_temp
            self.fuel_rate = pt.fuel_rate
            self.co2 = pt.co2
            self.altitude = pt.altitude

            self.lat, self.lng = advance_position(
                lat=self.lat,
                lng=self.lng,
                heading_deg=self.heading,
                speed_kmh=self.speed_kmh,
                dt=dt,
            )
        else:
            desired_accel = _lon.accel_mps2
            self.step_dynamics(
                steer_rad=self.steering_angle_rad,
                accel_mps2=desired_accel,
                dt=dt,
                heading_before=heading_before_update,
                update_heading=False,
            )

    def build_driver_context(self, dt_s: float) -> DriverContext:
        """Construct the DriverContext from the current world and sensor state."""
        _lane_clear_fn = (
            self._sense_lane_clear
            if self.traffic is not None else None
        )
        _target_heading = (
            self.calculate_bearing(self.lat, self.lng, self._last_waypoint_lat, self._last_waypoint_lng)
            if self._last_waypoint_dist > 5.0 else self.heading
        )
        return DriverContext(
            controller=self.controller,
            dt_s=dt_s,
            speed_kmh=self.speed_kmh,
            heading_deg=self.heading,
            steering_angle_rad=self.steering_angle_rad,
            acceleration_mps2=self.acceleration_mps2,
            ego_lat=self.lat,
            ego_lng=self.lng,
            station_m=self.current_station_m,
            lateral_offset_m=self.current_lateral_offset_m,
            last_lateral_accel_mps2=self.lateral_accel_mps2,
            dist_to_dest_m=self._last_dist,
            waypoint_dist_m=self._last_waypoint_dist,
            target_heading_deg=_target_heading,
            max_curvature_1pm=self._last_max_curvature,
            corner_turn_deg=self._last_corner_turn_deg,
            friction_mu=getattr(self, "friction_mu", 1.0),
            has_arrived=self.has_arrived,
            lane_clear_fn=_lane_clear_fn,
            prediction_result=self.prediction_result,
        )

    def step_world_sensors(self, dt: float, *, perceive: bool = True, predict: bool = True) -> None:
        """Advance world environmental state and sensors.

        Updates route projection, curvature, Frenet frame, traffic, forward
        lead range sensing, surround perception, and prediction.
        """
        # Navigation
        WAYPOINT_ARRIVAL_RADIUS = 15.0
        if self.route and self.route_index < len(self.route):
            PROJECTION_SEARCH_WAYPOINTS = 40
            search_hi = min(len(self.route), self.route_index + PROJECTION_SEARCH_WAYPOINTS)
            nearest_idx, nearest_dist = self.route_index, float('inf')
            for i in range(self.route_index, search_hi):
                d = self.calculate_distance(self.lat, self.lng, self.route[i][0], self.route[i][1])
                if d < nearest_dist:
                    nearest_dist, nearest_idx = d, i
            self.route_index = nearest_idx

            lookahead_m = max(self.LOOKAHEAD_MIN_M, self.LOOKAHEAD_K * (self.speed_kmh / 3.6))
            target_idx, accumulated = self.route_index, 0.0
            while target_idx < len(self.route) - 1 and accumulated < lookahead_m:
                p1, p2 = self.route[target_idx], self.route[target_idx + 1]
                accumulated += self.calculate_distance(p1[0], p1[1], p2[0], p2[1])
                target_idx += 1
            waypoint_lat, waypoint_lng = self.route[target_idx]
            waypoint_dist = self.calculate_distance(self.lat, self.lng, waypoint_lat, waypoint_lng)
        else:
            waypoint_lat, waypoint_lng = self.target_lat, self.target_lng
            waypoint_dist = self.calculate_distance(self.lat, self.lng, waypoint_lat, waypoint_lng)

        dist = self.calculate_distance(self.lat, self.lng, self.target_lat, self.target_lng)

        CORNER_LOOKAHEAD_M = 60.0
        MIN_SEGMENT_FOR_CURVATURE_M = 1.0
        SANE_MAX_TURN_PER_STEP_DEG = 55.0
        MIN_TURN_RADIUS_M = 8.0
        MAX_PLAUSIBLE_CURVATURE = 1.0 / MIN_TURN_RADIUS_M
        CURVATURE_WINDOW_M = 12.0
        corner_turn_deg = 0.0
        max_curvature = 0.0
        if self.route and len(self.route) > self.route_index + 2:
            steps = []
            accumulated_m = 0.0
            prev_bearing = None
            idx = self.route_index
            while idx < len(self.route) - 1 and accumulated_m < CORNER_LOOKAHEAD_M:
                p1, p2 = self.route[idx], self.route[idx + 1]
                seg_len = self.calculate_distance(p1[0], p1[1], p2[0], p2[1])
                accumulated_m += seg_len
                bearing = self.calculate_bearing(p1[0], p1[1], p2[0], p2[1])
                if prev_bearing is not None and seg_len >= MIN_SEGMENT_FOR_CURVATURE_M:
                    turn_deg = abs((bearing - prev_bearing + 180) % 360 - 180)
                    if turn_deg <= SANE_MAX_TURN_PER_STEP_DEG:
                        corner_turn_deg = max(corner_turn_deg, turn_deg)
                        steps.append((seg_len, math.radians(turn_deg)))
                prev_bearing = bearing
                idx += 1
            for i in range(len(steps)):
                win_len = 0.0
                win_turn = 0.0
                j = i
                while j < len(steps) and win_len < CURVATURE_WINDOW_M:
                    win_len += steps[j][0]
                    win_turn += steps[j][1]
                    j += 1
                if win_len >= MIN_SEGMENT_FOR_CURVATURE_M:
                    max_curvature = max(max_curvature, win_turn / win_len)
        self.path_curvature = min(max_curvature, MAX_PLAUSIBLE_CURVATURE)

        self._last_waypoint_lat = waypoint_lat
        self._last_waypoint_lng = waypoint_lng
        self._last_waypoint_dist = waypoint_dist
        self._last_dist = dist
        self._last_corner_turn_deg = corner_turn_deg
        self._last_max_curvature = max_curvature

        FRENET_SEARCH_BACKWARD_SLACK = 5
        if self.frenet_frame is not None:
            search_from = max(0, self.frenet_search_idx - FRENET_SEARCH_BACKWARD_SLACK)
            s_ego, d_ego, matched_idx = project_to_frenet(
                self.frenet_frame, self.lat, self.lng, search_start_idx=search_from,
            )
            self.frenet_search_idx = matched_idx
            self.current_station_m = s_ego
            self.current_lateral_offset_m = d_ego
        else:
            self.current_station_m = 0.0
            self.current_lateral_offset_m = 0.0

        if self.traffic is not None:
            ego_vehicle=None
            if self.frenet_frame is not None:
                from app.services.safety_geometry import Footprint
                from app.services.vehicle_geometry import lateral_half_extent,CHASSIS_HALF_LENGTH_M
                ex,ez=latlng_to_local(self.lat,self.lng,self.frenet_frame.origin_lat,self.frenet_frame.origin_lng)
                yaw=math.radians(self.heading)
                motion_yaw=yaw+sideslip(self.steering_angle_rad)
                speed=self.speed_kmh/3.6
                half_width=lateral_half_extent(0.,self.steering_angle_rad,self.WHEELBASE_M)
                ego_vehicle=WorldVehicleState(self.current_station_m,
                    Footprint(ex,ez,yaw,2*CHASSIS_HALF_LENGTH_M,2*half_width,
                              speed*math.sin(motion_yaw),-speed*math.cos(motion_yaw)))
            self.traffic.update(dt,self.current_station_m,ego_vehicle=ego_vehicle,frenet_frame=self.frenet_frame)
        if not self._sensor_outage_active():
            try:
                self.sensed_lead = (self.traffic.sense_lead_vehicle(
                    self.current_station_m,
                    ego_lane_offset=self.current_lateral_offset_m,
                    ego_target_lane_offset=self.lateral_target_d_m,
                ) if self.traffic is not None else None)
                self._forward_measurement_time_s = self.clock.sim_time_s
                self.sensor_faults['forward'] = None
            except Exception as error:
                self._record_sensor_fault('forward', error)
                self.sensed_lead = None
                self._forward_measurement_time_s = None

        if perceive:
            self.step_perception(dt)
        if predict:
            self.step_prediction(dt)

    def _sensor_outage_active(self) -> bool:
        start = self.sensor_outage_start_s
        return start is not None and start <= self.clock.sim_time_s < start + self.sensor_outage_duration_s

    def _sense_lane_clear(self, station_m: float, offset_m: float) -> bool:
        if self._sensor_outage_active() or self.traffic is None or self.frenet_frame is None:
            return False
        try:
            clear = bool(self.traffic.sense_lane_clear(station_m, offset_m))
            self.sensor_faults["lane"] = None
            return clear
        except Exception as error:
            self._record_sensor_fault('lane', error)
            return False

    def _record_sensor_fault(self, source: str, error: Exception) -> None:
        if self.sensor_faults[source] != type(error).__name__:
            logging.getLogger(__name__).exception('%s sensor producer failed; feed marked unavailable', source)
        self.sensor_faults[source] = type(error).__name__

    def step_perception(self, dt: float) -> None:
        if self.camera_controls and not self._sensor_outage_active():
            if self.control_camera is None:
                from app.services.control_camera_bridge import ControlCameraPipeline
                self.control_camera = ControlCameraPipeline(source="scene_RGB_camera_classical_CV" if self.scene_camera_controls else "synthetic_RGB_camera_classical_CV")
            if self.scene_camera_controls:
                self.control_camera.acquire_scene(self)
            else:
                self.control_camera.acquire(self)
        if self._sensor_outage_active():
            return  # Preserve producer time and last reading; do not refresh it.
        try:
            self.surround_tracks = self.surround_perception.step(
                self.frenet_frame, self.current_station_m, self.current_lateral_offset_m,
                self.traffic.npcs if self.traffic is not None else [], dt,
                capture_time_s=self.clock.sim_time_s,
            )
            self._safety_detections = tuple(SafetyDetection(
                d.x, d.z, d.vx, d.vz, *dimensions_for(d.entity_class)[:2])
                for d in self.surround_perception.last_detections)
            self._safety_measurement_time_s = self.surround_perception.last_capture_time_s
            self.sensor_faults['surround'] = None
        except Exception as error:
            self._record_sensor_fault('surround', error)
            self.surround_tracks = []
            self._safety_detections = None
            self._safety_measurement_time_s = None


    def step_prediction(self, dt: float) -> None:
        if self.controller != "legacy" and self.surround_tracks:
            self.prediction_result = self.prediction_engine.step(
                clock=self.clock,
                surround_tracks=self.surround_tracks,
                frenet_frame=self.frenet_frame,
                ego_lateral_offset_m=self.current_lateral_offset_m,
                dt=dt,
            )
        else:
            self.prediction_result = None

    def step_dynamics(
        self,
        steer_rad: float,
        accel_mps2: float,
        dt: float,
        heading_before: Optional[float] = None,
        update_heading: bool = True,
    ) -> None:
        """Apply an actuator command directly to drive vehicle dynamics.

        Owns the jerk-limited acceleration, bicycle heading integration,
        powertrain relaxation, and great-circle displacement.
        """
        if heading_before is None:
            heading_before = self.heading

        values = (dt, steer_rad, accel_mps2, self.speed_kmh,
                  self.acceleration_mps2, self.heading, self.lat, self.lng,
                  self.friction_mu,self.kinematic_sideslip_rad)
        if not all(math.isfinite(v) for v in values) or dt <= 0 or self.speed_kmh<0 or abs(self.kinematic_sideslip_rad)>abs(sideslip(self.MAX_STEER_RAD))+1e-9:
            raise ValueError("Dynamics requires finite state/commands and positive dt")

        max_accel_step = self.JERK_MAX_MPS3 * dt
        accel_at_tick_start = self.acceleration_mps2
        emergency = self.shield_verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE
        if emergency:
            # Collision mitigation takes precedence over the comfort limiter.
            self.acceleration_mps2 = -self.A_MAX_BRAKE_MPS2
            accel_at_tick_start = self.acceleration_mps2
        accel_error = accel_mps2 - self.acceleration_mps2
        if emergency:
            accel_error = 0.0
        self.acceleration_mps2 += max(-max_accel_step, min(max_accel_step, accel_error))

        old_speed_mps = self.speed_kmh / 3.6
        previous_beta_rad = self.kinematic_sideslip_rad
        previous_steer_rad = math.atan(2.*math.tan(self.kinematic_sideslip_rad))
        allocation = allocate_grip(speed_mps=old_speed_mps,
            previous_steer_rad=previous_steer_rad,requested_steer_rad=steer_rad,
            requested_acceleration_mps2=self.acceleration_mps2,
            friction_mu=self.friction_mu,dt_s=dt,wheelbase_m=self.WHEELBASE_M,
            max_steer_rad=self.MAX_STEER_RAD,steer_rate_radps=self.STEER_RATE_MAX_RADPS,
            emergency=emergency,max_braking_mps2=self.A_MAX_BRAKE_MPS2)
        self.acceleration_mps2 = allocation.acceleration_mps2
        if self.controller != "legacy":
            self.steering_angle_rad = allocation.steer_rad
            self.steering_angle = self.steering_angle_rad / self.MAX_STEER_RAD
            yaw_rate = old_speed_mps * math.cos(sideslip(self.steering_angle_rad)) * math.tan(self.steering_angle_rad) / self.WHEELBASE_M
            # The old inline update path has already integrated the requested
            # steer. Rebase on its supplied pre-update heading when allocation
            # changes the actual wheels; do not integrate that turn twice.
            self.heading = (heading_before + math.degrees(yaw_rate * dt) + 360) % 360

        v_mps = self.speed_kmh / 3.6
        v_mps += self.acceleration_mps2 * dt
        if v_mps <= 0.0:
            v_mps = 0.0
            if self.acceleration_mps2 < 0.0:
                self.acceleration_mps2 = min(0.0, self.acceleration_mps2 + max_accel_step)
        # Grip/recovery authority may override comfort jerk after a friction
        # change. Reapplying the old jerk clamp here would undo physical limits.
        if not (emergency or allocation.acceleration_limited or allocation.already_infeasible):
            self.acceleration_mps2 = max(accel_at_tick_start-max_accel_step,
                min(accel_at_tick_start+max_accel_step,self.acceleration_mps2))
        self.speed_kmh = min(v_mps * 3.6, 160.0)

        heading_delta_deg = (self.heading - heading_before + 180) % 360 - 180
        yaw_rate_radps = math.radians(heading_delta_deg) / dt if dt > 0 else 0.0
        self.yaw_rate_degps = math.degrees(yaw_rate_radps)
        self.lateral_accel_mps2 = abs((self.speed_kmh / 3.6) * yaw_rate_radps)
        self.signed_lateral_accel_mps2 = (self.speed_kmh / 3.6) * yaw_rate_radps
        self.kinematic_sideslip_rad = math.atan(0.5 * math.tan(self.steering_angle_rad))
        conservative_total = math.hypot(self.acceleration_mps2,allocation.lateral_bound_mps2)
        self.grip_budget_state = {
            "scope": "discrete_tick_conservative_no_slip_budget",
            "limit_mps2": allocation.limit_mps2,
            "lateral_bound_mps2": allocation.lateral_bound_mps2,
            "total_bound_mps2": conservative_total,
            "motion_lateral_acceleration_mps2": (self.speed_kmh/3.6)*(yaw_rate_radps+
                (self.kinematic_sideslip_rad-previous_beta_rad)/dt),
            "already_infeasible": allocation.already_infeasible,
            "recovery_active": allocation.already_infeasible,
            "within_modeled_budget": not allocation.already_infeasible and conservative_total <= allocation.limit_mps2+1e-9,
            "applied_tick_bound_within_budget": conservative_total <= allocation.limit_mps2+1e-9,
            "steering_limited": allocation.steering_limited,
            "acceleration_limited": allocation.acceleration_limited,
        }


        powertrain_decision = (
            "Accelerate" if self.acceleration_mps2 > 0.3
            else "Decelerate" if self.acceleration_mps2 < -0.3
            else "Maintain Speed"
        )
        pt = step_powertrain(
            speed_kmh=self.speed_kmh,
            ai_decision=powertrain_decision,
            rpm=self.rpm,
            coolant_temp=self.coolant_temp,
            fuel_rate=self.fuel_rate,
            co2=self.co2,
            altitude=self.altitude,
            dt=dt,
            rng=self._rng,
        )
        self.rpm = pt.rpm
        self.coolant_temp = pt.coolant_temp
        self.fuel_rate = pt.fuel_rate
        self.co2 = pt.co2
        self.altitude = pt.altitude

        self.lat, self.lng = advance_position(
            lat=self.lat,
            lng=self.lng,
            heading_deg=heading_before + heading_delta_deg / 2 + math.degrees(self.kinematic_sideslip_rad),
            speed_kmh=self.speed_kmh,
            dt=dt,
        )
        _, factors = wheel_geometry(self.steering_angle_rad, self.WHEELBASE_M)
        for i, factor in enumerate(factors):
            self.wheel_rotation_rad[i] += (self.speed_kmh / 3.6) * dt * factor / WHEEL_RADIUS_M

    def get_ml_features(self):
        rpm_delta = self.rpm - self.last_rpm
        co2_delta = self.co2 - self.last_co2
        fuel_delta = self.fuel_rate - self.last_fuel

        self.last_rpm = self.rpm
        self.last_co2 = self.co2
        self.last_fuel = self.fuel_rate

        return {
            'Altitude': round(self.altitude, 2),
            'CO2': round(self.co2, 2),
            'Coolant': round(self.coolant_temp, 2),
            'Litre per 100km(Instant)': round(self.fuel_rate, 2),
            'RPM': round(self.rpm, 2),
            'RPM_Delta': round(rpm_delta, 2),
            'CO2_Delta': round(co2_delta, 2),
            'Fuel_Rate_Delta': round(fuel_delta, 2)
        }

    def get_navigation_state(self):
        # Integrated center pose is authoritative. Frenet projection can clamp
        # at an endpoint and must never conceal actual longitudinal motion.
        world_x, world_z = (latlng_to_local(self.lat, self.lng, self.frenet_frame.origin_lat, self.frenet_frame.origin_lng)
                            if self.frenet_frame is not None else (0.0, 0.0))
        return {
            "lat": self.lat,
            "lng": self.lng,
            "target_lat": self.target_lat,
            "target_lng": self.target_lng,
            "heading": self.heading,
            "speed": self.speed_kmh,
            "steering": self.steering_angle,
            #  how far along the current route the car is. The full
            # waypoint list is pushed once (via a separate "route" WS
            # message, sent whenever a new route is fetched) rather than
            # resent every tick -- the frontend slices its stored route
            # array using this index instead of receiving hundreds of
            # lat/lng pairs at 10Hz.
            "route_index": self.route_index,
            "has_route": bool(self.route),
            "driving_state": self.driving_state,
            "station_m": self.current_station_m,
            # Deterministic sim time (ADR-001 item 4) -- single source of
            # truth for the protocol's simulation_time_s / tick, replacing
            # the loop's own predictions_count * 0.1 arithmetic.
            "simulation_time_s": self.clock.sim_time_s,
            "tick": self.clock.tick,
            # control/diagnostic state. Surfaced for the HMI  and
            # for the A/B evaluation in .
            "controller": self.controller,
            "acceleration": self.acceleration_mps2,
            "steering_angle_rad": self.steering_angle_rad,
            "path_curvature": self.path_curvature,
            "lateral_accel": self.lateral_accel_mps2,
            "grip_budget": dict(self.grip_budget_state),
            "control_perception": self.control_camera.state(self.clock.sim_time_s) if self.camera_controls and self.control_camera else {"mode": "scene_camera_prototype" if self.scene_camera_controls else "camera_prototype" if self.camera_controls else "map_schedule_baseline", "fresh": False, "source": "camera_pending" if self.camera_controls else "predefined_map_and_signal_schedule"},
            "speed_limit_reason": self.speed_limit_reason,
            #  forward range-sensor output (gap + relative speed to the
            # nearest same-lane vehicle ahead), or null if nothing is sensed.
            # This is DELIBERATELY the only NPC-related information available
            # through the normal navigation state -- see traffic.py.
            "sensed_lead_gap_m": self.sensed_lead.gap_m if self.sensed_lead else None,
            "sensed_lead_speed_kmh": self.sensed_lead.lead_speed_kmh if self.sensed_lead else None,
            # exact Frenet lateral offset (signed, metres, positive =
            # right/same-direction lane) and the planner's current tracked
            # target -- the frontend's hard-coded LANE_OFFSET_M render hack
            # is replaced by rendering AT this real value, so the HMI shows
            # the backend's actual planned lateral position, not an assumed
            # constant.
            "lateral_offset_m": self.current_lateral_offset_m,
            "lateral_target_m": self.lateral_target_d_m,
            "is_paused": self.is_paused,
            "active_scenario": self.active_scenario,
            "road_heading_deg": (
                (math.degrees(math.atan2(frenet_to_local_xz(self.frenet_frame, self.current_station_m, 0.0)[2],
                                        -frenet_to_local_xz(self.frenet_frame, self.current_station_m, 0.0)[3])) + 360.0) % 360.0
                if self.frenet_frame is not None and len(self.frenet_frame.station) >= 2 else self.heading
            ),
            "relative_heading_deg": (
                (self.heading - (
                    (math.degrees(math.atan2(frenet_to_local_xz(self.frenet_frame, self.current_station_m, 0.0)[2],
                                            -frenet_to_local_xz(self.frenet_frame, self.current_station_m, 0.0)[3])) + 360.0) % 360.0
                ) + 180.0) % 360.0 - 180.0
                if self.frenet_frame is not None and len(self.frenet_frame.station) >= 2 else 0.0
            ),
            "heading_deg": self.heading,
            "world_heading_deg": self.heading,
            "kinematic_sideslip_rad": self.kinematic_sideslip_rad,
            "lateral_accel_mps2": self.lateral_accel_mps2,
            "signed_lateral_accel": self.signed_lateral_accel_mps2,
            "signed_lateral_accel_mps2": self.signed_lateral_accel_mps2,
            "yaw_rate_degps": self.yaw_rate_degps,
            "wheels": {
                "steering_rad": list(wheel_geometry(self.steering_angle_rad, self.WHEELBASE_M)[0]),
                "rotation_rad": list(self.wheel_rotation_rad),
                "wheelbase_m": self.WHEELBASE_M, "track_width_m": TRACK_WIDTH_M,
                "radius_m": WHEEL_RADIUS_M, "pose_reference": "vehicle_center",
            },
            "friction_mu": max(0.1, min(1.0, getattr(self, "friction_mu", 1.0))),
            "friction_speed_derate_active": bool(getattr(self, "friction_speed_derate_active", False)),
            "friction_derate_kmh": round(float(getattr(self, "friction_derate_kmh", 0.0)), 2),
            "world_pos_x": world_x,
            "world_pos_z": world_z,
            "world_pos": {"x": world_x, "y": 0.0, "z": world_z},
        }

    def reset_state(
        self,
        station_m: float = 0.0,
        speed_kmh: float = 0.0,
        lateral_offset_m: float = EGO_LANE_OFFSET_M,
        heading_deg: Optional[float] = None,
        target_speed_kmh: float = 50.0,
    ):
        """Cleanly reset physical vehicle state for scenario execution without needing to refetch route."""
        values = (station_m, speed_kmh, lateral_offset_m, target_speed_kmh)
        if not all(math.isfinite(v) for v in values) or not 0 <= speed_kmh <= 160:
            raise ValueError("Reset requires finite state and speed in [0, 160] km/h")
        if heading_deg is not None and not math.isfinite(heading_deg):
            raise ValueError("Reset heading must be finite")
        self.current_station_m = station_m
        self.current_lateral_offset_m = lateral_offset_m
        self.lateral_target_d_m = lateral_offset_m
        self.speed_kmh = speed_kmh
        self.acceleration_mps2 = 0.0
        self.steering_angle = 0.0
        self.steering_angle_rad = 0.0
        self.kinematic_sideslip_rad = 0.0
        self.has_arrived = False
        self.frenet_search_idx = 0
        self.route_index = 0
        self.speed_limit_reason = "cruise"
        self.mapped_road_rules = None
        self.grip_budget_state = {"scope": "modeled_acceleration_budget", "already_infeasible": False}
        self.friction_mu = 1.0
        self.friction_speed_derate_active = False
        self.friction_curve_cap_kmh = None
        self.friction_derate_kmh = 0.0
        self.shield_verdict = safety_shield.ShieldVerdict(approved=True, risk_level=safety_shield.RISK_NONE)
        self.safety_monitor.reset()
        # A scenario reset should not leave stale track IDs/coasted history
        # from before the reset -- fresh engine, same as every other piece
        # of per-session state this method resets.
        self.surround_perception = SurroundPerceptionEngine(
            noise=getattr(self, "perception_noise", PerceptionNoiseConfig()), seed=self._seed,
        )
        self.surround_tracks = []
        self._safety_detections = None
        self._safety_measurement_time_s = None
        self._forward_measurement_time_s = None
        self.prediction_engine.reset()
        self.prediction_result = None
        self._driver.on_route_reset()
        self.rpm = 800.0
        self.co2 = 0.0
        self.fuel_rate = 0.0
        self.coolant_temp = 80.0
        self.last_rpm = self.rpm
        self.altitude = 162.5
        self.last_co2 = self.co2
        self.last_fuel = self.fuel_rate
        self.lateral_accel_mps2 = 0.0
        self.signed_lateral_accel_mps2 = 0.0
        self.yaw_rate_degps = 0.0
        self.wheel_rotation_rad = [0.0] * 4
        if self._seed is not None:
            self._rng = random.Random(self._seed)
        # A scenario reset returns deterministic sim time to zero too.
        self.clock = SimClock(dt_s=self.clock.dt_s)
        self.sensor_faults = {"forward": None, "surround": None, "lane": None}
        self.sensor_outage_start_s = None
        self.sensor_outage_duration_s = 0.
        self.camera_controls = False
        self.scene_camera_controls = False
        self.camera_required_for_drive = False
        self.scene_camera_pending = None
        self.camera_occluded = False
        self.camera_dropout = False
        self.control_camera = None

        if self.frenet_frame is not None:
            # Seed the bounded projection window from the reset station. Starting
            # at segment zero can project a distant reset onto an unrelated road
            # and invent a kilometre-scale lateral error on its first tick.
            self.frenet_search_idx = max(0, bisect_left(self.frenet_frame.station, station_m) - 1)
            self.route_index = self.frenet_search_idx
            self.lat, self.lng = frenet_to_latlng(self.frenet_frame, station_m, lateral_offset_m)
            if heading_deg is not None:
                self.heading = heading_deg
            else:
                _, _, dir_x, dir_z = frenet_to_local_xz(self.frenet_frame, station_m, lateral_offset_m)
                self.heading = (math.degrees(math.atan2(dir_x, -dir_z)) + 360.0) % 360.0

    def get_planner_candidates(self):
        """The last tick's scored lateral candidate set , for the HMI
         to render dimmed alternatives alongside the chosen path, and
        for the previous evaluation. Separate from get_navigation_state() to keep
        that payload small at 10Hz -- callers that don't need the full
        candidate breakdown (most ticks, most of the time) don't pay for it."""
        return [
            {
                "d_target": c.d_target,
                "cost": c.cost,
                "is_chosen": bool(getattr(c, "is_chosen", False)) or c.d_target == self.planner_chosen_d_m,
                "is_lane_change": c.is_lane_change,
            }
            for c in self.planner_candidates
        ]

    def get_safety_shield_state(self):
        """The last tick's independent Safety Shield verdict -- see
        safety_shield.py's module docstring for why this is evaluated
        separately from get_navigation_state()'s planner/IDM output rather
        than folded into it."""
        return {
            "approved": self.shield_verdict.approved,
            "risk_level": self.shield_verdict.risk_level,
            "reasons": self.shield_verdict.reasons,
            "override_action": self.shield_verdict.override_action,
            "ttc_s": self.shield_verdict.ttc_s,
            "overlap_detected": self.shield_verdict.overlap_detected,
            "body_contained": self.shield_verdict.body_contained,
            "swept_body_contained": self.shield_verdict.swept_body_contained,
            "footprint_collision_risk": self.shield_verdict.footprint_collision_risk,
        }

    def get_npc_states(self):
        """Full NPC state for the HMI renderer  -- a rendering
        concern, distinct from what the ego's own sensor may perceive for
        control purposes (get_navigation_state()'s sensed_lead_* fields)."""
        return self.traffic.get_npc_states() if self.traffic is not None else []

    def observe(self) -> SensorObservation:
        """Package the current post-perception state into the World -> Driver
        boundary type (``interfaces.SensorObservation``). Pure read, no side
        effects. This is the contract a CARLA world backend must also produce:
        the Driver never touches ``PhysicsEngine`` (or CARLA) directly, only
        the object this returns.

        Sensor-only by construction -- ``lead`` is the forward range return,
        ``surround_tracks`` is the ego's own tracker output, ``frenet_frame``
        is map geometry. No NPC list, no NPC identity, no absolute NPC pose.
        """
        lead = None
        if self.sensed_lead is not None:
            lead = LeadObservation(
                gap_m=self.sensed_lead.gap_m,
                lead_speed_mps=self.sensed_lead.lead_speed_kmh / 3.6,
            )
        adj_clear = self._sense_lane_clear(self.current_station_m, ADJACENT_LANE_OFFSET_M)
        from app.services.road_semantics import recorded_map
        self.mapped_road_rules = recorded_map().match(self.lat, self.lng, self.heading)
        return SensorObservation(
            clock=self.clock,
            ego=EgoProprioception(
                speed_mps=self.speed_kmh / 3.6,
                heading_deg=self.heading,
                accel_mps2=self.acceleration_mps2,
                steer_rad=self.steering_angle_rad,
                station_m=self.current_station_m,
                lateral_offset_m=self.current_lateral_offset_m,
            ),
            lead=lead,
            adjacent_lane_clear=adj_clear,
            surround_tracks=tuple(self.surround_tracks),
            safety_detections=getattr(self, "_safety_detections", None),
            safety_measurement_age_s=(max(0.0, self.clock.sim_time_s - self._safety_measurement_time_s)
                                      if self._safety_measurement_time_s is not None else math.inf),
            sensor_health=SensorHealth(self._forward_measurement_time_s, self._safety_measurement_time_s,
                camera_required=self.camera_required_for_drive,
                camera_required_views=(CAMERA_IDS if self.camera_required_for_drive else ()),
                camera_capture_times_s=tuple((name,frame.capture_time_s) for name,frame in
                    getattr(self.control_camera,'camera_frames',{}).items()),
                camera_capture_time_s=(self.control_camera.frame.capture_time_s
                    if self.scene_camera_controls and self.control_camera is not None
                    and self.control_camera.frame is not None else None)),
            occupancy=None,
            frenet_frame=self.frenet_frame,
            road_rules=self.mapped_road_rules,
            traffic_controls=self.observe_traffic_controls(),
            parking=self.observe_parking(),
        )

    def observe_parking(self):
        from app.services.parking import parking_observation
        return parking_observation(self)

    def scene_control_objects(self):
        """World-only display geometry. Never passed to Driver observations."""
        from app.services.junctions import simulated_signal_phase
        return [{"node_id":c.node_id,"kind":c.kind,"station_m":c.station_m,
            "stop_station_m":c.station_m-6.,
            "phase":simulated_signal_phase(c.node_id,self.clock.sim_time_s) if c.kind=="traffic_signals" else None}
            for c in self.route_controls if -20<=c.station_m-self.current_station_m<=200]

    def observe_traffic_controls(self):
        if self.camera_controls:
            if self.control_camera is None:
                from app.services.control_camera_bridge import ControlCameraPipeline
                self.control_camera = ControlCameraPipeline(source="scene_RGB_camera_classical_CV" if self.scene_camera_controls else "synthetic_RGB_camera_classical_CV")
            return self.control_camera.observations(self)
        from app.services.junctions import ControlObservation, simulated_signal_phase
        now = self.clock.sim_time_s
        return tuple(ControlObservation(c,
            simulated_signal_phase(c.node_id, now) if c.kind == "traffic_signals" else None,
            now) for c in self.route_controls
            if -20 <= c.station_m-self.current_station_m <= 200)

    def build_learning_observation(self):
        """Assemble the SENSOR-ONLY Observation a learned driving component
        is permitted to see, from the current perception/prediction/ego
        state. Not consumed by control (there is no learned component yet);
        this is the audited boundary -- see app/services/learning/ and
        docs/OBSERVATION_BOUNDARY.md. Returns None before a route/frame exists.
        """
        if self.frenet_frame is None:
            return None
        from app.services.learning import build_observation

        pr = self.prediction_result
        intents = dict(pr.intents) if pr is not None else {}
        lat_rate_by_track = dict(pr.lateral_rate_by_track) if pr is not None else {}
        long_acc_by_track = dict(pr.long_accel_by_track) if pr is not None else {}
        _, _, _tx, _tz = frenet_to_local_xz(self.frenet_frame, self.current_station_m, 0.0)
        road_h = math.atan2(_tx, -_tz)
        goal_dist = self.calculate_distance(self.lat, self.lng, self.target_lat, self.target_lng)

        # Road context AHEAD: signed curvature sampled at several look-ahead
        # distances (map data -- from the route FrenetFrame, not privileged),
        # and the distance to the next significant bend (|kappa| > 1/50 m),
        # capped at 200 m or the remaining route, whichever is shorter.
        _route_end_m = (
            self.frenet_frame.station[-1]
            if getattr(self.frenet_frame, "station", None) else self.current_station_m
        )
        _look = (0.0, 15.0, 30.0, 50.0, 80.0)
        _curv_profile = []
        for _L in _look:
            _s = min(self.current_station_m + _L, max(0.0, _route_end_m - 1.0))
            try:
                _curv_profile.append(float(signed_curvature_at(self.frenet_frame, _s)))
            except Exception:
                _curv_profile.append(0.0)
        _dist_to_maneuver = min(200.0, max(0.0, _route_end_m - self.current_station_m))
        _scan = 5.0
        while _scan <= min(200.0, _route_end_m - self.current_station_m - 1.0):
            try:
                if abs(signed_curvature_at(self.frenet_frame, self.current_station_m + _scan)) > (1.0 / 50.0):
                    _dist_to_maneuver = _scan
                    break
            except Exception:
                pass
            _scan += 5.0
        adj_clear = self._sense_lane_clear(self.current_station_m, ADJACENT_LANE_OFFSET_M)
        return build_observation(
            ego_speed_mps=self.speed_kmh / 3.6,
            ego_accel_mps2=self.acceleration_mps2,
            ego_lane_offset_m=self.current_lateral_offset_m,
            ego_lateral_rate_mps=getattr(self, "_lat_speed_mps", 0.0),
            ego_heading_err_rad=((math.radians(self.heading) - road_h + math.pi) % (2 * math.pi)) - math.pi,
            road_curvature_1pm=getattr(self, "_road_kappa_signed", 0.0) or self.path_curvature,
            friction_estimate=max(0.1, min(1.0, getattr(self, "friction_mu", 1.0))),
            dist_to_goal_m=goal_dist,
            lead_gap_m=self.sensed_lead.gap_m if self.sensed_lead else None,
            lead_speed_mps=(self.sensed_lead.lead_speed_kmh / 3.6) if self.sensed_lead else None,
            surround_tracks=self.surround_tracks,
            intent_by_track=intents,
            left_lane_clear=adj_clear,
            right_lane_clear=adj_clear,
            binding_constraint=self.speed_limit_reason,
            route_tangent_heading_rad=road_h,
            lateral_rate_by_track=lat_rate_by_track,
            long_accel_by_track=long_acc_by_track,
            curvature_profile_1pm=tuple(_curv_profile),
            curvature_profile_lookahead_m=_look,
            dist_to_next_maneuver_m=_dist_to_maneuver,
        )

    def get_surround_perception_state(self):
        """Confirmed surround tracks from Phase 6's 360-degree perception
        layer -- distinct from get_npc_states() (the full simulated-truth
        NPC list) and from sensed_lead (the forward-only IDM sensor); this
        is what the ego's own multi-sensor rig + tracker actually confirms,
        including detections behind/beside the ego the forward sensor never
        sees."""
        return self.surround_perception.get_state()

    def get_prediction_state(self):
        """Phase 7: per-agent forecasts + intent + the current proactive
        cut-in response. ``agents`` carries a 3 s / 0.1 s forecast trail and
        the top intents per tracked agent; ``cut_in`` summarises whether the
        ego is currently easing off for a developing merge."""
        pr = self.prediction_result
        if pr is None:
            return {"agents": [], "cut_in": {"active": False, "probability": 0.0,
                                             "track_id": None, "time_to_cross_s": None},
                    "proactive_decel_mps2": 0.0}
        agents = []
        for pred in pr.output.agents:
            agents.append({
                "track_id": pred.track_id,
                "intent": [{"label": k, "p": round(p, 3)} for k, p in pred.intent],
                "trail": [{"t": s.t_s, "x": round(s.x, 2), "z": round(s.z, 2)}
                          for s in pred.states],
            })
        return {
            "agents": agents,
            "cut_in": {
                "active": pr.proactive_decel_mps2 > 0.0,
                "probability": round(pr.cut_in_probability, 3),
                "track_id": pr.cut_in_track_id,
                "time_to_cross_s": (round(pr.time_to_cross_s, 2)
                                    if pr.time_to_cross_s is not None else None),
            },
            "proactive_decel_mps2": round(pr.proactive_decel_mps2, 2),
        }
