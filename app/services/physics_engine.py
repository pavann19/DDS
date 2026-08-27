import math
import time
from enum import Enum
from typing import Optional, List, Dict, Any

from app.services.traffic import TrafficModel, EGO_LANE_OFFSET_M, ADJACENT_LANE_OFFSET_M
from app.services.path_smoothing import smooth_route
from app.services.frenet import (
    build_frenet_frame,
    project_to_frenet,
    frenet_to_local_xz,
    frenet_to_latlng,
)
from app.services.car_following import idm_acceleration
from app.services import safety_shield
from app.services.world import advance_position, step_powertrain
from app.services.driver import plan_lateral_offset
from app.services.perception.perception_engine import SurroundPerceptionEngine
from app.services.planner import LANE_CENTER_D_M

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

    def __init__(self, start_lat=37.7749, start_lng=-122.4194, controller="bicycle"):
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
        self.acceleration_mps2 = 0.0
        # Diagnostics surfaced for the HMI / evaluation .
        self.path_curvature = 0.0       # 1/m, max |curvature| in the lookahead window
        self.lateral_accel_mps2 = 0.0   # v^2 * kappa, the comfort-limiting quantity
        self.speed_limit_reason = "cruise"  # which constraint is currently binding

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
        self.lateral_target_d_m = LANE_CENTER_D_M
        self.planner_candidates = []
        self.planner_chosen_d_m = LANE_CENTER_D_M
        # Default "nothing to report yet" verdict -- legacy never runs the
        # shield at all (it's the untouched P6-6 A/B control), and the
        # bicycle controller doesn't populate a real one until its first
        # tick with a route.
        self.shield_verdict = safety_shield.ShieldVerdict(approved=True, risk_level=safety_shield.RISK_NONE)
        self.driving_state = DrivingState.IDLE
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
        self.surround_perception = SurroundPerceptionEngine()
        self.surround_tracks = []

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
        self.frenet_search_idx = 0
        self.current_lateral_offset_m = 0.0
        self.lateral_target_d_m = LANE_CENTER_D_M
        self.planner_candidates = []
        self.planner_chosen_d_m = LANE_CENTER_D_M
        self.shield_verdict = safety_shield.ShieldVerdict(approved=True, risk_level=safety_shield.RISK_NONE)

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
        self.traffic = TrafficModel(total_length_m=total_length_m) if total_length_m > 0 else None
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

    def update(self, ai_decision: str, dt: Optional[float] = None):
        now = time.time()
        if dt is None:
            dt = min(now - self.last_update_time, 0.5)
        self.last_update_time = now
        if self.is_paused:
            return
        # Captured before any steering/heading integration so the realised yaw
        # rate (and hence lateral acceleration) can be measured at the end.
        heading_before_update = self.heading

        # Navigation
        #
        #  if a real road-following route is set, steer toward the
        # next waypoint in sequence (advancing once close enough) instead
        # of a straight bearing to the final destination -- this is what
        # makes the car actually follow real road geometry/turns rather
        # than cutting a straight line through buildings. `dist` (used
        # below for the cruise/braking speed curve) is always measured to
        # the FINAL destination, not the next waypoint, so the car doesn't
        # brake at every intermediate waypoint along the route.
        WAYPOINT_ARRIVAL_RADIUS = 15.0
        if self.route and self.route_index < len(self.route):
            # bug fix: advance along the route by PROJECTION (nearest
            # waypoint ahead), not by proximity to the current one.
            #
            # The previous logic advanced route_index only while within
            # WAYPOINT_ARRIVAL_RADIUS of the current target. That silently
            # depended on the car never overshooting -- true only because the
            # legacy point-mass controller could snap its heading instantly.
            # Under the physically-constrained bicycle model the car can and
            # does overshoot a tight corner, after which the old logic left the
            # index pinned and steered the car BACK to a waypoint behind it:
            # observed during early development as the car orbiting the
            # destination and never arriving (stuck at index 17 of 29).
            # Projection is also the natural precursor to the Frenet station
            # coordinate that the previous builds on.
            PROJECTION_SEARCH_WAYPOINTS = 40
            search_hi = min(len(self.route), self.route_index + PROJECTION_SEARCH_WAYPOINTS)
            nearest_idx, nearest_dist = self.route_index, float('inf')
            for i in range(self.route_index, search_hi):
                d = self.calculate_distance(self.lat, self.lng, self.route[i][0], self.route[i][1])
                if d < nearest_dist:
                    nearest_dist, nearest_idx = d, i
            # Monotonic: the search starts at the current index, so progress
            # along the route can never run backwards.
            self.route_index = nearest_idx

            # Steer toward a point a lookahead distance ahead of the projection
            # rather than at the nearest waypoint itself -- aiming directly at a
            # point a metre away produces violent steering demands. The lookahead
            # grows with speed. ( replaces this with a proper pure-pursuit
            # geometric law and a planned trajectory.)
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

        # Look ahead along the route for how sharp the nearest upcoming turn
        # is, so the car can brake for it in advance -- like a real driver,
        # not just steer through it at full cruise speed. Previously the
        # car held CRUISE_SPEED (50 km/h) through every corner regardless
        # of sharpness, which is why turns looked physically wrong (no
        # braking for corners at all). Uses the single sharpest bearing
        # change between consecutive route segments within the lookahead
        # window, not the cumulative turn, so a long gentle curve isn't
        # mistaken for one hard corner.
        CORNER_LOOKAHEAD_M = 60.0  # ~4s of braking distance at cruise speed -- enough for the speed lerp to actually slow down before reaching the corner
        CORNER_MAX_TURN_DEG = 90.0
        CORNER_MIN_SPEED = 15.0
        #  alongside the existing corner_turn_deg (still used by the
        # behavioural state machine), compute the path CURVATURE
        # kappa = d(heading)/d(arc length) [1/m]. Curvature is the physically
        # meaningful quantity: the comfort/grip limit on cornering speed is
        # v_max = sqrt(a_lat_max / kappa). That replaces the previous ad-hoc
        # linear interpolation between two hand-picked speeds.
        MIN_SEGMENT_FOR_CURVATURE_M = 1.0  # guard: OSRM emits near-duplicate points; a tiny
                                           # ds with any dtheta yields a spurious huge kappa
        corner_turn_deg = 0.0
        max_curvature = 0.0
        if self.route and len(self.route) > self.route_index + 2:
            accumulated_m = 0.0
            prev_bearing = None
            idx = self.route_index
            while idx < len(self.route) - 1 and accumulated_m < CORNER_LOOKAHEAD_M:
                p1, p2 = self.route[idx], self.route[idx + 1]
                seg_len = self.calculate_distance(p1[0], p1[1], p2[0], p2[1])
                accumulated_m += seg_len
                bearing = self.calculate_bearing(p1[0], p1[1], p2[0], p2[1])
                if prev_bearing is not None:
                    turn = abs((bearing - prev_bearing + 180) % 360 - 180)
                    corner_turn_deg = max(corner_turn_deg, turn)
                    if seg_len >= MIN_SEGMENT_FOR_CURVATURE_M:
                        max_curvature = max(max_curvature, math.radians(turn) / seg_len)
                prev_bearing = bearing
                idx += 1
        self.path_curvature = max_curvature

        #  exact Frenet projection of the ego's real position, when a
        # route (and therefore a Frenet frame) exists. This REPLACES the old
        # `station_distances[route_index]` approximation -- see the long
        # comment this used to carry, now obsolete: that approximation was
        # only ever accurate to within one inter-waypoint spacing because it
        # snapped to the nearest WAYPOINT rather than projecting onto the
        # route geometry itself. Computed once here (regardless of
        # controller) since it is strictly more accurate for BOTH the 
        # A/B legacy and bicycle conditions and does not touch either
        # controller's own speed/steering formulas -- it only feeds the
        # sensor/traffic station bookkeeping both already depended on.
        FRENET_SEARCH_BACKWARD_SLACK = 5  # segments of backward slack against overshoot
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

        # Advance traffic and refresh the forward range sensor from the
        # ego's own station_m and real-time lateral offset -- the same quantities
        # just computed above, so the sensor is always consistent with the position
        # actually being driven. Placed here BEFORE candidate planning and IDM speed
        # control so both see the real, current-tick lead vehicle rather than last-tick's state.
        #
        # Real bug fix: this used to sense against a HARDCODED lane position
        # (EGO_LANE_OFFSET_M, a constant 1.75m) instead of the ego's actual real-time
        # current_lateral_offset_m. The ego's real lateral position legitimately
        # fluctuates (cornering, and now real lane changes), so once it drifted toward
        # the far lane the sensor kept checking the near lane: it detected nothing
        # actually in front of the car, and detected phantom traffic in a lane the car
        # wasn't in. This is why the car could drive straight through an NPC that was
        # visually right in front of it -- IDM never saw it, because the sensor was
        # never told where the car actually was.
        if self.traffic is not None:
            self.traffic.update(dt, self.current_station_m)
            self.sensed_lead = self.traffic.sense_lead_vehicle(
                self.current_station_m, ego_lane_offset=self.current_lateral_offset_m
            )
        else:
            self.sensed_lead = None

        # Phase 6: the 360-degree surround perception layer (sensor_rig +
        # multi-class tracking + occupancy grid), separate from the forward
        # sensor above -- sense_lead_vehicle() is what IDM/the planner
        # actually consume for car-following and is left untouched; this is
        # additional situational awareness (blind-spot/rear traffic) for the
        # HMI, not yet wired into any driving decision.
        if self.traffic is not None and self.frenet_frame is not None:
            self.surround_tracks = self.surround_perception.step(
                self.frenet_frame, self.current_station_m, self.current_lateral_offset_m,
                self.traffic.npcs, dt,
            )
        else:
            self.surround_tracks = []

        if waypoint_dist > 5.0:
            target_heading = self.calculate_bearing(self.lat, self.lng, waypoint_lat, waypoint_lng)
            diff = (target_heading - self.heading + 180) % 360 - 180

            if self.controller == "legacy":
                # Pre-P6 point-mass steering: heading is nudged directly, with no
                # steering geometry. Retained as the A/B control for .
                turn_rate = max(0, min(1, self.speed_kmh / 20.0)) * 45.0 * dt
                if abs(diff) > turn_rate:
                    turn = turn_rate if diff > 0 else -turn_rate
                else:
                    turn = diff
                self.heading = (self.heading + turn + 360) % 360
                self.steering_angle = turn / max(0.01, turn_rate) if turn_rate > 0 else 0
            else:
                # kinematic bicycle model. The controller commands a front-wheel
                # ANGLE; heading is then a consequence of vehicle geometry and speed
                # (yaw_rate = v*tan(delta)/L) rather than being written directly.
                v_mps = self.speed_kmh / 3.6
                # Speed-dependent steering limit. The binding constraint on a real
                # vehicle is grip/comfort, not the steering rack: with
                #     a_lat = v^2 * tan(delta) / L  <=  A_LAT_MAX
                # the admissible wheel angle shrinks as speed rises,
                #     delta_max(v) = atan(A_LAT_MAX * L / v^2).
                # Without this, a 35 deg wheel angle at 50 km/h implies ~200 deg/s
                # of yaw and >20 m/s^2 lateral acceleration -- measured during
                # development, and worse than the legacy controller it
                # replaces. The geometric MAX_STEER_RAD still applies at parking
                # speeds, where it is the real limit.
                if v_mps > 0.5:
                    grip_steer_limit = math.atan(self.A_LAT_MAX_MPS2 * self.WHEELBASE_M / (v_mps ** 2))
                    steer_limit = min(self.MAX_STEER_RAD, grip_steer_limit)
                else:
                    steer_limit = self.MAX_STEER_RAD

                if self.frenet_frame is not None:
                    #  Frenet local planner + pure pursuit. This REPLACES
                    # the proportional heading controller below (which chased
                    # a raw route waypoint, with no notion of "lane") for any
                    # routed bicycle-controller drive. Candidates are lateral
                    # OFFSETS from the route centreline; the winner is
                    # followed geometrically, which is what fixes centreline
                    # driving -- the car now targets a LANE, not the road's
                    # raw polyline.
                    #
                    # Extracted verbatim into driver/lateral_planner.py
                    # (ADR-001 item 3). The lane-clear query stays here
                    # because it touches self.traffic; the planner is handed
                    # only the resulting bool, never the NPC list.
                    lead_gap_m = self.sensed_lead.gap_m if self.sensed_lead else None
                    adjacent_lane_clear = (
                        self.traffic.sense_lane_clear(self.current_station_m, ADJACENT_LANE_OFFSET_M)
                        if self.traffic is not None else False
                    )
                    lateral_plan = plan_lateral_offset(
                        current_lateral_offset_m=self.current_lateral_offset_m,
                        lead_gap_m=lead_gap_m,
                        adjacent_lane_clear=adjacent_lane_clear,
                        lateral_target_d_m=self.lateral_target_d_m,
                        frenet_frame=self.frenet_frame,
                        current_station_m=self.current_station_m,
                        ego_lat=self.lat,
                        ego_lng=self.lng,
                        heading_deg=self.heading,
                        v_mps=v_mps,
                        dt=dt,
                        steer_limit_rad=steer_limit,
                        lateral_target_rate_mps=self.LATERAL_TARGET_RATE_MPS,
                        pp_lookahead_k=self.PP_LOOKAHEAD_K,
                        pp_lookahead_min_m=self.PP_LOOKAHEAD_MIN_M,
                        wheelbase_m=self.WHEELBASE_M,
                    )
                    self.planner_candidates = lateral_plan.candidates
                    self.planner_chosen_d_m = lateral_plan.chosen_d_m
                    self.lateral_target_d_m = lateral_plan.lateral_target_d_m
                    desired_steer = lateral_plan.desired_steer_rad
                else:
                    # No route: fall back to the previous proportional heading
                    # controller chasing target_lat/target_lng directly (a
                    # Frenet frame needs a route to project onto).
                    desired_steer = max(-steer_limit,
                                        min(steer_limit, self.STEER_KP * math.radians(diff)))

                # Rate-limit the wheel: a real steering actuator cannot step instantly.
                max_step = self.STEER_RATE_MAX_RADPS * dt
                steer_error = desired_steer - self.steering_angle_rad
                self.steering_angle_rad += max(-max_step, min(max_step, steer_error))
                # Re-clamp after rate limiting: if the car accelerated, the previous
                # wheel angle may now exceed what is admissible at the new speed.
                self.steering_angle_rad = max(-steer_limit, min(steer_limit, self.steering_angle_rad))

                yaw_rate = v_mps * math.tan(self.steering_angle_rad) / self.WHEELBASE_M
                self.heading = (self.heading + math.degrees(yaw_rate * dt) + 360) % 360
                self.steering_angle = self.steering_angle_rad / self.MAX_STEER_RAD
        else:
            self.steering_angle = 0.0
            if self.controller != "legacy":
                # Ease the wheel back to centre rather than snapping it.
                max_step = self.STEER_RATE_MAX_RADPS * dt
                self.steering_angle_rad -= max(-max_step, min(max_step, self.steering_angle_rad))

        # State Machine Logic
        # Keyed off has_arrived rather than the raw distance so that a car which
        # coasts a few metres past the destination before stopping still reports
        # ARRIVED, instead of falling back to FINAL_APPROACH as though it were
        # still driving there.
        if self.has_arrived:
            self.driving_state = (DrivingState.ARRIVED if self.speed_kmh < 1.0
                                  else DrivingState.ARRIVING)
        elif dist < 40.0:
            self.driving_state = DrivingState.FINAL_APPROACH
        elif self.speed_kmh < 10.0 and dist > 40.0:
            self.driving_state = DrivingState.DEPARTING
        elif corner_turn_deg > 15.0 and waypoint_dist < CORNER_LOOKAHEAD_M:
            self.driving_state = DrivingState.APPROACHING_TURN
        elif abs(self.steering_angle) > 0.15:
            self.driving_state = DrivingState.TURNING
        else:
            self.driving_state = DrivingState.CRUISING
            
        if self.speed_kmh < 1.0 and dist > 10.0 and not self.route:
            self.driving_state = DrivingState.IDLE

        # Speed & Engine
        #
        # The car now has its own baseline cruise-toward-destination speed,
        # like a real adaptive-cruise/autopilot controller, instead of speed
        # being driven *solely* by the ML decision. Previously target_speed
        # was ONLY set by ai_decision ('Accelerate' -> +5, 'Decelerate' ->
        # -8, else -> hold current speed) -- since the classifier predicts
        # "Maintain Speed" from idle-looking telemetry (RPM~800, near-zero
        # deltas), a car starting at rest fed that exact idle reading back
        # every tick, so it could never bootstrap out of "Maintain Speed" ->
        # target_speed stays at the current speed (0) -> forever stationary.
        # This is why the dashboard never actually showed the car moving.
        # The ML decision still matters -- it now modulates the baseline
        # (Accelerate pushes above cruise speed, Decelerate pulls below it)
        # so it's visibly reflected in how fast the car speeds up/slows
        # down, without being the only thing that can start motion at all.
        CRUISE_SPEED = 50.0
        if dist < 10.0:
            self.has_arrived = True

        if self.has_arrived:
            # Arrived -- come to a stop and STAY stopped until a new
            # destination is set (a real "parking spot" selection UI is a
            # separate feature; this is the arrival/stop behaviour).
            base_target_speed = 0.0
        elif dist < 40.0:
            # Final approach: bleed speed off smoothly instead of cruising
            # at full speed right up to the destination and slamming to 0.
            base_target_speed = CRUISE_SPEED * ((dist - 10.0) / 30.0)
        else:
            base_target_speed = CRUISE_SPEED

        if self.has_arrived:
            # Arrival overrides the ML decision: an 'Accelerate' prediction must
            # not pull the stopped car away from its destination.
            target_speed = 0.0
        elif ai_decision == 'Accelerate':
            target_speed = min(base_target_speed + 15.0, 120.0)
        elif ai_decision == 'Decelerate':
            target_speed = max(base_target_speed - 20.0, 0.0)
        else:
            target_speed = base_target_speed

        # Cornering speed cap -- applied AFTER the AI decision's modulation,
        # as a hard physical constraint (grip/turning radius), not just
        # another input the AI's Accelerate call can override.
        self.speed_limit_reason = "cruise"
        if self.controller == "legacy":
            # Pre-P6 heuristic: linear interpolation between two hand-picked
            # speeds based on the sharpest turn angle in the lookahead window.
            # Retained as the A/B control for .
            if corner_turn_deg > 0:
                corner_factor = 1.0 - min(corner_turn_deg, CORNER_MAX_TURN_DEG) / CORNER_MAX_TURN_DEG
                corner_speed_cap = CORNER_MIN_SPEED + (CRUISE_SPEED - CORNER_MIN_SPEED) * corner_factor
                target_speed = min(target_speed, corner_speed_cap)
        else:
            #  derive the cornering limit from physics instead. Holding
            # lateral acceleration at or below A_LAT_MAX gives
            #     v_max = sqrt(a_lat_max / kappa)
            # so a gentle curve barely slows the car while a tight intersection
            # turn slows it a lot -- with no hand-tuned constants.
            if max_curvature > 1e-6:
                v_max_mps = math.sqrt(self.A_LAT_MAX_MPS2 / max_curvature)
                curve_cap_kmh = max(self.MIN_CORNER_SPEED_KMH, v_max_mps * 3.6)
                if curve_cap_kmh < target_speed:
                    target_speed = curve_cap_kmh
                    self.speed_limit_reason = "lateral_accel_limit"

            # Real bug fix: speed and steering were fully decoupled. Most
            # visible right after a stop/new destination, when the car's
            # current heading can be badly mismatched from the route's
            # actual initial direction -- the bicycle model correctly
            # cannot yaw while nearly stationary (yaw_rate = v*tan(delta)/L),
            # so as it accelerated hard toward cruise speed while still
            # pointed the wrong way, it travelled fast in the wrong
            # direction before steering authority caught up. Measured on
            # the real default SF route: lateral offset reached 26m (the
            # modelled road is only 7m wide) in the first ~12s before the
            # car reoriented and converged back to normal (1-3m) tracking
            # for the rest of the drive -- exactly the reported "car goes
            # off the road" behaviour, and specifically a start-of-drive
            # issue, not a persistent one. Fixed the same way the existing
            # curvature cap already works, just keyed off REALISED
            # steering demand (this tick's steering_angle_rad, computed
            # earlier in this same update()) instead of upcoming path
            # curvature -- a real driver naturally slows while correcting
            # a large heading error rather than flooring it.
            # Recomputed independently rather than reusing the earlier
            # steering block's `steer_limit` -- that variable is only ever
            # assigned when waypoint_dist > 5.0 this tick, so relying on it
            # here would risk a NameError on a tick where it wasn't set.
            v_mps_for_cap = self.speed_kmh / 3.6
            if v_mps_for_cap > 0.5:
                current_steer_limit = min(
                    self.MAX_STEER_RAD,
                    math.atan(self.A_LAT_MAX_MPS2 * self.WHEELBASE_M / (v_mps_for_cap ** 2)),
                )
            else:
                current_steer_limit = self.MAX_STEER_RAD

            if current_steer_limit > 1e-6:
                steer_fraction = abs(self.steering_angle_rad) / current_steer_limit
                if steer_fraction > self.TRACKING_ERROR_STEER_FRACTION_THRESHOLD:
                    severity = (steer_fraction - self.TRACKING_ERROR_STEER_FRACTION_THRESHOLD) / \
                               (1.0 - self.TRACKING_ERROR_STEER_FRACTION_THRESHOLD)
                    severity = min(1.0, severity)
                    # Squared, not linear: a large heading error (severity
                    # near 1) needs to cut speed hard and fast, not
                    # gradually -- linear falloff measured 19m+ excursions
                    # on a worst-case (180deg start-heading mismatch) route.
                    tracking_cap_kmh = max(self.MIN_CORNER_SPEED_KMH,
                                           CRUISE_SPEED * (1.0 - severity) ** 2)
                    if tracking_cap_kmh < target_speed:
                        target_speed = tracking_cap_kmh
                        self.speed_limit_reason = "tracking_correction"

        if target_speed < base_target_speed and self.speed_limit_reason == "cruise":
            self.speed_limit_reason = "ai_decelerate" if ai_decision == 'Decelerate' else "approach"

        if self.controller == "legacy":
            speed_diff = target_speed - self.speed_kmh
            self.speed_kmh += speed_diff * dt * 2.0
            self.speed_kmh = max(0.0, min(self.speed_kmh, 160.0))
        else:
            # jerk-limited longitudinal control. The previous model wrote
            # speed directly (speed += (target - speed) * dt * 2.0), which allows
            # unbounded acceleration and unbounded jerk -- the discontinuities
            # that read on screen as flickering/teleporting motion. Here the
            # controller commands an ACCELERATION, that acceleration is bounded,
            # and its rate of change (jerk) is bounded too, so velocity is C1
            # continuous.
            v_mps = self.speed_kmh / 3.6
            target_mps = target_speed / 3.6
            desired_accel = self.SPEED_KP * (target_mps - v_mps)

            # IDM car-following: this REPLACES the previous behaviour where
            # traffic.py's sensed_lead was computed every tick, exposed to
            # the lateral planner (P6-2), and then never touched
            # longitudinal control at all -- the ego never actually slowed
            # for a real gap, regardless of how close it got. Standard IDM
            # composition: take the more conservative (smaller) of "what the
            # cruise/AI-decision controller wants" and "what's required to
            # not run into the sensed lead vehicle" (Treiber, Hennecke &
            # Helbing, 2000). Legacy is untouched -- it never reads
            # sensed_lead at all, exactly like before.
            if self.sensed_lead is not None:
                idm_accel = idm_acceleration(
                    v_mps=v_mps,
                    v0_mps=target_mps,
                    gap_m=self.sensed_lead.gap_m,
                    lead_speed_mps=self.sensed_lead.lead_speed_kmh / 3.6,
                    a_max_mps2=self.A_MAX_ACCEL_MPS2,
                )
                if idm_accel is not None:
                    desired_accel = min(desired_accel, idm_accel)
                    if desired_accel < 0 and self.speed_limit_reason == "cruise":
                        self.speed_limit_reason = "car_following"

            # Safety Shield: an INDEPENDENT check of the ego's actual
            # physical state, run AFTER the planner/IDM have already
            # decided -- not more logic folded into their own cost
            # functions. self.lateral_accel_mps2 here is last tick's
            # value (this tick's isn't computed until after movement,
            # below) -- a deliberate one-tick lag, the same acceptable
            # staleness pattern already used elsewhere in this method for
            # jerk/curvature-limited quantities that can't change
            # violently tick to tick. See safety_shield.py's module
            # docstring for why this is a separate module, not more
            # planner logic: it re-derives risk from raw physical
            # quantities instead of trusting the planner's own bookkeeping.
            self.shield_verdict = safety_shield.evaluate(
                ego_speed_mps=v_mps,
                lateral_offset_m=self.current_lateral_offset_m,
                lateral_accel_mps2=self.lateral_accel_mps2,
                sensed_lead_gap_m=self.sensed_lead.gap_m if self.sensed_lead else None,
                sensed_lead_speed_mps=(self.sensed_lead.lead_speed_kmh / 3.6) if self.sensed_lead else None,
            )
            if self.shield_verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE:
                # min() composition, same principle as IDM above: the
                # shield can only ever make the car brake HARDER than
                # already planned, never accelerate harder -- it forces
                # maximum physical braking regardless of what the
                # cruise/IDM composition above computed. Appropriate here
                # because this path is only reached for an imminent
                # collision (TTC critical) -- stopping IS the right call.
                desired_accel = min(desired_accel, -self.A_MAX_BRAKE_MPS2)
                self.speed_limit_reason = "safety_shield_override"
            elif self.shield_verdict.override_action == safety_shield.OVERRIDE_RECOVER_LOW_SPEED:
                # Deliberately NOT a full stop -- braking all the way to
                # zero here (road-boundary/hard-lateral-accel violations)
                # would remove the only thing that lets the car steer back
                # under control (yaw_rate = v*tan(delta)/L needs forward
                # speed): a real livelock found live, where the car froze
                # off-road, permanently re-triggering the same override
                # forever. Proportional control toward a low but nonzero
                # recovery floor instead, same min() composition principle.
                recovery_target_mps = self.MIN_CORNER_SPEED_KMH / 3.6
                recovery_accel = self.SPEED_KP * (recovery_target_mps - v_mps)
                desired_accel = min(desired_accel, recovery_accel)
                self.speed_limit_reason = "safety_shield_override"

            desired_accel = max(-self.A_MAX_BRAKE_MPS2,
                                min(self.A_MAX_ACCEL_MPS2, desired_accel))

            max_accel_step = self.JERK_MAX_MPS3 * dt
            accel_error = desired_accel - self.acceleration_mps2
            self.acceleration_mps2 += max(-max_accel_step, min(max_accel_step, accel_error))

            v_mps += self.acceleration_mps2 * dt
            if v_mps <= 0.0:
                # Stopped: clear any wound-up braking demand so pulling away
                # again starts from zero acceleration rather than a negative one.
                v_mps = 0.0
                if self.acceleration_mps2 < 0.0:
                    self.acceleration_mps2 = 0.0
            self.speed_kmh = min(v_mps * 3.6, 160.0)

        # Diagnostic: the lateral acceleration ACTUALLY being experienced right
        # now, a_lat = v * yaw_rate, derived from the realised heading change.
        # Deliberately NOT v^2 * max_curvature: max_curvature is the sharpest
        # curvature anywhere in the 60 m LOOKAHEAD, so that form spikes as soon
        # as a corner comes into view while the car is still on straight road,
        # which misreports comfort. Computing it from the realised yaw rate also
        # makes the metric controller-agnostic, so the previous A/B compares like
        # with like.
        heading_delta_deg = (self.heading - heading_before_update + 180) % 360 - 180
        yaw_rate_radps = math.radians(heading_delta_deg) / dt if dt > 0 else 0.0
        self.lateral_accel_mps2 = abs((self.speed_kmh / 3.6) * yaw_rate_radps)
        
        # Powertrain / emissions relaxation-integration -- extracted verbatim
        # into world/vehicle_dynamics.py (ADR-001 item 2). Same arithmetic,
        # same RNG call order (idle-RPM jitter then altitude drift).
        pt = step_powertrain(
            speed_kmh=self.speed_kmh,
            ai_decision=ai_decision,
            rpm=self.rpm,
            coolant_temp=self.coolant_temp,
            fuel_rate=self.fuel_rate,
            co2=self.co2,
            altitude=self.altitude,
            dt=dt,
        )
        self.rpm = pt.rpm
        self.coolant_temp = pt.coolant_temp
        self.fuel_rate = pt.fuel_rate
        self.co2 = pt.co2
        self.altitude = pt.altitude

        # Movement -- great-circle displacement, also extracted verbatim.
        self.lat, self.lng = advance_position(
            lat=self.lat,
            lng=self.lng,
            heading_deg=self.heading,
            speed_kmh=self.speed_kmh,
            dt=dt,
        )

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
            # control/diagnostic state. Surfaced for the HMI  and
            # for the A/B evaluation in .
            "controller": self.controller,
            "acceleration": self.acceleration_mps2,
            "steering_angle_rad": self.steering_angle_rad,
            "path_curvature": self.path_curvature,
            "lateral_accel": self.lateral_accel_mps2,
            "speed_limit_reason": self.speed_limit_reason,
            #  forward range-sensor output (gap + relative speed to the
            # nearest same-lane vehicle ahead), or null if nothing is sensed.
            # This is DELIBERATELY the only NPC-related information available
            # through the normal navigation state -- see traffic.py.
            "sensed_lead_gap_m": self.sensed_lead.gap_m if self.sensed_lead else None,
            "sensed_lead_speed_kmh": self.sensed_lead.lead_speed_kmh if self.sensed_lead else None,
            #  exact Frenet lateral offset (signed, metres, positive =
            # right/same-direction lane) and the planner's current tracked
            # target -- the frontend's hard-coded LANE_OFFSET_M render hack
            # is replaced by rendering AT this real value, so the HMI shows
            # the backend's actual planned lateral position, not an assumed
            # constant.
            "lateral_offset_m": self.current_lateral_offset_m,
            "lateral_target_m": self.lateral_target_d_m,
            "is_paused": self.is_paused,
            "active_scenario": self.active_scenario,
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
        self.current_station_m = station_m
        self.current_lateral_offset_m = lateral_offset_m
        self.lateral_target_d_m = lateral_offset_m
        self.speed_kmh = speed_kmh
        self.acceleration_mps2 = 0.0
        self.steering_angle = 0.0
        self.steering_angle_rad = 0.0
        self.has_arrived = False
        self.frenet_search_idx = 0
        self.route_index = 0
        self.speed_limit_reason = "cruise"
        self.shield_verdict = safety_shield.ShieldVerdict(approved=True, risk_level=safety_shield.RISK_NONE)
        # A scenario reset should not leave stale track IDs/coasted history
        # from before the reset -- fresh engine, same as every other piece
        # of per-session state this method resets.
        self.surround_perception = SurroundPerceptionEngine()
        self.surround_tracks = []

        if self.frenet_frame is not None:
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
                "is_chosen": c.d_target == self.planner_chosen_d_m,
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
        }

    def get_npc_states(self):
        """Full NPC state for the HMI renderer  -- a rendering
        concern, distinct from what the ego's own sensor may perceive for
        control purposes (get_navigation_state()'s sensed_lead_* fields)."""
        return self.traffic.get_npc_states() if self.traffic is not None else []

    def get_surround_perception_state(self):
        """Confirmed surround tracks from Phase 6's 360-degree perception
        layer -- distinct from get_npc_states() (the full simulated-truth
        NPC list) and from sensed_lead (the forward-only IDM sensor); this
        is what the ego's own multi-sensor rig + tracker actually confirms,
        including detections behind/beside the ego the forward sensor never
        sees."""
        return self.surround_perception.get_state()
