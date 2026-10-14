"""
Scenario Engine: Deterministic, repeatable scenario execution platform for DDS V2.

Allows selecting and running curated driving scenarios:
1. Normal Cruising: Open road with light traffic, testing speed maintenance and lane-centring.
2. Traffic Overtake: Slow lead vehicle encountered; adjacent lane clear; triggers candidate-path lane-change selection.
3. Emergency Cut-in & Brake: Sudden intrusion / hard braking violating TTC; triggers Safety Shield EMERGENCY_BRAKE override.
4. Queue Stop-and-Go: Dense traffic queue ahead; tests IDM car-following standstill and smooth resumption.

All scenarios use fixed seeds and reproducible initial configurations.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Any
import math
import uuid
from datetime import datetime, timezone

from app.services.traffic import (
    TrafficModel,
    NpcVehicle,
    EGO_LANE_OFFSET_M,
    ADJACENT_LANE_OFFSET_M,
    LANE_OFFSETS,
    DENSITY_NPC_COUNTS,
)
from app.services.physics_engine import PhysicsEngine
from app.services.road_routes import real_road_route
from app.services.safety_shield import (
    RISK_CRITICAL,
    OVERRIDE_EMERGENCY_BRAKE,
    TTC_CRITICAL_S,
)


@dataclass
class ScenarioDefinition:
    id: str
    name: str
    category: str  # "normal" | "traffic" | "maneuver" | "safety_critical"
    description: str
    seed: int
    default_initial_speed_kmh: float
    default_density: str  # "low" | "medium" | "high"
    setup_fn: Callable[["ScenarioEngine", PhysicsEngine, str, Optional[float]], None]
    tick_fn: Optional[Callable[["ScenarioEngine", PhysicsEngine, int, float], Optional[dict]]] = None


def configure_driving_environment_traffic(physics, density, traffic_end_m=None):
    """Install ordinary finite background actors for this recorded route.

    Destination edits call this with the full new route extent. Parking setup
    passes its verified approach boundary. Neither call scripts ego decisions.
    """
    from app.services.road_semantics import recorded_map
    from app.services.frenet import frenet_to_latlng, frenet_to_local_xz
    frame = physics.frenet_frame
    if frame is None or density not in DENSITY_NPC_COUNTS:
        raise ValueError("Finite traffic requires a route and supported density")
    end = frame.total_length_m if traffic_end_m is None else traffic_end_m
    if not math.isfinite(end) or not 0 < end <= frame.total_length_m:
        raise ValueError("Traffic extent must be within the recorded route")
    traffic = TrafficModel(total_length_m=end, seed=physics._seed, density=density,
                           finite_world=True, world_controls=physics.route_controls)
    # Retain this route value, never a mutable physics/session pointer.
    traffic_frame = physics.frenet_frame
    def mapped_limit(station):
        lat, lng = frenet_to_latlng(traffic_frame, station, EGO_LANE_OFFSET_M)
        _, _, dx, dz = frenet_to_local_xz(traffic_frame, station, EGO_LANE_OFFSET_M)
        rule = recorded_map().match(lat, lng, math.degrees(math.atan2(dx, -dz)))
        return rule.speed_limit_kmh if rule is not None else None
    traffic.speed_limit_at_station = mapped_limit
    # Uniform whole-route spawning can leave a long route empty locally.
    # Seed up to four ordinary actors along the initial 40–130 m approach;
    # distribute the remainder over the route, never recycling/teleporting them.
    count = len(traffic.npcs)
    near_count = min(4, count)
    if end >= 160.:
        stations = [40. + i * 30. for i in range(near_count)]
        stations += [160. + (end - 160.) * (i + 1) / (count - near_count + 1)
                     for i in range(count - near_count)]
    else:
        # Short routes cannot support arbitrary density with safe spacing.
        count = min(count, max(0, int((end - 15.) / 12.)))
        traffic.npcs = traffic.npcs[:count]
        stations = [15. + 12. * i for i in range(count)]
    for npc, station in zip(traffic.npcs, stations):
        npc.station_m = station
        # Lane centre is a declared simulator assumption. Outer +/-5.25 m
        # lanes and oncoming actors cannot be justified on every access road.
        npc.lane_offset = npc.base_lane_offset = EGO_LANE_OFFSET_M
        npc.wander_amplitude_m = 0.
        npc.speed_kmh = abs(npc.speed_kmh)
        npc.desired_speed_kmh = abs(npc.desired_speed_kmh)
        lat, lng = frenet_to_latlng(physics.frenet_frame, npc.station_m, npc.lane_offset)
        _, _, dx, dz = frenet_to_local_xz(physics.frenet_frame, npc.station_m, npc.lane_offset)
        rule = recorded_map().match(lat, lng, math.degrees(math.atan2(dx, -dz)))
        if rule is not None and rule.speed_limit_kmh is not None:
            npc.speed_kmh = min(npc.speed_kmh, rule.speed_limit_kmh)
            npc.desired_speed_kmh = min(npc.desired_speed_kmh, rule.speed_limit_kmh)
    physics.traffic = traffic
    return traffic


class ScenarioEngine:
    def __init__(self):
        self.scenarios: Dict[str, ScenarioDefinition] = {}
        self.active_scenario_id: Optional[str] = None
        self.active_density: str = "medium"
        self.active_initial_speed_kmh: float = 45.0
        self.tick_count: int = 0
        self.elapsed_time_s: float = 0.0
        self.is_paused: bool = False
        self.events_log: List[dict] = []
        self._scenario_data: Dict[str, Any] = {}
        self.active_params: Dict[str, Any] = {}

        self._register_built_in_scenarios()

    def _register_built_in_scenarios(self):
        self.register_scenario(ScenarioDefinition(
            id="driving_environment", name="Real-road driving environment",
            category="normal",
            description="Recorded highway, streets and parking mission with seeded simulated traffic, sensors and a 3D scene camera. Driving decisions belong to the intelligent controller; world traffic uses a modeled lane, not surveyed lane geometry.",
            seed=711, default_initial_speed_kmh=0., default_density="medium",
            setup_fn=self._setup_driving_environment, tick_fn=None))
        self.register_scenario(ScenarioDefinition(
            id="recorded_signal", name="East Millbrae · Simulated Signal",
            category="safety_critical", description="Recorded mapped control; signal cycle belongs to the simulated world. Supports synthetic camera mode.",
            seed=810, default_initial_speed_kmh=18., default_density="low",
            setup_fn=self._setup_recorded_signal))
        self.register_scenario(ScenarioDefinition(
            id="mapped_parking", name="Mapped Road to Parking",
            category="maneuver",
            description="Recorded US 101 exit, Broadway and Old Bayshore Highway to a mapped parking area. Simulated signals and sensors; planner-selected unmarked parking pose.",
            seed=707, default_initial_speed_kmh=0., default_density="low",
            setup_fn=self._setup_mapped_parking, tick_fn=None))
        self.register_scenario(ScenarioDefinition(
            id="recorded_stop", name="Ray Drive · Recorded Stop",
            category="maneuver",
            description="Recorded Ray Drive and OSM stop control. Simulated vehicle/sensors; stopping position is a modeled offset, not a surveyed stop line.",
            seed=808, default_initial_speed_kmh=18., default_density="low",
            setup_fn=self._setup_recorded_stop))
        self.register_scenario(ScenarioDefinition(
            id="recorded_yield_conflict", name="Recorded Yield · Simulated Conflict",
            category="safety_critical",
            description="Recorded OSM road/yield control with an explicitly simulated stationary conflict vehicle. Conflict clears at 20 simulated seconds.",
            seed=809, default_initial_speed_kmh=18., default_density="low",
            setup_fn=self._setup_recorded_yield_conflict,
            tick_fn=self._tick_recorded_yield_conflict))
        # 1. Normal Cruising
        self.register_scenario(
            ScenarioDefinition(
                id="normal_cruising",
                name="Normal Open-Road Cruising",
                category="normal",
                description="Open-road driving with free-flowing traffic. Demonstrates smooth lane-centring, cruise speed tracking, and high efficiency.",
                seed=42,
                default_initial_speed_kmh=45.0,
                default_density="low",
                setup_fn=self._setup_normal_cruising,
                tick_fn=self._tick_normal_cruising,
            )
        )

        # 2. Traffic Overtake (Lane Change)
        self.register_scenario(
            ScenarioDefinition(
                id="traffic_overtake",
                name="Slow Lead Overtake (Lane Change)",
                category="maneuver",
                description="Approaching a slow lead vehicle (22 km/h). The Frenet candidate planner detects blocked lane and chooses adjacent lane with highlighted trajectory.",
                seed=101,
                default_initial_speed_kmh=42.0,
                default_density="medium",
                setup_fn=self._setup_traffic_overtake,
                tick_fn=self._tick_traffic_overtake,
            )
        )

        # 3. Emergency Cut-in & Brake (Safety Shield)
        self.register_scenario(
            ScenarioDefinition(
                id="emergency_cut_in",
                name="Emergency Cut-in & Brake",
                category="safety_critical",
                description="Lead vehicle aggressively cuts in and decelerates hard, dropping TTC < 2.0s. The independent Safety Shield engages EMERGENCY_BRAKE override.",
                seed=202,
                default_initial_speed_kmh=48.0,
                default_density="medium",
                setup_fn=self._setup_emergency_cut_in,
                tick_fn=self._tick_emergency_cut_in,
            )
        )

        # 4. Queue Stop-and-Go
        self.register_scenario(
            ScenarioDefinition(
                id="queue_stop_and_go",
                name="Congested Queue (Stop & Go)",
                category="traffic",
                description="A queue of stationary vehicles ahead. Tests IDM car-following deceleration to standstill, safe gap hold, and resumption.",
                seed=303,
                default_initial_speed_kmh=35.0,
                default_density="high",
                setup_fn=self._setup_queue_stop_and_go,
                tick_fn=self._tick_queue_stop_and_go,
            )
        )

        # 5. Erratic Cut-in (Dynamic Safety-Critical)
        self.register_scenario(
            ScenarioDefinition(
                id="erratic_cut_in",
                name="Erratic High-Speed Cut-in",
                category="safety_critical",
                description="Aggressive adjacent vehicle performs an abrupt, high-lateral-velocity cut-in with heavy deceleration, challenging proactive prediction and Safety Shield emergency braking.",
                seed=404,
                default_initial_speed_kmh=50.0,
                default_density="medium",
                setup_fn=self._setup_erratic_cut_in,
                tick_fn=self._tick_erratic_cut_in,
            )
        )

        # 6. Highway Merge & Interleaving
        self.register_scenario(
            ScenarioDefinition(
                id="highway_merge",
                name="Highway Merge & Interleaving",
                category="maneuver",
                description="On-ramp merge vehicle seeks gap acceptance; ego smoothly adapts speed via IDM car-following without triggering safety alarms.",
                seed=505,
                default_initial_speed_kmh=52.0,
                default_density="medium",
                setup_fn=self._setup_highway_merge,
                tick_fn=self._tick_highway_merge,
            )
        )

        # 7. Inattentive Lane Straddler
        self.register_scenario(
            ScenarioDefinition(
                id="lane_straddler",
                name="Inattentive Lane Straddler",
                category="traffic",
                description="Lead vehicle straddles the lane line with periodic lateral wandering, testing Frenet candidate generation and defensive spacing.",
                seed=606,
                default_initial_speed_kmh=42.0,
                default_density="low",
                setup_fn=self._setup_lane_straddler,
                tick_fn=self._tick_lane_straddler,
            )
        )

        # 8. Low-Friction Wet Curve
        self.register_scenario(
            ScenarioDefinition(
                id="low_friction_curve",
                name="Low-Friction Wet Curve",
                category="safety_critical",
                description="Wet road surface (mu = 0.5) degrades tire adhesion on a curved section, forcing dynamic lateral acceleration de-rating and proactive speed reduction.",
                seed=707,
                default_initial_speed_kmh=60.0,
                default_density="low",
                setup_fn=self._setup_low_friction_curve,
                tick_fn=self._tick_low_friction_curve,
            )
        )

    def register_scenario(self, scenario: ScenarioDefinition):
        self.scenarios[scenario.id] = scenario

    def list_scenarios(self) -> List[dict]:
        return [
            {
                "id": s.id,
                "name": s.name,
                "category": s.category,
                "description": s.description,
                "seed": s.seed,
                "default_initial_speed_kmh": s.default_initial_speed_kmh,
                "default_density": s.default_density,
            }
            for s in self.scenarios.values()
        ]

    def get_active_scenario(self) -> Optional[ScenarioDefinition]:
        if not self.active_scenario_id:
            return None
        return self.scenarios.get(self.active_scenario_id)

    def get_state(self) -> dict:
        active = self.get_active_scenario()
        return {
            "id": self.active_scenario_id,
            "name": active.name if active else "Free Drive",
            "category": active.category if active else "normal",
            "description": active.description if active else "Standard unscripted driving",
            "is_paused": self.is_paused,
            "tick": self.tick_count,
            "elapsed_s": round(self.elapsed_time_s, 2),
            "density": self.active_density,
            "initial_speed_kmh": self.active_initial_speed_kmh,
            "status": self._scenario_data.get("status", "running" if active else "idle"),
            "milestone": self._scenario_data.get("milestone"),
            "params": getattr(self, "active_params", {}),
            "route_name": self._scenario_data.get("route_name"),
            "route_way_id": self._scenario_data.get("route_way_id"),
            "recorded_control_node_id": self._scenario_data.get("recorded_control_node_id"),
            "conflict_source": self._scenario_data.get("conflict_source"),
            "environment": self._scenario_data.get("environment"),
        }

    def load_scenario(
        self,
        scenario_id: str,
        physics: PhysicsEngine,
        density: Optional[str] = None,
        initial_speed_kmh: Optional[float] = None,
        params: Optional[Dict[str, Any]] = None,
    ) -> dict:
        if scenario_id not in self.scenarios:
            raise ValueError(f"Unknown scenario ID: '{scenario_id}'. Available: {list(self.scenarios.keys())}")

        scenario = self.scenarios[scenario_id]
        speed = initial_speed_kmh if initial_speed_kmh is not None else scenario.default_initial_speed_kmh
        if not math.isfinite(speed) or not 0 <= speed <= 160:
            raise ValueError("Initial speed must be finite and between 0 and 160 km/h")
        if density is not None and density not in DENSITY_NPC_COUNTS:
            raise ValueError("Unknown traffic density")
        # Reject malformed inputs before touching scenario or world state.
        if params is not None and not isinstance(params, dict):
            raise ValueError("Scenario params must be an object")
        params = dict(params or {})
        allowed = {"rain_enabled", "scene_camera_controls", "camera_controls", "camera_occluded", "camera_dropout", "sensor_noise_enabled", "sensor_dropout_rate", "friction_mu",
                   "position_sigma_m", "position_sigma_per_100m", "velocity_sigma_mps",
                   "dropout_prob", "occlusion_enabled", "occlusion_half_angle_deg",
                   "latency_ticks", "false_positive_rate", "sensor_outage_start_s", "sensor_outage_duration_s"}
        if params.keys() - allowed:
            raise ValueError("Unknown scenario parameter")
        for key, value in params.items():
            if key in ("rain_enabled", "sensor_noise_enabled", "occlusion_enabled", "scene_camera_controls", "camera_controls", "camera_occluded", "camera_dropout"):
                if not isinstance(value, bool):
                    raise ValueError(f"{key} must be boolean")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key} must be a finite number")
            upper = {"sensor_dropout_rate": 1, "dropout_prob": 1, "friction_mu": 1,
                     "occlusion_half_angle_deg": 180, "latency_ticks": 1000}.get(key, 1000)
            lower = 0.1 if key == "friction_mu" else 0
            if not lower <= value <= upper:
                raise ValueError(f"{key} outside supported range [{lower}, {upper}]")
            if key == "latency_ticks" and not isinstance(value, int):
                raise ValueError("latency_ticks must be an integer")
        if scenario_id == "driving_environment":
            if speed != 0.:
                raise ValueError("Driving environment must start stationary")
            # Explicit advanced CPU/map mode overrides the scene-camera default.
            # Persist normalized defaults so reset reproduces the same boundary.
            if "scene_camera_controls" not in params:
                params["scene_camera_controls"] = "camera_controls" not in params
        physics._seed = scenario.seed
        self.active_scenario_id = scenario_id
        self.active_density = density or scenario.default_density
        self.active_initial_speed_kmh = (
            initial_speed_kmh if initial_speed_kmh is not None else scenario.default_initial_speed_kmh
        )
        self.active_params = params or {}
        self.tick_count = 0
        self.elapsed_time_s = 0.0
        self.is_paused = False
        self.events_log.clear()
        self._scenario_data = {
            "status": "active",
            "milestone": "Scenario Initialized",
            "flags": {},
        }

        # Every built-in scenario restarts on recorded road geometry, never on
        # a generated arc or the preceding scenario's route.
        road = real_road_route("wet_curve" if scenario_id == "low_friction_curve" else "highway")
        physics.set_destination(*road[-1])
        physics.set_route(road)
        # Apply setup function
        scenario.setup_fn(self, physics, self.active_density, self.active_initial_speed_kmh)

        # Apply runtime parameters if provided
        if "sensor_noise_enabled" in self.active_params and physics.traffic:
            physics.traffic.sensor_noise_enabled = bool(self.active_params["sensor_noise_enabled"])
        if "sensor_dropout_rate" in self.active_params and physics.traffic:
            physics.traffic.sensor_dropout_rate = float(self.active_params["sensor_dropout_rate"])
        if "friction_mu" in self.active_params:
            physics.friction_mu = max(0.1, min(1.0, float(self.active_params["friction_mu"])))
        _pn = {k: v for k, v in self.active_params.items() if k in (
            "position_sigma_m", "position_sigma_per_100m", "velocity_sigma_mps",
            "dropout_prob", "occlusion_enabled", "occlusion_half_angle_deg",
            "latency_ticks", "false_positive_rate",
        )}
        if _pn:
            from app.services.perception.perception_engine import PerceptionNoiseConfig
            physics.set_perception_noise(PerceptionNoiseConfig(**_pn))

        physics.scene_camera_controls = self.active_params.get("scene_camera_controls", False)
        physics.camera_controls = physics.scene_camera_controls or self.active_params.get("camera_controls", False)
        physics.camera_required_for_drive = scenario_id == "driving_environment" and physics.scene_camera_controls
        physics.camera_occluded = self.active_params.get("camera_occluded", False)
        physics.camera_dropout = self.active_params.get("camera_dropout", False)
        physics.sensor_outage_start_s = self.active_params.get("sensor_outage_start_s")
        physics.sensor_outage_duration_s = self.active_params.get("sensor_outage_duration_s", 0.)

        # Notify physics engine of active scenario
        physics.is_paused = self.is_paused
        physics.active_scenario = self.get_state()

        event = self._create_event(
            event_type="SCENARIO_LOADED",
            cause=f"Scenario '{scenario.name}' loaded",
            actor="SYSTEM",
            decision=f"Initial speed {self.active_initial_speed_kmh} km/h, density {self.active_density}",
            metadata={"scenario_id": scenario_id, "seed": scenario.seed, "params": self.active_params},
        )
        self.events_log.append(event)
        return event

    def reset(self, physics: PhysicsEngine) -> dict:
        """Reset current scenario back to tick 0 with exact seed and initial conditions."""
        if not self.active_scenario_id:
            physics.reset_state(station_m=0.0, speed_kmh=0.0, lateral_offset_m=EGO_LANE_OFFSET_M)
            return {"status": "reset", "scenario": None}

        return self.load_scenario(
            self.active_scenario_id,
            physics,
            density=self.active_density,
            initial_speed_kmh=self.active_initial_speed_kmh,
            params=getattr(self, "active_params", None),
        )

    def pause(self, physics: PhysicsEngine) -> None:
        self.is_paused = True
        physics.is_paused = True
        if physics.active_scenario:
            physics.active_scenario["is_paused"] = True

    def resume(self, physics: PhysicsEngine) -> None:
        self.is_paused = False
        physics.is_paused = False
        if physics.active_scenario:
            physics.active_scenario["is_paused"] = False

    def update(self, physics: PhysicsEngine, dt: float, *, clock=None) -> Optional[dict]:
        """Called once per simulation tick (e.g. 10 Hz) when running."""
        if self.is_paused or not self.active_scenario_id:
            return None

        self.tick_count += 1
        self.elapsed_time_s = clock.sim_time_s if clock is not None else self.elapsed_time_s + dt

        scenario = self.get_active_scenario()
        event = None
        if scenario and scenario.tick_fn:
            event = scenario.tick_fn(self, physics, self.tick_count, self.elapsed_time_s)
            if event:
                self.events_log.append(event)

        # Sync active scenario state to physics engine
        physics.active_scenario = self.get_state()
        return event

    def _create_event(
        self,
        event_type: str,
        cause: str,
        actor: str = "EGO",
        decision: str = "NONE",
        confidence: float = 1.0,
        metadata: Optional[dict] = None,
    ) -> dict:
        return {
            "type": "event",
            "event": {
                "event_id": str(uuid.uuid4()),
                "simulation_id": "sim_active",
                "run_id": self.active_scenario_id or "free_drive",
                "tick": self.tick_count,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "type": event_type,
                "actor": actor,
                "cause": cause,
                "decision": decision,
                "confidence": confidence,
                "metadata": metadata or {},
            },
        }

    # =========================================================================
    # Scenario 1: Normal Cruising
    # =========================================================================
    def _setup_recorded_intersection(self, physics, way_id, node_id, initial_speed_kmh, conflict):
        from app.services.road_semantics import recorded_map
        way = next(w for w in recorded_map().features if w["id"] == way_id)
        road = [tuple(point) for point in way["geometry"]]
        physics.set_destination(*road[-1])
        physics.set_route(road)
        physics.reset_state(station_m=0., speed_kmh=initial_speed_kmh,
                            lateral_offset_m=EGO_LANE_OFFSET_M)
        control = next(c for c in physics.route_controls if c.node_id == node_id)
        traffic = TrafficModel(total_length_m=physics.frenet_frame.total_length_m,
                               seed=physics._seed, density="low")
        actors = [NpcVehicle("fixture-yield-conflict", -3.5, 0., control.station_m,
                             desired_speed_kmh=0., prevent_recycle=True)] if conflict else []
        traffic.spawn_scripted_npcs(actors)
        physics.traffic = traffic
        self._scenario_data.update(route_name=way["tags"].get("name"),
            route_way_id=way_id, recorded_control_node_id=node_id,
            conflict_source="explicit_simulated_stationary_actor" if conflict else None)

    def _setup_recorded_signal(self, engine, physics, density, initial_speed_kmh):
        self._setup_recorded_intersection(physics, 396939375, 6386743913,
                                         initial_speed_kmh, False)

    def _setup_recorded_stop(self, engine, physics, density, initial_speed_kmh):
        self._setup_recorded_intersection(physics, 130716952, 65424265,
                                         initial_speed_kmh, False)

    def _setup_recorded_yield_conflict(self, engine, physics, density, initial_speed_kmh):
        self._setup_recorded_intersection(physics, 370279592, 6383816258,
                                         initial_speed_kmh, True)

    def _tick_recorded_yield_conflict(self, engine, physics, tick, sim_time_s):
        flags = self._scenario_data.setdefault("flags", {})
        if sim_time_s >= 20. and not flags.get("conflict_cleared"):
            physics.traffic.npcs = [n for n in physics.traffic.npcs if n.id != "fixture-yield-conflict"]
            flags["conflict_cleared"] = True
            self._scenario_data["milestone"] = "Simulated conflict cleared"
            return self._create_event(event_type="SIMULATED_YIELD_CONFLICT_CLEARED",
                actor="TRAFFIC", cause="Explicit fixture schedule reached 20 simulated seconds",
                decision="Remove simulated stationary conflict actor",
                metadata={"simulated_time_s": sim_time_s,
                          "source": "explicit_simulated_stationary_actor"})
        return None

    def _setup_driving_environment(self, engine, physics, density, initial_speed_kmh):
        from app.services.parking import parking_plan, install_parking
        plan = parking_plan()
        physics.set_destination(*plan.waypoints[-1])
        physics.set_route(plan.waypoints)
        install_parking(physics, plan)
        physics.reset_state(station_m=0., speed_kmh=0., lateral_offset_m=EGO_LANE_OFFSET_M)
        # World extent excludes the dedicated parking approach and maneuver.
        # Ordinary seeded traffic keeps IDM, with finite exits; no tick scripts,
        # ego-speed buffers, forced hazards, or timed clearing are installed.
        traffic_end = physics.parking_mission['approach_start_m']
        traffic = configure_driving_environment_traffic(physics, density, traffic_end)
        self._scenario_data.update(route_name="Recorded highway to mapped parking",
            environment={
                "map_source": "recorded_OSM_geometry", "vehicle_source": "simulated_vehicle",
                "traffic_source": "seeded_TrafficModel_IDM", "sensor_source": "simulated_sensors",
                "camera_source": "rendered_3D_scene_RGB_default", "scripted_tick_events": False,
                "lane_source": "single_same_direction_modeled_lane_not_surveyed",
                "traffic_extent_end_m": traffic_end, "npc_count": len(traffic.npcs),
                "npc_limitations": "Simplified world signal/STOP response and mapped speed target; no intersection conflict negotiation, surveyed lanes, or curvature-aware NPC steering.",
                "traffic_lifecycle": "finite_route_no_wrap_or_recycling",
                "weather_source": "friction_parameter_only_no_weather_sensor",
                "parking_pose_source": plan.pose_source})

    def _setup_mapped_parking(self, engine, physics, density, initial_speed_kmh):
        from app.services.parking import parking_plan, install_parking
        plan = parking_plan()
        road = plan.waypoints[len(real_road_route())-1:]
        physics.set_destination(*road[-1])
        physics.set_route(road)
        install_parking(physics, plan)
        physics.reset_state(station_m=0., speed_kmh=0., lateral_offset_m=EGO_LANE_OFFSET_M)
        # This repeatable mission starts with no moving traffic. The sensor
        # boundary remains active, and injected obstacles still veto motion.
        physics.traffic = TrafficModel(total_length_m=physics.station_distances[-1], seed=707, density="low")
        physics.traffic.npcs = []

    def _setup_normal_cruising(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh if initial_speed_kmh is not None else 45.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )

        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        # Initialize deterministic traffic with clear buffer ahead of ego
        traffic = TrafficModel(total_length_m=total_len, seed=42, density=density)
        # Ensure lead vehicle in ego lane is well ahead (> 90m)
        npcs = []
        for i, npc in enumerate(traffic.npcs):
            if abs(npc.lane_offset - EGO_LANE_OFFSET_M) < 0.2:
                # Place far ahead
                station = max(90.0, 90.0 + i * 80.0)
                speed = 50.0 + (i % 3) * 3.0
                npcs.append(
                    NpcVehicle(
                        id=f"npc-normal-{i}",
                        lane_offset=npc.lane_offset,
                        speed_kmh=speed,
                        station_m=station,
                        desired_speed_kmh=speed,
                    )
                )
            else:
                npcs.append(npc)
        traffic.spawn_scripted_npcs(npcs)
        physics.traffic = traffic

    def _tick_normal_cruising(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})
        if tick == 30 and not flags.get("cruising_confirmed"):
            flags["cruising_confirmed"] = True
            self._scenario_data["milestone"] = "Cruising smoothly in lane centre"
            return self._create_event(
                event_type="CRUISING_STABLE",
                cause="Ego vehicle maintaining target cruise speed",
                decision="Maintain Speed",
                metadata={"speed_kmh": round(physics.speed_kmh, 1), "lateral_m": round(physics.current_lateral_offset_m, 2)},
            )
        return None

    # =========================================================================
    # Scenario 2: Traffic Overtake (Lane Change)
    # =========================================================================
    def _setup_traffic_overtake(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 42.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )

        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=101, density=density)

        # Place a slow lead vehicle at station = 30m moving at 20 km/h in ego lane
        lead_car = NpcVehicle(
            id="npc-slow-lead",
            lane_offset=EGO_LANE_OFFSET_M,
            speed_kmh=20.0,
            station_m=30.0,
            desired_speed_kmh=20.0,
            prevent_recycle=True,
        )
        # Ensure adjacent passing lane near ego is completely clear
        other_npcs = [lead_car]
        for npc in traffic.npcs:
            if npc.id == "npc-slow-lead":
                continue
            if abs(npc.lane_offset - ADJACENT_LANE_OFFSET_M) < 0.2:
                # Move to oncoming lane so passing corridor is wide open
                npc.lane_offset = -1.75
                npc.base_lane_offset = -1.75
                npc.wander_amplitude_m = 0.0
            other_npcs.append(npc)

        traffic.spawn_scripted_npcs(other_npcs)
        physics.traffic = traffic
        self._scenario_data["slow_lead_id"] = "npc-slow-lead"

    def _tick_traffic_overtake(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        # Check if lane change is initiated
        candidates = physics.get_planner_candidates()
        chosen = next((c for c in candidates if c["is_chosen"]), None)
        is_changing = chosen and chosen.get("is_lane_change", False)

        if is_changing and not flags.get("lane_change_started"):
            flags["lane_change_started"] = True
            self._scenario_data["milestone"] = "Lane change chosen: Overtaking slow lead"
            return self._create_event(
                event_type="LANE_CHANGE_INITIATED",
                cause="Current lane blocked by slow lead vehicle (20 km/h)",
                actor="PLANNER",
                decision="Change to Adjacent Lane",
                confidence=0.96,
                metadata={
                    "gap_m": round(physics.sensed_lead.gap_m, 1) if physics.sensed_lead else None,
                    "target_lateral_d": chosen.get("d_target"),
                },
            )

        # Check if ego reached adjacent lane
        if flags.get("lane_change_started") and not flags.get("lane_change_completed"):
            if abs(physics.current_lateral_offset_m - ADJACENT_LANE_OFFSET_M) < 0.4:
                flags["lane_change_completed"] = True
                self._scenario_data["milestone"] = "Lane change complete: Passing vehicle"
                return self._create_event(
                    event_type="LANE_CHANGE_COMPLETED",
                    cause="Vehicle established in adjacent passing lane",
                    actor="EGO",
                    decision="Cruise in Passing Lane",
                    metadata={"lateral_offset_m": round(physics.current_lateral_offset_m, 2)},
                )

        return None

    # =========================================================================
    # Scenario 3: Emergency Cut-in & Brake (Safety Shield)
    # =========================================================================
    def _setup_emergency_cut_in(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 48.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )

        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=202, density=density)

        # Place the cut-in vehicle in adjacent lane just slightly ahead
        cut_in_vehicle = NpcVehicle(
            id="npc-cut-in",
            lane_offset=ADJACENT_LANE_OFFSET_M,
            speed_kmh=48.0,
            station_m=28.0,
            desired_speed_kmh=48.0,
            prevent_recycle=True,
        )
        npcs = [cut_in_vehicle] + [n for n in traffic.npcs if n.id != "npc-cut-in"]
        traffic.spawn_scripted_npcs(npcs)
        physics.traffic = traffic
        self._scenario_data["cut_in_id"] = "npc-cut-in"

    def _tick_emergency_cut_in(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        # At tick 12 (~1.2s), the adjacent vehicle abruptly swerves into ego lane and brakes hard
        if tick == 12 and not flags.get("cut_in_triggered"):
            flags["cut_in_triggered"] = True
            for npc in physics.traffic.npcs:
                if npc.id == "npc-cut-in":
                    npc.lane_offset = EGO_LANE_OFFSET_M  # cut directly into ego lane
                    npc.speed_kmh = 8.0  # sudden hard braking
                    npc.desired_speed_kmh = 8.0
                    break

            self._scenario_data["milestone"] = "Vehicle cut into lane and braked hard"
            return self._create_event(
                event_type="VEHICLE_CUT_IN",
                cause="Adjacent vehicle abruptly entered ego lane at short gap",
                actor="NPC",
                decision="Swerve and Hard Brake",
                metadata={"lead_id": "npc-cut-in", "lead_speed_kmh": 8.0},
            )

        # Check for Safety Shield override engagement
        shield = physics.shield_verdict
        if (
            shield.risk_level == RISK_CRITICAL
            or shield.override_action == OVERRIDE_EMERGENCY_BRAKE
        ) and not flags.get("shield_engaged"):
            flags["shield_engaged"] = True
            self._scenario_data["milestone"] = "Safety Shield OVERRIDE: Emergency braking engaged"
            return self._create_event(
                event_type="SAFETY_SHIELD_OVERRIDE",
                cause=f"TTC violation: {shield.ttc_s:.2f}s < {TTC_CRITICAL_S}s",
                actor="SAFETY_SHIELD",
                decision="EMERGENCY_BRAKE",
                confidence=1.0,
                metadata={
                    "ttc_s": shield.ttc_s,
                    "reasons": shield.reasons,
                    "deceleration_mps2": round(physics.acceleration_mps2, 2),
                },
            )

        return None

    # =========================================================================
    # Scenario 4: Queue Stop-and-Go
    # =========================================================================
    def _setup_queue_stop_and_go(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 35.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )

        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=303, density=density)

        # Queue of 3 vehicles in ego lane:
        # Car 1: stationary at station = 42m
        # Car 2: stationary at station = 48m
        # Car 3: stationary at station = 54m
        queue_npcs = [
            NpcVehicle(
                id="npc-queue-1",
                lane_offset=EGO_LANE_OFFSET_M,
                speed_kmh=0.0,
                station_m=42.0,
                desired_speed_kmh=0.0,
                prevent_recycle=True,
            ),
            NpcVehicle(
                id="npc-queue-2",
                lane_offset=EGO_LANE_OFFSET_M,
                speed_kmh=0.0,
                station_m=48.0,
                desired_speed_kmh=0.0,
                prevent_recycle=True,
            ),
            NpcVehicle(
                id="npc-queue-3",
                lane_offset=EGO_LANE_OFFSET_M,
                speed_kmh=0.0,
                station_m=54.0,
                desired_speed_kmh=0.0,
                prevent_recycle=True,
            ),
        ]
        # Also block the adjacent lane so ego must queue rather than pass
        blocking_car = NpcVehicle(
            id="npc-adjacent-block",
            lane_offset=ADJACENT_LANE_OFFSET_M,
            speed_kmh=0.0,
            station_m=35.0,
            desired_speed_kmh=0.0,
            prevent_recycle=True,
        )
        traffic.spawn_scripted_npcs(queue_npcs + [blocking_car])
        physics.traffic = traffic

    def _tick_queue_stop_and_go(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        # When ego comes to a stop behind queue
        if physics.speed_kmh < 1.0 and not flags.get("standstill_reached"):
            flags["standstill_reached"] = True
            self._scenario_data["milestone"] = "Smoothly stopped behind traffic queue"
            return self._create_event(
                event_type="QUEUE_STANDSTILL",
                cause="IDM car-following brought vehicle to complete stop behind queue",
                actor="EGO",
                decision="Decelerate to 0 km/h",
                metadata={"sensed_gap_m": round(physics.sensed_lead.gap_m, 1) if physics.sensed_lead else None},
            )

        # At tick 95, the queue starts moving again
        if tick == 95 and not flags.get("queue_resumed"):
            flags["queue_resumed"] = True
            for npc in physics.traffic.npcs:
                if "queue" in npc.id or npc.id == "npc-adjacent-block":
                    npc.desired_speed_kmh = 35.0
                    npc.speed_kmh = 15.0

            self._scenario_data["milestone"] = "Traffic queue moving: Resuming cruise"
            return self._create_event(
                event_type="QUEUE_RESUMED",
                cause="Lead traffic accelerated away",
                actor="TRAFFIC",
                decision="Resume Cruising",
                metadata={"new_lead_speed_kmh": 35.0},
            )

        return None

    # =========================================================================
    # Scenario 5: Erratic Cut-in
    # =========================================================================
    def _setup_erratic_cut_in(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 50.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )
        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=404, density=density)
        cut_in_car = NpcVehicle(
            id="npc-erratic-cutin",
            lane_offset=ADJACENT_LANE_OFFSET_M,
            speed_kmh=52.0,
            station_m=28.0,
            desired_speed_kmh=52.0,
            prevent_recycle=True,
            aggressiveness=0.85,
        )
        traffic.spawn_scripted_npcs([cut_in_car])
        physics.traffic = traffic

    def _tick_erratic_cut_in(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        # Between tick 10 and 22 (~1.0s to ~2.2s), the adjacent car rapidly sweeps across lanes and brakes
        if 10 <= tick <= 24:
            for npc in physics.traffic.npcs:
                if npc.id == "npc-erratic-cutin":
                    # Lateral progression toward ego lane
                    progress = min(1.0, (tick - 10) / 10.0)
                    npc.lane_offset = ADJACENT_LANE_OFFSET_M + (EGO_LANE_OFFSET_M - ADJACENT_LANE_OFFSET_M) * progress
                    npc.lateral_speed_mps = -1.75
                    if tick >= 14:
                        npc.desired_speed_kmh = 10.0
                        npc.speed_kmh = max(10.0, npc.speed_kmh - 3.5)
                    break

            if tick == 10 and not flags.get("cut_in_started"):
                flags["cut_in_started"] = True
                self._scenario_data["milestone"] = "Aggressive vehicle initiated sudden cut-in"
                return self._create_event(
                    event_type="VEHICLE_CUT_IN",
                    cause="High-speed adjacent vehicle executed sudden high-lateral-velocity cut-in",
                    actor="NPC",
                    decision="Aggressive Cut-in & Deceleration",
                    metadata={"lead_id": "npc-erratic-cutin", "lateral_rate_mps": 1.75},
                )

        shield = physics.shield_verdict
        if (
            shield.risk_level == RISK_CRITICAL
            or shield.override_action == OVERRIDE_EMERGENCY_BRAKE
        ) and not flags.get("shield_engaged"):
            flags["shield_engaged"] = True
            self._scenario_data["milestone"] = "Safety Shield OVERRIDE: Emergency brake triggered"
            return self._create_event(
                event_type="SAFETY_SHIELD_OVERRIDE",
                cause=f"TTC critical violation: {shield.ttc_s:.2f}s < {TTC_CRITICAL_S}s",
                actor="SAFETY_SHIELD",
                decision="EMERGENCY_BRAKE",
                confidence=1.0,
                metadata={
                    "ttc_s": shield.ttc_s,
                    "reasons": shield.reasons,
                    "deceleration_mps2": round(physics.acceleration_mps2, 2),
                },
            )
        return None

    # =========================================================================
    # Scenario 6: Highway Merge & Interleaving
    # =========================================================================
    def _setup_highway_merge(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 52.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )
        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=505, density=density)
        merge_car = NpcVehicle(
            id="npc-merge-lead",
            lane_offset=ADJACENT_LANE_OFFSET_M,
            speed_kmh=48.0,
            station_m=34.0,
            desired_speed_kmh=48.0,
            prevent_recycle=True,
            aggressiveness=0.45,
        )
        traffic.spawn_scripted_npcs([merge_car])
        physics.traffic = traffic

    def _tick_highway_merge(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        # Track the real safety envelope through the merge window so
        # MERGE_COMPLETED can report what actually happened rather than
        # asserting "no violation" by fiat.
        if 15 <= tick <= 46:
            gap = physics.sensed_lead.gap_m if physics.sensed_lead is not None else None
            if gap is not None:
                flags["min_gap_m"] = min(flags.get("min_gap_m", 1e9), gap)
            if not physics.shield_verdict.approved:
                flags["shield_vetoed_during_merge"] = True

        # At tick 15, vehicle begins gentle merging maneuver over 3 seconds (30 ticks)
        if 15 <= tick <= 45:
            for npc in physics.traffic.npcs:
                if npc.id == "npc-merge-lead":
                    progress = (tick - 15) / 30.0
                    npc.lane_offset = ADJACENT_LANE_OFFSET_M + (EGO_LANE_OFFSET_M - ADJACENT_LANE_OFFSET_M) * progress
                    npc.lateral_speed_mps = -0.58
                    break

            if tick == 15 and not flags.get("merge_started"):
                flags["merge_started"] = True
                self._scenario_data["milestone"] = "Highway merge vehicle entering corridor"
                return self._create_event(
                    event_type="MERGE_CORRIDOR_ENTRY",
                    cause="On-ramp vehicle entering main lane corridor",
                    actor="TRAFFIC",
                    decision="Smooth Lane Merge",
                    metadata={"lead_id": "npc-merge-lead", "speed_kmh": 48.0},
                )

        if tick > 45 and not flags.get("merge_completed"):
            flags["merge_completed"] = True
            for npc in physics.traffic.npcs:
                if npc.id == "npc-merge-lead":
                    npc.lane_offset = EGO_LANE_OFFSET_M
                    npc.lateral_speed_mps = 0.0
                    break
            min_gap = flags.get("min_gap_m")
            vetoed = flags.get("shield_vetoed_during_merge", False)
            envelope_held = (not vetoed) and (min_gap is None or min_gap > 6.0)
            meta = {
                "ego_speed_kmh": round(physics.speed_kmh, 1),
                "min_sensed_gap_m": round(min_gap, 1) if min_gap is not None else None,
                "shield_vetoed_during_merge": vetoed,
            }
            if envelope_held:
                self._scenario_data["milestone"] = "Merge completed, safety envelope held"
                return self._create_event(
                    event_type="MERGE_COMPLETED",
                    cause="Lead vehicle completed merge; ego held gap > 6 m and Safety Shield never vetoed",
                    actor="EGO",
                    decision="Maintain Smooth Follow",
                    metadata=meta,
                )
            self._scenario_data["milestone"] = "Merge completed but safety margin was tight"
            return self._create_event(
                event_type="MERGE_SAFETY_MARGIN_TIGHT",
                cause=f"Merge completed with reduced margin (min gap {min_gap:.1f} m, shield vetoed={vetoed})"
                      if min_gap is not None else "Merge completed with Safety Shield intervention",
                actor="EGO",
                decision="Merge Absorbed With Tight Margin",
                metadata=meta,
            )
        return None

    # =========================================================================
    # Scenario 7: Inattentive Lane Straddler
    # =========================================================================
    def _setup_lane_straddler(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 42.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )
        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=606, density=density)
        # Nominal offset only ~0.9 m outside the ego lane centre with a 0.7 m
        # wander amplitude, so the straddler periodically crosses the lane
        # line into the ego's path; slow enough (28 km/h vs 42) that the ego
        # must actually give way rather than sail past.
        straddler = NpcVehicle(
            id="npc-straddler",
            lane_offset=EGO_LANE_OFFSET_M + 0.9,
            speed_kmh=28.0,
            station_m=25.0,
            desired_speed_kmh=28.0,
            prevent_recycle=True,
            wander_amplitude_m=0.7,
            wander_freq_rad=0.35,
        )
        traffic.spawn_scripted_npcs([straddler])
        physics.traffic = traffic

    def _tick_lane_straddler(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        if tick == 5 and not flags.get("straddler_notified"):
            flags["straddler_notified"] = True
            self._scenario_data["milestone"] = "Lead vehicle straddling lane boundary detected"
            return self._create_event(
                event_type="LANE_STRADDLER_DETECTED",
                cause="Lead NPC exhibiting lateral wandering across lane demarcation",
                actor="PERCEPTION",
                decision="Track Defensive Clearance",
                metadata={"nominal_offset_m": round(EGO_LANE_OFFSET_M + 0.9, 2), "drift_amplitude_m": 0.7},
            )
        return None

    # =========================================================================
    # Scenario 8: Low-Friction Wet Curve
    # =========================================================================
    def _setup_low_friction_curve(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        # load_scenario installed the recorded US 101 approach curve.
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 55.0,
            lateral_offset_m=EGO_LANE_OFFSET_M,
        )
        physics.friction_mu = 0.5  # Wet surface adhesion (mu in [0.1, 1.0])
        total_len = physics.station_distances[-1] if physics.station_distances else 2000.0
        traffic = TrafficModel(total_length_m=total_len, seed=707, density=density)
        physics.traffic = traffic

    def _tick_low_friction_curve(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        tick: int,
        sim_time_s: float,
    ) -> Optional[dict]:
        flags = self._scenario_data.setdefault("flags", {})

        if tick == 2 and not flags.get("friction_advisory"):
            flags["friction_advisory"] = True
            self._scenario_data["milestone"] = "Road friction reduced: mu = 0.5 (wet asphalt)"
            return self._create_event(
                event_type="LOW_FRICTION_ADVISORY",
                cause="Environmental sensor detects wet road surface",
                actor="ENVIRONMENT",
                decision="De-rate lateral acceleration envelope",
                metadata={"friction_mu": 0.5, "max_lat_accel_mps2": round(physics.A_LAT_MAX_MPS2 * 0.5, 2)},
            )

        # Fire on the RECORDED de-rate fact, not speed_limit_reason: the
        # tracking-correction cap routinely overwrites the "low_friction_curvature"
        # label on a real curve even though the friction de-rate did lower the
        # cornering cap.
        if getattr(physics, "friction_speed_derate_active", False) and not flags.get("speed_derated"):
            flags["speed_derated"] = True
            cap = getattr(physics, "friction_curve_cap_kmh", None)
            drop = getattr(physics, "friction_derate_kmh", 0.0)
            self._scenario_data["milestone"] = f"Wet-curve de-rate removed ~{drop:.0f} km/h of cornering speed"
            return self._create_event(
                event_type="SPEED_DE_RATED",
                cause="Cornering lateral-acceleration budget cut by low friction (mu = 0.5)",
                actor="PHYSICS_PLANNER",
                decision=f"Cornering speed cap {cap:.1f} km/h" if cap is not None else "Cornering speed de-rated",
                metadata={
                    "friction_mu": round(getattr(physics, "friction_mu", 0.5), 2),
                    "wet_curve_cap_kmh": round(cap, 1) if cap is not None else None,
                    "derate_kmh": round(drop, 1),
                    "speed_kmh": round(physics.speed_kmh, 1),
                },
            )
        return None




# Global singleton instance for easy import and sharing
scenario_engine = ScenarioEngine()
