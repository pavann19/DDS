"""Authoritative Simulation Session (100 Hz scheduling clock, driver/world separation).

Owns:
- WorldBackend: DeterministicSimBackend(mode="external")
- Driver: simulator-independent decision maker
- SimClock & MultiRateExecutor: 100 Hz scheduling clock
    - 50 Hz: control / safety / physics
    - 20 Hz: perception + track history
    - 10 Hz: behaviour / planning
- ScenarioEngine: scenario state & triggers
- CommandQueue: operator/client commands processed at tick boundaries
- Bounded queues for analytics, persistence, and WebSocket subscribers
"""
from __future__ import annotations

import asyncio
import json
import sys
import copy
import logging
import math
import time
import uuid
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

from app.core.database import DriverScoreLog, TelemetryLog
from app.services.carla_adapter import ControlCommand, DeterministicSimBackend
from app.services.driver.context import DriverContext
from app.services.driver.driver import Driver
from app.services.driver_scoring import DriverScorer
from app.services.executor import (
    BASE_HZ,
    CONTROL_HZ,
    PERCEPTION_HZ,
    PLANNING_HZ,
    MultiRateExecutor,
)
from app.services.inference import pipeline
from app.services.interfaces import ActuatorCommand, SensorObservation, SimClock
from app.services.routing import get_route
from app.services.road_routes import real_road_route, real_road_name, real_road_continuation
from app.services.road_semantics import recorded_map
from app.services.scenario_engine import ScenarioEngine
from app.services.journey_planning import journey_plans
from app.services.vehicle_signals import route_turns, vehicle_signals

logger = logging.getLogger(__name__)


@dataclass
class TimingStats:
    sim_time_s: float = 0.0
    wall_time_s: float = 0.0
    real_time_factor: float = 1.0
    is_slowdown: bool = False
    slowdown_factor: float = 1.0
    scheduling_lateness_ms: float = 0.0
    max_scheduling_lateness_ms: float = 0.0


class SubscriberQueue(asyncio.Queue):
    """Coalesce replaceable state only; reliable-message overflow is explicit."""
    failed = False

    def put_nowait(self, item):
        if self.failed:
            return
        if self.full():
            old = next((p for p in self._queue if p.get("type") == "state"), None)
            if old is not None:
                self._queue.remove(old)
                self.task_done()
            elif item.get("type") == "state":
                return
            else:
                self.failed = True
                while not self.empty():
                    self.get_nowait()
                    self.task_done()
                item = {"type": "subscriber_overflow"}
        super().put_nowait(item)


class SimulationSession:
    """The authoritative simulation session.

    Owns the world, clock, driver, scenario, and command queue.
    """

    def __init__(
        self,
        *,
        seed: Optional[int] = None,
        real_time: bool = True,
        session_id: Optional[str] = None,
    ) -> None:
        self.session_id = session_id or str(uuid.uuid4())
        self.seed = seed
        self.real_time = real_time
        self.is_paused: bool = False
        self._running: bool = False

        # WorldBackend in external-driver mode
        self.world = DeterministicSimBackend(mode="external", seed=seed)

        # Simulator-independent Driver (shared with physics engine for property forwarding)
        self.driver = self.world.physics._driver
        self.scenario_engine = ScenarioEngine()

        # Multi-rate executor (100 Hz base scheduling clock)
        self.executor = MultiRateExecutor(base_hz=BASE_HZ, seed=seed)

        # Queues
        import collections
        self.command_queue: collections.deque = collections.deque()
        self.analytics_queue: Optional[asyncio.Queue] = None
        self.persistence_queue: Optional[asyncio.Queue] = None
        self.persistence_failed = False
        self._subscribers: Dict[str, asyncio.Queue] = {}
        self.epoch = 0
        self.route_version = 1
        self._route_generation = 0
        self.route_status = "ready"
        self.route_steps = []
        self._signal_route_key = None
        self._signal_turns = []
        self._route_request = None

        # Pipeline state handoffs
        self.latest_observation: Optional[SensorObservation] = None
        self.latest_command: Optional[ActuatorCommand] = None
        self.latest_payload: Optional[Dict[str, Any]] = None
        self.plan_tick = None

        # Analytics / Scorer state (updated by background worker)
        self.scorer = DriverScorer(window_size=60)
        self.latest_analytics: Dict[str, Any] = {
            "decision": "Unavailable",
            "confidence": 0.0,
            "shap": {},
            "anomaly": {"is_anomaly": False, "type": "NORMAL"},
            "driver_score": {},
        }

        # Timing
        self.timing = TimingStats()
        self._wall_start: float = time.perf_counter()
        self._sim_start: float = 0.0

        # Background tasks
        self._main_task: Optional[asyncio.Task] = None
        self._analytics_task: Optional[asyncio.Task] = None
        self._persistence_task: Optional[asyncio.Task] = None
        self._background_tasks: List[asyncio.Task] = []

        # Setup initial route
        self._init_default_route()
        self._route_geometry = tuple(self.world.physics.route)

        # Register multi-rate stages
        self._register_stages()

    def _init_default_route(self) -> None:
        from app.services.parking import parking_plan, install_parking
        plan = parking_plan()
        hw = plan.waypoints
        self.world.reset(route_waypoints=hw, start_speed_mps=0.0, seed=self.seed)
        install_parking(self.world.physics, plan)
        self.route_steps = [{"instruction": "US 101 → Broadway → Old Bayshore Highway → mapped parking area", "type": "depart"}]
        self.latest_observation = self.world.observe()

    def _register_stages(self) -> None:
        """Register the pipeline stages with the 100 Hz executor in execution order.

        1. Perception (20 Hz)
        2. Behaviour / Planning (10 Hz)
        3. Control / Safety / Physics (50 Hz)
        """
        self.executor.add_stage("perception", PERCEPTION_HZ, self._stage_perception)
        self.executor.add_stage("planning", PLANNING_HZ, self._stage_planning)
        self.executor.add_stage("control_safety_physics", CONTROL_HZ, self._stage_control_physics)
        self.executor.add_stage("clock", BASE_HZ, self._stage_clock)

    def _stage_clock(self, clock: SimClock) -> None:
        self.world.physics.clock = clock
        self.scenario_engine.elapsed_time_s = clock.sim_time_s
        self.world.physics.active_scenario = self.scenario_engine.get_state()

    # ------------------------------------------------------------------
    # Multi-Rate Pipeline Stages
    # ------------------------------------------------------------------
    def _stage_perception(self, clock: SimClock) -> None:
        """Perception + Track History (20 Hz)."""
        self.world.physics.clock = clock
        self.world.physics.step_perception(.05)
        self.latest_observation = self.world.observe()

    def _stage_planning(self, clock: SimClock) -> None:
        """Prediction -> Behaviour Choice -> Trajectory Planning (10 Hz)."""
        self.world.physics.clock = clock
        self.world.physics.step_prediction(.10)
        obs = self.world.observe()
        ctx = self.world.physics.build_driver_context(dt_s=0.10)

        # Let the Driver run candidate planning and lateral plan
        _lat = self.driver.decide_lateral(obs, ctx)
        self.plan_tick = clock.tick
        self.world.physics.lateral_target_d_m = self.driver.lateral_target_d_m

        # Update driving state machine on physics
        dist = self.world.physics._last_dist
        corner_turn_deg = self.world.physics._last_corner_turn_deg
        waypoint_dist = self.world.physics._last_waypoint_dist

        from app.services.physics_engine import DrivingState

        if self.world.physics.has_arrived:
            self.world.physics.driving_state = (
                DrivingState.ARRIVED if self.world.physics.speed_kmh < 1.0 else DrivingState.ARRIVING
            )
        elif dist < 40.0:
            self.world.physics.driving_state = DrivingState.FINAL_APPROACH
        elif self.world.physics.speed_kmh < 10.0 and dist > 40.0:
            self.world.physics.driving_state = DrivingState.DEPARTING
        elif corner_turn_deg > 15.0 and waypoint_dist < 60.0:
            self.world.physics.driving_state = DrivingState.APPROACHING_TURN
        elif abs(self.world.physics.steering_angle) > 0.15:
            self.world.physics.driving_state = DrivingState.TURNING
        else:
            self.world.physics.driving_state = DrivingState.CRUISING

        if self.world.physics.speed_kmh < 1.0 and dist > 10.0 and not self.world.physics.route:
            self.world.physics.driving_state = DrivingState.IDLE

        parking = self.world.physics.observe_parking()
        if (parking and parking['parked']) or dist < (.7 if self.world.physics.parking_mission else 10.0):
            self.world.physics.has_arrived = True

    def _stage_control_physics(self, clock: SimClock) -> None:
        """Controller -> Independent Safety Check -> World Integration (50 Hz)."""
        if self.latest_observation is None:
            self.latest_observation = self.world.observe()
        # Fresh ego/forward range at control rate; tracked surroundings at 20 Hz.
        obs = self.world.observe()
        ctx = self.world.physics.build_driver_context(dt_s=0.02)

        # 1. Controller & Safety Shield -> ActuatorCommand
        cmd = self.driver.control(obs, ctx)
        self.latest_command = cmd

        # 2. WorldBackend.apply_control()
        self.world.apply_control(cmd)

        # 3. World Integration (50 Hz, dt = 0.02s)
        self.world.step(0.02, clock=clock, update_sensors=False)

        # 4. Scenario Engine update
        evt = self.scenario_engine.update(self.world.physics, 0.10, clock=clock) if clock.is_rate_tick(10.0) else None
        if evt:
            self._broadcast_event(evt)

    # ------------------------------------------------------------------
    # Step Execution (1 complete pipeline = 10 substeps = 0.10s)
    # ------------------------------------------------------------------
    def step_pipeline(self, num_substeps: int = 10) -> List[str]:
        """Advance the complete simulation pipeline by `num_substeps`.

        At 100 Hz base clock, 10 substeps = 0.10s, ensuring:
        - 1 planning tick (10 Hz)
        - 2 perception ticks (20 Hz)
        - 5 control / safety / physics ticks (50 Hz)
        """
        self.process_pending_commands()
        all_ran = []
        for _ in range(num_substeps):
            ran = self.executor.step()
            all_ran.extend(ran)

        # Emit telemetry snapshot after the pipeline cycle
        self._emit_telemetry_snapshot()
        return all_ran

    # ------------------------------------------------------------------
    # Command Queue & Tick Boundaries
    # ------------------------------------------------------------------
    def enqueue_command(self, cmd: Dict[str, Any], *, subscriber_id=None) -> None:
        """Queue an operator/client command for processing at the next tick boundary."""
        if len(self.command_queue) >= 256:
            self.command_reply(subscriber_id, cmd.get("command_id"), "rejected", "Command queue full")
            return
        self.command_queue.append({**cmd, "_subscriber_id": subscriber_id})

    def command_reply(self, subscriber_id, command_id, status, message=None):
        reply = {"type": "command_result", "command_id": command_id, "status": status,
                 "message": message, "epoch": self.epoch, "route_version": self.route_version,
                 "tick": self.executor.clock.tick, "simulation_time_s": self.executor.clock.sim_time_s}
        q = self._subscribers.get(subscriber_id)
        if q is not None:
            q.put_nowait(reply)

    def process_pending_commands(self) -> None:
        """Drain and process commands at the tick boundary."""
        # Bound command work so a backlog cannot monopolize a control tick.
        for _ in range(min(4, len(self.command_queue))):
            try:
                cmd = self.command_queue.popleft()
                result = self._handle_command(cmd)
                if cmd.get("type") != "apply_route":
                    self.command_reply(cmd.get("_subscriber_id"), cmd.get("command_id"), result or "applied")
            except Exception as e:
                self.command_reply(cmd.get("_subscriber_id"), cmd.get("command_id"), "rejected", str(e))
                logger.warning("Command rejected: %s", e)

    def _handle_command(self, payload: Dict[str, Any]) -> None:
        cmd_type = payload.get("type")
        if cmd_type == "pause_simulation":
            self.is_paused = True
            self.scenario_engine.pause(self.world.physics)
            self._emit_telemetry_snapshot()
            logger.info("Simulation paused at tick boundary")
        elif cmd_type == "resume_simulation":
            if self.route_status == "pending":
                raise ValueError("Wait for route preparation before resuming")
            self.is_paused = False
            self.scenario_engine.resume(self.world.physics)
            self._emit_telemetry_snapshot()
            logger.info("Simulation resumed at tick boundary")
        elif cmd_type == "reset_simulation":
            self.reset()
            logger.info("Simulation reset at tick boundary")
        elif cmd_type == "set_destination":
            lat = payload.get("lat")
            lng = payload.get("lng")
            if lat is None or lng is None or not math.isfinite(lat) or not math.isfinite(lng) or not -90 <= lat <= 90 or not -180 <= lng <= 180:
                raise ValueError("Invalid destination coordinates")
            if lat is not None and lng is not None:
                self._supersede_route_request()
                self._route_generation += 1
                generation = self._route_generation
                self._route_request = payload
                self.route_status = "pending"
                self.is_paused = True
                self.scenario_engine.pause(self.world.physics)
                try:
                    loop = asyncio.get_running_loop()
                    t = loop.create_task(
                        self._fetch_and_apply_route(self.world.physics.lat, self.world.physics.lng, lat, lng, generation)
                    )
                    self._background_tasks.append(t)
                    t.add_done_callback(self._background_tasks.remove)
                except RuntimeError:
                    self.route_status = "failed"
                    raise ValueError("Routing requires an active event loop")
                return "pending"
        elif cmd_type == "start_journey":
            plan_id = payload.get("plan_id")
            if not isinstance(plan_id, str) or not 16 <= len(plan_id) <= 128:
                raise ValueError("Invalid route preview token")
            if self.scenario_engine.active_scenario_id != "driving_environment":
                raise ValueError("Start a journey from the combined driving environment")
            if self.route_status == "pending":
                raise ValueError("Wait for the current route request to finish")
            plan = journey_plans.get(plan_id)
            self.is_paused = True
            self.scenario_engine.pause(self.world.physics)
            # The operator explicitly selected a new simulated spawn point.
            # PhysicsEngine.set_route snaps it to the returned road geometry.
            self.apply_route(plan.waypoints, [json.loads(step) for step in plan.steps],
                             start_from_route_origin=True)
            journey_plans.consume(plan_id)
            self.is_paused = False
            self.scenario_engine.resume(self.world.physics)
            self._emit_telemetry_snapshot()
            return "applied"
        elif cmd_type == "apply_route":
            if payload.get("generation") != self._route_generation:
                return "superseded"
            waypoints = payload.get("waypoints")
            steps = payload.get("steps", [])
            try:
                if not waypoints:
                    raise ValueError("Route fetch failed; previous route retained, simulation paused")
                self.apply_route(waypoints, steps)
                self.route_status = "ready"
                request = self._route_request or {}
                self.command_reply(request.get("_subscriber_id"), request.get("command_id"), "applied")
            except Exception as e:
                self.route_status = "failed"
                request = self._route_request or {}
                self.command_reply(request.get("_subscriber_id"), request.get("command_id"), "rejected", str(e))
            self._route_request = None
        elif cmd_type == "load_scenario":
            scenario_id = payload.get("scenario_id")
            density = payload.get("traffic_density")
            initial_speed = payload.get("initial_speed_kmh")
            params = payload.get("params")
            if scenario_id:
                candidate_physics = copy.deepcopy(self.world.physics)
                candidate_scenario = copy.deepcopy(self.scenario_engine)
                evt = candidate_scenario.load_scenario(
                    scenario_id,
                    candidate_physics,
                    density=density,
                    initial_speed_kmh=initial_speed,
                    params=params,
                )
                self.world.physics = candidate_physics
                self.driver = candidate_physics._driver
                self.scenario_engine = candidate_scenario
                self._reset_run_handoffs()
                if scenario_id == "driving_environment":
                    self.is_paused = True
                    self.scenario_engine.pause(self.world.physics)
                if evt:
                    self._broadcast_event(evt)
                self._emit_telemetry_snapshot()
                logger.info(f"Scenario loaded at tick boundary: {scenario_id}")
        elif cmd_type == "set_cruise_speed":
            speed = payload.get("speed_kmh")
            if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or not 10 <= speed <= 120:
                raise ValueError("Cruise speed must be a finite number between 10 and 120 km/h")
            self.driver.cruise_speed_kmh = float(speed)
            self._emit_telemetry_snapshot()
        elif cmd_type == "step_simulation":
            if self.route_status == "pending":
                raise ValueError("Wait for route preparation before stepping")
            # Single-step advances the complete simulation pipeline (0.10s)
            self.is_paused = False
            self.scenario_engine.resume(self.world.physics)
            self.step_pipeline(num_substeps=10)
            self.is_paused = True
            self.scenario_engine.pause(self.world.physics)
            self._emit_telemetry_snapshot()
            logger.info("Simulation stepped 1 tick (complete pipeline)")
        else:
            raise ValueError("Unsupported command type")

    async def _fetch_and_apply_route(
        self, origin_lat: float, origin_lng: float, dest_lat: float, dest_lng: float, generation: int
    ) -> None:
        try:
            route_data = await get_route(origin_lat, origin_lng, dest_lat, dest_lng)
            waypoints, steps = route_data if route_data else ([], [])
            self.enqueue_command({"type": "apply_route", "generation": generation, "waypoints": waypoints, "steps": steps})
        except Exception as e:
            logger.error(f"Failed to fetch route: {e}", exc_info=True)
            self.enqueue_command({"type": "apply_route", "generation": generation, "waypoints": []})

    def apply_route(self, waypoints: Sequence[Tuple[float, float]], steps: List[Any], *, start_from_route_origin: bool = False) -> None:
        import copy
        from app.services.frenet import project_to_frenet, frenet_to_latlng
        if len(waypoints) < 2 or not all(len(p) == 2 and math.isfinite(p[0]) and math.isfinite(p[1]) and -90 <= p[0] <= 90 and -180 <= p[1] <= 180 for p in waypoints):
            raise ValueError("Invalid route geometry")
        old = self.world.physics
        candidate = copy.deepcopy(old)
        candidate.set_route(list(waypoints))
        if start_from_route_origin:
            candidate.speed_kmh = 0.0
            candidate.acceleration_mps2 = 0.0
            candidate.steering_angle_rad = 0.0
            candidate.steering_angle = 0.0
            candidate.has_arrived = False
        else:
            station, offset, index = project_to_frenet(candidate.frenet_frame, old.lat, old.lng, search_window=len(candidate.route))
            if abs(offset) > 7.0:
                raise ValueError("New route is outside the current road corridor; explicit scenario restart required")
            projected = frenet_to_latlng(candidate.frenet_frame, station, offset)
            if old.calculate_distance(old.lat, old.lng, *projected) > 1.:
                raise ValueError("New route does not contain current position; previous route retained")
            candidate.lat, candidate.lng, candidate.heading = old.lat, old.lng, old.heading
            candidate.current_station_m, candidate.current_lateral_offset_m = station, offset
            candidate.frenet_search_idx = index
        candidate.target_lat, candidate.target_lng = waypoints[-1]
        candidate.has_arrived = False
        if self.scenario_engine.active_scenario_id == "driving_environment":
            from app.services.scenario_engine import configure_driving_environment_traffic
            configure_driving_environment_traffic(candidate, self.scenario_engine.active_density)
            # World signal time follows the session, including a route change
            # midway through a drive. Do not restart NPC lights independently.
            candidate.traffic.elapsed_time_s = self.executor.clock.sim_time_s
            candidate.control_camera = None
            candidate.scene_camera_pending = None
        candidate.step_world_sensors(0.)
        # Commit all geometry, traffic and driver references together at a boundary.
        self.world.physics = candidate
        self.driver = candidate._driver
        self.world._last_cmd = None
        self.latest_command = None
        self.plan_tick = None
        self.latest_observation = self.world.observe()
        if self.scenario_engine.active_scenario_id == "driving_environment":
            environment = self.scenario_engine._scenario_data.get("environment", {})
            environment.update(traffic_extent_end_m=candidate.traffic.total_length_m,
                               npc_count=len(candidate.traffic.npcs),
                               mission_source="operator_route_no_verified_parking_goal")
        self.route_version += 1
        self.epoch += 1
        self.route_steps = steps
        self._route_geometry = tuple(self.world.physics.route)
        self._broadcast_event({
            "type": "route",
            "epoch": self.epoch, "route_version": self.route_version,
            "waypoints": [[lat, lng] for lat, lng in self.world.physics.route],
            "road_context": self.road_context(),
            "steps": steps,
        })
        self._emit_telemetry_snapshot()

    def reset(self) -> None:
        """Reset world, driver, scenario, and clock."""
        evt = self.scenario_engine.reset(self.world.physics)
        self.driver.on_route_reset()
        self._reset_run_handoffs()
        if self.scenario_engine.active_scenario_id == "driving_environment":
            self.is_paused = True
            self.scenario_engine.pause(self.world.physics)
        if evt and isinstance(evt, dict) and "type" in evt:
            self._broadcast_event(evt)
        self._emit_telemetry_snapshot()

    def _supersede_route_request(self) -> None:
        if self._route_request is not None:
            request = self._route_request
            self.command_reply(request.get("_subscriber_id"), request.get("command_id"),
                               "superseded", "A newer destination or scenario replaced this request")
            self._route_request = None

    def _reset_run_handoffs(self) -> None:
        self._supersede_route_request()
        self.epoch += 1
        self._route_generation += 1
        self.route_status = "ready"
        self._run_generation = getattr(self, "_run_generation", 0) + 1
        self.is_paused = False
        self.scenario_engine.resume(self.world.physics)
        self.world._last_cmd = None
        self.latest_command = None
        self.plan_tick = None
        self.scorer = DriverScorer(window_size=60)
        self.latest_analytics = {"decision": "Unavailable", "confidence": 0.0,
                                 "shap": {}, "anomaly": {}, "driver_score": {}}
        for queue in (self.analytics_queue, self.persistence_queue):
            if queue is not None:
                while not queue.empty():
                    queue.get_nowait()
                    queue.task_done()
        self.executor = MultiRateExecutor(base_hz=BASE_HZ, seed=self.seed)
        self._register_stages()
        self._wall_start = time.perf_counter()
        self._sim_start = 0.0
        self.timing = TimingStats()
        # Scenario setup replaces traffic after reset_state. Refresh sensors
        # before the first command so no lead from the previous run is reused.
        self.world.physics.step_world_sensors(0.0)
        self.world.physics.clock = self.executor.clock
        self.latest_observation = self.world.observe()

        geometry = tuple(self.world.physics.route)
        if geometry != self._route_geometry:
            self.route_version += 1
            profile = "wet_curve" if self.scenario_engine.active_scenario_id == "low_friction_curve" else "highway"
            scenario = self.scenario_engine.get_state()
            recorded_label = scenario.get("route_name") or "Mapped yield approach"
            instruction = (recorded_label if scenario["id"] in {"recorded_stop", "recorded_yield_conflict"}
                else "Broadway → Old Bayshore Highway → mapped parking area" if self.world.physics.parking_mission
                else real_road_name(profile))
            self.route_steps = [{"instruction": instruction, "type": "depart"}]
            self._route_geometry = geometry
        self._broadcast_event({"type": "route", "epoch": self.epoch,
                               "route_version": self.route_version,
                               "waypoints": [list(p) for p in geometry], "road_context": self.road_context(), "steps": self.route_steps})

    # ------------------------------------------------------------------
    # Telemetry Snapshot & Decoupled Queues
    # ------------------------------------------------------------------
    def _emit_telemetry_snapshot(self) -> None:
        started = time.perf_counter()
        try:
            self._build_and_dispatch_telemetry()
        finally:
            self.executor.record_external_timing("telemetry_build_dispatch",
                (time.perf_counter() - started) * 1000, 100.0)

    def _build_and_dispatch_telemetry(self) -> None:
        payload = self.build_telemetry_payload()
        self.latest_payload = payload
        sample_key = (self.epoch, self.route_version, self.executor.clock.tick)
        new_sample = sample_key != getattr(self, "_last_sample_key", None)
        self._last_sample_key = sample_key
        # Feature deltas are stateful: sample once and share the same reading.
        features = self.world.physics.get_ml_features() if new_sample else None

        # 1. Enqueue for background analytics (drop oldest if queue is full)
        if self.analytics_queue is not None and new_sample:
            if self.analytics_queue.full():
                try:
                    self.analytics_queue.get_nowait()
                    self.analytics_queue.task_done()
                except asyncio.QueueEmpty:
                    pass
            try:
                self.analytics_queue.put_nowait((features, time.time()))
            except (asyncio.QueueFull, RuntimeError):
                pass

        # 2. Enqueue for background persistence (drop oldest if full)
        if self.persistence_queue is not None and new_sample:
            if self.persistence_queue.full():
                try:
                    self.persistence_queue.get_nowait()
                    self.persistence_queue.task_done()
                except asyncio.QueueEmpty:
                    pass
            try:
                self.persistence_queue.put_nowait((self.session_id, time.time(), features, self.latest_analytics))
            except (asyncio.QueueFull, RuntimeError):
                pass

        # 3. Broadcast to each subscriber (drop oldest if subscriber is slow)
        for sub_id, q in list(self._subscribers.items()):
            try:
                q.put_nowait(payload)
            except (asyncio.QueueFull, RuntimeError):
                pass

    def _broadcast_event(self, event: Dict[str, Any]) -> None:
        for sub_id, q in list(self._subscribers.items()):
            try:
                q.put_nowait(event)
            except asyncio.QueueFull:
                pass

    def build_telemetry_payload(self) -> Dict[str, Any]:
        physics = self.world.physics
        nav_state = physics.get_navigation_state()
        rules = getattr(physics, "mapped_road_rules", None)
        current_time = time.time()
        iso_time = datetime.fromtimestamp(current_time, tz=timezone.utc).isoformat().replace("+00:00", "Z")

        # NPCs
        npc_states = physics.get_npc_states()
        traffic_data = []
        for npc in npc_states:
            speed = npc.get("speed_kmh", 0)
            is_oncoming = speed < 0
            rel_heading = npc.get("relative_heading_deg", 0.0)
            base_yaw = 180 if is_oncoming else 0
            traffic_data.append({
                "id": f"npc_{npc['id']}",
                "pose": {"x": npc.get("lane_offset", 0), "y": 0, "z": -npc.get("station_m", 0)},
                "yaw": base_yaw + rel_heading,
                "relative_heading_deg": rel_heading,
                "lateral_speed_mps": npc.get("lateral_speed_mps", 0.0),
                "velocity": abs(speed) / 3.6,
                "acceleration": 0,
                "frenet": {"s": npc.get("station_m", 0), "d": npc.get("lane_offset", 0)},
            })

        # Perception
        perception_data = []
        sensed_gap = nav_state.get("sensed_lead_gap_m")
        if sensed_gap is not None and sensed_gap < 100.0:
            lead_speed_mps = (nav_state.get("sensed_lead_speed_kmh") or 0.0) / 3.6
            ego_speed_mps = physics.speed_kmh / 3.6
            rel_v = lead_speed_mps - ego_speed_mps
            lead_station = nav_state.get("station_m", 0) + sensed_gap
            lead_lane = nav_state.get("lateral_offset_m", 0)

            perception_data.append({
                "id": "sensed_lead_vehicle",
                "type": "VEHICLE",
                "pose": {"x": lead_lane, "y": 0, "z": -lead_station},
                "yaw": 0,
                "velocity": lead_speed_mps,
                "acceleration": 0,
                "frenet": {"s": lead_station, "d": lead_lane},
                "distance": round(sensed_gap, 1),
                "rel_velocity": round(rel_v, 1),
                "confidence": 0.98,
            })

        # Trajectory corridor
        trajectory = []
        committed = self.driver.spatiotemporal_traj
        if committed is not None:
            from app.services.planner.spatiotemporal import sample_trajectory_state
            for i in range(21):
                t = self.driver._maneuver_t + i * .1
                if t > max(committed.t_lat_s, committed.t_lon_s):
                    break
                station, offset, _, _ = sample_trajectory_state(committed, t)
                trajectory.append({"x": offset, "y": .02,
                                   "z": -station, "time_offset_s": i * .1})

        planner_candidates = physics.get_planner_candidates()
        chosen_candidate = next((c for c in planner_candidates if c.get("is_chosen")), None)
        is_changing_lane = bool(chosen_candidate and chosen_candidate.get("is_lane_change"))
        signal_route_key = (self.epoch, self.route_version, id(physics.frenet_frame))
        if signal_route_key != self._signal_route_key:
            self._signal_turns = route_turns(physics.frenet_frame, self.route_steps)
            self._signal_route_key = signal_route_key

        ego_state = {
            "id": "ego_1",
            "geographic_position": {"lat": physics.lat, "lng": physics.lng},
            "pose": {"x": nav_state.get("lateral_offset_m", 0), "y": 0, "z": -nav_state.get("station_m", 0)},
            "world_pos": nav_state.get("world_pos", {"x": 0.0, "y": 0.0, "z": 0.0}),
            "yaw": nav_state.get("relative_heading_deg", 0.0),
            "world_heading_deg": nav_state.get("world_heading_deg", 0.0),
            "road_heading_deg": nav_state.get("road_heading_deg", 0.0),
            "relative_heading_deg": nav_state.get("relative_heading_deg", 0.0),
            "steering_angle_rad": nav_state.get("steering_angle_rad", 0.0),
            "wheels": nav_state.get("wheels"),
            "kinematic_sideslip_rad": nav_state.get("kinematic_sideslip_rad", 0.0),
            "lateral_accel_mps2": nav_state.get("lateral_accel_mps2", 0.0),
            "grip_budget": nav_state.get("grip_budget"),
            "control_perception": nav_state.get("control_perception"),
            "signals": vehicle_signals(physics, self.driver, self._signal_turns),
            "signed_lateral_accel_mps2": nav_state.get("signed_lateral_accel_mps2", 0.0),
            "yaw_rate_degps": nav_state.get("yaw_rate_degps", 0.0),
            "velocity": physics.speed_kmh / 3.6,
            "acceleration": getattr(physics, "acceleration_mps2", 0),
            "frenet": {"s": nav_state.get("station_m", 0), "d": nav_state.get("lateral_offset_m", 0)},
            "decision": self.latest_analytics.get("decision", "Unavailable"),
            "confidence": self.latest_analytics.get("confidence", 0.0),
            "sensor_health": {**asdict(self.driver.sensor_guard.status),
                              "faults": dict(physics.sensor_faults)},
            "camera_view_health": list(self.driver.sensor_guard.status.camera_view_health),
            "speed_limit_reason": nav_state.get("speed_limit_reason"),
            "target_velocity": self.driver.target_speed_mps,
            "sensed_lead_gap_m": nav_state.get("sensed_lead_gap_m"),
            "driving_state": physics.driving_state.value,
            "has_arrived": physics.has_arrived,
            "remaining_distance_m": max(0.0, physics._last_dist),
            "parking_status": (physics.observe_parking() or {}).get("status", "unavailable_no_verified_parking_map"),
            "parking": physics.observe_parking(),
            "parking_area": [list(p) for p in physics.parking_mission['polygon_latlng']] if physics.parking_mission else None,
            "junction": self.driver.junctions.status,
            "configured_cruise_speed_kmh": self.driver.cruise_speed_kmh,
            "road_semantics": {"source": "recorded OpenStreetMap tags", "posted_speed_limit_kmh": rules.speed_limit_kmh if rules else None,
                               "status": "mapped_rules_partial" if rules else "unknown_or_ambiguous",
                               "way_id": rules.way_id if rules else None,
                               "lane_count": rules.lane_count if rules else None,
                               "oneway": rules.oneway if rules else None,
                               "map_timestamp": rules.map_timestamp if rules else None,
                               "map_version": rules.map_version if rules else None,
                               "recorded_width_m": rules.recorded_width_m if rules else None,
                               "width_tag": rules.width_tag if rules else None,
                               "lane_width_verified": False, "parking_verified": False,
                               "recorded_control_count": recorded_map().recorded_control_count,
                               "traffic_control_behavior_verified": False,
                               "active_route_control_count": len(physics.route_controls),
                               "signal_phase_source": "deterministic_simulated_infrastructure",
                               "physical_autonomy_supported": False},

            "control_source": self.latest_command.source if self.latest_command else "unavailable",
            "requested_acceleration": self.latest_command.accel_mps2 if self.latest_command else None,
            "steering_angle": nav_state.get("steering_angle_rad", 0.0),
            "throttle": max(0.0, getattr(physics, "acceleration_mps2", 0) / 3.0),
            "brake": max(0.0, -getattr(physics, "acceleration_mps2", 0) / 4.5),
        }

        return {
            "protocol_version": "3.0",
            "simulation_id": self.session_id,
            "run_id": "live",
            "epoch": self.epoch,
            "route_version": self.route_version,
            "route_status": self.route_status,
            "tick": self.executor.clock.tick // 10,
            "simulation_time_s": self.executor.clock.sim_time_s,
            "timing": {
                "scheduling_lateness_ms": self.timing.scheduling_lateness_ms,
                "max_scheduling_lateness_ms": self.timing.max_scheduling_lateness_ms,
                "stages": self.executor.timing_summary(),
                "world_time_s": physics.clock.sim_time_s,
                "scenario_time_s": self.scenario_engine.elapsed_time_s,
                "sim_time_s": self.executor.clock.sim_time_s,
                "wall_time_s": round(self.timing.wall_time_s, 3),
                "real_time_factor": round(self.timing.real_time_factor, 3),
                "is_slowdown": self.timing.is_slowdown,
                "slowdown_factor": round(self.timing.slowdown_factor, 2),
            },
            "timestamp": iso_time,
            "coordinate_frame": "dds_world_v1",
            "units": "SI",
            "type": "state",
            "channels": {
                "pose": {
                    "ego": ego_state,
                },
                "semantic": {
                    "traffic": traffic_data,
                    "perception": perception_data,
                    "planner": {
                        "trajectory": trajectory,
                        "trajectory_source": "committed_polynomials" if committed is not None else "unavailable",
                        "source_tick": self.plan_tick,
                        "lateral_target_m": self.driver.lateral_target_d_m,
                        "lane_state": self.driver._lc_sm.state.value,
                        "semantic_origin_m": self.driver._lc_sm.origin_lane_d_m,
                        "semantic_adjacent_m": self.driver._lc_sm.adjacent_lane_d_m,
                        "required_destination_m": self.driver._lc_sm.required_destination_d_m,
                        "joint_plan_status": "unavailable" if self.driver._joint_plan_unavailable else "committed" if committed is not None else "lane_following",
                        "checked_horizon_s": getattr(committed, "checked_horizon_s", None),
                        "initial_speed_recovery": bool(getattr(committed, "initial_speed_recovery", False)),
                        "continuous_modeled_corridor_verified": bool(getattr(committed, "continuous_modeled_corridor_verified", False)),
                        "continuous_route_station_verified": bool(getattr(committed, "continuous_route_station_verified", False)),
                        "initial_actual_steering_checked": bool(getattr(committed, "initial_actual_steering_checked", False)),
                        "sampled_modeled_vehicle_envelope_verified": bool(getattr(committed, "sampled_modeled_vehicle_envelope_verified", False)),
                        "full_cartesian_swept_corridor_verified": bool(getattr(committed, "full_cartesian_swept_corridor_verified", False)),
                        "modeled_straight_vehicle_width_m": float(getattr(committed, "modeled_straight_vehicle_width_m", 2.18)),
                        "continuous_min_half_width_margin_m": float(getattr(committed, "continuous_min_half_width_margin_m", 1.09)),
                        "feasibility_friction_mu": getattr(committed, "feasibility_friction_mu", None),
                        "peak_combined_lateral_accel_mps2": getattr(committed, "peak_lat_accel_mps2", None),
                        "lookahead_point": trajectory[-1] if trajectory else {"x": 0, "y": 0, "z": 0},
                        "lane_center": nav_state.get("lateral_offset_m", 0),
                        "curvature": getattr(physics, "path_curvature", 0),
                        "candidates": planner_candidates,
                        "is_changing_lane": is_changing_lane,
                    },
                    "safety_shield": physics.get_safety_shield_state(),
                    "scenario": self.scenario_engine.get_state(),
                    "driver_analytics": {
                        "decision": self.latest_analytics.get("decision", "Unavailable"),
                        "confidence": self.latest_analytics.get("confidence", 0.0),
                        "shap": self.latest_analytics.get("shap", {}),
                        "anomaly": self.latest_analytics.get("anomaly", {}),
                        "driver_score": self.latest_analytics.get("driver_score", {}),
                    },
                },
                "heavy": {
                    "scene_controls": physics.scene_control_objects(),
                    "surround_perception": physics.get_surround_perception_state(),
                    "prediction": physics.get_prediction_state(),
                },
            },
        }

    # ------------------------------------------------------------------
    # Subscribers (WebSockets)
    # ------------------------------------------------------------------
    def road_context(self):
        route = self.world.physics.route
        if not route:
            return []
        if self.world.physics.parking_mission:
            mission = self.world.physics.parking_mission
            from bisect import bisect_left
            index = max(2, min(len(route)-1, bisect_left(self.world.physics.frenet_frame.station, mission['maneuver_start_m'])))
            # Road/aisle centreline ends where the off-aisle maneuver begins.
            # The remaining vehicle trajectory is not fabricated asphalt.
            return [list(p) for p in route[:index+1]]
        for name in ("highway", "wet_curve"):
            recorded = real_road_route(name)
            if tuple(route[0]) == recorded[0] and tuple(route[-1]) == recorded[-1]:
                return [list(p) for p in route] + [list(p) for p in real_road_continuation(name)]
        return []  # Unknown continuation is never extrapolated.

    def subscribe(self, subscriber_id: Optional[str] = None) -> Tuple[str, asyncio.Queue]:
        sub_id = subscriber_id or str(uuid.uuid4())
        q: asyncio.Queue = SubscriberQueue(maxsize=10)
        self._subscribers[sub_id] = q
        # Send initial route event if route exists
        if self.world.physics.route:
            q.put_nowait({
                "type": "route",
                "epoch": self.epoch, "route_version": self.route_version,
                "waypoints": [[lat, lng] for lat, lng in self.world.physics.route],
            "road_context": self.road_context(),
                "steps": self.route_steps,
            })
        return sub_id, q

    def unsubscribe(self, subscriber_id: str) -> None:
        self._subscribers.pop(subscriber_id, None)

    # ------------------------------------------------------------------
    # Background Workers: Analytics & Persistence
    # ------------------------------------------------------------------
    async def _analytics_worker(self) -> None:
        """Runs ML pipeline and driver scoring off the control path."""
        while self._running:
            acquired = False
            try:
                features, timestamp = await self.analytics_queue.get()
                acquired = True
                generation = getattr(self, "_run_generation", 0)
                if pipeline.is_ready():
                    result = await asyncio.to_thread(pipeline.predict, features)
                    if generation != getattr(self, "_run_generation", 0):
                        continue
                    decision = result["prediction"]
                    confidence = result["confidence"]
                    anomaly = result["anomaly_result"]
                    shap = result["shap_result"]
                    clean_input = result["clean_input"]

                    self.scorer.add_reading(clean_input, decision, confidence, anomaly)
                    score_data = self.scorer.calculate_score()

                    self.latest_analytics = {
                        "decision": decision,
                        "confidence": confidence,
                        "shap": shap,
                        "anomaly": anomaly,
                        "driver_score": score_data,
                    }
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in analytics worker: {e}", exc_info=True)
            finally:
                if acquired:
                    self.analytics_queue.task_done()

    async def _persistence_worker(self) -> None:
        """Batches and commits DB logs off the control path."""
        from app.core.database import persist_stream_batch

        pending_logs: List[TelemetryLog] = []
        pending_scores: List[DriverScoreLog] = []
        self.persistence_failed = False

        while self._running or not self.persistence_queue.empty():
            try:
                try:
                    item = await asyncio.wait_for(self.persistence_queue.get(), timeout=1.0)
                    session_id, current_time, features, analytics = item
                    decision = analytics.get("decision", "Unavailable")
                    confidence = analytics.get("confidence")
                    shap = analytics.get("shap", {})
                    anomaly = analytics.get("anomaly") or {}
                    score_data = analytics.get("driver_score") or {}

                    pending_logs.append(
                        TelemetryLog(
                            session_id=session_id,
                            timestamp=current_time,
                            features=features,
                            prediction=decision,
                            confidence=confidence,
                            shap_values=shap,
                            is_anomaly=anomaly.get("is_anomaly"),
                            anomaly_type=anomaly.get("type"),
                        )
                    )
                    if (isinstance(score_data.get('score'), (int, float)) and
                        not isinstance(score_data.get('score'), bool) and
                        math.isfinite(score_data['score']) and 0 <= score_data['score'] <= 100 and
                        isinstance(score_data.get('rating'), str) and score_data['rating']):
                        pending_scores.append(DriverScoreLog(
                            session_id=session_id,
                            timestamp=current_time,
                            score=score_data['score'],
                            rating=score_data['rating'],
                            breakdown=score_data.get("breakdown", {}),
                        ))
                    self.persistence_queue.task_done()
                except asyncio.TimeoutError:
                    pass

                if len(pending_logs) >= 10:
                    await persist_stream_batch(pending_logs, pending_scores)
                    pending_logs = []
                    pending_scores = []
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.persistence_failed = True
                pending_logs, pending_scores = [], []
                logger.error(f"Error in persistence worker: {e}", exc_info=True)

        if pending_logs or pending_scores:
            try:
                await persist_stream_batch(pending_logs, pending_scores)
            except Exception:
                self.persistence_failed = True
                logger.exception('Final persistence flush failed')

    # ------------------------------------------------------------------
    # Simulation Loop & Timing
    # ------------------------------------------------------------------
    async def run_loop(self) -> None:
        """Authoritative simulation loop running at 100 Hz scheduling clock."""
        self._running = True
        self._wall_start = time.perf_counter()
        self._sim_start = self.executor.clock.sim_time_s

        if self.analytics_queue is None:
            self.analytics_queue = asyncio.Queue(maxsize=10)
        if self.persistence_queue is None:
            self.persistence_queue = asyncio.Queue(maxsize=50)

        from app.core.database import create_stream_session, finalize_stream_session
        await create_stream_session(self.session_id)

        self._analytics_task = asyncio.create_task(self._analytics_worker())
        self._persistence_task = asyncio.create_task(self._persistence_worker())

        # Step in chunks of 0.02s (2 substeps of 100 Hz = 50 Hz control rate)
        substep_chunk = 2
        chunk_dt = 0.02

        try:
            while self._running:
                chunk_wall_start = time.perf_counter()

                if self.real_time and not self.is_paused:
                    expected = self._wall_start + self.executor.clock.sim_time_s - self._sim_start
                    lateness = max(0.0, (chunk_wall_start - expected) * 1000)
                    self.timing.scheduling_lateness_ms = lateness
                    self.timing.max_scheduling_lateness_ms = max(self.timing.max_scheduling_lateness_ms, lateness)
                    self.executor.record_external_timing('scheduler_lateness', lateness, chunk_dt * 1000)
                self.process_pending_commands()
                if not self.is_paused:
                    for _ in range(substep_chunk):
                        self.executor.step()

                    # Telemetry broadcast at 10 Hz
                    if self.executor.clock.is_rate_tick(10.0):
                        self._emit_telemetry_snapshot()
                else:
                    # Heartbeats keep paused telemetry distinguishable from a dead stream.
                    self._emit_telemetry_snapshot()
                    await asyncio.sleep(0.10)
                    self._wall_start += time.perf_counter() - chunk_wall_start
                    continue

                # Timing & Slowdown Calculation
                sim_elapsed = self.executor.clock.sim_time_s - self._sim_start
                wall_elapsed = time.perf_counter() - self._wall_start
                self.timing.sim_time_s = self.executor.clock.sim_time_s
                self.timing.wall_time_s = wall_elapsed

                if self.real_time:
                    # Pace against the run deadline, not a fresh relative sleep
                    # after every chunk. Timer overshoot must not accumulate as
                    # permanent drift; simulation dt remains fixed.
                    sleep_time = self._wall_start + sim_elapsed - time.perf_counter()

                    if wall_elapsed > 0.05:
                        rtf = sim_elapsed / max(wall_elapsed, 1e-6)
                        self.timing.real_time_factor = rtf
                        self.timing.is_slowdown = rtf < 0.95
                        self.timing.slowdown_factor = max(1.0, 1.0 / max(rtf, 1e-3))

                    if sleep_time > 0 and not self.is_paused:
                        await self._pace_until(self._wall_start + sim_elapsed)
                    else:
                        await asyncio.sleep(0)
                else:
                    self.timing.real_time_factor = sim_elapsed / max(wall_elapsed, 1e-6)
                    await asyncio.sleep(0)

        except asyncio.CancelledError:
            pass
        finally:
            self._running = False
            if self._analytics_task:
                self._analytics_task.cancel()
            # A cancelled SQLite commit can leave its worker connection in
            # flight while finalization opens another writer. Let the single
            # persistence worker finish its current transaction and flush;
            # its queue wait is bounded to one second after _running clears.
            for t in self._background_tasks:
                t.cancel()
            tasks = [t for t in [self._analytics_task, self._persistence_task, *self._background_tasks] if t]
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
            finalized = await finalize_stream_session(
                self.session_id, "persistence_failed" if getattr(self, 'persistence_failed', False) else "completed"
            )
            if finalized is False:
                self.persistence_failed = True

    @staticmethod
    def _wait_until(deadline: float) -> None:
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)

    async def _pace_until(self, deadline: float) -> None:
        if sys.platform == 'win32':
            # Recheck after worker dispatch; no busy spin or loop blocking.
            await asyncio.to_thread(self._wait_until, deadline)
        else:
            await asyncio.sleep(max(0., deadline - time.perf_counter()))

    def start(self) -> None:
        """Start the simulation loop in the current asyncio event loop."""
        if self._main_task is None or self._main_task.done():
            self._main_task = asyncio.create_task(self.run_loop())

    def stop(self) -> None:
        """Stop the simulation loop."""
        self._running = False
        if self._main_task and not self._main_task.done():
            self._main_task.cancel()


# Global Authoritative Simulation Session singleton
_GLOBAL_SESSION: Optional[SimulationSession] = None


def get_simulation_session() -> SimulationSession:
    """Retrieve or create the global authoritative SimulationSession."""
    global _GLOBAL_SESSION
    if _GLOBAL_SESSION is None:
        _GLOBAL_SESSION = SimulationSession(real_time=True)
    return _GLOBAL_SESSION


def reset_simulation_session(*, seed: Optional[int] = None, real_time: bool = True) -> SimulationSession:
    """Create a fresh SimulationSession (useful for tests or hard reloads)."""
    global _GLOBAL_SESSION
    if _GLOBAL_SESSION is not None:
        _GLOBAL_SESSION.stop()
    _GLOBAL_SESSION = SimulationSession(seed=seed, real_time=real_time)
    return _GLOBAL_SESSION
