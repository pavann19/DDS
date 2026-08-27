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

        self._register_built_in_scenarios()

    def _register_built_in_scenarios(self):
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
        }

    def load_scenario(
        self,
        scenario_id: str,
        physics: PhysicsEngine,
        density: Optional[str] = None,
        initial_speed_kmh: Optional[float] = None,
    ) -> dict:
        if scenario_id not in self.scenarios:
            raise ValueError(f"Unknown scenario ID: '{scenario_id}'. Available: {list(self.scenarios.keys())}")

        scenario = self.scenarios[scenario_id]
        self.active_scenario_id = scenario_id
        self.active_density = density or scenario.default_density
        self.active_initial_speed_kmh = (
            initial_speed_kmh if initial_speed_kmh is not None else scenario.default_initial_speed_kmh
        )
        self.tick_count = 0
        self.elapsed_time_s = 0.0
        self.is_paused = False
        self.events_log.clear()
        self._scenario_data = {
            "status": "active",
            "milestone": "Scenario Initialized",
            "flags": {},
        }

        # Apply setup function
        scenario.setup_fn(self, physics, self.active_density, self.active_initial_speed_kmh)

        # Notify physics engine of active scenario
        physics.is_paused = self.is_paused
        physics.active_scenario = self.get_state()

        event = self._create_event(
            event_type="SCENARIO_LOADED",
            cause=f"Scenario '{scenario.name}' loaded",
            actor="SYSTEM",
            decision=f"Initial speed {self.active_initial_speed_kmh} km/h, density {self.active_density}",
            metadata={"scenario_id": scenario_id, "seed": scenario.seed},
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

    def update(self, physics: PhysicsEngine, dt: float) -> Optional[dict]:
        """Called once per simulation tick (e.g. 10 Hz) when running."""
        if self.is_paused or not self.active_scenario_id:
            return None

        self.tick_count += 1
        self.elapsed_time_s += dt

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
    def _setup_normal_cruising(
        self,
        engine: "ScenarioEngine",
        physics: PhysicsEngine,
        density: str,
        initial_speed_kmh: Optional[float],
    ):
        physics.reset_state(
            station_m=0.0,
            speed_kmh=initial_speed_kmh or 45.0,
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


# Global singleton instance for easy import and sharing
scenario_engine = ScenarioEngine()
