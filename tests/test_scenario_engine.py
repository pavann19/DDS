"""
Tests for Scenario Engine (Phase 5 of DDS V2 Roadmap).

Verifies:
1. Deterministic scenario registry and metadata schemas.
2. Exact reproducibility across multiple runs with fixed seeds.
3. Normal cruising scenario maintains speed and lane-centring with zero overrides.
4. Traffic overtake scenario: slow lead vehicle triggers planner lane-change selection.
5. Emergency cut-in scenario: aggressive intrusion triggers Safety Shield EMERGENCY_BRAKE.
6. Queue stop-and-go scenario: IDM brings vehicle to standstill and smoothly resumes.
7. Pause, resume, and reset controls.
8. REST API /api/scenarios endpoint.
9. WebSocket scenario command processing.
"""
from importlib.metadata import version

import pytest
import asyncio
from fastapi.testclient import TestClient

from app.main import app
from app.services.physics_engine import PhysicsEngine
from app.services.scenario_engine import ScenarioEngine, scenario_engine
from app.services.traffic import EGO_LANE_OFFSET_M, ADJACENT_LANE_OFFSET_M
from app.services.safety_shield import RISK_CRITICAL, OVERRIDE_EMERGENCY_BRAKE


def _starlette_testclient_ws_portal_hangs() -> bool:
    try:
        return int(version("starlette").split(".")[0]) >= 1
    except Exception:
        return False


def _create_test_route_physics():
    """Helper to initialize physics engine with a synthetic 1 km straight route."""
    physics = PhysicsEngine()
    # Synthetic route of 200 points spaced 5m apart along latitude (approx 1000m)
    waypoints = [(37.7749 + i * 0.000045, -122.4194) for i in range(200)]
    physics.set_route(waypoints)
    return physics


def test_scenario_registry_and_metadata():
    engine = ScenarioEngine()
    scenarios = engine.list_scenarios()
    scenario_ids = [s["id"] for s in scenarios]

    expected_ids = ["normal_cruising", "traffic_overtake", "emergency_cut_in", "queue_stop_and_go"]
    for sid in expected_ids:
        assert sid in scenario_ids, f"Scenario '{sid}' missing from registry"

    for s in scenarios:
        assert s["id"]
        assert s["name"]
        assert s["category"] in ["normal", "traffic", "maneuver", "safety_critical"]
        assert s["description"]
        assert isinstance(s["seed"], int)
        assert s["default_initial_speed_kmh"] > 0
        assert s["default_density"] in ["low", "medium", "high"]


def test_scenario_deterministic_reproducibility():
    """Two independent setups with identical seed must yield identical initial NPC states."""
    engine_1 = ScenarioEngine()
    physics_1 = _create_test_route_physics()
    engine_1.load_scenario("traffic_overtake", physics_1, density="medium", initial_speed_kmh=40.0)

    engine_2 = ScenarioEngine()
    physics_2 = _create_test_route_physics()
    engine_2.load_scenario("traffic_overtake", physics_2, density="medium", initial_speed_kmh=40.0)

    npcs_1 = physics_1.get_npc_states()
    npcs_2 = physics_2.get_npc_states()

    assert len(npcs_1) == len(npcs_2)
    for n1, n2 in zip(npcs_1, npcs_2):
        assert n1["id"] == n2["id"]
        assert pytest.approx(n1["station_m"], rel=1e-4) == n2["station_m"]
        assert pytest.approx(n1["lane_offset"], rel=1e-4) == n2["lane_offset"]
        assert pytest.approx(n1["speed_kmh"], rel=1e-4) == n2["speed_kmh"]


def test_normal_cruising_scenario():
    """Normal cruising scenario maintains target cruise speed and lane centring with zero shield interventions."""
    engine = ScenarioEngine()
    physics = _create_test_route_physics()
    engine.load_scenario("normal_cruising", physics, initial_speed_kmh=45.0)

    # Run for 35 ticks (simulating 3.5s)
    events = []
    for _ in range(35):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)

    # Verify no safety shield overrides
    shield = physics.shield_verdict
    assert shield.approved is True
    assert shield.risk_level != RISK_CRITICAL
    assert shield.override_action is None

    # Verify vehicle is cruising stably near lane center (1.75m)
    assert abs(physics.current_lateral_offset_m - EGO_LANE_OFFSET_M) < 0.6
    assert physics.speed_kmh > 35.0

    # Verify milestone event was emitted
    event_types = [e["event"]["type"] for e in events]
    assert "CRUISING_STABLE" in event_types


def test_traffic_overtake_scenario_triggers_lane_change():
    """Approaching slow lead vehicle triggers candidate planner blocked lane and selects adjacent lane."""
    engine = ScenarioEngine()
    physics = _create_test_route_physics()
    engine.load_scenario("traffic_overtake", physics, initial_speed_kmh=42.0)

    lane_change_initiated = False
    events = []

    # Run simulation for up to 60 ticks
    for _ in range(60):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)

        physics.update("Maintain Speed", dt=0.1)

        candidates = physics.get_planner_candidates()
        chosen = next((c for c in candidates if c["is_chosen"]), None)
        if chosen and chosen.get("is_lane_change"):
            lane_change_initiated = True
            break

    assert lane_change_initiated, "Candidate planner did not trigger a lane-change candidate"

    # Advance more ticks to verify lateral transition towards adjacent lane (5.25m)
    for _ in range(40):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)

    assert physics.current_lateral_offset_m > 3.0, (
        f"Lateral offset did not shift toward adjacent lane: {physics.current_lateral_offset_m}m"
    )
    event_types = [e["event"]["type"] for e in events]
    assert "LANE_CHANGE_INITIATED" in event_types


def test_emergency_cut_in_scenario_triggers_safety_shield():
    """Cut-in vehicle abruptly entering ego lane forces Safety Shield EMERGENCY_BRAKE override."""
    engine = ScenarioEngine()
    physics = _create_test_route_physics()
    engine.load_scenario("emergency_cut_in", physics, initial_speed_kmh=48.0)

    shield_engaged = False
    events = []

    # Run 30 ticks
    for _ in range(30):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)

        shield = physics.shield_verdict
        if shield.risk_level == RISK_CRITICAL and shield.override_action == OVERRIDE_EMERGENCY_BRAKE:
            shield_engaged = True
            break

    assert shield_engaged, "Safety Shield did not engage emergency brake on critical cut-in"

    # Advance 15 more ticks to allow jerk-limited deceleration ramp down to emergency brake limit
    for _ in range(15):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)

    assert physics.acceleration_mps2 < -3.0, f"Braking acceleration not applied: {physics.acceleration_mps2}"

    # Verify no collision occurred (ego station strictly behind lead car)
    lead_npc = next((n for n in physics.traffic.npcs if n.id == "npc-cut-in"), None)
    assert lead_npc is not None
    assert physics.current_station_m < lead_npc.station_m, "Ego collided with cut-in vehicle"

    event_types = [e["event"]["type"] for e in events]
    assert "VEHICLE_CUT_IN" in event_types


def test_queue_stop_and_go_scenario():
    """IDM car-following stops vehicle behind queue and resumes when queue moves."""
    engine = ScenarioEngine()
    physics = _create_test_route_physics()
    engine.load_scenario("queue_stop_and_go", physics, initial_speed_kmh=35.0)

    stopped = False
    events = []

    # Run up to 95 ticks to reach standstill behind queue
    for _ in range(95):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)
        if physics.speed_kmh < 1.0:
            stopped = True
            break

    assert stopped, f"Ego did not decelerate to standstill behind queue: speed={physics.speed_kmh} km/h"
    lead_npc = next((n for n in physics.traffic.npcs if n.id == "npc-queue-1"), None)
    assert lead_npc is not None
    gap = lead_npc.station_m - physics.current_station_m
    assert gap > 1.5, f"Ego violated safe standoff buffer: gap={gap}m"

    # Run past tick 95 to observe queue movement and resumption
    for _ in range(35):
        evt = engine.update(physics, 0.1)
        if evt:
            events.append(evt)
        physics.update("Maintain Speed", dt=0.1)

    event_types = [e["event"]["type"] for e in events]
    assert "QUEUE_RESUMED" in event_types


def test_simulation_pause_resume_reset():
    """Pause freezes physical motion; resume continues; reset restores initial state."""
    engine = ScenarioEngine()
    physics = _create_test_route_physics()
    engine.load_scenario("normal_cruising", physics, initial_speed_kmh=45.0)

    # Initial station
    s0 = physics.current_station_m
    assert s0 == 0.0

    # Run 5 ticks
    for _ in range(5):
        engine.update(physics, 0.1)
        physics.update("Maintain Speed", dt=0.1)

    s1 = physics.current_station_m
    assert s1 > s0

    # Pause simulation
    engine.pause(physics)
    assert engine.is_paused is True
    assert physics.is_paused is True

    # Try advancing 5 ticks while paused
    for _ in range(5):
        engine.update(physics, 0.1)
        physics.update("Maintain Speed", dt=0.1)

    # Station should not have moved while paused
    assert physics.current_station_m == s1

    # Resume simulation
    engine.resume(physics)
    assert engine.is_paused is False
    assert physics.is_paused is False

    for _ in range(5):
        engine.update(physics, 0.1)
        physics.update("Maintain Speed", dt=0.1)

    assert physics.current_station_m > s1

    # Reset simulation
    engine.reset(physics)
    assert physics.current_station_m == 0.0
    assert engine.tick_count == 0


def test_rest_api_scenarios_endpoint():
    """GET /api/scenarios returns 200 OK and valid scenario definitions."""
    client = TestClient(app)
    resp = client.get("/api/scenarios")
    assert resp.status_code == 200
    data = resp.json()
    assert isinstance(data, list)
    assert len(data) >= 4
    ids = [item["id"] for item in data]
    assert "normal_cruising" in ids
    assert "traffic_overtake" in ids
    assert "emergency_cut_in" in ids
    assert "queue_stop_and_go" in ids


@pytest.mark.skipif(
    _starlette_testclient_ws_portal_hangs(),
    reason="Starlette 1.x TestClient websocket portal teardown deadlocks (harness bug, not app)",
)
def test_websocket_scenario_commands(stub_inference):
    """WebSocket handles load_scenario, pause_simulation, and reset_simulation commands.
    ML pipeline stubbed (see conftest.stub_inference) -- this checks command
    handling + protocol v3 payload, not the classifier."""
    client = TestClient(app)
    with client.websocket_connect("/ws/telemetry") as ws:
        # Receive initial route or state
        msg = ws.receive_json()
        assert msg["type"] in ["route", "state"]

        # Send load_scenario command
        ws.send_json({
            "type": "load_scenario",
            "scenario_id": "emergency_cut_in",
            "traffic_density": "medium",
            "initial_speed_kmh": 48.0
        })

        # Expect scenario loaded event or state with scenario
        scenario_confirmed = False
        for _ in range(15):
            reply = ws.receive_json()
            if reply.get("type") == "event" and reply["event"]["type"] == "SCENARIO_LOADED":
                assert reply["event"]["metadata"]["scenario_id"] == "emergency_cut_in"
                scenario_confirmed = True
                break
            if reply.get("type") == "state" and reply.get("channels", {}).get("semantic", {}).get("scenario", {}).get("id") == "emergency_cut_in":
                scenario_confirmed = True
                break

        assert scenario_confirmed, "WebSocket did not confirm scenario load"

        # Send pause_simulation
        ws.send_json({"type": "pause_simulation"})
        # Read next state to confirm is_paused
        paused_confirmed = False
        for _ in range(10):
            reply = ws.receive_json()
            if reply.get("type") == "state":
                sc = reply.get("channels", {}).get("semantic", {}).get("scenario", {})
                if sc.get("is_paused") is True:
                    paused_confirmed = True
                    break
        assert paused_confirmed, "WebSocket state did not reflect is_paused=True"

        # Send resume_simulation
        ws.send_json({"type": "resume_simulation"})
        resumed_confirmed = False
        for _ in range(10):
            reply = ws.receive_json()
            if reply.get("type") == "state":
                sc = reply.get("channels", {}).get("semantic", {}).get("scenario", {})
                if sc.get("is_paused") is False:
                    resumed_confirmed = True
                    break
        assert resumed_confirmed, "WebSocket state did not reflect is_paused=False"
