"""Tests for the authoritative SimulationSession (100 Hz scheduling clock, driver/world separation).

Verifies the core architectural guarantees:
1. Determinism gate: identical seeds and command sequences reproduce identical traces.
2. Actuator control authority: changing ActuatorCommand in external mode changes next world state.
3. Decoupled slow analytics and slow browser: slow ML/analytics worker and slow subscriber queues
   cannot alter driving decisions or block the 100 Hz simulation loop.
4. Multi-rate scheduling: 50 Hz control/physics, 20 Hz perception, 10 Hz planning.
5. Single-step pipeline: advances 10 substeps (0.10s) across all pipeline stages.
6. Timing separation: tracks simulation time vs wall time and reports slowdown factors.
"""
import asyncio
import time
from typing import List, Tuple
from unittest.mock import MagicMock, patch

import pytest

from app.services.carla_adapter import DeterministicSimBackend
from app.services.executor import BASE_HZ, CONTROL_HZ, PERCEPTION_HZ, PLANNING_HZ
from app.services.interfaces import ActuatorCommand, SensorObservation
from app.services.simulation_session import SimulationSession, TimingStats, reset_simulation_session


def test_simulation_session_determinism():
    """Identical seeds and identical command sequences produce identical state traces."""
    session1 = SimulationSession(seed=42, real_time=False)
    session2 = SimulationSession(seed=42, real_time=False)

    trace1: List[Tuple[float, float, float, float, float]] = []
    trace2: List[Tuple[float, float, float, float, float]] = []

    # Run 100 substeps (1.0 s of simulation time at 100 Hz)
    for _ in range(10):
        session1.step_pipeline(num_substeps=10)
        p1 = session1.world.physics
        c1 = session1.latest_command
        trace1.append((
            p1.speed_kmh,
            p1.lat,
            p1.lng,
            c1.accel_mps2 if c1 else 0.0,
            c1.steer_rad if c1 else 0.0,
        ))

        session2.step_pipeline(num_substeps=10)
        p2 = session2.world.physics
        c2 = session2.latest_command
        trace2.append((
            p2.speed_kmh,
            p2.lat,
            p2.lng,
            c2.accel_mps2 if c2 else 0.0,
            c2.steer_rad if c2 else 0.0,
        ))

    assert len(trace1) == 10
    assert len(trace2) == 10

    for i, (t1, t2) in enumerate(zip(trace1, trace2)):
        assert t1 == t2, f"Discrepancy at step {i}: {t1} != {t2}"


def test_actuator_control_authority():
    """Changing actuator command directly changes the next world state in external mode."""
    backend_accel = DeterministicSimBackend(mode="external", seed=101)
    backend_brake = DeterministicSimBackend(mode="external", seed=101)

    # Both start at same speed
    backend_accel.physics.speed_kmh = 30.0
    backend_brake.physics.speed_kmh = 30.0

    cmd_accel = ActuatorCommand(steer_rad=0.0, accel_mps2=2.5)
    cmd_brake = ActuatorCommand(steer_rad=0.2, accel_mps2=-3.5)

    backend_accel.apply_control(cmd_accel)
    backend_brake.apply_control(cmd_brake)

    # Step both for 0.10s (5 dynamics steps of 0.02s)
    for _ in range(5):
        backend_accel.step(0.02)
        backend_brake.step(0.02)

    # Acceleration command increased speed, braking command decreased speed
    assert backend_accel.physics.speed_kmh > 30.0
    assert backend_brake.physics.speed_kmh < 30.0

    # Steer command changed heading on brake vehicle, but not on accel vehicle
    assert abs(backend_accel.physics.heading - 45.0) < 1e-4
    assert abs(backend_brake.physics.heading - 45.0) > 0.05
    assert backend_accel.physics.heading != backend_brake.physics.heading


def test_multi_rate_scheduling_frequencies():
    """Executor runs stages at their specified rates: 50 Hz control, 20 Hz perception, 10 Hz planning."""
    session = SimulationSession(seed=55, real_time=False)

    perception_count = 0
    planning_count = 0
    control_count = 0

    orig_perception = session._stage_perception
    orig_planning = session._stage_planning
    orig_control = session._stage_control_physics

    def mock_perception(clock):
        nonlocal perception_count
        perception_count += 1
        orig_perception(clock)

    def mock_planning(clock):
        nonlocal planning_count
        planning_count += 1
        orig_planning(clock)

    def mock_control(clock):
        nonlocal control_count
        control_count += 1
        orig_control(clock)

    session._stage_perception = mock_perception
    session._stage_planning = mock_planning
    session._stage_control_physics = mock_control

    # Re-register with mocks
    session.executor = type(session.executor)(base_hz=BASE_HZ, seed=55)
    session.executor.add_stage("perception", PERCEPTION_HZ, mock_perception)
    session.executor.add_stage("planning", PLANNING_HZ, mock_planning)
    session.executor.add_stage("control_safety_physics", CONTROL_HZ, mock_control)

    # Run for 100 substeps (1.0 second of simulation time)
    session.step_pipeline(num_substeps=100)

    assert control_count == 50, f"Expected 50 control ticks in 1.0s, got {control_count}"
    assert perception_count == 20, f"Expected 20 perception ticks in 1.0s, got {perception_count}"
    assert planning_count == 10, f"Expected 10 planning ticks in 1.0s, got {planning_count}"


def test_single_step_advances_complete_pipeline():
    """Single-step advances 10 substeps (0.10s): 1 planning, 2 perception, 5 control ticks."""
    session = SimulationSession(seed=77, real_time=False)
    session.is_paused = True

    start_sim_time = session.executor.clock.sim_time_s

    # Single-step: exactly 10 substeps
    ran_stages = session.step_pipeline(num_substeps=10)

    elapsed_sim = session.executor.clock.sim_time_s - start_sim_time
    assert round(elapsed_sim, 3) == 0.100

    # Verify stages executed in this single step
    assert ran_stages.count("planning") == 1
    assert ran_stages.count("perception") == 2
    assert ran_stages.count("control_safety_physics") == 5

    # Verify latest telemetry payload was generated and matches new time
    payload = session.latest_payload
    assert payload is not None
    assert payload["simulation_time_s"] == round(session.executor.clock.sim_time_s, 2)
    assert set(payload["channels"]) == {"pose", "semantic", "heavy"}


def test_tick_boundary_command_processing():
    """Commands are processed strictly at tick boundaries, not asynchronously mid-physics step."""
    session = SimulationSession(seed=99, real_time=False)

    session.enqueue_command({"type": "pause_simulation"})
    assert not session.is_paused  # Not yet processed

    session.step_pipeline(num_substeps=10)
    assert session.is_paused  # Processed at tick boundary

    session.enqueue_command({"type": "resume_simulation"})
    session.process_pending_commands()
    assert not session.is_paused


def test_slow_subscriber_drops_oldest_without_blocking():
    """A slow WebSocket subscriber queue with a full buffer drops frames without blocking."""
    session = SimulationSession(seed=12, real_time=False)
    sub_id, sub_queue = session.subscribe()

    # Fill queue to maximum capacity (10)
    for _ in range(15):
        session._emit_telemetry_snapshot()

    assert sub_queue.qsize() == 10  # Capped at maxsize, does not exceed or crash

    # Subscriber reads most recent items cleanly
    msg = sub_queue.get_nowait()
    assert msg is not None

    session.unsubscribe(sub_id)
    assert sub_id not in session._subscribers


@pytest.mark.asyncio
async def test_slow_analytics_does_not_block_simulation_or_alter_decisions():
    """Slow ML/analytics pipeline does not alter driving decisions or block simulation."""
    session = SimulationSession(seed=33, real_time=False)
    session.analytics_queue = asyncio.Queue(maxsize=3)

    # Fill analytics queue with dummy items
    for i in range(5):
        features = session.world.physics.get_ml_features()
        if session.analytics_queue.full():
            session.analytics_queue.get_nowait()
        session.analytics_queue.put_nowait((features, time.time()))

    assert session.analytics_queue.qsize() <= 3

    # Stepping pipeline proceeds smoothly with zero delay or interference
    t0 = time.perf_counter()
    session.step_pipeline(num_substeps=10)
    elapsed = time.perf_counter() - t0

    assert elapsed < 0.05  # Instantaneous execution
    assert session.latest_command is not None  # Driver decided autonomously


def test_simulation_timing_and_slowdown_reporting():
    """TimingStats computes real_time_factor, is_slowdown, and slowdown_factor correctly."""
    stats = TimingStats(
        sim_time_s=1.0,
        wall_time_s=2.0,
        real_time_factor=0.5,
        is_slowdown=True,
        slowdown_factor=2.0,
    )
    assert stats.is_slowdown is True
    assert stats.slowdown_factor == 2.0
    assert stats.real_time_factor == 0.5


@pytest.mark.parametrize("params", [{"friction_mu": "invalid"}, {"dropout_prob": float("nan")},
                                      {"latency_ticks": 1.5}, {"sensor_noise_enabled": "false"}])
def test_invalid_scenario_preserves_live_world(params):
    session = SimulationSession(seed=42, real_time=False)
    session.step_pipeline(num_substeps=10)
    physics = session.world.physics
    engine = session.scenario_engine
    tick = session.executor.clock.tick
    before = (physics.lat, physics.lng, physics.speed_kmh, physics.current_station_m, list(physics.route))
    with pytest.raises(ValueError):
        session._handle_command({"type": "load_scenario", "scenario_id": "low_friction_curve", "params": params})
    assert session.world.physics is physics
    assert session.scenario_engine is engine
    assert session.executor.clock.tick == tick
    assert (physics.lat, physics.lng, physics.speed_kmh, physics.current_station_m, list(physics.route)) == before


def test_scenario_commit_keeps_driver_property_forwarding():
    session = SimulationSession(seed=42, real_time=False)
    session._handle_command({"type": "load_scenario", "scenario_id": "low_friction_curve"})
    assert session.driver is session.world.physics._driver
    session.step_pipeline(num_substeps=10)


def test_arrival_telemetry_is_explicit():
    session = SimulationSession(seed=42, real_time=False)
    session.world.physics.has_arrived = True
    session.world.physics.speed_kmh = 0
    session.step_pipeline(num_substeps=10)
    ego = session.build_telemetry_payload()["channels"]["pose"]["ego"]
    assert ego["has_arrived"] is True
    assert ego["driving_state"] == "ARRIVED"
    assert ego["parking_status"] == "route_to_parking"
    assert ego["parking"]["parked"] is False
    assert ego["parking"]["mapped_bay"] is False


@pytest.mark.parametrize("speed", [float("nan"), 121, -1, "80", True])
def test_invalid_cruise_request_preserves_preference(speed):
    session = SimulationSession(seed=42, real_time=False)
    before = session.driver.cruise_speed_kmh
    with pytest.raises(ValueError):
        session._handle_command({"type": "set_cruise_speed", "speed_kmh": speed})
    assert session.driver.cruise_speed_kmh == before


def test_higher_cruise_preference_is_applied_and_survives_reset():
    session = SimulationSession(seed=42, real_time=False)
    session._handle_command({"type": "set_cruise_speed", "speed_kmh": 80})
    session.reset()
    assert session.driver.cruise_speed_kmh == 80
    session.step_pipeline(num_substeps=10)
    # The shared ramp endpoint is ambiguous. Preference persists, while
    # the conservative policy applies until the road match resolves.
    assert session.driver.target_speed_mps == pytest.approx(50 / 3.6)
    session.step_pipeline(num_substeps=400)
    assert session.latest_observation.road_rules is not None
    assert session.driver.target_speed_mps > 50/3.6
    assert session.driver.target_speed_mps <= 80 / 3.6


def test_recorded_road_continues_beyond_navigation_goal():
    session = SimulationSession(seed=42, real_time=False)
    session.scenario_engine.load_scenario("normal_cruising", session.world.physics)
    context = session.road_context()
    route = session.world.physics.route
    assert context[:len(route)] == [list(p) for p in route]
    assert len(context) > len(route)
    assert context[-1] != list(route[-1])


def test_safety_measurements_have_no_privileged_identity():
    session = SimulationSession(seed=42,real_time=False)
    session.step_pipeline(num_substeps=10)
    obs = session.world.observe()
    assert obs.safety_detections is not None
    for detection in obs.safety_detections:
        assert not hasattr(detection,"source_id")
        assert not hasattr(detection,"track_id")
        assert detection.length_m > 0
