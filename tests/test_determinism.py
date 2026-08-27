"""ADR-001 Phase 6.5, Action Item 8 -- the determinism gate (6.5.2).

Same seed + same scenario => bit-identical ego trajectory across two
independent runs. This is the test that proves the restructure did not
introduce hidden nondeterminism: fixed-step SimClock (item 4), seeded
powertrain RNG (item 4), and no wall-clock reads on the explicit-dt path.

Run on the explicit-dt path (physics.update(action, dt=...)); the legacy
wall-clock path in update() is retained for existing _tick() tests and is
intentionally out of scope for this gate.
"""
import pytest

from app.services.physics_engine import PhysicsEngine
from app.services.scenario_engine import ScenarioEngine

DT = 0.1
_STRAIGHT_ROUTE = [(37.7749 + i * 0.000045, -122.4194) for i in range(220)]


def _ego_sample(engine: PhysicsEngine):
    return (
        engine.lat,
        engine.lng,
        engine.heading,
        engine.speed_kmh,
        engine.current_station_m,
        engine.current_lateral_offset_m,
        engine.acceleration_mps2,
        engine.steering_angle_rad,
        engine.rpm,
        engine.co2,
        engine.altitude,
        engine.clock.tick,
    )


def _run_free_drive(seed: int, ticks: int = 300):
    engine = PhysicsEngine(seed=seed)
    engine.set_destination(*_STRAIGHT_ROUTE[-1])
    engine.set_route(_STRAIGHT_ROUTE)
    trace = []
    for _ in range(ticks):
        engine.update("Accelerate", dt=DT)
        trace.append(_ego_sample(engine))
    return trace


def _run_scenario(scenario_id: str, seed: int, ticks: int = 220):
    engine = PhysicsEngine(seed=seed)
    engine.set_route(_STRAIGHT_ROUTE)
    scen = ScenarioEngine()
    scen.load_scenario(scenario_id, engine, density="medium", initial_speed_kmh=45.0)
    trace = []
    for _ in range(ticks):
        scen.update(engine, DT)
        engine.update("Maintain Speed", dt=DT)
        trace.append(
            _ego_sample(engine)
            + (
                engine.shield_verdict.risk_level,
                engine.shield_verdict.override_action,
                engine.speed_limit_reason,
            )
        )
    return trace


def test_free_drive_is_bit_identical_across_runs():
    a = _run_free_drive(seed=1234)
    b = _run_free_drive(seed=1234)
    assert a == b, "same seed + same inputs must reproduce the ego trajectory exactly"


def test_free_drive_diverges_for_a_different_seed():
    a = _run_free_drive(seed=1234)
    c = _run_free_drive(seed=9999)
    assert a != c, "powertrain RNG must actually be seeded (different seed -> different run)"


@pytest.mark.parametrize(
    "scenario_id",
    ["normal_cruising", "traffic_overtake", "emergency_cut_in", "queue_stop_and_go"],
)
def test_scenario_replay_is_bit_identical(scenario_id):
    a = _run_scenario(scenario_id, seed=202)
    b = _run_scenario(scenario_id, seed=202)
    assert a == b, f"scenario '{scenario_id}' must replay bit-identically at a fixed seed"


def test_scenario_replay_covers_safety_shield_state():
    """The emergency cut-in drives the SafetyMonitor into an override; that
    verdict sequence must also be reproducible, not just the kinematics."""
    a = _run_scenario("emergency_cut_in", seed=202)
    b = _run_scenario("emergency_cut_in", seed=202)
    assert [row[12:] for row in a] == [row[12:] for row in b]
    # Sanity: the override actually fires somewhere in the run.
    assert any(row[13] is not None for row in a)
