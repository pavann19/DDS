"""Unit tests for the multi-rate deterministic executor (ADR-001 item 4).

Covers the two properties the ADR relies on:
* stages fire at their declared rate off fixed-step sim time, and
* the schedule is bit-identical run to run (no wall clock, no hidden RNG).
Also covers the seeded-RNG path added to PhysicsEngine for the determinism
gate; the full bit-identical-trajectory test is Action Item 8.
"""
import pytest

from app.services.executor import (
    BASE_HZ,
    CONTROL_HZ,
    PERCEPTION_HZ,
    PLANNING_HZ,
    MultiRateExecutor,
)
from app.services.physics_engine import PhysicsEngine


def _record_schedule(seconds=1.0, seed=None):
    ex = MultiRateExecutor(seed=seed)
    log = []
    for hz, name in ((PERCEPTION_HZ, "perception"), (PLANNING_HZ, "planning"), (CONTROL_HZ, "control")):
        ex.add_stage(name, hz, (lambda n: lambda clk: log.append((clk.tick, n)))(name))
    substeps = ex.run_for(seconds)
    return substeps, log, ex.clock


def test_stage_counts_match_declared_rates_over_one_second():
    substeps, log, clock = _record_schedule(1.0)
    assert substeps == 100
    assert clock.tick == 100
    assert clock.sim_time_s == pytest.approx(1.0)
    counts = {}
    for _tick, name in log:
        counts[name] = counts.get(name, 0) + 1
    assert counts == {"perception": 20, "planning": 10, "control": 50}


def test_schedule_is_bit_identical_across_runs():
    _, log_a, _ = _record_schedule(2.0, seed=1)
    _, log_b, _ = _record_schedule(2.0, seed=2)  # different seed, same schedule
    assert log_a == log_b


def test_rate_that_does_not_divide_base_is_rejected():
    ex = MultiRateExecutor()
    # 100 / 30 is not an integer -> not representable on the base grid.
    with pytest.raises(ValueError):
        ex.add_stage("bad", 30.0, lambda clk: None)


def test_planning_and_control_align_on_shared_ticks():
    """A 10 Hz stage tick must always coincide with a 50 Hz stage tick
    (10 divides 50) -- needed so the controller always has a fresh plan."""
    _, log, _ = _record_schedule(1.0)
    control_ticks = {t for t, n in log if n == "control"}
    planning_ticks = {t for t, n in log if n == "planning"}
    assert planning_ticks <= control_ticks


def test_base_hz_is_one_hundred():
    assert BASE_HZ == 100.0


# --- PhysicsEngine seeded-RNG determinism (partial; full gate = item 8) ---

def _run_seeded(seed, ticks=120):
    eng = PhysicsEngine(seed=seed)
    eng.set_destination(eng.lat + 0.02, eng.lng + 0.02)
    trace = []
    for _ in range(ticks):
        eng.update("Maintain Speed", dt=0.1)
        trace.append((eng.lat, eng.lng, eng.speed_kmh, eng.rpm, eng.altitude))
    return trace, eng


def test_same_seed_same_dt_reproduces_trajectory_bitwise():
    a, _ = _run_seeded(7)
    b, _ = _run_seeded(7)
    assert a == b


def test_different_seed_diverges_on_powertrain_noise():
    a, _ = _run_seeded(7)
    c, _ = _run_seeded(8)
    assert a != c


def test_seeded_engine_advances_deterministic_sim_clock():
    _, eng = _run_seeded(7, ticks=50)
    # 50 updates * (0.1 / 0.02 = 5) substeps.
    assert eng.clock.tick == 250
    assert eng.clock.sim_time_s == pytest.approx(5.0)
