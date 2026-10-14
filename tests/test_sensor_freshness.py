"""Faults are injected into the actual simulator producer; no fake fresh feed."""
from dataclasses import replace
import math

import pytest

from app.services.driver.sensor_freshness import SensorFreshnessGuard
from app.services.interfaces import SensorHealth, SimClock
from app.services.physics_engine import PhysicsEngine
from app.services.simulation_session import SimulationSession


def fresh_guard():
    g = SensorFreshnessGuard()
    for t in (0., .05, .1):
        g.evaluate(t, SensorHealth(t, t))
    assert g.status.ready
    return g


def test_distinct_complete_samples_required_for_recovery():
    g = fresh_guard()
    assert not g.evaluate(.3, SensorHealth(.1, .1)).ready
    assert g.status.reason == 'sensor_stale'
    for t in (.30, .31, .32):
        assert not g.evaluate(t, SensorHealth(t, .30)).ready
    assert g.status.recovery_samples == 1
    assert not g.evaluate(.35, SensorHealth(.35, .35)).ready
    assert g.evaluate(.40, SensorHealth(.40, .40)).ready


def test_fast_feed_catching_up_does_not_count_same_surround_sample_twice():
    g = SensorFreshnessGuard()
    assert not g.evaluate(.30, SensorHealth(.29, .30)).ready
    assert not g.evaluate(.32, SensorHealth(.31, .30)).ready
    assert g.status.recovery_samples == 1
    assert not g.evaluate(.35, SensorHealth(.34, .35)).ready
    assert not g.evaluate(.37, SensorHealth(.36, .35)).ready
    assert g.status.recovery_samples == 2
    assert g.evaluate(.40, SensorHealth(.39, .40)).ready


@pytest.mark.parametrize('stamp', [math.nan, math.inf, -.5, 1.1, None, 'invalid', True])
def test_invalid_or_missing_source_never_appears_fresh(stamp):
    g = fresh_guard()
    assert not g.evaluate(1., SensorHealth(1., stamp)).ready
    assert g.status.reason != 'sensor_ready'


def test_backwards_capture_rejected_and_clock_reset_requires_recovery():
    g = fresh_guard()
    assert not g.evaluate(.12, SensorHealth(.12, .09)).ready
    assert g.status.reason == 'sensor_timestamp_invalid'
    assert not g.evaluate(0., SensorHealth(0., 0.)).ready
    assert g.status.recovery_samples == 1


def test_unmonitored_legacy_adapter_is_explicit_not_healthy():
    s = SensorFreshnessGuard().evaluate(0., None)
    assert not s.monitored and not s.ready
    assert s.reason == 'sensor_unmonitored'
    g = fresh_guard()
    dropped = g.evaluate(.12, None)
    assert dropped.monitored and not dropped.ready
    assert dropped.reason == 'sensor_missing'


def test_initial_and_reset_missing_measurement_has_infinite_age():
    p = PhysicsEngine(seed=42)
    assert p.observe().safety_measurement_age_s == math.inf
    assert p.observe().sensor_health.surround_capture_time_s is None
    p._safety_measurement_time_s = 12.
    p.reset_state()
    assert p.observe().sensor_health.surround_capture_time_s is None
    assert p.observe().safety_measurement_age_s == math.inf


def test_lane_sensor_exception_fails_closed(monkeypatch):
    p = PhysicsEngine(seed=42)
    p.set_route([(37., -122.), (37.01, -122.)])
    def broken(*_a, **_kw):
        raise RuntimeError('synthetic sensor producer failure')
    p.step_world_sensors(.02)
    monkeypatch.setattr(p.traffic, 'sense_lane_clear', broken)
    assert not p.observe().adjacent_lane_clear
    assert not p.build_driver_context(.1).lane_clear_fn(0., 5.25)


def session_with_fault(start=1., duration=4.):
    s = SimulationSession(seed=42, real_time=False)
    s.scenario_engine.load_scenario('normal_cruising', s.world.physics,
        params={'sensor_outage_start_s': start, 'sensor_outage_duration_s': duration})
    return s


def test_real_pipeline_fault_preserves_producer_time_and_stops_then_recovers():
    s = session_with_fault()
    p = s.world.physics
    rows = []
    for _ in range(70):
        s.step_pipeline(10)
        rows.append((s.executor.clock.sim_time_s, p.speed_kmh, p.acceleration_mps2,
                     s.driver.target_speed_mps, s.driver.speed_limit_reason,
                     s.driver.sensor_guard.status, p._safety_measurement_time_s))
    stopped = [r for r in rows if 1.3 < r[0] < 5.]
    assert stopped and all(not r[5].ready for r in stopped)
    assert all(r[3] == 0. for r in stopped)
    assert all(r[2] <= 0. for r in stopped)
    assert len({r[6] for r in stopped}) == 1
    assert min(r[1] for r in stopped) < rows[8][1]
    assert any(r[5].ready for r in rows if r[0] > 5.3)
    assert all(abs(b[2]-a[2])/.1 <= 2.5 + 1e-8 for a,b in zip(rows, rows[1:]))


def test_fresh_empty_detection_set_is_ready_and_paused_time_does_not_age():
    s = session_with_fault(start=100.)
    p = s.world.physics
    p.traffic.npcs = []
    for _ in range(5):
        s.step_pipeline(10)
    obs = p.observe()
    assert obs.safety_detections == ()
    assert s.driver.sensor_guard.status.ready
    for _ in range(20):
        assert s.driver.sensor_guard.evaluate(obs.clock.sim_time_s, obs.sensor_health).ready


def test_emergency_shield_retains_stronger_braking_while_sensors_stale():
    from app.services.interfaces import LeadObservation
    s = session_with_fault()
    for _ in range(5):
        s.step_pipeline(10)
    p = s.world.physics
    obs = replace(p.observe(), lead=LeadObservation(.1, 0.),
                  sensor_health=SensorHealth(None, None))
    ctx = p.build_driver_context(.02)
    cmd = s.driver.control(obs, ctx)
    assert cmd.accel_mps2 == -p.A_MAX_BRAKE_MPS2
    assert s.driver.target_speed_mps == 0.
    assert cmd.speed_limit_reason == 'safety_shield_override'


@pytest.mark.parametrize('source', ['forward', 'surround'])
def test_sensor_producer_exception_does_not_abort_control_pipeline(monkeypatch, source):
    s = session_with_fault(start=100.)
    for _ in range(5):
        s.step_pipeline(10)
    p = s.world.physics
    before = p.speed_kmh
    def broken(*_a, **_kw):
        raise RuntimeError('synthetic producer exception')
    if source == 'surround':
        monkeypatch.setattr(p.surround_perception, 'step', broken)
    else:
        monkeypatch.setattr(p.traffic, 'sense_lead_vehicle', broken)
    for _ in range(20):
        s.step_pipeline(10)
    assert s.driver.target_speed_mps == 0.
    assert not s.driver.sensor_guard.status.ready
    assert p.sensor_faults[source] == 'RuntimeError'
    assert p.speed_kmh < before
    assert math.isfinite(p.lat) and math.isfinite(p.lng)
    assert s.executor.clock.sim_time_s > 2.


def test_delivery_latency_uses_original_capture_time_not_delivery_time():
    s = SimulationSession(seed=42, real_time=False)
    s.scenario_engine.load_scenario('normal_cruising', s.world.physics,
                                   params={'latency_ticks': 4})
    p = s.world.physics
    for _ in range(6):
        s.step_pipeline(10)
    obs = p.observe()
    assert obs.safety_measurement_age_s >= .2 - 1e-8
    assert not s.driver.sensor_guard.status.ready
    assert s.driver.sensor_guard.status.reason == 'sensor_stale'
    assert s.driver.target_speed_mps == 0.


def test_configured_long_outage_reaches_full_stop_before_recovery():
    s = session_with_fault(start=1., duration=9.)
    p = s.world.physics
    p.traffic.npcs = []
    for _ in range(90):
        s.step_pipeline(10)
    assert p.speed_kmh == 0.
    assert s.driver.target_speed_mps == 0.
    assert s.driver.speed_limit_reason == 'sensor_stale'
    for _ in range(30):
        s.step_pipeline(10)
    assert s.driver.sensor_guard.status.ready
    assert p.speed_kmh > 0.
