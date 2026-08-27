"""Unit tests for app/services/driver/safety_monitor.py (ADR-001 item 6).

The monitor is a thin, independent node around safety_shield.evaluate + the
veto composition. These lock veto-only authority (it can only brake harder)
and parity with the shield primitive. Behaviour-preservation vs. the former
inline block is covered by tests/test_physics_engine.py's shield tests
staying green.
"""
import pytest

from app.services import safety_shield
from app.services.driver import SafetyMonitor


def _step(monitor, **over):
    kwargs = dict(
        ego_speed_mps=15.0,
        lateral_offset_m=1.75,
        lateral_accel_mps2=0.0,
        sensed_lead_gap_m=None,
        sensed_lead_speed_mps=None,
        desired_accel=1.5,
        a_max_brake_mps2=4.5,
        recovery_speed_kmh=8.0,
        speed_kp=0.6,
    )
    kwargs.update(over)
    return monitor.step(**kwargs)


def test_no_hazard_passes_accel_through_unchanged():
    m = SafetyMonitor()
    accel, reason = _step(m, desired_accel=1.5)
    assert accel == 1.5
    assert reason is None
    assert m.verdict.approved is True


def test_imminent_collision_forces_max_brake():
    m = SafetyMonitor()
    # gap 6 m, lead stopped, ego 15 m/s => TTC = 0.4 s < critical.
    accel, reason = _step(
        m, sensed_lead_gap_m=6.0, sensed_lead_speed_mps=0.0, desired_accel=2.0,
    )
    assert accel == -4.5
    assert reason == "safety_shield_override"
    assert m.verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE


def test_off_road_triggers_recover_low_speed_not_full_stop():
    m = SafetyMonitor()
    accel, reason = _step(
        m, lateral_offset_m=safety_shield.ROAD_BOUNDARY_HARD_LIMIT_M + 2.0,
        ego_speed_mps=15.0, desired_accel=1.0,
    )
    assert m.verdict.override_action == safety_shield.OVERRIDE_RECOVER_LOW_SPEED
    assert reason == "safety_shield_override"
    # Proportional control toward the ~2.2 m/s recovery floor:
    # 0.6 * (8/3.6 - 15) ~= -7.67. The monitor does NOT clamp (the caller's
    # [-a_brake, +a_accel] clamp is downstream, unchanged) -- it only vetoes
    # downward, so the value is the raw proportional demand.
    assert accel == pytest.approx(0.6 * (8.0 / 3.6 - 15.0))
    assert accel < 0.0


def test_veto_only_never_increases_accel():
    m = SafetyMonitor()
    # Even with a hazard, if the planner already wants to brake harder than
    # the recovery target, the monitor must not pull it back up.
    accel, _ = _step(
        m, lateral_offset_m=safety_shield.ROAD_BOUNDARY_HARD_LIMIT_M + 2.0,
        desired_accel=-4.0,
    )
    assert accel <= -4.0


def test_reset_restores_clean_verdict():
    m = SafetyMonitor()
    _step(m, sensed_lead_gap_m=5.0, sensed_lead_speed_mps=0.0)
    assert m.verdict.approved is False
    m.reset()
    assert m.verdict.approved is True
    assert m.verdict.risk_level == safety_shield.RISK_NONE
