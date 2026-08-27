"""Unit tests for app/services/world/vehicle_dynamics.py (ADR-001 item 2).

These lock the extracted world-side integration math independently of
PhysicsEngine. The behaviour-preservation guarantee itself is covered by
tests/test_physics_engine.py continuing to pass unchanged; this file adds
direct coverage of the pure functions.
"""
import math
import random

import pytest

from app.services.world.vehicle_dynamics import advance_position, step_powertrain


def test_powertrain_coolant_stays_within_clamp():
    rpm, coolant, fuel, co2, alt = 800.0, 80.0, 0.0, 0.0, 162.5
    rng = random.Random(0)
    for _ in range(500):
        s = step_powertrain(
            speed_kmh=90.0, ai_decision="Accelerate",
            rpm=rpm, coolant_temp=coolant, fuel_rate=fuel, co2=co2, altitude=alt,
            dt=0.1, rng=rng,
        )
        rpm, coolant, fuel, co2, alt = s.rpm, s.coolant_temp, s.fuel_rate, s.co2, s.altitude
    assert 70.0 <= coolant <= 110.0


def test_powertrain_idle_coolant_does_not_pin_to_ceiling():
    """Regression parity: at speed 0 there is still a passive cooling floor,
    so coolant must not climb monotonically to the 110 clamp."""
    rpm, coolant, fuel, co2, alt = 800.0, 80.0, 0.0, 0.0, 162.5
    rng = random.Random(1)
    for _ in range(2000):
        s = step_powertrain(
            speed_kmh=0.0, ai_decision="Maintain Speed",
            rpm=rpm, coolant_temp=coolant, fuel_rate=fuel, co2=co2, altitude=alt,
            dt=0.1, rng=rng,
        )
        rpm, coolant, fuel, co2, alt = s.rpm, s.coolant_temp, s.fuel_rate, s.co2, s.altitude
    assert coolant < 110.0
    assert coolant <= 87.0


def test_powertrain_altitude_drift_is_bounded_per_tick():
    rng = random.Random(2)
    s = step_powertrain(
        speed_kmh=50.0, ai_decision="Maintain Speed",
        rpm=2000.0, coolant_temp=85.0, fuel_rate=5.0, co2=120.0, altitude=162.5,
        dt=0.1, rng=rng,
    )
    assert abs(s.altitude - 162.5) <= 0.1


def test_advance_position_is_noop_when_stopped():
    assert advance_position(lat=37.0, lng=-122.0, heading_deg=90.0, speed_kmh=0.0, dt=0.1) == (37.0, -122.0)


def test_advance_position_moves_roughly_speed_times_dt():
    lat0, lng0 = 37.7749, -122.4194
    # 36 km/h = 10 m/s, 1.0 s => ~10 m displacement.
    lat1, lng1 = advance_position(lat=lat0, lng=lng0, heading_deg=0.0, speed_kmh=36.0, dt=1.0)
    # heading 0 = due north => latitude increases, longitude ~unchanged.
    assert lat1 > lat0
    assert lng1 == pytest.approx(lng0, abs=1e-9)
    meters = (lat1 - lat0) * (math.pi / 180.0) * 6371000.0
    assert meters == pytest.approx(10.0, rel=1e-3)
