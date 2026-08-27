"""World-side vehicle dynamics (ADR-001, Phase 6.5, Action Item 2).

The parts of ``PhysicsEngine.update()`` that integrate the physical vehicle
forward, extracted **verbatim** as pure functions:

* :func:`step_powertrain` -- RPM / coolant / fuel / CO2 / altitude first-order
  relaxation, exactly the block that previously lived inline.
* :func:`advance_position` -- great-circle displacement of the ego by
  ``speed * dt`` along its current heading.

No planner, perception, or safety logic lives here, and no wall clock -- ``dt``
is always passed in. ``PhysicsEngine`` delegates to these and keeps its public
surface identical; the arithmetic and the RNG call sequence are unchanged, so
this is strictly behaviour-preserving.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

EARTH_RADIUS_M = 6371000.0


@dataclass
class PowertrainState:
    """The five relaxation-integrated engine/emissions signals.

    Field names match the ``PhysicsEngine`` attributes they map back onto
    (``rpm``, ``coolant_temp``, ``fuel_rate``, ``co2``, ``altitude``) so the
    facade assignment is a trivial unpack.
    """

    rpm: float
    coolant_temp: float
    fuel_rate: float
    co2: float
    altitude: float


def step_powertrain(
    *,
    speed_kmh: float,
    ai_decision: str,
    rpm: float,
    coolant_temp: float,
    fuel_rate: float,
    co2: float,
    altitude: float,
    dt: float,
    rng: random.Random | None = None,
) -> PowertrainState:
    """Advance the powertrain/emissions signals one tick.

    Verbatim port of the former inline block (physics_engine.py). ``rng``
    defaults to the ``random`` module so the call sequence
    (``uniform(-10, 10)`` for idle RPM jitter when stopped, then
    ``uniform(-0.1, 0.1)`` for altitude drift) is byte-for-byte what it was;
    the multi-rate executor (Action Item 4) is what will later thread a
    seeded generator through here for the determinism gate.
    """
    _rng = rng if rng is not None else random

    if speed_kmh < 1:
        target_rpm = 800.0 + _rng.uniform(-10, 10)
    else:
        gear_speed = speed_kmh % 30.0
        target_rpm = 1000 + (gear_speed / 30.0) * 3000
        if ai_decision == "Accelerate":
            target_rpm += 500

    rpm = rpm + (target_rpm - rpm) * dt * 5.0

    # Coolant, Fuel, CO2
    heat_gen = (rpm / 4000.0) * 2.0
    # Passive radiator/fan cooling floor so coolant doesn't run away to the
    # clamp ceiling while idling at speed_kmh == 0.
    cooling = 0.5 + (speed_kmh / 120.0) * 1.5
    coolant_temp = coolant_temp + (heat_gen - cooling) * dt
    coolant_temp = max(70.0, min(coolant_temp, 110.0))

    load_factor = (rpm / 4000.0) + (1.0 if ai_decision == "Accelerate" else 0.0)
    target_fuel = 2.0 + load_factor * 8.0 if speed_kmh > 1 else 1.0
    fuel_rate = fuel_rate + (target_fuel - fuel_rate) * dt * 2.0

    target_co2 = fuel_rate * 25.0
    co2 = co2 + (target_co2 - co2) * dt * 2.0

    altitude = altitude + _rng.uniform(-0.1, 0.1)

    return PowertrainState(
        rpm=rpm,
        coolant_temp=coolant_temp,
        fuel_rate=fuel_rate,
        co2=co2,
        altitude=altitude,
    )


def advance_position(
    *,
    lat: float,
    lng: float,
    heading_deg: float,
    speed_kmh: float,
    dt: float,
) -> tuple[float, float]:
    """Move ``(lat, lng)`` forward by ``speed_kmh/3.6 * dt`` metres along
    ``heading_deg`` on a great circle. Verbatim port of the former inline
    movement block; returns the ego position unchanged when stopped.
    """
    if speed_kmh <= 0:
        return lat, lng

    speed_mps = speed_kmh / 3.6
    dist_moved = speed_mps * dt

    R = EARTH_RADIUS_M
    brng = math.radians(heading_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lng)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(dist_moved / R)
        + math.cos(lat1) * math.sin(dist_moved / R) * math.cos(brng)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(dist_moved / R) * math.cos(lat1),
        math.cos(dist_moved / R) - math.sin(lat1) * math.sin(lat2),
    )

    return math.degrees(lat2), math.degrees(lon2)
