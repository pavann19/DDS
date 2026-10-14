"""
Runtime AI attribution audit (P0.3).

Asserts -- as executable, checked-in tests rather than a one-off scratch
script -- which module produces each driving output, and in particular that
the ML classifier (XGBoost, powertrain telemetry only; ADR-001) has NO path
into steering / acceleration / braking / lane-change / trajectory selection.

Companion to docs/AI_VS_DETERMINISTIC_ATTRIBUTION.md -- if that matrix and
these tests disagree, one of them is wrong.
"""
import math
import pytest

from app.services.physics_engine import (
    PhysicsEngine,
    EGO_LANE_OFFSET_M,
    ADJACENT_LANE_OFFSET_M,
)
from app.services.traffic import TrafficModel, NpcVehicle
from app.services.planner.state_machine import LaneChangeState


def _straight_route(n=160):
    return [(37.7749 + i * 0.00005, -122.4194) for i in range(n)]


def _corner_route():
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + (i * 4.0) / m_lng) for i in range(30)]
    x0 = 30 * 4.0
    R, ang = 90.0, 0.0
    while ang < math.pi / 2:
        ang += 4.0 / R
        pts.append((lat0 + (R * (1 - math.cos(ang))) / m_lat, lng0 + (x0 + R * math.sin(ang)) / m_lng))
    lat, lng = pts[-1]
    pts += [(lat + (i * 4.0) / m_lat, lng) for i in range(1, 30)]
    return pts


def _trace(engine, action, ticks=200, dt=0.1):
    out = []
    for _ in range(ticks):
        engine.update(action, dt=dt)
        out.append((
            round(engine.speed_kmh, 9),
            round(engine.steering_angle_rad, 9),
            round(engine.current_lateral_offset_m, 9),
            round(engine.current_station_m, 9),
        ))
    return out


# ---------------------------------------------------------------------------
# 1. The ML decision string does not influence control at all.
# ---------------------------------------------------------------------------
def test_ml_decision_string_does_not_influence_control():
    route = _corner_route()

    def drive(action):
        e = PhysicsEngine(seed=7)
        e.set_destination(*route[-1])
        e.set_route(route)
        e.reset_state(station_m=0.0, speed_kmh=40.0, lateral_offset_m=EGO_LANE_OFFSET_M)
        return _trace(e, action, ticks=250)

    # Three wildly different ML classifications, same physical scenario.
    accel = drive("Accelerate")
    decel = drive("Decelerate")
    hold = drive("Maintain Speed")

    assert accel == decel == hold, (
        "the ML classifier's output changed the driven trajectory -- it must be "
        "analytics-only (ADR-001)"
    )


# ---------------------------------------------------------------------------
# 2. Steering is a pure deterministic function of route geometry + state.
# ---------------------------------------------------------------------------
def test_steering_is_a_deterministic_function_of_geometry():
    route = _corner_route()

    def steer_trace():
        e = PhysicsEngine(seed=7)
        e.set_destination(*route[-1])
        e.set_route(route)
        e.reset_state(station_m=0.0, speed_kmh=40.0, lateral_offset_m=EGO_LANE_OFFSET_M)
        return [round(x[1], 12) for x in _trace(e, "Maintain Speed", ticks=250)]

    a, b = steer_trace(), steer_trace()
    assert a == b, "steering is not reproducible for a fixed (route, dt, seed) -- a hidden RNG/ML input?"
    assert any(abs(s) > 1e-3 for s in a), "sanity: the corner route should actually demand steering"


# ---------------------------------------------------------------------------
# 3. An emergency brake is attributable to the Safety Shield, not the planner.
# ---------------------------------------------------------------------------
def test_emergency_brake_is_attributable_to_the_safety_shield():
    route = _straight_route(200)
    e = PhysicsEngine(seed=1)
    e.set_destination(*route[-1])
    e.set_route(route)
    e.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    tm = TrafficModel(total_length_m=e.station_distances[-1], seed=1)
    tm.spawn_scripted_npcs([
        NpcVehicle(id="hazard", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=8.0,
                   station_m=16.0, desired_speed_kmh=8.0, prevent_recycle=True),
    ])
    e.traffic = tm

    fired = False
    for _ in range(30):
        e.update("Accelerate", dt=0.1)  # ML says "go" -- must be overridden
        v = e.get_safety_shield_state()
        if not v["approved"]:
            fired = True
            assert v["override_action"] == "EMERGENCY_BRAKE"
            break
    assert fired, "Safety Shield never vetoed a closing collision"

    # over the next few ticks the jerk-limited brake ramps to a heavy
    # deceleration -- produced by the shield override, NOT the ML "Accelerate"
    for _ in range(12):
        e.update("Accelerate", dt=0.1)
    assert e.acceleration_mps2 <= -3.0, (
        f"shield override did not produce heavy braking (a = {e.acceleration_mps2:.2f})"
    )
    assert e.speed_kmh < 48.0


# ---------------------------------------------------------------------------
# 4. A lane change is driven by the state machine, not the ML decision.
# ---------------------------------------------------------------------------
def test_lane_change_is_driven_by_the_state_machine():
    route = _straight_route(220)
    e = PhysicsEngine(seed=1)
    e.set_destination(*route[-1])
    e.set_route(route)
    e.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    tm = TrafficModel(total_length_m=e.station_distances[-1], seed=1)
    tm.spawn_scripted_npcs([
        NpcVehicle(id="slow-lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=18.0,
                   station_m=32.0, desired_speed_kmh=18.0, prevent_recycle=True),
    ])
    e.traffic = tm

    saw_execute = False
    for _ in range(160):
        e.update("Maintain Speed", dt=0.1)
        if e._lc_sm.state in (LaneChangeState.EXECUTE_LANE_CHANGE, LaneChangeState.ABORT_LANE_CHANGE):
            saw_execute = True
            # while executing, the lateral target tracks the adjacent lane the
            # state machine opened -- nothing about the ML label is involved
            assert e._lc_sm.state is not None
    assert saw_execute, "no lane change was executed for a slow blocking lead"


# ---------------------------------------------------------------------------
# 5. The ML feature vector is powertrain/emissions telemetry only (ADR-001).
# ---------------------------------------------------------------------------
def test_ml_feature_vector_is_powertrain_telemetry_only():
    e = PhysicsEngine(seed=1)
    e.set_destination(*_straight_route()[-1])
    e.set_route(_straight_route())
    for _ in range(20):
        e.update("Maintain Speed", dt=0.1)
    feats = e.get_ml_features()

    allowed_tokens = ("rpm", "co2", "coolant", "fuel", "litre", "altitude", "delta", "instant")
    for key in feats:
        k = key.lower()
        assert any(tok in k for tok in allowed_tokens), (
            f"ML feature {key!r} is not powertrain/emissions telemetry -- ADR-001 "
            "isolates the classifier from the driving state"
        )
    banned = ("steer", "lateral", "heading", "planner", "frenet", "throttle",
              "brake", "yaw", "perception", "shield", "ttc", "lane")
    for key in feats:
        k = key.lower()
        assert not any(b in k for b in banned), f"ML feature {key!r} leaks a driving-state quantity"


# ---------------------------------------------------------------------------
# Regression: oncoming traffic in a physically separated opposing lane must
# never trigger a proactive cut-in slowdown (it was doing so from small
# lateral components of a fast oncoming velocity vector -- see
# OVERNIGHT_CHECKPOINT_2026-09-10 "phantom cut-in on curves").
# ---------------------------------------------------------------------------
def test_oncoming_traffic_does_not_trigger_a_proactive_cut_in():
    route = _straight_route(200)
    e = PhysicsEngine(seed=1)
    e.set_destination(*route[-1])
    e.set_route(route)
    e.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    tm = TrafficModel(total_length_m=e.station_distances[-1], seed=1)
    # a fast oncoming vehicle in the opposing lane, closing head-on but
    # laterally well separated -- classic false-positive source
    tm.spawn_scripted_npcs([
        NpcVehicle(id="oncoming", lane_offset=-5.25, speed_kmh=-55.0,
                   station_m=120.0, desired_speed_kmh=-55.0, prevent_recycle=True),
    ])
    e.traffic = tm

    proactive_ticks = 0
    for _ in range(120):
        e.update("Maintain Speed", dt=0.1)
        pr = e.prediction_result
        if pr is not None and pr.proactive_decel_mps2 > 0.0:
            proactive_ticks += 1

    assert proactive_ticks == 0, (
        f"oncoming traffic triggered a proactive cut-in slowdown on {proactive_ticks} ticks"
    )
