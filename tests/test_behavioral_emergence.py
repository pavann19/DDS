"""
Behavioral Emergence Test Suite (P1)
=====================================
Verifies that driving decisions (car-following, lane change initiation,
blockage suppression, dynamic abort, and safety shield vetoes) emerge
organically from the perception -> decision -> safety -> planner pipeline,
rather than from predetermined scenario scripts or hardcoded timers.
"""

import pytest
from app.services.physics_engine import (
    PhysicsEngine,
    EGO_LANE_OFFSET_M,
    ADJACENT_LANE_OFFSET_M,
)
from app.services.traffic import TrafficModel, NpcVehicle
from app.services.planner.state_machine import LaneChangeState


def _straight_route(n=200):
    return [(37.7749 + i * 0.0001, -122.4194) for i in range(n)]


def _make_engine(lead_station_m: float, lead_speed_kmh: float, adjacent_blocker_station_m: float = None, adjacent_blocker_speed_kmh: float = 20.0):
    """Construct an engine on the freeway with precise NPC placement."""
    route = _straight_route(200)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    engine.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    total_len = engine.station_distances[-1] if engine.station_distances else 2000.0
    traffic = TrafficModel(total_length_m=total_len, seed=42, density="sparse")

    npcs = [
        NpcVehicle(
            id="npc-lead",
            lane_offset=EGO_LANE_OFFSET_M,
            speed_kmh=lead_speed_kmh,
            station_m=lead_station_m,
            desired_speed_kmh=lead_speed_kmh,
            prevent_recycle=True,
        )
    ]

    if adjacent_blocker_station_m is not None:
        npcs.append(
            NpcVehicle(
                id="npc-blocker",
                lane_offset=ADJACENT_LANE_OFFSET_M,
                speed_kmh=adjacent_blocker_speed_kmh,
                station_m=adjacent_blocker_station_m,
                desired_speed_kmh=adjacent_blocker_speed_kmh,
                prevent_recycle=True,
            )
        )

    traffic.spawn_scripted_npcs(npcs)
    engine.traffic = traffic
    return engine


def test_lane_change_trigger_scales_with_gap():
    """
    Anti-scripting proof:
    If lane changes were triggered by a static timer/tick, varying the initial
    distance to a slow lead vehicle would have NO effect on trigger time.
    Instead, trigger time must scale continuously with initial gap:
    T_trigger ~ (d_initial - d_threshold) / (v_ego - v_lead)
    """
    gaps = [32.0, 42.0, 52.0]
    trigger_ticks = []

    for gap in gaps:
        engine = _make_engine(lead_station_m=gap, lead_speed_kmh=20.0)
        dt = 0.1
        triggered_at_tick = None

        for tick in range(1, 150):
            engine.update("Maintain Speed", dt=dt)

            # Check if ego's state machine entered prepare or execute lane change
            if engine._lc_sm.state in (
                LaneChangeState.PREPARE_LANE_CHANGE,
                LaneChangeState.EXECUTE_LANE_CHANGE,
            ):
                triggered_at_tick = tick
                break

        assert triggered_at_tick is not None, f"Failed to trigger lane change for gap={gap}m"
        trigger_ticks.append(triggered_at_tick)

    # Monotonic scaling: larger gap must require strictly more simulation ticks to trigger
    assert trigger_ticks[0] < trigger_ticks[1] < trigger_ticks[2], (
        f"Trigger ticks {trigger_ticks} do not scale with gaps {gaps}. "
        "Decisions must emerge dynamically from distance closing."
    )

    # When gap is far beyond trigger threshold (> 120m), lane change should not trigger in short horizons
    far_engine = _make_engine(lead_station_m=120.0, lead_speed_kmh=20.0)
    for _ in range(30):
        far_engine.update("Maintain Speed", dt=0.1)
    assert far_engine._lc_sm.state is LaneChangeState.LANE_KEEP, (
        "Vehicle should remain in LANE_KEEP when lead is far ahead (>100m)."
    )


def test_lane_change_suppression_on_adjacent_blockage():
    """
    Perceptual emergence proof:
    When ego encounters a slow lead vehicle, but perception observes that the
    adjacent passing lane is blocked by another vehicle alongside ego, ego
    MUST NOT initiate a lane change. It must decelerate via IDM car-following.
    """
    # Slow lead at 30m, blocker alongside at station 10m moving at matching speed (20 km/h)
    engine = _make_engine(
        lead_station_m=30.0,
        lead_speed_kmh=20.0,
        adjacent_blocker_station_m=10.0,
        adjacent_blocker_speed_kmh=20.0,
    )

    dt = 0.1
    for tick in range(40):
        engine.update("Maintain Speed", dt=dt)
        # Ego must never execute a lane change into an occupied lane
        assert engine._lc_sm.state is not LaneChangeState.EXECUTE_LANE_CHANGE, (
            f"Ego illegally initiated lane change at tick {tick} while adjacent lane was blocked!"
        )
        assert abs(engine.current_lateral_offset_m - EGO_LANE_OFFSET_M) < 0.5, (
            "Ego lateral position deviated significantly while adjacent lane was blocked"
        )

    # Ego speed must have decelerated to match the slow lead vehicle
    assert engine.speed_kmh < 40.0, (
        f"Ego failed to decelerate behind slow lead: speed is {engine.speed_kmh:.1f} km/h"
    )


def test_dynamic_lane_change_abort_emergence():
    """
    Dynamic replanning proof:
    If adjacent lane becomes suddenly blocked mid-maneuver (< 85% completion),
    the state machine must autonomously switch to ABORT_LANE_CHANGE and
    re-plan a smooth return trajectory back to the origin lane.
    """
    engine = _make_engine(lead_station_m=30.0, lead_speed_kmh=20.0)
    dt = 0.1

    # Run until lane change executes and starts moving laterally
    entered_execute = False
    for tick in range(40):
        engine.update("Maintain Speed", dt=dt)
        if engine._lc_sm.state is LaneChangeState.EXECUTE_LANE_CHANGE:
            entered_execute = True
            break

    assert entered_execute, "Ego should have initiated lane change for slow lead"

    # Advance 10 ticks into the maneuver
    for _ in range(10):
        engine.update("Maintain Speed", dt=dt)

    # Now dynamically inject an obstacle vehicle in the adjacent lane near ego
    engine.traffic.spawn_scripted_npcs([
        NpcVehicle(
            id="npc-lead",
            lane_offset=EGO_LANE_OFFSET_M,
            speed_kmh=20.0,
            station_m=engine.current_station_m + 30.0,
            desired_speed_kmh=20.0,
            prevent_recycle=True,
        ),
        NpcVehicle(
            id="npc-sudden-blocker",
            lane_offset=ADJACENT_LANE_OFFSET_M,
            speed_kmh=30.0,
            station_m=engine.current_station_m + 10.0,  # Critical closing zone
            desired_speed_kmh=30.0,
            prevent_recycle=True,
        ),
    ])

    # Next update must detect blockage and abort
    engine.update("Maintain Speed", dt=dt)
    assert engine._lc_sm.state in (LaneChangeState.ABORT_LANE_CHANGE, LaneChangeState.LANE_KEEP), (
        f"State machine did not abort when adjacent lane became blocked: state={engine._lc_sm.state}"
    )

    # Continue stepping: the abort must arrest the outward lateral velocity
    # the committed maneuver built up and then return to the origin lane.
    # That takes longer than an instant -- the vehicle cannot reverse
    # lateral momentum for free -- but it must complete and settle.
    returned_tick = None
    for i in range(60):
        engine.update("Maintain Speed", dt=dt)
        if abs(engine.current_lateral_offset_m - EGO_LANE_OFFSET_M) < 0.5:
            returned_tick = i
            break
    assert returned_tick is not None, (
        f"Ego did not return to the origin lane after aborting: "
        f"lateral={engine.current_lateral_offset_m:.2f} m"
    )
    # and it stays there (no oscillation back out)
    for _ in range(20):
        engine.update("Maintain Speed", dt=dt)
        assert abs(engine.current_lateral_offset_m - EGO_LANE_OFFSET_M) < 0.7


def test_safety_shield_independent_override():
    """
    Independent supervisor authority proof:
    The Safety Shield must override the planner into EMERGENCY_BRAKE whenever
    RSS distance or TTC threshold (< 2.0s) is breached, independent of scenario.
    """
    route = _straight_route(200)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    engine.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    # Place a slow hazard ahead (18m ahead moving at 10 km/h vs ego at 50 km/h -> TTC ~ 1.6s)
    total_len = 2000.0
    traffic = TrafficModel(total_length_m=total_len, seed=123, density="sparse")
    traffic.spawn_scripted_npcs([
        NpcVehicle(
            id="npc-hazard",
            lane_offset=EGO_LANE_OFFSET_M,
            speed_kmh=10.0,
            station_m=18.0,
            desired_speed_kmh=10.0,
            prevent_recycle=True,
        )
    ])
    engine.traffic = traffic

    engine.update("Maintain Speed", dt=0.1)

    shield = engine.get_safety_shield_state()
    assert shield["approved"] is False, "Safety Shield should have vetoed the planner!"
    assert shield["override_action"] == "EMERGENCY_BRAKE", (
        f"Expected EMERGENCY_BRAKE override, got {shield['override_action']}"
    )

    # Over subsequent ticks of sustained emergency braking, acceleration ramps down via jerk limits
    for _ in range(12):
        engine.update("Maintain Speed", dt=0.1)
    assert engine.acceleration_mps2 <= -3.0, (
        f"Expected sustained heavy braking <= -3.0 m/s^2, got {engine.acceleration_mps2:.2f} m/s^2"
    )
    assert engine.speed_kmh < 43.0, "Ego must rapidly decelerate under emergency brake override"


def test_unscripted_free_drive_autonomy():
    """
    Zero-script free-drive proof:
    Run the simulation with NO scenario script or scenario engine active.
    The vehicle must cruise, steer, avoid obstacles, and remain safe purely
    through the autonomous closed loop.
    """
    route = _straight_route(300)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    engine.active_scenario = None  # Explicitly None
    engine.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=EGO_LANE_OFFSET_M)

    # Default traffic model with multiple NPC vehicles
    traffic = TrafficModel(total_length_m=2000.0, seed=99, density="medium")
    engine.traffic = traffic

    # Run for 100 ticks (10 seconds)
    for tick in range(100):
        engine.update("Maintain Speed", dt=0.1)

        # Invariants: no collisions, speed within legal highway bounds, lateral position bounded
        assert 0.0 <= engine.speed_kmh <= 65.0
        assert -3.0 <= engine.current_lateral_offset_m <= 7.0
        assert abs(engine.steering_angle_rad) <= engine.MAX_STEER_RAD
        assert abs(engine.signed_lateral_accel_mps2) <= engine.A_LAT_MAX_MPS2 + 0.5
