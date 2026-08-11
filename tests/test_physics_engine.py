"""
Unit tests for app/services/physics_engine.py, including a P1-7
regression test for the idle-coolant-runaway fix.
"""
import time
import pytest
from app.services.physics_engine import PhysicsEngine


def _tick(engine, action, dt, ticks):
    """Advance the engine `ticks` times with a fixed dt, bypassing the
    wall-clock-based dt in update() so tests are deterministic."""
    for _ in range(ticks):
        engine.last_update_time = time.time() - dt
        engine.update(action)


def test_initial_altitude_is_in_training_distribution():
    """P1-3 regression: Altitude used to start at 10.0, wildly outside the
    training data's observed 128-203 range."""
    engine = PhysicsEngine()
    assert 128 <= engine.altitude <= 203


def test_coolant_stays_within_clamp_bounds():
    engine = PhysicsEngine()
    _tick(engine, "Maintain Speed", dt=0.1, ticks=50)
    assert 70.0 <= engine.coolant_temp <= 110.0


def test_idle_coolant_does_not_run_away_to_clamp_ceiling():
    """P1-7 regression: at speed 0, cooling used to be exactly 0, so
    coolant climbed monotonically to the 110.0 clamp ceiling. It must now
    stay well below the training-data-observed max (87C) under sustained
    idle."""
    engine = PhysicsEngine()
    engine.speed_kmh = 0.0
    # dist < 10 keeps target_speed at 0, so the vehicle stays parked.
    engine.target_lat = engine.lat
    engine.target_lng = engine.lng

    _tick(engine, "Maintain Speed", dt=0.1, ticks=1000)

    assert engine.speed_kmh == 0.0, "test setup must keep the vehicle at idle"
    assert engine.coolant_temp < 110.0, "coolant must not pin at the clamp ceiling while idling"
    assert engine.coolant_temp <= 87.0, "coolant must stay within the training-data-observed max while idling"


def test_car_moves_toward_a_distant_destination_even_if_ai_only_ever_says_maintain_speed():
    """Regression test: target_speed used to be driven SOLELY by ai_decision
    ('Accelerate' -> +5/tick, 'Decelerate' -> -8/tick, else -> hold current
    speed). Since the classifier predicts "Maintain Speed" from idle-looking
    telemetry, a car starting at rest fed that same idle reading back every
    tick could never leave 0 km/h -- "Maintain Speed" held the target at the
    current speed forever. The physics engine must now have its own
    baseline cruise-toward-destination speed so the car actually drives,
    with the AI decision only modulating that baseline."""
    engine = PhysicsEngine()
    engine.target_lat = engine.lat + 0.02  # a destination ~2km away
    engine.target_lng = engine.lng + 0.02
    start_lat, start_lng = engine.lat, engine.lng

    _tick(engine, "Maintain Speed", dt=0.1, ticks=100)  # worst case: never told to accelerate

    assert engine.speed_kmh > 0.0, "car must move even if the AI only ever predicts Maintain Speed"
    assert (engine.lat, engine.lng) != (start_lat, start_lng), "car must actually change position"


def test_accelerate_decision_pushes_speed_above_decelerate_decision():
    """The AI decision should still visibly modulate speed relative to the
    baseline cruise target, even though it's no longer the sole driver."""
    accel_engine = PhysicsEngine()
    accel_engine.target_lat = accel_engine.lat + 0.02
    accel_engine.target_lng = accel_engine.lng + 0.02
    _tick(accel_engine, "Accelerate", dt=0.1, ticks=50)

    decel_engine = PhysicsEngine()
    decel_engine.target_lat = decel_engine.lat + 0.02
    decel_engine.target_lng = decel_engine.lng + 0.02
    _tick(decel_engine, "Decelerate", dt=0.1, ticks=50)

    assert accel_engine.speed_kmh > decel_engine.speed_kmh


def test_car_stops_on_arrival_at_destination():
    engine = PhysicsEngine()
    engine.target_lat = engine.lat  # already "arrived"
    engine.target_lng = engine.lng
    _tick(engine, "Maintain Speed", dt=0.1, ticks=50)
    assert engine.speed_kmh == 0.0


def test_speed_never_exceeds_clamp():
    engine = PhysicsEngine()
    _tick(engine, "Accelerate", dt=0.5, ticks=200)
    assert 0.0 <= engine.speed_kmh <= 160.0


def test_set_route_and_navigation_state_exposes_progress():
    engine = PhysicsEngine()
    waypoints = [(37.776, -122.419), (37.780, -122.415), (37.8199, -122.4783)]
    engine.set_destination(37.8199, -122.4783)
    engine.set_route(waypoints)

    nav = engine.get_navigation_state()
    assert nav["has_route"] is True
    assert nav["route_index"] == 0


def test_no_route_falls_back_to_straight_line_navigation():
    engine = PhysicsEngine()
    engine.set_destination(engine.lat + 0.01, engine.lng + 0.01)
    nav = engine.get_navigation_state()
    assert nav["has_route"] is False

    _tick(engine, "Maintain Speed", dt=0.1, ticks=50)
    assert engine.speed_kmh > 0.0  # still drives, just via straight-line bearing


def test_car_advances_through_route_waypoints_in_sequence():
    engine = PhysicsEngine()
    # Waypoints placed roughly along the path the car will actually take,
    # close enough together that normal driving reaches each in sequence.
    waypoints = [
        (engine.lat, engine.lng),
        (engine.lat + 0.0005, engine.lng + 0.0005),
        (engine.lat + 0.001, engine.lng + 0.001),
        (engine.lat + 0.005, engine.lng + 0.005),
    ]
    engine.set_destination(*waypoints[-1])
    engine.set_route(waypoints)

    assert engine.route_index == 0
    _tick(engine, "Maintain Speed", dt=0.1, ticks=400)  # 40s of driving
    assert engine.route_index > 0, "car should have advanced past the first waypoint(s)"


def test_new_destination_clears_the_previous_route():
    engine = PhysicsEngine()
    engine.set_destination(37.8199, -122.4783)
    engine.set_route([(37.776, -122.419), (37.8199, -122.4783)])
    assert engine.get_navigation_state()["has_route"] is True

    engine.set_destination(37.8, -122.45)
    nav = engine.get_navigation_state()
    assert nav["has_route"] is False
    assert nav["route_index"] == 0


def test_car_slows_down_for_a_sharp_upcoming_turn():
    """Regression test: the car used to hold CRUISE_SPEED (50 km/h) through
    every corner regardless of sharpness -- no braking for turns at all.
    A route with a ~90 degree corner right after the car's current position
    should cap speed well below cruise, even with the AI predicting
    Accelerate (cornering is a physical constraint, not something the AI
    decision can override)."""
    engine = PhysicsEngine()
    # A route with realistically-dense waypoints (~11m apart, matching real
    # OSRM route density -- see _evidence/P3-1-backend) that goes straight
    # east for ~110m, then turns sharply north for ~110m -- a ~90 degree
    # corner within the cornering lookahead window.
    corner_route = [(37.7749, -122.4194 + i * 0.0001) for i in range(10)]
    turn_lng = corner_route[-1][1]
    corner_route += [(37.7749 + i * 0.0001, turn_lng) for i in range(1, 10)]
    engine.set_destination(*corner_route[-1])
    engine.set_route(corner_route)

    _tick(engine, "Accelerate", dt=0.1, ticks=100)  # 10s, worst case: AI says floor it

    assert engine.speed_kmh < 40.0, "car must slow for a sharp upcoming turn, even when the AI predicts Accelerate"


def test_car_holds_cruise_speed_on_a_straight_route():
    """A straight route (no sharp turns) should NOT trigger the cornering
    speed cap -- the car should reach full cruise speed."""
    engine = PhysicsEngine()
    straight_route = [(37.7749 + i * 0.0001, -122.4194) for i in range(30)]
    engine.set_destination(*straight_route[-1])
    engine.set_route(straight_route)

    _tick(engine, "Maintain Speed", dt=0.1, ticks=200)  # 20s

    assert engine.speed_kmh > 45.0, "a straight route should not be capped by the cornering logic"


def test_get_ml_features_returns_expected_keys():
    engine = PhysicsEngine()
    _tick(engine, "Maintain Speed", dt=0.1, ticks=5)
    features = engine.get_ml_features()
    for key in ["RPM", "Coolant", "CO2", "Litre per 100km(Instant)", "Altitude", "RPM_Delta", "CO2_Delta", "Fuel_Rate_Delta"]:
        assert key in features


def test_get_navigation_state_returns_expected_keys():
    engine = PhysicsEngine()
    nav = engine.get_navigation_state()
    for key in ["lat", "lng", "heading", "speed", "steering"]:
        assert key in nav


# ---------------------------------------------------------------------------
# P6-1: kinematic bicycle model + jerk-limited longitudinal control.
#
# These lock in PHYSICAL guarantees the pre-P6 point-mass controller did not
# provide. Measured during development, the legacy controller reached ~22 m/s^2
# acceleration (>2 g) and ~195 m/s^3 jerk -- the discontinuities that read on
# screen as flickering/teleporting motion.
# ---------------------------------------------------------------------------

def _corner_route():
    """~120 m east, then a ~90 degree left turn and ~150 m north, with
    realistically dense waypoints (matching real OSRM route density)."""
    route = [(37.7749, -122.4194 + i * 0.0001) for i in range(15)]
    turn_lng = route[-1][1]
    route += [(37.7749 + i * 0.0001, turn_lng) for i in range(1, 15)]
    return route


def _straight_route(n=100):
    return [(37.7749 + i * 0.0001, -122.4194) for i in range(n)]


def _drive(engine, decision, ticks, dt=0.1):
    """Drive, returning per-tick (speed_mps, accel, lateral_accel) traces."""
    speeds, accels, lat_accels = [], [], []
    for _ in range(ticks):
        engine.last_update_time = time.time() - dt
        engine.update(decision)
        speeds.append(engine.speed_kmh / 3.6)
        accels.append(engine.acceleration_mps2)
        lat_accels.append(engine.lateral_accel_mps2)
    return speeds, accels, lat_accels


def test_longitudinal_acceleration_stays_within_limits():
    engine = PhysicsEngine()
    engine.set_destination(*_corner_route()[-1])
    engine.set_route(_corner_route())
    _, accels, _ = _drive(engine, "Accelerate", ticks=400)
    assert max(accels) <= PhysicsEngine.A_MAX_ACCEL_MPS2 + 1e-6
    assert min(accels) >= -PhysicsEngine.A_MAX_BRAKE_MPS2 - 1e-6


def test_jerk_stays_within_limit():
    """The defining fix of P6-1: bounded d(acceleration)/dt makes velocity C1
    continuous, which is what removes the visual flicker."""
    dt = 0.1
    engine = PhysicsEngine()
    engine.set_destination(*_corner_route()[-1])
    engine.set_route(_corner_route())
    _, accels, _ = _drive(engine, "Accelerate", ticks=400, dt=dt)
    jerks = [abs(accels[i] - accels[i - 1]) / dt for i in range(1, len(accels))]
    # 1% tolerance: the engine derives its own dt from the wall clock, which is
    # a few microseconds LONGER than the dt the test divides by here, so this
    # measurement slightly over-estimates the true jerk. The engine's internal
    # clamp itself is exact.
    assert max(jerks) <= PhysicsEngine.JERK_MAX_MPS3 * 1.01


def test_lateral_acceleration_respects_comfort_limit():
    """Cornering speed is now derived from a_lat = v^2 * kappa <= A_LAT_MAX
    rather than a hand-tuned speed lookup. A small tolerance is allowed for
    discrete-time integration overshoot."""
    engine = PhysicsEngine()
    engine.set_destination(*_corner_route()[-1])
    engine.set_route(_corner_route())
    _, _, lat_accels = _drive(engine, "Accelerate", ticks=500)
    settled = lat_accels[20:]  # skip the initial heading-alignment transient
    assert max(settled) <= PhysicsEngine.A_LAT_MAX_MPS2 * 1.25


def test_bicycle_controller_is_far_smoother_than_legacy():
    """Headline P6-1 comparison; the quantitative version becomes the P6-6
    A/B evaluation. Acceleration is derived from the speed trace so both
    controllers are measured identically (legacy writes speed directly and
    never populates acceleration_mps2)."""
    dt = 0.1

    def peak_jerk(controller):
        engine = PhysicsEngine(controller=controller)
        engine.set_destination(*_corner_route()[-1])
        engine.set_route(_corner_route())
        speeds, _, _ = _drive(engine, "Accelerate", ticks=400, dt=dt)
        accels = [(speeds[i] - speeds[i - 1]) / dt for i in range(1, len(speeds))]
        return max(abs(accels[i] - accels[i - 1]) / dt for i in range(1, len(accels)))

    assert peak_jerk("bicycle") < peak_jerk("legacy") / 10.0


def test_legacy_controller_remains_selectable_as_ab_control():
    """P6-6 needs the pre-P6 controller as its experimental control condition."""
    engine = PhysicsEngine(controller="legacy")
    engine.set_destination(*_straight_route()[-1])
    engine.set_route(_straight_route())
    _drive(engine, "Maintain Speed", ticks=100)
    assert engine.speed_kmh > 0.0
    assert engine.get_navigation_state()["controller"] == "legacy"


def test_bicycle_model_cannot_yaw_while_stationary():
    """Physical property of the bicycle model: yaw_rate = v*tan(delta)/L, so a
    stationary car cannot rotate however hard the wheel is turned."""
    engine = PhysicsEngine()
    engine.set_destination(37.7849, -122.4094)  # far away, heading error is large
    engine.speed_kmh = 0.0
    heading_before = engine.heading
    for _ in range(20):
        engine.last_update_time = time.time() - 0.1
        engine.update("Decelerate")  # keep it stopped
        engine.speed_kmh = 0.0
    assert engine.heading == pytest.approx(heading_before)


def test_arrival_is_latched_so_the_car_does_not_orbit_the_destination():
    """Regression: target speed used to be a pure function of the CURRENT
    distance, so a car coasting a few metres past its destination saw the
    distance grow, re-accelerated to cruise, turned around and orbited
    forever. Observed once the bicycle model made overshoot possible."""
    route = _corner_route()
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=600)  # long enough to orbit if unlatched
    assert engine.has_arrived
    assert engine.speed_kmh == pytest.approx(0.0, abs=0.5)
    final_dist = engine.calculate_distance(engine.lat, engine.lng,
                                           engine.target_lat, engine.target_lng)
    assert final_dist < 30.0, "car should stop near the destination, not orbit away from it"


def test_setting_a_new_destination_rearms_after_arrival():
    route = _corner_route()
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=600)
    assert engine.has_arrived

    engine.set_destination(37.7949, -122.4094)
    assert not engine.has_arrived
    _drive(engine, "Maintain Speed", ticks=150)
    assert engine.speed_kmh > 5.0, "car must drive again once given a new destination"


def test_route_progress_recovers_after_overshooting_a_corner():
    """Regression: route_index only advanced while within 15 m of the current
    waypoint, so an overshoot left it pinned and steered the car backwards.
    Progress is now by projection onto the route."""
    route = _corner_route()
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=500)
    assert engine.route_index >= len(route) - 2, "car should progress to the end of the route"


# ---------------------------------------------------------------------------
# P6-1b: server-side NPC traffic + forward range sensor, wired into
# PhysicsEngine. See app/services/traffic.py for the underlying model tests.
# ---------------------------------------------------------------------------

def test_set_route_spawns_traffic_for_a_nonzero_length_route():
    engine = PhysicsEngine()
    route = _straight_route()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    assert engine.traffic is not None
    assert len(engine.get_npc_states()) > 0


def test_no_route_means_no_traffic():
    engine = PhysicsEngine()
    assert engine.traffic is None
    assert engine.get_npc_states() == []
    nav = engine.get_navigation_state()
    assert nav["sensed_lead_gap_m"] is None
    assert nav["sensed_lead_speed_kmh"] is None


def test_sensed_lead_is_surfaced_in_navigation_state():
    """End-to-end: a same-lane NPC placed just ahead of the ego is visible
    through get_navigation_state()'s sensed_lead_* fields (not through any
    other route to NPC data)."""
    from app.services.traffic import NpcVehicle, EGO_LANE_OFFSET_M

    engine = PhysicsEngine()
    route = _straight_route()
    engine.set_destination(*route[-1])
    engine.set_route(route)

    engine.traffic.npcs = [NpcVehicle(
        id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=20.0,
        station_m=engine.current_station_m + 30.0,
    )]
    engine.last_update_time = time.time() - 0.1
    engine.update("Maintain Speed")

    nav = engine.get_navigation_state()
    assert nav["sensed_lead_gap_m"] is not None
    assert nav["sensed_lead_speed_kmh"] == pytest.approx(20.0)


def test_new_route_respawns_traffic_for_the_new_length():
    engine = PhysicsEngine()
    route_a = _straight_route(n=50)
    engine.set_destination(*route_a[-1])
    engine.set_route(route_a)
    traffic_a = engine.traffic
    assert traffic_a is not None

    route_b = _corner_route()
    engine.set_destination(*route_b[-1])
    engine.set_route(route_b)
    assert engine.traffic is not traffic_a
    assert engine.sensed_lead is None


def test_station_m_advances_with_real_movement_even_while_route_index_is_small():
    """Regression: station_m was computed as
    station_distances[route_index] - waypoint_dist, where waypoint_dist is the
    distance to a LOOKAHEAD point (potentially many waypoints ahead), not to
    route_index's own waypoint -- and since station_distances[0] == 0.0,
    while route_index sat at 0 early in a drive this produced 0 regardless of
    real movement. Silently broke P6-1b traffic recycling/sensing, which is
    keyed off the ego's station."""
    engine = PhysicsEngine()
    route = _straight_route(n=80)
    engine.set_destination(*route[-1])
    engine.set_route(route)

    _drive(engine, "Maintain Speed", ticks=40)
    assert engine.speed_kmh > 5.0, "test setup: car must actually be moving"
    assert engine.current_station_m > 0.0, "station must reflect real forward progress, not stay pinned at 0"


def test_car_does_not_spiral_off_a_long_dense_route():
    """Regression (P6-1d): the steering lookahead was `max(8, 0.8*v)` ~= 11 m,
    which only worked because it happened to step past one raw OSRM waypoint
    (~21 m average spacing), giving ~21 m of lookahead by accident. Uniform
    5 m resampling removed that accident, leaving a genuinely 11 m lookahead --
    under a second of travel at cruise -- and the car over-steered, overshot
    and spiralled away from the route (cross-track grew 6 m -> 60 m while the
    heading rotated through 100+ degrees). Lookahead is now an explicit
    distance, decoupled from waypoint spacing."""
    engine = PhysicsEngine()
    # A long, densely-sampled straight route (5 m spacing, like the smoothed
    # OSRM output) that the car must track without diverging.
    route = [(37.7749 + i * 0.000045, -122.4194) for i in range(300)]
    engine.set_destination(*route[-1])
    engine.set_route(route)

    _drive(engine, "Maintain Speed", ticks=400)

    cross_track = min(
        engine.calculate_distance(engine.lat, engine.lng, route[j][0], route[j][1])
        for j in range(len(route))
    )
    assert cross_track < 15.0, "car must track a dense route, not spiral away from it"


# ---------------------------------------------------------------------------
# P6-2: Frenet local planner + pure-pursuit lateral control. Replaces the
# P6-1 proportional heading controller for any ROUTED bicycle-controller
# drive. See app/services/frenet.py and app/services/planner.py for the
# underlying unit tests of the projection math and candidate scoring; these
# tests lock in the integration into PhysicsEngine.update().
# ---------------------------------------------------------------------------

def test_bicycle_controller_settles_near_the_lane_centre_on_a_straight_route():
    """The headline P6-2 fix: the car targets a LANE (d ~= LANE_CENTER_D_M),
    not the raw route centreline (d = 0), which is what P6-1 actually drove."""
    from app.services.planner import LANE_CENTER_D_M
    route = _straight_route(n=150)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=400)

    settled_offsets = []
    for _ in range(50):
        engine.last_update_time = time.time() - 0.1
        engine.update("Maintain Speed")
        settled_offsets.append(engine.current_lateral_offset_m)

    # Pure pursuit is known to settle with a small non-zero steady-state
    # lateral offset (a function of lookahead distance and the rate-limited
    # lateral TARGET tracking, not a bug) -- the acceptance bar is "bounded
    # lateral error" (PHASE_6_TASK_BOARD.md's P6-2 acceptance), not exact
    # convergence to zero error. 1.5m tolerance (well under half a 3.5m lane)
    # comfortably separates "on centreline" (d~=0, the P6-1 behaviour this
    # replaces) from "in the lane" (d~=3.5).
    mean_offset = sum(settled_offsets) / len(settled_offsets)
    assert mean_offset == pytest.approx(LANE_CENTER_D_M, abs=1.5)
    assert max(abs(o - LANE_CENTER_D_M) for o in settled_offsets) < 2.5


def test_frenet_lateral_offset_is_near_zero_for_the_legacy_controller_on_centreline():
    """Legacy is the P6-6 A/B control and must keep chasing the raw
    centreline (its steering law is untouched by P6-2) -- the more accurate
    Frenet station/offset bookkeeping is a diagnostic improvement layered
    underneath it, not a behaviour change."""
    route = _straight_route(n=100)
    engine = PhysicsEngine(controller="legacy")
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=200)
    assert abs(engine.current_lateral_offset_m) < 1.0
    assert engine.planner_candidates == [], "legacy must never touch the P6-2 planner"


def test_planner_candidates_are_populated_and_include_the_chosen_path():
    route = _straight_route(n=100)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=50)

    candidates = engine.get_planner_candidates()
    assert len(candidates) > 1, "candidate SET (plural) must be exposed, not just the winner"
    chosen = [c for c in candidates if c["is_chosen"]]
    assert len(chosen) == 1
    assert chosen[0]["d_target"] == pytest.approx(engine.planner_chosen_d_m)


def test_navigation_state_exposes_lateral_offset_for_the_frontend():
    """The frontend's hard-coded LANE_OFFSET_M render hack (P6-2's build
    note) is replaced by rendering at this real backend value."""
    route = _straight_route(n=100)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=50)
    nav = engine.get_navigation_state()
    assert "lateral_offset_m" in nav
    assert nav["lateral_offset_m"] == pytest.approx(engine.current_lateral_offset_m)


def test_pure_pursuit_steering_output_is_continuous_no_teleport_jumps():
    """Acceptance: 'steering output is continuous'. The rate limiter is
    structural (same clamp P6-1 already relies on), but this locks in that
    the NEW Frenet/pure-pursuit path also respects it, tick to tick."""
    dt = 0.1
    route = _corner_route()
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)

    prev = engine.steering_angle_rad
    for _ in range(300):
        # Measure the ACTUAL dt update() will see (like the jerk test above):
        # under full-suite wall-clock load this loop's own overhead can push
        # real elapsed time past the nominal 0.1s, which would make a
        # nominal-dt bound flaky rather than a genuine regression signal.
        set_time = time.time()
        engine.last_update_time = set_time - dt
        engine.update("Accelerate")
        actual_dt = max(dt, time.time() - set_time)
        max_allowed_step = PhysicsEngine.STEER_RATE_MAX_RADPS * actual_dt * 1.01
        assert abs(engine.steering_angle_rad - prev) <= max_allowed_step
        prev = engine.steering_angle_rad


def test_station_m_from_frenet_projection_advances_smoothly_with_speed():
    """Regression-of-improvement: current_station_m now comes from an exact
    continuous projection (frenet.py), not a waypoint-snapped approximation
    -- consecutive ticks should advance roughly with distance actually
    travelled, not in discrete waypoint-sized steps."""
    dt = 0.1
    route = _straight_route(n=150)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=100)  # get up to cruising speed

    prev_station = engine.current_station_m
    for _ in range(20):
        engine.last_update_time = time.time() - dt
        engine.update("Maintain Speed")
        speed_mps = engine.speed_kmh / 3.6
        # Generous slack: the test drives dt via wall-clock arithmetic
        # (last_update_time = time.time() - dt), so the ACTUAL measured dt
        # inside update() is dt plus whatever wall-clock time this test loop
        # itself consumes -- not a station-tracking precision bound.
        max_possible_advance = speed_mps * dt + 2.0
        assert 0.0 <= engine.current_station_m - prev_station <= max_possible_advance
        prev_station = engine.current_station_m


def test_no_route_bicycle_steering_falls_back_to_proportional_heading_control():
    """A Frenet frame needs a route to project onto -- without one, the P6-1
    proportional heading controller chasing target_lat/target_lng directly
    must still work exactly as before P6-2."""
    engine = PhysicsEngine()
    engine.target_lat = engine.lat + 0.02
    engine.target_lng = engine.lng + 0.02
    _drive(engine, "Accelerate", ticks=100)
    assert engine.speed_kmh > 0.0
    assert engine.frenet_frame is None
    assert engine.current_lateral_offset_m == 0.0
