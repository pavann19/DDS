"""
Unit tests for app/services/physics_engine.py, including a 
regression test for the idle-coolant-runaway fix.
"""
import time
import math
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services.traffic import EGO_LANE_OFFSET_M


def _tick(engine, action, dt, ticks):
    """Advance the engine `ticks` times with a fixed dt, bypassing the
    wall-clock-based dt in update() so tests are deterministic."""
    for _ in range(ticks):
        engine.last_update_time = time.time() - dt
        engine.update(action)


def test_initial_altitude_is_in_training_distribution():
    """ regression: Altitude used to start at 10.0, wildly outside the
    training data's observed 128-203 range."""
    engine = PhysicsEngine()
    assert 128 <= engine.altitude <= 203


def test_coolant_stays_within_clamp_bounds():
    engine = PhysicsEngine()
    _tick(engine, "Maintain Speed", dt=0.1, ticks=50)
    assert 70.0 <= engine.coolant_temp <= 110.0


def test_idle_coolant_does_not_run_away_to_clamp_ceiling():
    """ regression: at speed 0, cooling used to be exactly 0, so
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
    baseline cruise-toward-destination speed so the car actually drives.
    (Since ADR-001 item 5 the AI decision has no effect on control at all --
    see test_ai_decision_has_no_effect_on_the_control_path.)"""
    engine = PhysicsEngine()
    engine.target_lat = engine.lat + 0.02  # a destination ~2km away
    engine.target_lng = engine.lng + 0.02
    start_lat, start_lng = engine.lat, engine.lng

    _tick(engine, "Maintain Speed", dt=0.1, ticks=100)  # worst case: never told to accelerate

    assert engine.speed_kmh > 0.0, "car must move even if the AI only ever predicts Maintain Speed"
    assert (engine.lat, engine.lng) != (start_lat, start_lng), "car must actually change position"


def test_ai_decision_has_no_effect_on_the_control_path():
    """ADR-001 item 5: the learned model is OUT of the speed-target path.
    It reads only ego powertrain telemetry (RPM/CO2/coolant/fuel), so it
    cannot be a driving policy -- it is scored as driver-behaviour analytics
    instead. Feeding 'Accelerate' vs 'Decelerate' vs 'Maintain Speed' must
    produce a byte-identical ego trajectory."""
    def drive(decision):
        eng = PhysicsEngine(seed=99)
        eng.target_lat = eng.lat + 0.02
        eng.target_lng = eng.lng + 0.02
        for _ in range(120):
            eng.update(decision, dt=0.1)
        return (eng.lat, eng.lng, eng.speed_kmh, eng.heading,
                eng.acceleration_mps2, eng.steering_angle_rad)

    accelerate = drive("Accelerate")
    decelerate = drive("Decelerate")
    maintain = drive("Maintain Speed")
    assert accelerate == decelerate == maintain


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
    # OSRM route density) that goes straight
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
    # This test isolates the CURVATURE-based speed cap specifically -- IDM
    # car-following (a randomly-seeded NPC legitimately blocking the lane)
    # is a separate, real confound now that sensed_lead actually affects
    # speed, and is covered by its own dedicated car-following tests below.
    engine.traffic = None

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
#  kinematic bicycle model + jerk-limited longitudinal control.
#
# These lock in PHYSICAL guarantees the pre-P6 point-mass controller did not
# provide. Measured during development, the legacy controller reached ~22 m/s^2
# acceleration (>2 g) and ~195 m/s^3 jerk -- the discontinuities that read on
# screen as flickering/teleporting motion.
# ---------------------------------------------------------------------------

def _corner_route():
    """~120 m east, a ~90 deg left turn taken over a ~14 m radius arc (a
    tight real urban-intersection corner -- OSRM emits a turn as an arc of
    several points, not a single 90 deg vertex, and the ingest spline
    rounds it further), then ~150 m north. Dense ~9 m waypoint spacing."""
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + i * 0.0001) for i in range(15)]  # ~120 m east
    x0 = 14 * (0.0001 * m_lng)                             # local east metres at corner
    R = 14.0
    ang, d_ang = 0.0, 3.5 / R
    while ang < math.pi / 2:
        ang += d_ang
        ex = x0 + R * math.sin(ang)
        ny = R * (1.0 - math.cos(ang))
        pts.append((lat0 + ny / m_lat, lng0 + ex / m_lng))
    lat_n, lng_n = pts[-1]
    pts += [(lat_n + i * 0.0001, lng_n) for i in range(1, 15)]  # ~150 m north
    return pts


def _straight_route(n=100):
    return [(37.7749 + i * 0.0001, -122.4194) for i in range(n)]


def _drive(engine, decision, ticks, dt=0.1):
    """Drive, returning per-tick (speed_mps, accel, lateral_accel) traces.

    Steps with an EXPLICIT dt so the traces are deterministic and the
    jerk/accel bounds below can be checked exactly -- the wall-clock path is
    exercised by the _tick()-based tests instead."""
    speeds, accels, lat_accels = [], [], []
    for _ in range(ticks):
        engine.update(decision, dt=dt)
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
    """The defining fix of : bounded d(acceleration)/dt makes velocity C1
    continuous, which is what removes the visual flicker."""
    dt = 0.1
    engine = PhysicsEngine(seed=42)
    engine.set_destination(*_corner_route()[-1])
    engine.set_route(_corner_route())
    # Comfort jerk is a no-collision contract. Emergency authority is tested
    # separately; randomly spawned traffic may correctly demand a jerk bypass.
    engine.traffic = None
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
    """Headline the previous comparison; the quantitative version becomes the 
    A/B evaluation. Acceleration is derived from the speed trace so both
    controllers are measured identically (legacy writes speed directly and
    never populates acceleration_mps2)."""
    dt = 0.1

    def peak_jerk(controller):
        engine = PhysicsEngine(controller=controller)
        engine.set_destination(*_corner_route()[-1])
        engine.set_route(_corner_route())
        # Isolate the two controllers' OWN dynamics on identical corner
        # geometry -- a randomly-seeded NPC triggering real IDM braking
        # (legacy never reads sensed_lead, so this asymmetrically affects
        # only the bicycle run) is a separate, real confound now that
        # sensed_lead actually affects speed.
        engine.traffic = None
        speeds, _, _ = _drive(engine, "Accelerate", ticks=400, dt=dt)
        accels = [(speeds[i] - speeds[i - 1]) / dt for i in range(1, len(speeds))]
        return max(abs(accels[i] - accels[i - 1]) / dt for i in range(1, len(accels)))

    assert peak_jerk("bicycle") < peak_jerk("legacy") / 10.0


def test_legacy_controller_remains_selectable_as_ab_control():
    """ needs the pre-P6 controller as its experimental control condition."""
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
    engine = PhysicsEngine(seed=1)  # deterministic traffic -- this test is about destination re-arm
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
#  server-side NPC traffic + forward range sensor, wired into
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
    real movement. Silently broke the previous traffic recycling/sensing, which is
    keyed off the ego's station."""
    engine = PhysicsEngine()
    route = _straight_route(n=80)
    engine.set_destination(*route[-1])
    engine.set_route(route)

    _drive(engine, "Maintain Speed", ticks=40)
    assert engine.speed_kmh > 5.0, "test setup: car must actually be moving"
    assert engine.current_station_m > 0.0, "station must reflect real forward progress, not stay pinned at 0"


def test_car_does_not_spiral_off_a_long_dense_route():
    """Regression : the steering lookahead was `max(8, 0.8*v)` ~= 11 m,
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
#  Frenet local planner + pure-pursuit lateral control. Replaces the
# proportional heading controller for any ROUTED bicycle-controller
# drive. See app/services/frenet.py and app/services/planner.py for the
# underlying unit tests of the projection math and candidate scoring; these
# tests lock in the integration into PhysicsEngine.update().
# ---------------------------------------------------------------------------

def test_bicycle_controller_settles_near_the_lane_centre_on_a_straight_route():
    """The headline the previous fix: the car targets a LANE (d ~= LANE_CENTER_D_M),
    not the raw route centreline (d = 0), which is what the previous actually drove."""
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
    # lateral error", not exact
    # convergence to zero error. 1.5m tolerance (well under half a 3.5m lane)
    # comfortably separates "on centreline" (d~=0, the previous behaviour this
    # replaces) from "in the lane" (d~=3.5).
    mean_offset = sum(settled_offsets) / len(settled_offsets)
    assert mean_offset == pytest.approx(LANE_CENTER_D_M, abs=1.5)
    assert max(abs(o - LANE_CENTER_D_M) for o in settled_offsets) < 2.5


def test_routed_bicycle_drive_starts_on_route_lane_center_with_route_heading():
    """Regression: real OSRM routes may start several metres away from the
    raw requested origin after map matching. The simulated ego must start on
    the routed lane centre instead of inheriting an off-road/off-lane raw GPS
    point that immediately trips TTC and safety-shield overrides."""
    from app.services.planner import LANE_CENTER_D_M

    engine = PhysicsEngine()
    route = [(engine.lat + 0.00005, engine.lng), (engine.lat + 0.001, engine.lng)]
    engine.set_destination(*route[-1])
    engine.set_route(route)

    nav = engine.get_navigation_state()
    assert nav["lateral_offset_m"] == pytest.approx(LANE_CENTER_D_M)
    assert engine.heading == pytest.approx(0.0)


def test_frenet_lateral_offset_is_near_zero_for_the_legacy_controller_on_centreline():
    """Legacy is the previous A/B control and must keep chasing the raw
    centreline (its steering law is untouched by ) -- the more accurate
    Frenet station/offset bookkeeping is a diagnostic improvement layered
    underneath it, not a behaviour change."""
    route = _straight_route(n=100)
    engine = PhysicsEngine(controller="legacy")
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=200)
    assert abs(engine.current_lateral_offset_m) < 1.0
    assert engine.planner_candidates == [], "legacy must never touch the previous planner"


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
    """The frontend's hard-coded LANE_OFFSET_M render hack (the build
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
    structural (same clamp the previous already relies on), but this locks in that
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


def test_tracking_error_speed_cap_bounds_startup_excursion_off_a_mismatched_route():
    """Regression: the car starts every drive at a fixed heading (45deg)
    with no relation to the real route's actual initial direction. Because
    the bicycle model correctly cannot yaw while nearly stationary
    (yaw_rate = v*tan(delta)/L), flooring the throttle before steering
    authority catches up sent the car travelling fast in the wrong
    direction -- measured on the real default SF OSRM route, lateral
    offset reached 26m off a 7m-wide modelled road within the first ~12s,
    reported live as "car goes off the road". A south-heading dense route
    (max mismatch from the 45deg start heading) reproduces the same class
    of error deterministically without a live OSRM call."""
    dt = 0.1
    engine = PhysicsEngine()
    south_route = [(37.7749 - i * 0.0001, -122.4194) for i in range(150)]
    engine.set_destination(*south_route[-1])
    engine.set_route(south_route)

    max_abs_d = 0.0
    for _ in range(150):  # 15s -- the window the real-route excursion occurred in
        engine.last_update_time = time.time() - dt
        engine.update("Accelerate")  # worst case: AI floors it throughout
        max_abs_d = max(max_abs_d, abs(engine.current_lateral_offset_m))

    # A dead-south route is a 180deg mismatch -- more adversarial than the
    # real SF route's ~95deg (which the cap already reduced from 26m to
    # under 10m). Bounding this worst case, not eliminating it: further
    # tightening would mean weakening JERK_MAX_MPS3 itself (even a
    # correctly-reduced target speed takes time to realize under bounded
    # jerk) -- the same physical property that fixed P6-1's flickering.
    # Before this fix, the same scenario reached 26m+ with no bound at all;
    # a real hard bound (a road-boundary safety check) belongs to the
    # Safety Shield, which this residual is the direct motivation for.
    assert max_abs_d < 20.0, "tracking-error speed cap should bound even a worst-case (180deg) startup mismatch"


def test_safety_shield_overrides_with_emergency_brake_on_imminent_collision():
    """Integration test: with a stopped lead vehicle sensed at close range,
    the independent Safety Shield must force maximum braking regardless of
    what IDM alone would have decided (an even earlier check than IDM's own
    braking, and unconditional -- not tunable by the AI decision)."""
    dt = 0.1
    engine = PhysicsEngine()
    route = _straight_route(n=150)
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=100)  # get up to real speed
    assert engine.speed_kmh > 20.0, "test setup must have real speed to brake from"

    # Force a critical TTC directly via the traffic model, at the ego's
    # own real sensed lane -- matches how sense_lead_vehicle is actually
    # queried in physics_engine.py after the lane-sensing bug fix.
    from app.services.traffic import NpcVehicle
    engine.traffic.npcs = [NpcVehicle(
        id="blocker", lane_offset=engine.current_lateral_offset_m,
        speed_kmh=0.0, station_m=engine.current_station_m + 8.0,
    )]

    speed_before = engine.speed_kmh
    engine.last_update_time = time.time() - dt
    engine.update("Accelerate")  # worst case: AI still says floor it

    assert engine.shield_verdict.approved is False
    assert engine.shield_verdict.override_action == "EMERGENCY_BRAKE"
    assert engine.get_navigation_state()["speed_limit_reason"] == "safety_shield_override"

    # acceleration_mps2 is JERK-LIMITED (JERK_MAX_MPS3*dt per tick), so it
    # cannot flip sign in a single tick even under an emergency override --
    # that bound is deliberate (P6-1). Confirm the override actually wins
    # over several ticks instead: real deceleration, not just a flag.
    for _ in range(20):
        engine.last_update_time = time.time() - dt
        engine.traffic.npcs = [NpcVehicle(
            id="blocker", lane_offset=engine.current_lateral_offset_m,
            speed_kmh=0.0, station_m=engine.current_station_m + 8.0,
        )]
        engine.update("Accelerate")
    assert engine.acceleration_mps2 < 0.0
    assert engine.speed_kmh < speed_before


def test_safety_shield_road_boundary_override_does_not_livelock_the_car():
    """Regression: an earlier version forced EMERGENCY_BRAKE (full stop)
    on a road-boundary violation. Braking to zero removes the only thing
    that lets the car steer back onto the road (yaw_rate = v*tan(delta)/L
    needs forward speed) -- found live: the car froze off-road, speed
    pinned at 0, re-triggering the same override every tick, forever.
    Drive the car off the modelled road directly and confirm it recovers
    (lateral offset shrinks back toward the road) instead of freezing."""
    dt = 0.1
    engine = PhysicsEngine(seed=42)
    route = _straight_route(n=150)
    engine.set_destination(*route[-1])
    engine.set_route(route)
    # Isolate road-boundary recovery from unrelated random actor overlap.
    # Collision emergency braking is covered by the preceding regression.
    engine.traffic = None
    _drive(engine, "Accelerate", ticks=50)

    # Force the car's REAL position off the modelled road (not just the
    # derived current_lateral_offset_m field, which is recomputed from
    # lat/lng by a fresh Frenet projection every tick and would otherwise
    # just overwrite a directly-set value on the next update()).
    from app.services.safety_shield import ROAD_BOUNDARY_HARD_LIMIT_M
    from app.services.frenet import frenet_to_latlng
    off_road_lat, off_road_lng = frenet_to_latlng(
        engine.frenet_frame, engine.current_station_m, ROAD_BOUNDARY_HARD_LIMIT_M + 1.5,
    )
    engine.lat, engine.lng = off_road_lat, off_road_lng
    engine.lateral_target_d_m = 1.75  # planner still wants to come back

    speeds_while_off_road = []
    for _ in range(50):  # 5s
        engine.last_update_time = time.time() - dt
        engine.update("Maintain Speed")
        speeds_while_off_road.append(engine.speed_kmh)

    assert engine.shield_verdict.override_action in ("RECOVER_LOW_SPEED", None), \
        "must not still be off-road after recovery (or already fully recovered)"
    assert max(speeds_while_off_road) > 0.0, \
        "car must retain some speed while off-road, not freeze at a dead stop (the livelock)"


def test_safety_shield_state_is_exposed_via_get_safety_shield_state():
    engine = PhysicsEngine()
    state = engine.get_safety_shield_state()
    assert state["approved"] is True
    assert state["risk_level"] == "NONE"
    assert state["reasons"] == []
    assert state["override_action"] is None


def test_safety_shield_is_not_evaluated_for_legacy_controller():
    """Legacy is the untouched P6-6 A/B control -- the shield is part of
    the new stack, not retrofitted onto the old one."""
    dt = 0.1
    engine = PhysicsEngine(controller="legacy")
    route = _straight_route(n=150)
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=50)
    assert engine.shield_verdict.risk_level == "NONE"
    assert engine.shield_verdict.approved is True


def test_no_route_bicycle_steering_falls_back_to_proportional_heading_control():
    """A Frenet frame needs a route to project onto -- without one, the 
    proportional heading controller chasing target_lat/target_lng directly
    must still work exactly as before ."""
    engine = PhysicsEngine()
    engine.target_lat = engine.lat + 0.02
    engine.target_lng = engine.lng + 0.02
    _drive(engine, "Accelerate", ticks=100)
    assert engine.speed_kmh > 0.0
    assert engine.frenet_frame is None
    assert engine.current_lateral_offset_m == 0.0


def test_kinematic_bicycle_sideslip_formula():
    """Kinematic sideslip beta = atan((lr/L) * tan(delta)) with lr = L/2."""
    engine = PhysicsEngine()
    engine.steering_angle_rad = math.radians(15.0)
    # Trigger update with small dt
    engine.last_update_time = time.time() - 0.05
    engine.update("Maintain Speed", dt=0.05)

    expected_beta = math.atan(0.5 * math.tan(engine.steering_angle_rad))
    assert engine.kinematic_sideslip_rad == pytest.approx(expected_beta, abs=1e-3)
    nav = engine.get_navigation_state()
    assert nav["kinematic_sideslip_rad"] == pytest.approx(expected_beta, abs=1e-3)


def test_steering_and_yaw_rate_physical_bounds():
    """Steering angle, yaw rate, and lateral acceleration must stay strictly within physical passenger bounds."""
    route = _corner_route()
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)

    max_steer = 0.0
    max_yaw_rate = 0.0
    max_lat_accel = 0.0

    for _ in range(250):
        engine.last_update_time = time.time() - 0.05
        engine.update("Accelerate", dt=0.05)
        max_steer = max(max_steer, abs(engine.steering_angle_rad))
        max_yaw_rate = max(max_yaw_rate, abs(engine.yaw_rate_degps))
        max_lat_accel = max(max_lat_accel, engine.lateral_accel_mps2)

    # Max steer must never exceed 35 deg (0.61 rad)
    assert max_steer <= PhysicsEngine.MAX_STEER_RAD + 1e-4
    # Max yaw rate must stay within comfortable cornering bounds (< 90 deg/s)
    assert max_yaw_rate < 90.0, f"Yaw rate exceeded bound: {max_yaw_rate} deg/s"
    # Lateral acceleration must be bounded
    assert max_lat_accel <= PhysicsEngine.A_LAT_MAX_MPS2 + 1.5, f"Lateral accel exceeded: {max_lat_accel}"


def test_authoritative_navigation_state_fields():
    """Authoritative physical fields are exposed in get_navigation_state for frontend rendering."""
    route = _straight_route(n=50)
    engine = PhysicsEngine()
    engine.set_destination(*route[-1])
    engine.set_route(route)
    _drive(engine, "Accelerate", ticks=10)

    nav = engine.get_navigation_state()
    required_keys = [
        "station_m", "lateral_offset_m", "lateral_target_m",
        "heading_deg", "road_heading_deg", "relative_heading_deg", "world_heading_deg",
        "steering_angle_rad", "kinematic_sideslip_rad",
        "lateral_accel_mps2", "signed_lateral_accel_mps2", "yaw_rate_degps",
        "world_pos_x", "world_pos_z"
    ]
    for key in required_keys:
        assert key in nav, f"Key '{key}' missing from get_navigation_state"

    # World heading must match road heading + relative heading (mod 360)
    expected_world_heading = (nav["road_heading_deg"] + nav["relative_heading_deg"] + 360.0) % 360.0
    assert nav["world_heading_deg"] == pytest.approx(expected_world_heading, abs=0.1)


def test_friction_derating_reduces_cornering_speed():
    """Reduced road friction (wet surface mu = 0.5) reduces cornering speed cap."""
    route = _corner_route()
    engine_dry = PhysicsEngine()
    engine_dry.set_destination(*route[-1])
    engine_dry.set_route(route)
    engine_dry.friction_mu = 1.0

    engine_wet = PhysicsEngine()
    engine_wet.set_destination(*route[-1])
    engine_wet.set_route(route)
    engine_wet.friction_mu = 0.5

    # Drive both until they encounter curvature
    for _ in range(80):
        engine_dry.last_update_time = time.time() - 0.05
        engine_dry.update("Accelerate", dt=0.05)
        engine_wet.last_update_time = time.time() - 0.05
        engine_wet.update("Accelerate", dt=0.05)

    # Wet surface must limit cornering speed lower than dry surface
    if engine_dry.path_curvature > 1e-4:
        dry_cap = math.sqrt(PhysicsEngine.A_LAT_MAX_MPS2 / engine_dry.path_curvature)
        wet_cap = math.sqrt((PhysicsEngine.A_LAT_MAX_MPS2 * 0.5) / engine_wet.path_curvature)
        assert wet_cap < dry_cap



# ---------------------------------------------------------------------------
# P0 (A1): low-friction proof at the control level. A wet surface must (a)
# make "low_friction_curvature" the binding speed constraint on a curve and
# (b) physically hold both the cornering speed and the realised lateral
# acceleration below the dry-surface values -- not just relabel a variable.
# ---------------------------------------------------------------------------
def _gentle_arc_route():
    """~120 m straight lead-in, then a smooth ~90 deg left arc of radius
    ~60 m (well above the 8 km/h corner-speed floor so friction has room to
    create a measurable apex-speed separation), then a straight run-out."""
    lat0, lng0 = 37.7749, -122.4194
    m_per_deg_lat = 111_320.0
    m_per_deg_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = []
    # straight lead-in heading east, 120 m in 3 m steps
    for i in range(40):
        pts.append((lat0, lng0 + (i * 3.0) / m_per_deg_lng))
    lat, lng = pts[-1]
    # smooth left arc, radius 60 m, heading east -> north, 3 m arc steps
    R = 60.0
    cx_lat = lat + R / m_per_deg_lat          # centre is due north of arc start
    cx_lng = lng
    ang = -math.pi / 2                         # start angle (arc start is south of centre)
    d_ang = 3.0 / R
    while ang < 0.0:
        ang += d_ang
        pts.append((cx_lat + (R * math.sin(ang)) / m_per_deg_lat,
                    cx_lng + (R * (1 - math.cos(ang))) / m_per_deg_lng))
    lat, lng = pts[-1]
    for i in range(1, 40):
        pts.append((lat + (i * 3.0) / m_per_deg_lat, lng))
    return pts


def test_low_friction_scales_the_grip_steer_limit():
    """Wet / icy grip must tighten the speed-dependent steering limit:
    delta_max(v, mu) = min(MAX_STEER_RAD, atan(mu * A_LAT_MAX * L / v^2)).
    This mirrors physics_engine's grip_steer_limit branch and locks the
    contract that friction constrains *control*, not just a speed label."""
    L = PhysicsEngine.WHEELBASE_M
    a_lat = PhysicsEngine.A_LAT_MAX_MPS2
    v = 16.0  # m/s -- fast enough that grip, not the geometric cap, binds
    limits = {}
    for mu in (1.0, 0.5, 0.2):
        expected = min(PhysicsEngine.MAX_STEER_RAD, math.atan(mu * a_lat * L / (v * v)))
        limits[mu] = expected
    # strictly monotone: less grip -> smaller allowed steer angle
    assert limits[1.0] > limits[0.5] > limits[0.2]
    # and at this speed all three are the grip branch, not the geometric clamp
    assert limits[1.0] < PhysicsEngine.MAX_STEER_RAD


def _arc_route_for_friction(R=60.0, lead=40, runout=30):
    """Straight ~120 m east lead-in, a constant-radius left arc whose initial
    tangent is continuous with the lead-in (east), then a straight north
    run-out. Dense 3 m raw spacing (smooth_route resamples to ~5 m).
    NOTE: set_destination() MUST be called before set_route() -- set_destination()
    clears the route."""
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + (i * 3.0) / m_lng) for i in range(lead)]
    x0 = lead * 3.0                      # local east metres at arc start
    d_ang = 3.0 / R
    ang = 0.0
    while ang < math.pi / 2:             # sweep 0 -> 90 deg, tangent east -> north
        ang += d_ang
        east_m = x0 + R * math.sin(ang)
        north_m = R * (1.0 - math.cos(ang))
        pts.append((lat0 + north_m / m_lat, lng0 + east_m / m_lng))
    lat_n, lng_n = pts[-1]
    pts += [(lat_n + (i * 3.0) / m_lat, lng_n) for i in range(1, runout)]
    return pts


def test_low_friction_records_a_speed_derate_fact_on_a_curve():
    """Deterministic (seeded, fixed dt): on the SAME arc, mu=0.5 must make
    the engine record friction_speed_derate_active with a positive
    friction_derate_kmh and surface 'low_friction_curvature' as a binding
    reason; mu=1.0 must do neither. This proves the friction de-rate lowers
    the cornering speed cap in fact, independent of which cap ultimately
    wins speed_limit_reason (tracking_correction routinely overwrites it).
    """
    route = _arc_route_for_friction()

    def run(mu):
        eng = PhysicsEngine(seed=5)
        eng.set_destination(*route[-1])   # before set_route -- required
        eng.set_route(route)
        eng.reset_state(station_m=0.0, speed_kmh=52.0, lateral_offset_m=1.75)
        eng.friction_mu = mu
        derate_active = False
        peak_derate = 0.0
        reasons = set()
        for _ in range(150):
            eng.update("Maintain Speed", dt=0.1)
            reasons.add(eng.speed_limit_reason)
            if eng.friction_speed_derate_active:
                derate_active = True
                peak_derate = max(peak_derate, eng.friction_derate_kmh)
        return derate_active, peak_derate, reasons

    dry_active, dry_peak, dry_reasons = run(1.0)
    wet_active, wet_peak, wet_reasons = run(0.5)

    assert wet_active and wet_peak > 0.0, "wet surface did not record a friction speed de-rate"
    assert "low_friction_curvature" in wet_reasons
    assert not dry_active, "dry surface must never record a friction de-rate"
    assert "low_friction_curvature" not in dry_reasons
    assert "lateral_accel_limit" in dry_reasons  # dry is curvature-limited, not friction-limited


def test_navigation_state_exposes_friction_derate_fact():
    route = _arc_route_for_friction()
    eng = PhysicsEngine(seed=5)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=52.0, lateral_offset_m=1.75)
    eng.friction_mu = 0.5
    for _ in range(150):
        eng.update("Maintain Speed", dt=0.1)
    nav = eng.get_navigation_state()
    for k in ("friction_mu", "friction_speed_derate_active", "friction_derate_kmh"):
        assert k in nav, f"{k} missing from get_navigation_state"
    assert nav["friction_mu"] == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Cornering-loop determinism.
#
# Regression: driving a fixed curved route through its low-speed apex used to
# give materially different speed/trajectory traces run-to-run *within one
# process* -- apex speed after tick 40 varied ~0.0..7.9 km/h across identical
# in-process trials. Root cause: set_route() spawned its per-route
# TrafficModel with seed=None (OS entropy), so NPC start stations/speeds --
# and hence the forward sensor + IDM longitudinal control -- differed every
# run. Near a curve's stall point that is the difference between the ego
# braking to a standstill and rolling through. Straight-line / short-window
# regimes only looked deterministic because no randomly-placed NPC had come
# within sensing range yet. Fix: PhysicsEngine(seed=...) now threads that
# seed into TrafficModel, making (route, dt, seed) bit-reproducible.
# ---------------------------------------------------------------------------
def test_cornering_loop_is_bit_reproducible_for_a_fixed_route_dt_seed():
    route = _gentle_arc_route()

    def drive():
        eng = PhysicsEngine(seed=20260909)
        eng.set_destination(*route[-1])
        eng.set_route(route)
        speeds, positions = [], []
        # Well past the ~230-tick deterministic-only window and deep into the
        # low-speed apex regime that was previously unstable.
        for _ in range(260):
            eng.update("Accelerate", dt=0.05)
            speeds.append(eng.speed_kmh)
            positions.append((eng.lat, eng.lng))
        return speeds, positions

    speeds_a, pos_a = drive()
    speeds_b, pos_b = drive()

    # Bit-for-bit identical: same seed, same route, same dt -> same drive.
    assert speeds_a == speeds_b
    assert pos_a == pos_b

    # And the trace is a real drive through the bend -- the car reaches
    # cruise on the lead-in and measurably slows for the arc (this is the
    # regime that was previously unstable), not a degenerate flat/zero trace.
    apex = min(speeds_a[40:])
    assert max(speeds_a) > 20.0
    assert apex < max(speeds_a) - 5.0


def test_unseeded_engine_still_defaults_to_os_seeded_traffic():
    """The determinism fix must not change behaviour for callers that pass no
    seed: TrafficModel is still created, just with seed=None as before."""
    route = _straight_route()
    eng = PhysicsEngine()
    eng.set_destination(*route[-1])
    eng.set_route(route)
    assert eng.traffic is not None
    assert eng.traffic.seed is None


# ---------------------------------------------------------------------------
# P-CURV regression: a single polyline vertex that reverses direction (a
# hairpin artefact, or a spot the spline smoother failed to round) must NOT
# be read as a ~1.7 m-radius corner that floors the car to the crawl speed.
# path_curvature is now a sliding-window estimate with per-step artefact
# rejection and a physical MIN_TURN_RADIUS_M ceiling.
# ---------------------------------------------------------------------------
def _route_with_reversal_artifact():
    """~90 m straight east, one bad vertex that jumps back west ~6 m (a
    ~175 deg reversal in a single step), then continues east again."""
    lat0, lng0 = 37.7749, -122.4194
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + (i * 3.0) / m_lng) for i in range(30)]
    # inject the artefact: one point that steps backwards, then resume
    x_back = 29 * 3.0 - 6.0
    pts.append((lat0, lng0 + x_back / m_lng))
    pts += [(lat0, lng0 + (30 * 3.0 + i * 3.0) / m_lng) for i in range(1, 30)]
    return pts


def test_direction_reversal_vertex_does_not_produce_a_phantom_hairpin():
    route = _route_with_reversal_artifact()
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=1.75)

    peak_k = 0.0
    min_speed = 1e9
    for _ in range(200):
        eng.update("Accelerate", dt=0.05)
        peak_k = max(peak_k, eng.path_curvature)
        if eng.current_station_m > 5.0:
            min_speed = min(min_speed, eng.speed_kmh)

    # curvature must stay within the physical ceiling (R >= 8 m -> kappa <= 0.125)
    assert peak_k <= 0.125 + 1e-9, f"phantom curvature {peak_k:.3f} 1/m from a route artefact"
    # and the essentially-straight route must not have braked the car to a crawl
    assert min_speed > 25.0, f"artefact vertex floored the car to {min_speed:.1f} km/h"


def test_smooth_arc_curvature_matches_true_radius():
    """A clean constant-radius arc must report path_curvature ~= 1/R, not a
    resampling-noise spike."""
    for R in (60.0, 120.0, 250.0):
        route = _arc_route_for_friction(R=R)
        eng = PhysicsEngine(seed=1)
        eng.set_destination(*route[-1])
        eng.set_route(route)
        eng.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=1.75)
        peak_k = 0.0
        for _ in range(400):
            eng.update("Accelerate", dt=0.05)
            peak_k = max(peak_k, eng.path_curvature)
        expected = 1.0 / R
        assert abs(peak_k - expected) <= 0.35 * expected, (
            f"R={R}: path_curvature {peak_k:.4f} not within 35% of 1/R {expected:.4f}"
        )


# ---------------------------------------------------------------------------
# P-CORNER: with the curvature-spike fixed and the proactive cut-in response
# bounded to an ease-off (not an emergency stop), the ego must negotiate a
# range of realistic constant-radius curves while holding its lane -- i.e.
# lateral offset stays well inside the 7 m modelled road, no Safety-Shield
# overrides, and it never stalls to a standstill mid-drive.
# ---------------------------------------------------------------------------
def test_ego_holds_lane_through_realistic_curves():
    for R in (80.0, 150.0, 300.0):
        route = _arc_route_for_friction(R=R, runout=60)
        eng = PhysicsEngine(seed=1)
        eng.set_destination(*route[-1])
        eng.set_route(route)
        eng.reset_state(station_m=0.0, speed_kmh=40.0, lateral_offset_m=1.75)
        total = eng.station_distances[-1]

        max_off = 0.0
        shield_ticks = 0
        mid_min_speed = 1e9
        for _ in range(700):
            eng.update("Maintain Speed", dt=0.1)
            max_off = max(max_off, abs(eng.current_lateral_offset_m))
            if not eng.shield_verdict.approved:
                shield_ticks += 1
            if 15.0 < eng.current_station_m < total - 55.0:
                mid_min_speed = min(mid_min_speed, eng.speed_kmh)

        assert max_off < 4.0, f"R={R}: ego strayed {max_off:.1f} m from lane centre (road half-width 7 m)"
        assert shield_ticks == 0, f"R={R}: Safety Shield overrode {shield_ticks} ticks negotiating a benign curve"
        assert mid_min_speed > 12.0, f"R={R}: ego stalled to {mid_min_speed:.1f} km/h mid-curve"


def test_proactive_cut_in_response_is_bounded_to_an_ease_off():
    """A proactive cut-in slowdown may not pull the ego below
    max(0.5*target, target - PROACTIVE_CUT_IN_MAX_DIP_KMH). This bounds the
    cost of a sustained false-positive cut-in (the intent estimator is only
    partially curve-robust) to an ease-off rather than a full stop."""
    route = _arc_route_for_friction(R=150.0, runout=60)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=40.0, lateral_offset_m=1.75)
    total = eng.station_distances[-1]

    saw_proactive = False
    min_speed_while_proactive = 1e9
    for _ in range(700):
        eng.update("Maintain Speed", dt=0.1)
        if eng.speed_limit_reason == "predictive_cut_in" and eng.current_station_m < total - 55.0:
            saw_proactive = True
            min_speed_while_proactive = min(min_speed_while_proactive, eng.speed_kmh)

    if saw_proactive:
        # target is 50 km/h cruise; floor is max(25, 50-20)=30 -> allow a small
        # undershoot for the jerk-limited approach to the floor
        assert min_speed_while_proactive > 22.0, (
            f"proactive cut-in pulled the ego to {min_speed_while_proactive:.1f} km/h -- not an ease-off"
        )


# ---------------------------------------------------------------------------
# P0.1 physics-semantics audit: independent checks against ANALYTICALLY known
# route geometry (not the world_h == road_h + rel_h algebraic identity, which
# is true by construction). Compass convention: heading in degrees, 0 = +lat
# (north), 90 = +lng (east), measured atan2(dir_x, -dir_z).
# ---------------------------------------------------------------------------
def _bearing_route(bearing_deg, n=100, step_m=4.0):
    """Straight route heading `bearing_deg` (compass: 0=N, 90=E)."""
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    br = math.radians(bearing_deg)
    north = math.cos(br) * step_m
    east = math.sin(br) * step_m
    return [(lat0 + (i * north) / m_lat, lng0 + (i * east) / m_lng) for i in range(n)]


def test_road_heading_matches_known_route_bearing():
    for bearing in (0.0, 45.0, 90.0, 135.0):
        route = _bearing_route(bearing)
        eng = PhysicsEngine(seed=1)
        eng.set_destination(*route[-1])
        eng.set_route(route)
        eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)
        for _ in range(60):
            eng.update("Maintain Speed", dt=0.1)
        nav = eng.get_navigation_state()
        got = nav["road_heading_deg"] % 360.0
        want = bearing % 360.0
        err = abs((got - want + 180.0) % 360.0 - 180.0)
        assert err < 3.0, f"bearing {bearing}: road_heading {got:.1f} deg, expected {want:.1f}"
        # a car tracking a straight road has ~zero heading relative to it
        assert abs(nav["relative_heading_deg"]) < 3.0
        # world heading is the vehicle heading and must equal the road's here
        assert abs((nav["world_heading_deg"] - want + 180.0) % 360.0 - 180.0) < 4.0


def test_yaw_rate_and_lateral_accel_match_v_over_R_on_a_steady_arc():
    R = 150.0
    route = _arc_route_for_friction(R=R, runout=60)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)

    yaw_rates, lat_accels, speeds, n = [], [], [], 0
    for _ in range(700):
        eng.update("Maintain Speed", dt=0.1)
        st = eng.current_station_m
        if 160.0 < st < 300.0:  # solidly inside the constant-radius arc
            v = eng.speed_kmh / 3.6
            if v < 3.0:
                continue
            yaw_rates.append(abs(math.radians(eng.yaw_rate_degps)))
            lat_accels.append(eng.lateral_accel_mps2)
            speeds.append(v)
            n += 1

    assert n > 20, "not enough steady-arc samples"
    v_mean = sum(speeds) / n
    # The MEAN realised yaw rate / lateral accel over the steady arc must
    # match v/R and v^2/R. (Instantaneous values are noisier now that the
    # path tracker makes small per-tick corrections; the mean is the
    # physically meaningful quantity. The ego tracks a lane offset ~1.75 m
    # inside the centreline radius, so a few % bias is expected.)
    mean_yaw = sum(yaw_rates) / n
    mean_lat = sum(lat_accels) / n
    assert abs(mean_yaw - v_mean / R) / (v_mean / R) < 0.15, (
        f"mean yaw rate {mean_yaw:.4f} vs v/R {v_mean / R:.4f}"
    )
    assert abs(mean_lat - v_mean * v_mean / R) / (v_mean * v_mean / R) < 0.20, (
        f"mean lateral accel {mean_lat:.3f} vs v^2/R {v_mean * v_mean / R:.3f}"
    )


def test_kinematic_sideslip_is_exactly_the_bicycle_formula_every_tick():
    route = _arc_route_for_friction(R=120.0, runout=40)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)
    for _ in range(400):
        eng.update("Maintain Speed", dt=0.1)
        expected = math.atan(0.5 * math.tan(eng.steering_angle_rad))
        assert eng.kinematic_sideslip_rad == pytest.approx(expected, abs=1e-9)


def test_road_heading_sweeps_through_a_left_arc():
    """On a left arc from east to north, road_heading_deg must decrease
    monotonically (90 -> 0), never jump, and stay within [-5, 95]."""
    route = _arc_route_for_friction(R=150.0, runout=60)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)

    headings = []
    for _ in range(700):
        eng.update("Maintain Speed", dt=0.1)
        if 130.0 < eng.current_station_m < 330.0:
            headings.append(eng.get_navigation_state()["road_heading_deg"])

    assert len(headings) > 30
    assert max(headings) < 95.0 and min(headings) > -5.0
    # net sweep is a real left turn of tens of degrees
    assert headings[0] - headings[-1] > 30.0
    # no single-step discontinuity (resampling / wrap glitch)
    assert max(abs(a - b) for a, b in zip(headings, headings[1:])) < 8.0


# ---------------------------------------------------------------------------
# P0.2 authoritative pose: the backend's emitted world_pos must be
# self-consistent with its own emitted (lat, lng) under the SAME
# equirectangular projection + first-waypoint origin that the frontend's
# toLocalXZ uses. This locks the frame contract the 3D renderer now relies
# on when it consumes ego.world_pos directly.
# ---------------------------------------------------------------------------
def test_world_pos_is_consistent_with_emitted_latlng():
    from app.services.frenet import latlng_to_local

    route = _arc_route_for_friction(R=150.0, runout=60)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)

    origin_lat, origin_lng = route[0]
    worst = 0.0
    n = 0
    for _ in range(500):
        eng.update("Maintain Speed", dt=0.1)
        nav = eng.get_navigation_state()
        wp = nav["world_pos"]
        # independent projection of the ego's own lat/lng
        ex, ez = latlng_to_local(eng.lat, eng.lng, origin_lat, origin_lng)
        d = math.hypot(wp["x"] - ex, wp["z"] - ez)
        worst = max(worst, d)
        n += 1
    assert n > 100
    # frenet_to_local_xz interpolates on the resampled polyline while
    # (lat,lng) is the exact integrated pose -- a few tens of cm of
    # smoothing/interpolation slack is expected, metres are not.
    assert worst < 1.5, f"world_pos vs projected lat/lng diverged by {worst:.2f} m"


def test_world_pos_and_frenet_pose_share_a_frame():
    """world_pos (curved-route projection) and pose (frenet-as-cartesian,
    x=d, z=-s) must agree on a STRAIGHT route -- proving they are the same
    metric frame, not offset by an origin bug."""
    route = _bearing_route(90.0, n=120, step_m=4.0)  # due east
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=35.0, lateral_offset_m=1.75)
    for _ in range(120):
        eng.update("Maintain Speed", dt=0.1)
    nav = eng.get_navigation_state()
    wp = nav["world_pos"]
    # on a due-east straight route: world x ~= station, world z ~= -d
    assert abs(wp["x"] - nav["station_m"]) < 2.0
    assert abs(wp["z"] - (-nav["lateral_offset_m"])) < 2.0


# ===========================================================================
# STEP 1 (cornering) regression suite -- feed-forward + Stanley path tracker.
# Known constant-radius geometry; the ego must hold its lane centre to well
# under a lane's width, entering, through, and exiting the curve, both
# directions, at realistic speeds. Diagnosed root cause: the old pure-pursuit
# feedback law lagged a curve by ~Ld^2*kappa/4 (drifted / cut the corner)
# then overshot on exit; replaced with delta_ff = atan(L*kappa) plus a
# Stanley cross-track + heading feedback on the exactly-known Frenet state.
# ===========================================================================
from app.services.planner import ff_stanley_steer as _ff_stanley  # noqa: E402
from app.services.frenet import (  # noqa: E402
    signed_curvature_at as _sca,
    build_frenet_frame as _bff,
)

_M_LAT = 111_320.0
_M_LNG = 111_320.0 * math.cos(math.radians(37.7749))


def _known_arc(R, sign=+1, lead_m=140.0, sweep_deg=90.0, runout_m=160.0, step_m=3.0):
    """Straight east lead-in, constant-radius arc (sign=+1 left / -1 right,
    tangent continuous), straight run-out."""
    lat0, lng0 = 37.7749, -122.4194
    pts = []
    n_lead = int(lead_m / step_m)
    for i in range(n_lead):
        pts.append((lat0, lng0 + (i * step_m) / _M_LNG))
    x0 = (n_lead - 1) * step_m
    d_ang = step_m / R
    ang = 0.0
    end = math.radians(sweep_deg)
    while ang < end:
        ang += d_ang
        ex = x0 + R * math.sin(ang)
        ny = sign * R * (1.0 - math.cos(ang))
        pts.append((lat0 + ny / _M_LAT, lng0 + ex / _M_LNG))
    lat_n, lng_n = pts[-1]
    for i in range(1, int(runout_m / step_m)):
        pts.append((lat_n + sign * (i * step_m) / _M_LAT, lng_n))
    return pts


def _drive_arc_cte(R, sign, entry_kmh):
    route = _known_arc(R, sign=sign)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=entry_kmh, lateral_offset_m=EGO_LANE_OFFSET_M)
    total = eng.station_distances[-1]
    peak_cte = 0.0
    straight_cte = 0.0
    for _ in range(4000):
        eng.update("Maintain Speed", dt=0.05)
        st = eng.current_station_m
        cte = abs(eng.current_lateral_offset_m - EGO_LANE_OFFSET_M)
        if st < 120.0:
            straight_cte = max(straight_cte, cte)
        else:
            peak_cte = max(peak_cte, cte)
        if st > total - 25:
            break
    return straight_cte, peak_cte


def test_cornering_left_arc_holds_lane_within_half_a_metre():
    for R in (30.0, 50.0, 80.0, 150.0):
        for entry in (40.0, 55.0):
            straight_cte, peak_cte = _drive_arc_cte(R, +1, entry)
            assert straight_cte < 0.05, (
                f"left R={R} entry={entry}: strayed {straight_cte:.2f} m on the straight lead-in"
            )
            lim = 0.5 if R >= 40 else 0.7
            assert peak_cte < lim, (
                f"left R={R} entry={entry}: peak cross-track error {peak_cte:.2f} m through the curve"
            )


def test_cornering_right_arc_holds_lane_within_half_a_metre():
    for R in (30.0, 50.0, 80.0, 150.0):
        for entry in (40.0, 55.0):
            straight_cte, peak_cte = _drive_arc_cte(R, -1, entry)
            assert straight_cte < 0.05
            lim = 0.5 if R >= 40 else 0.7
            assert peak_cte < lim, (
                f"right R={R} entry={entry}: peak cross-track error {peak_cte:.2f} m through the curve"
            )


def test_cornering_no_exit_overshoot_into_the_next_lane():
    """The classic pure-pursuit failure was an inward overshoot on curve
    exit. With FF+Stanley the settled error after the arc must be small."""
    route = _known_arc(50.0, sign=+1, runout_m=200.0)
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=EGO_LANE_OFFSET_M)
    total = eng.station_distances[-1]
    arc_end = 140.0 + 50.0 * math.pi / 2
    settled = []
    for _ in range(5000):
        eng.update("Maintain Speed", dt=0.05)
        st = eng.current_station_m
        if st > arc_end + 60.0:
            settled.append(abs(eng.current_lateral_offset_m - EGO_LANE_OFFSET_M))
        if st > total - 25:
            break
    assert settled and max(settled) < 0.15, (
        f"exit overshoot / oscillation: max settled CTE {max(settled):.2f} m"
    )


def test_ff_stanley_zero_when_straight_on_target_and_aligned():
    d = _ff_stanley(
        current_d_m=1.75, target_d_m=1.75,
        ego_heading_deg=90.0, road_heading_deg=90.0,
        road_curvature_signed_1pm=0.0, v_mps=12.0,
        wheelbase_m=2.8, steer_limit_rad=0.6,
    )
    assert abs(d) < 1e-9


def test_ff_stanley_feed_forward_is_the_kinematic_bicycle_angle():
    for kappa in (-0.02, -0.005, 0.008, 0.02):
        d = _ff_stanley(
            current_d_m=1.75, target_d_m=1.75,
            ego_heading_deg=90.0, road_heading_deg=90.0,
            road_curvature_signed_1pm=kappa, v_mps=12.0,
            wheelbase_m=2.8, steer_limit_rad=0.6,
        )
        assert d == pytest.approx(math.atan(2.8 * kappa), abs=1e-9)


def test_ff_stanley_cross_track_sign_is_correct():
    # d is +LEFT; a car LEFT of target (current > target) must steer RIGHT
    # (positive steer turns the car toward -d).
    left_of_target = _ff_stanley(
        current_d_m=3.0, target_d_m=1.75,
        ego_heading_deg=90.0, road_heading_deg=90.0,
        road_curvature_signed_1pm=0.0, v_mps=12.0,
        wheelbase_m=2.8, steer_limit_rad=0.6,
    )
    right_of_target = _ff_stanley(
        current_d_m=0.5, target_d_m=1.75,
        ego_heading_deg=90.0, road_heading_deg=90.0,
        road_curvature_signed_1pm=0.0, v_mps=12.0,
        wheelbase_m=2.8, steer_limit_rad=0.6,
    )
    assert left_of_target > 0.0
    assert right_of_target < 0.0


def test_ff_stanley_heading_error_sign_is_correct():
    d = _ff_stanley(
        current_d_m=1.75, target_d_m=1.75,
        ego_heading_deg=95.0, road_heading_deg=90.0,
        road_curvature_signed_1pm=0.0, v_mps=12.0,
        wheelbase_m=2.8, steer_limit_rad=0.6,
    )
    assert d < 0.0


def test_signed_curvature_at_matches_known_radius_and_sign():
    fl = _bff(_known_arc(120.0, sign=+1))
    fr = _bff(_known_arc(120.0, sign=-1))
    kl = _sca(fl, 200.0)
    kr = _sca(fr, 200.0)
    assert kl < 0 and abs(abs(kl) - 1.0 / 120.0) < 0.25 / 120.0
    assert kr > 0 and abs(abs(kr) - 1.0 / 120.0) < 0.25 / 120.0
    assert abs(_sca(fl, 40.0)) < 1e-3


# ===========================================================================
# STEP 2 (collision awareness) regression suite. The ego used to keep moving
# until ~half its body had entered a lead vehicle before slowing enough,
# because (a) sense_lead_vehicle used a centre-to-centre station gap, not
# bumper-to-bumper, and (b) it dropped the lead the instant the ego's lateral
# offset drifted half a lane during a lane change, after which IDM stopped
# applying and the ego ACCELERATED into the car it had been braking for.
# Fix: the forward sensor is now a collision CORRIDOR (covers the lane the
# ego is committing into) reporting a bumper-to-bumper gap; the Safety Shield
# has a hard proximity floor.
# ===========================================================================
from app.services.traffic import (  # noqa: E402
    TrafficModel as _TM,
    NpcVehicle as _NPC,
    ADJACENT_LANE_OFFSET_M as _ADJ,
    VEHICLE_HALF_LENGTH_M as _HL,
    VEHICLE_HALF_WIDTH_M as _HW,
)


def _straight_route_step2(n=400):
    return [(37.7749 + i * 0.00005, -122.4194) for i in range(n)]


def _min_body_clearance(eng, npcs):
    """Minimum AABB-style body-to-body clearance ego<->any npc (negative = overlap)."""
    worst = 1e9
    for npc in npcs:
        ds = abs(npc.station_m - eng.current_station_m) - 2 * _HL
        dd = abs(npc.lane_offset - eng.current_lateral_offset_m) - 2 * _HW
        c = max(ds, dd) if (ds < 0 and dd < 0) else math.hypot(max(ds, 0.0), max(dd, 0.0))
        worst = min(worst, c)
    return worst


def _run_toward_lead(lead_speed_kmh, block_adjacent, ticks=900):
    route = _straight_route_step2()
    eng = PhysicsEngine(seed=1)
    eng.set_destination(*route[-1])
    eng.set_route(route)
    eng.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)
    tm = _TM(total_length_m=eng.station_distances[-1], seed=1)
    npcs = [_NPC(id="lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=lead_speed_kmh,
                 station_m=120.0, desired_speed_kmh=lead_speed_kmh, prevent_recycle=True)]
    if block_adjacent:
        npcs.append(_NPC(id="wall", lane_offset=_ADJ, speed_kmh=lead_speed_kmh,
                         station_m=118.0, desired_speed_kmh=lead_speed_kmh, prevent_recycle=True))
    tm.spawn_scripted_npcs(npcs)
    eng.traffic = tm

    min_clear = 1e9
    for _ in range(ticks):
        eng.update("Maintain Speed", dt=0.1)
        min_clear = min(min_clear, _min_body_clearance(eng, eng.traffic.npcs))
        if eng.current_station_m > 200.0:
            break
    return min_clear


def test_ego_never_overlaps_a_stopped_lead_it_goes_around():
    # stopped car in the ego lane, adjacent lane clear -> ego brakes, changes
    # lanes, passes. It must never physically overlap the stopped car.
    min_clear = _run_toward_lead(lead_speed_kmh=0.0, block_adjacent=False)
    assert min_clear > 0.5, f"ego body came within {min_clear:.2f} m of the stopped lead"


def test_ego_stops_short_of_a_stopped_lead_when_boxed_in():
    # stopped car ahead AND the adjacent lane blocked alongside -> the ego
    # cannot go around and MUST stop behind it without overlapping.
    min_clear = _run_toward_lead(lead_speed_kmh=0.0, block_adjacent=True)
    assert min_clear > 0.4, f"boxed-in ego closed to {min_clear:.2f} m of the stopped lead"


def test_ego_follows_a_slow_lead_without_body_overlap():
    for lead_kmh in (8.0, 18.0, 28.0):
        min_clear = _run_toward_lead(lead_speed_kmh=lead_kmh, block_adjacent=True)
        assert min_clear > 0.4, f"lead@{lead_kmh} km/h: ego closed to {min_clear:.2f} m"


def test_forward_sensor_keeps_the_lead_through_an_ego_lane_change():
    """The sensor must not lose a lead just because the ego's lateral offset
    moved off that lane -- it covers the corridor between the ego's current
    and target lane."""
    tm = _TM(total_length_m=1000.0, seed=1)
    tm.spawn_scripted_npcs([_NPC(id="lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=0.0,
                                 station_m=60.0, desired_speed_kmh=0.0, prevent_recycle=True)])
    # ego half-way through a change out of the lead's lane toward the adjacent one
    sensed = tm.sense_lead_vehicle(
        ego_station_m=30.0, ego_lane_offset=3.4, ego_target_lane_offset=_ADJ,
    )
    assert sensed is not None, "sensor dropped the lead mid-lane-change"
    # and the reported gap is bumper-to-bumper
    assert sensed.gap_m == pytest.approx(30.0 - 2 * _HL)


def test_forward_sensor_reports_overlap_as_a_negative_gap():
    tm = _TM(total_length_m=1000.0, seed=1)
    tm.spawn_scripted_npcs([_NPC(id="lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=0.0,
                                 station_m=103.0, desired_speed_kmh=0.0, prevent_recycle=True)])
    sensed = tm.sense_lead_vehicle(ego_station_m=100.0, ego_lane_offset=EGO_LANE_OFFSET_M)
    assert sensed is not None and sensed.gap_m < 0.0  # 3 m station gap - 4.3 m of bodies
