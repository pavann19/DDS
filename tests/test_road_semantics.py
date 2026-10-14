import dataclasses
import math

import pytest

from app.services.road_semantics import RoadMap, parse_speed, recorded_map


def feature(id=1, **tags):
    return {"id": id, "timestamp": "2026-01-01T00:00:00Z",
            "tags": {"highway": "motorway", "maxspeed": "65 mph", "lanes": "5", "oneway": "yes", **tags},
            "geometry": [[37.6, -122.38], [37.601, -122.38]]}


@pytest.mark.parametrize("tag,expected", [("65 mph", 104.60736), ("80", 80), ("50 km/h", 50),
                                           ("signals", None), ("none", None), ("0", None), ("nan", None)])
def test_map_speed_parsing_preserves_unknown(tag, expected):
    assert parse_speed(tag) == expected


def test_matching_rejects_wrong_direction_far_and_nonfinite():
    road = RoadMap({"features": [feature()]})
    match = road.match(37.6005, -122.38, 0)
    assert match.way_id == 1 and match.lane_count == 5
    assert road.match(37.6005, -122.38, 180) is None
    assert road.match(38, -122.38, 0) is None
    assert road.match(float("nan"), -122.38, 0) is None


def test_incompatible_overlaid_ways_are_unknown():
    road = RoadMap({"features": [feature(), feature(2, maxspeed="30")]})
    assert road.match(37.6005, -122.38, 0) is None


def test_recorded_route_coverage_is_measured_not_assumed():
    from app.services.road_routes import real_road_route
    points = real_road_route()
    matches = []
    for a, b in zip(points, points[1:]):
        h = math.degrees(math.atan2((b[1] - a[1]) * math.cos(math.radians(a[0])), b[0] - a[0]))
        matches.append(recorded_map().match(*a, h))
    # Shared ramp/road endpoints have incompatible/absent tags: keep them
    # unknown. This pins measured coverage rather than assuming a full map.
    assert [i for i, match in enumerate(matches) if match is None] == [0, 24, 33, 38, 41, 42]
    assert all(match.speed_limit_kmh == pytest.approx(104.60736) for match in matches if match)


def test_longitudinal_driver_respects_mapped_limit():
    from app.services.physics_engine import PhysicsEngine
    physics = PhysicsEngine(seed=42)
    physics.update("Maintain Speed", .02)
    physics._driver.cruise_speed_kmh = 120
    obs = dataclasses.replace(physics.observe(), road_rules=RoadMap({"features": [feature()]}).match(37.6005, -122.38, 0))
    ctx = physics.build_driver_context(.02)
    ctx = dataclasses.replace(ctx, has_arrived=False, dist_to_dest_m=1000, max_curvature_1pm=0,
                              speed_kmh=80, controller="trajectory")
    physics._driver.decide_longitudinal(obs, ctx)
    assert physics._driver.target_speed_mps * 3.6 <= 104.60736


def test_map_gap_holds_cap_then_expires_and_reset_clears_it():
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SimClock
    physics = PhysicsEngine(seed=42)
    driver = physics._driver
    driver.cruise_speed_kmh = 120
    obs = dataclasses.replace(physics.observe(), road_rules=RoadMap({"features": [feature()]}).match(37.6005, -122.38, 0))
    assert driver.cruise_cap(obs) == (104.60736, "mapped_speed_limit")
    missing = dataclasses.replace(obs, road_rules=None,
        ego=dataclasses.replace(obs.ego, station_m=obs.ego.station_m + 20),
        clock=SimClock(sim_time_s=obs.clock.sim_time_s + 1))
    assert driver.cruise_cap(missing) == (104.60736, "map_uncertainty_hold")
    expired = dataclasses.replace(missing, clock=SimClock(sim_time_s=obs.clock.sim_time_s + 3))
    assert driver.cruise_cap(expired) == (50, "map_unknown_conservative")
    driver.on_route_reset()
    assert driver.cruise_cap(missing) == (50, "map_unknown_conservative")
