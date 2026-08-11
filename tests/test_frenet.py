"""
Unit tests for app/services/frenet.py -- the exact station/lateral
projection P6-2 builds its planner on top of.
"""
import math
import pytest

from app.services.frenet import (
    build_frenet_frame,
    project_to_frenet,
    frenet_to_latlng,
    frenet_to_local_xz,
    latlng_to_local,
)


def _straight_east_route(n=20, spacing_deg=0.0001):
    lat0, lng0 = 37.7749, -122.4194
    return [(lat0, lng0 + i * spacing_deg) for i in range(n)]


def _corner_route():
    route = [(37.7749, -122.4194 + i * 0.0001) for i in range(15)]
    turn_lng = route[-1][1]
    route += [(37.7749 + i * 0.0001, turn_lng) for i in range(1, 15)]
    return route


def test_build_frenet_frame_station_matches_route_length():
    route = _straight_east_route()
    frame = build_frenet_frame(route)
    assert frame is not None
    assert frame.station[0] == 0.0
    assert frame.total_length_m == pytest.approx(frame.station[-1])
    # Roughly 0.0001 deg longitude at this latitude ~= 8.8 m/segment * 19 segments
    assert frame.total_length_m > 100.0


def test_build_frenet_frame_returns_none_for_degenerate_route():
    assert build_frenet_frame([]) is None
    assert build_frenet_frame([(1.0, 2.0)]) is None


def test_projection_on_the_centreline_has_zero_lateral_offset():
    route = _straight_east_route()
    frame = build_frenet_frame(route)
    lat, lng = route[10]
    s, d, idx = project_to_frenet(frame, lat, lng)
    assert d == pytest.approx(0.0, abs=1e-6)
    assert s == pytest.approx(frame.station[10], abs=1e-6)


def test_projection_lateral_sign_is_consistent_with_offset_application():
    """Whatever sign convention frenet_to_local_xz uses to APPLY a lateral
    offset must be the same sign project_to_frenet reports when reading one
    back -- otherwise a round trip through frenet space would flip sides,
    which is exactly the class of frontend/backend sign disagreement P6-1b
    and P6-1d's work was about avoiding."""
    route = _straight_east_route()
    frame = build_frenet_frame(route)
    s_query = frame.station[10]

    for d_applied in (-3.5, -1.0, 1.0, 3.5):
        x, z, _, _ = frenet_to_local_xz(frame, s_query, d_applied)
        lat, lng = frenet_to_latlng(frame, s_query, d_applied)
        s_back, d_back, _ = project_to_frenet(frame, lat, lng, search_start_idx=8)
        assert d_back == pytest.approx(d_applied, abs=1e-3)
        assert s_back == pytest.approx(s_query, abs=1e-3)


def test_projection_and_inverse_round_trip_on_a_corner():
    """The route this project actually drives has a real turn in it -- the
    round trip must hold there too, not just on a straight segment."""
    route = _corner_route()
    frame = build_frenet_frame(route)
    for i in (5, 14, 15, 20, len(route) - 2):
        lat, lng = route[i]
        s, d, idx = project_to_frenet(frame, lat, lng)
        assert d == pytest.approx(0.0, abs=1e-4)
        back_lat, back_lng = frenet_to_latlng(frame, s, d)
        assert back_lat == pytest.approx(lat, abs=1e-9)
        assert back_lng == pytest.approx(lng, abs=1e-9)


def test_station_is_monotonic_non_decreasing_along_the_route():
    route = _corner_route()
    frame = build_frenet_frame(route)
    prev_s = -1.0
    for lat, lng in route:
        s, _, _ = project_to_frenet(frame, lat, lng, search_start_idx=0, search_window=len(route))
        assert s >= prev_s - 1e-6
        prev_s = s


def test_project_to_frenet_search_window_is_respected():
    """A far-ahead point must not be found if the search window doesn't reach
    it -- this is what keeps the per-tick projection cheap on long routes."""
    route = _straight_east_route(n=100)
    frame = build_frenet_frame(route)
    lat, lng = route[80]
    s, d, idx = project_to_frenet(frame, lat, lng, search_start_idx=0, search_window=10)
    # Clamped search only reaches index ~10, so it must NOT snap onto the
    # true (much further away) closest segment.
    assert idx <= 10
