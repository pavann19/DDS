"""
Unit tests for app/services/path_smoothing.py : centripetal
Catmull-Rom smoothing + uniform arc-length resampling of OSRM route polylines.
"""
import math
import pytest

from app.services.path_smoothing import (
    smooth_route, path_length_m, max_deviation_m,
    _to_local_xy, DEFAULT_SPACING_M,
)


def _straight(n=20, step=0.0002):
    return [(37.7749 + i * step, -122.4194) for i in range(n)]


def _corner():
    route = [(37.7749, -122.4194 + i * 0.0002) for i in range(10)]
    turn_lng = route[-1][1]
    route += [(37.7749 + i * 0.0002, turn_lng) for i in range(1, 10)]
    return route


def _spacings(route):
    o = route[0]
    local = [_to_local_xy(a, b, o[0], o[1]) for a, b in route]
    return [math.dist(local[i - 1], local[i]) for i in range(1, len(local))]


def test_too_few_points_returned_unchanged():
    """Must never become a new failure mode for routing (routing.py is
    contractually fail-soft)."""
    assert smooth_route([]) == []
    one = [(37.7749, -122.4194)]
    assert smooth_route(one) == one
    two = [(37.7749, -122.4194), (37.7750, -122.4194)]
    assert smooth_route(two) == two


def test_resampled_spacing_is_uniform():
    """Resampling is by ARC length, but this measures straight-line (chord)
    distance between consecutive points. On a curve the chord is necessarily
    shorter than the arc, so exact equality only holds on straight stretches:
    at the test corner (radius ~4.7 m) a 5 m arc subtends ~61 deg, giving a
    ~4.77 m chord. Hence: never longer than the target, and close to it."""
    smoothed = smooth_route(_corner())
    # Drop the final partial segment, which lands exactly on the endpoint.
    full = _spacings(smoothed)[:-1]
    assert max(full) <= DEFAULT_SPACING_M + 0.05
    assert min(full) > DEFAULT_SPACING_M * 0.9


def test_spacing_is_exact_on_a_straight_stretch():
    """With no curvature there is no chord-vs-arc gap, so spacing should hit
    the target essentially exactly."""
    full = _spacings(smooth_route(_straight()))[:-1]
    assert max(full) == pytest.approx(DEFAULT_SPACING_M, abs=0.01)
    assert min(full) == pytest.approx(DEFAULT_SPACING_M, abs=0.01)


def test_route_length_is_preserved():
    original = _corner()
    smoothed = smooth_route(original)
    ol, sl = path_length_m(original), path_length_m(smoothed)
    assert abs(sl - ol) / ol < 0.02, "smoothing must not materially change route length"


def test_endpoints_are_preserved_exactly():
    """The destination must not move -- arrival logic keys off it."""
    original = _corner()
    smoothed = smooth_route(original)
    assert smoothed[0] == pytest.approx(original[0])
    assert smoothed[-1] == pytest.approx(original[-1])


def test_smoothed_path_stays_close_to_the_original_route():
    """Fidelity: a spline through real OSRM waypoints must stay on the real
    road. Tolerance is well inside a 3.5 m lane."""
    original = _corner()
    smoothed = smooth_route(original)
    assert max_deviation_m(original, smoothed) < 3.0


def test_a_straight_route_stays_straight():
    """Centripetal Catmull-Rom must not introduce overshoot/wobble on
    collinear input -- a classic failure of the uniform parameterisation."""
    original = _straight()
    smoothed = smooth_route(original)
    lngs = [lng for _, lng in smoothed]
    assert max(lngs) - min(lngs) < 1e-6, "a straight road must not acquire lateral wobble"


def test_duplicate_points_are_tolerated():
    """OSRM emits near-duplicate vertices; coincident control points make
    spline knot spacing degenerate if not handled."""
    route = _corner()
    with_dupes = []
    for p in route:
        with_dupes.extend([p, p])  # every point duplicated
    smoothed = smooth_route(with_dupes)
    assert len(smoothed) > len(route)
    assert all(math.isfinite(a) and math.isfinite(b) for a, b in smoothed)


def test_sparse_route_is_densified():
    original = _corner()
    smoothed = smooth_route(original)
    assert len(smoothed) > len(original)


def test_spacing_parameter_is_respected():
    smoothed_fine = smooth_route(_corner(), spacing_m=2.0)
    smoothed_coarse = smooth_route(_corner(), spacing_m=10.0)
    assert len(smoothed_fine) > len(smoothed_coarse)
    assert _spacings(smoothed_coarse)[0] == pytest.approx(10.0, abs=0.1)


def test_no_zero_length_segments_in_output():
    """Zero-length segments are what produce spurious infinite curvature
    downstream -- the exact problem this task exists to eliminate."""
    smoothed = smooth_route(_corner())
    assert min(_spacings(smoothed)) > 0.0
