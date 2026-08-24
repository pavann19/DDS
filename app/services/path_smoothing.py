"""
Spline smoothing + uniform arc-length resampling of OSRM route polylines .

Why this exists
---------------
OSRM returns a route as a polyline whose vertices are spaced according to the
road network's own geometry, not according to anything we want. Measured on
this project's default San Francisco route, consecutive vertex gaps run
13.7 m, 8.7 m, 34.8 m, 128.8 m, 40.3 m -- an order of magnitude of variation,
plus occasional near-duplicate points. Rendering a road ribbon straight from
those vertices produces visibly faceted geometry, and differentiating them to
get curvature produces garbage: a near-zero `ds` with any heading change at
all yields an enormous spurious curvature (which is why
`physics_engine.py` carries a `MIN_SEGMENT_FOR_CURVATURE_M` guard).

Resampling the path at uniform arc length along a smooth interpolating spline
fixes both at the source: the ribbon becomes continuous, and curvature becomes
a well-conditioned quantity that P6-2's Frenet planner can actually rely on.

Why CENTRIPETAL Catmull-Rom specifically
----------------------------------------
Uniform Catmull-Rom overshoots badly and can form cusps or self-intersections
when control points are unevenly spaced -- precisely this data's failure mode
(see the 8.7 m -> 128.8 m jump above). The centripetal parameterisation
(alpha = 0.5) is the variant with a proof that it produces no cusps and no
self-intersections for any input point configuration, which is exactly the
robustness property needed for arbitrary real-world routes. Implemented via
the Barry-Goldman pyramidal formulation.

Fidelity note (for the report)
------------------------------
Smoothing does not make the route "less real". The OSRM vertices are samples
of a real road; a straight chord between two samples is itself an
approximation, and on a curved road a smooth interpolant is generally the
*better* approximation of the true road, not a worse one. What smoothing does
change is that hard vertices at intersections get rounded -- which is also
closer to reality, since real vehicles turn through a corner radius rather
than a mathematical point. `max_deviation_m()` is provided so this can be
measured and reported rather than asserted.
"""
import math
from typing import List, Sequence, Tuple

LatLng = Tuple[float, float]

EARTH_RADIUS_M = 6371000.0

# Target spacing of the resampled path. ~5 m is fine enough that the rendered
# ribbon reads as a smooth curve and curvature is well-conditioned, while
# keeping the point count modest (a 6 km route -> ~1200 points, sent once).
DEFAULT_SPACING_M = 5.0

# Points closer together than this are treated as duplicates. Coincident or
# near-coincident control points make the spline's knot spacing degenerate.
MIN_POINT_SEPARATION_M = 0.5

# Centripetal parameterisation. 0.0 = uniform, 0.5 = centripetal, 1.0 = chordal.
CENTRIPETAL_ALPHA = 0.5

# Samples evaluated per spline span before arc-length resampling. The spline is
# walked densely, then points are emitted at uniform arc length along that walk.
SAMPLES_PER_SPAN = 24


def _to_local_xy(lat: float, lng: float, origin_lat: float, origin_lng: float) -> Tuple[float, float]:
    """Equirectangular projection to local metres about an origin. Adequate at
    city scale (a few km), which is this project's operating range; matches the
    approximation already used by the frontend's `toLocalXZ`."""
    x = math.radians(lng - origin_lng) * math.cos(math.radians(origin_lat)) * EARTH_RADIUS_M
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def _to_lat_lng(x: float, y: float, origin_lat: float, origin_lng: float) -> LatLng:
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    lng = origin_lng + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lng


def _dedupe(points: Sequence[Tuple[float, float]], min_sep: float) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for p in points:
        if not out or math.dist(out[-1], p) >= min_sep:
            out.append(p)
    return out


def _catmull_rom_span(p0, p1, p2, p3, n_samples: int, alpha: float) -> List[Tuple[float, float]]:
    """Sample the centripetal Catmull-Rom segment between p1 and p2.

    Barry-Goldman pyramidal formulation: knots are spaced by the alpha-power of
    the distance between control points, which is what makes the centripetal
    (alpha=0.5) variant cusp-free on unevenly spaced input.
    """
    def knot(t_prev, a, b):
        d = math.dist(a, b)
        return t_prev + (d ** alpha if d > 0 else 1e-6)

    t0 = 0.0
    t1 = knot(t0, p0, p1)
    t2 = knot(t1, p1, p2)
    t3 = knot(t2, p2, p3)

    def lerp(a, b, ta, tb, t):
        if tb - ta == 0:
            return a
        w = (t - ta) / (tb - ta)
        return (a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w)

    out = []
    for i in range(n_samples):
        t = t1 + (t2 - t1) * (i / n_samples)
        a1 = lerp(p0, p1, t0, t1, t)
        a2 = lerp(p1, p2, t1, t2, t)
        a3 = lerp(p2, p3, t2, t3, t)
        b1 = lerp(a1, a2, t0, t2, t)
        b2 = lerp(a2, a3, t1, t3, t)
        out.append(lerp(b1, b2, t1, t2, t))
    return out


def smooth_route(
    waypoints: Sequence[LatLng],
    spacing_m: float = DEFAULT_SPACING_M,
) -> List[LatLng]:
    """Return the route resampled at ~uniform `spacing_m` arc length along a
    centripetal Catmull-Rom spline through the input waypoints.

    Endpoints are preserved exactly (the destination must not move). Input of
    fewer than 3 usable points is returned unchanged -- there is nothing to
    interpolate, and callers must keep working (this function must never be a
    new failure mode for routing; see routing.py's fail-soft contract).
    """
    if not waypoints or len(waypoints) < 3:
        return list(waypoints)

    origin_lat, origin_lng = waypoints[0]
    local = [_to_local_xy(lat, lng, origin_lat, origin_lng) for lat, lng in waypoints]
    local = _dedupe(local, MIN_POINT_SEPARATION_M)
    if len(local) < 3:
        return list(waypoints)

    # Duplicate the end control points so the spline spans the full path
    # (Catmull-Rom needs a neighbour on each side of every rendered span).
    padded = [local[0]] + local + [local[-1]]

    dense: List[Tuple[float, float]] = []
    for i in range(len(padded) - 3):
        dense.extend(_catmull_rom_span(
            padded[i], padded[i + 1], padded[i + 2], padded[i + 3],
            SAMPLES_PER_SPAN, CENTRIPETAL_ALPHA,
        ))
    dense.append(local[-1])

    # Walk the dense polyline and emit a point every `spacing_m` of arc length.
    # `residual` is arc length accumulated since the LAST EMITTED point, which
    # may span several dense segments; `pos` is progress within the current
    # segment. Conflating the two (treating leftover distance as an offset into
    # the next segment) makes the walk drift and skip whole segments whenever
    # spacing is uneven -- measured during development as a 4.5 km gap and 23%
    # of the route's length silently lost.
    resampled = [dense[0]]
    residual = 0.0
    for i in range(1, len(dense)):
        a, b = dense[i - 1], dense[i]
        seg = math.dist(a, b)
        if seg <= 0:
            continue
        pos = 0.0
        while residual + (seg - pos) >= spacing_m:
            pos += spacing_m - residual
            w = pos / seg
            resampled.append((a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w))
            residual = 0.0
        residual += seg - pos
    if math.dist(resampled[-1], local[-1]) > 1e-6:
        resampled.append(local[-1])  # preserve the exact destination

    return [_to_lat_lng(x, y, origin_lat, origin_lng) for x, y in resampled]


def path_length_m(waypoints: Sequence[LatLng]) -> float:
    """Total arc length of a lat/lng polyline, in metres."""
    if len(waypoints) < 2:
        return 0.0
    origin_lat, origin_lng = waypoints[0]
    local = [_to_local_xy(lat, lng, origin_lat, origin_lng) for lat, lng in waypoints]
    return sum(math.dist(local[i - 1], local[i]) for i in range(1, len(local)))


def max_deviation_m(original: Sequence[LatLng], smoothed: Sequence[LatLng]) -> float:
    """Largest distance from any ORIGINAL waypoint to the smoothed path.

    This is the honest fidelity measure for the report: it answers "how far
    did smoothing move the route away from the road OSRM actually gave us?".
    Expected to be small on straight/gently-curved stretches and largest at
    sharp intersection vertices, which smoothing deliberately rounds.
    """
    if not original or len(smoothed) < 2:
        return 0.0
    origin_lat, origin_lng = original[0]
    o = [_to_local_xy(lat, lng, origin_lat, origin_lng) for lat, lng in original]
    s = [_to_local_xy(lat, lng, origin_lat, origin_lng) for lat, lng in smoothed]

    def point_to_segment(p, a, b):
        ax, ay = a
        bx, by = b
        px, py = p
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        if denom == 0:
            return math.dist(p, a)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
        return math.dist(p, (ax + t * dx, ay + t * dy))

    worst = 0.0
    for p in o:
        best = min(point_to_segment(p, s[i - 1], s[i]) for i in range(1, len(s)))
        worst = max(worst, best)
    return worst
