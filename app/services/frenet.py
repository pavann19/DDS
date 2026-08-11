"""
Frenet (station-lateral) frame utilities for P6-2's local planner.

`current_station_m` (P6-1b) and its `station_distances[route_index]`
approximation were always a stopgap -- accurate only to within one
inter-waypoint spacing, and P6-1b's own comment said as much: "P6-2's proper
Frenet frame will replace this with exact segment projection." This module
is that replacement: an exact perpendicular projection of the ego's real
position onto the route polyline, giving a continuous signed lateral offset
`d` as well as a continuous station `s` (no longer snapped to a waypoint).

Built in a local metric (x, z) plane -- the same equirectangular projection
frontend/DriveScene.tsx's `toLocalXZ` uses, anchored at the route's first
waypoint -- because Frenet geometry (cross-track distance, quintic lateral
profiles, pure-pursuit lookahead) is inherently planar/metric; doing it in
lat/lng directly would require re-deriving spherical cross-track formulas
for no benefit at these distances (a few km at most).

Sign convention: positive `d` is to the RIGHT of the direction of travel.
This deliberately matches two other places in the codebase that already
picked this convention independently and must keep agreeing with it:
`traffic.py`'s `LANE_OFFSETS`/`EGO_LANE_OFFSET_M` (positive = same-direction
lane, right-hand traffic) and the frontend's `RoadMesh`/`SimulatedTraffic`
`right = (dir.z, 0, -dir.x)` offset vector. If this module's `d` sign ever
disagreed with those, the ego, the NPCs, and the road-edge rendering would
each be using a different idea of "which side is which" -- exactly the class
of backend/frontend disagreement P6-1b's station bug and P6-1d's smoothing
work were about.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

EARTH_RADIUS_M = 6371000.0


def latlng_to_local(lat: float, lng: float, origin_lat: float, origin_lng: float) -> Tuple[float, float]:
    lat_rad = math.radians(origin_lat)
    x = (lng - origin_lng) * math.cos(lat_rad) * (math.pi / 180.0) * EARTH_RADIUS_M
    z = -(lat - origin_lat) * (math.pi / 180.0) * EARTH_RADIUS_M
    return x, z


def local_to_latlng(x: float, z: float, origin_lat: float, origin_lng: float) -> Tuple[float, float]:
    lat_rad = math.radians(origin_lat)
    lng = origin_lng + x / (math.cos(lat_rad) * (math.pi / 180.0) * EARTH_RADIUS_M)
    lat = origin_lat - z / ((math.pi / 180.0) * EARTH_RADIUS_M)
    return lat, lng


@dataclass
class FrenetFrame:
    points_xz: List[Tuple[float, float]]  # local (x, z) metres, one per route waypoint
    station: List[float]                  # cumulative arc length (metres), same length as points_xz
    origin_lat: float
    origin_lng: float

    @property
    def total_length_m(self) -> float:
        return self.station[-1] if self.station else 0.0


def build_frenet_frame(route: List[Tuple[float, float]]) -> Optional[FrenetFrame]:
    """route: the SAME smoothed waypoint list PhysicsEngine.route already
    holds (post path_smoothing.smooth_route) -- projecting against the
    smoothed route, not the raw OSRM polyline, is what gives Frenet a
    well-conditioned (non-faceted) frame to work in, per P6-1d."""
    if not route or len(route) < 2:
        return None
    origin_lat, origin_lng = route[0]
    points_xz = [latlng_to_local(lat, lng, origin_lat, origin_lng) for lat, lng in route]
    station = [0.0]
    for i in range(1, len(points_xz)):
        x1, z1 = points_xz[i - 1]
        x2, z2 = points_xz[i]
        station.append(station[-1] + math.hypot(x2 - x1, z2 - z1))
    return FrenetFrame(points_xz=points_xz, station=station, origin_lat=origin_lat, origin_lng=origin_lng)


def project_to_frenet(
    frame: FrenetFrame,
    lat: float,
    lng: float,
    search_start_idx: int = 0,
    search_window: int = 60,
) -> Tuple[float, float, int]:
    """Project a real-world (lat, lng) onto the route polyline by exact
    clamped point-to-segment distance. Returns (s, d, segment_idx).

    search_start_idx/search_window bound the search to segments at or ahead
    of the caller's last known position (same windowed-search principle
    P6-1's route_index projection already uses) -- projecting against the
    whole route every tick would be wasteful and, worse, could snap onto a
    geometrically-nearby-but-wrong part of a route that loops back on
    itself."""
    px, pz = latlng_to_local(lat, lng, frame.origin_lat, frame.origin_lng)
    n = len(frame.points_xz)
    lo = max(0, min(search_start_idx, n - 2))
    hi = min(n - 1, lo + search_window)

    best_dist_sq = float("inf")
    best_s = frame.station[lo]
    best_d = 0.0
    best_idx = lo

    for i in range(lo, hi):
        ax, az = frame.points_xz[i]
        bx, bz = frame.points_xz[i + 1]
        dx, dz = bx - ax, bz - az
        seg_len_sq = dx * dx + dz * dz
        if seg_len_sq < 1e-9:
            continue
        t = ((px - ax) * dx + (pz - az) * dz) / seg_len_sq
        t = max(0.0, min(1.0, t))
        cx, cz = ax + t * dx, az + t * dz
        dist_sq = (px - cx) ** 2 + (pz - cz) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            seg_len = math.sqrt(seg_len_sq)
            best_s = frame.station[i] + t * seg_len
            right_x, right_z = dz / seg_len, -dx / seg_len
            best_d = (px - cx) * right_x + (pz - cz) * right_z
            best_idx = i

    return best_s, best_d, best_idx


def frenet_to_local_xz(frame: FrenetFrame, s: float, d: float) -> Tuple[float, float, float, float]:
    """Inverse of project_to_frenet: (s, d) -> (x, z, dir_x, dir_z) in the
    local metric frame, plus the route's unit tangent direction at that
    station (needed by pure-pursuit to reason about heading)."""
    n = len(frame.station)
    s = max(0.0, min(s, frame.station[-1]))
    idx = 0
    while idx < n - 2 and frame.station[idx + 1] < s:
        idx += 1
    ax, az = frame.points_xz[idx]
    bx, bz = frame.points_xz[idx + 1]
    seg_len = frame.station[idx + 1] - frame.station[idx]
    t = (s - frame.station[idx]) / seg_len if seg_len > 1e-9 else 0.0
    dx, dz = bx - ax, bz - az
    dir_len = math.hypot(dx, dz)
    if dir_len < 1e-9:
        dir_x, dir_z = 0.0, 1.0
    else:
        dir_x, dir_z = dx / dir_len, dz / dir_len
    cx, cz = ax + t * dx, az + t * dz
    right_x, right_z = dir_z, -dir_x
    x = cx + d * right_x
    z = cz + d * right_z
    return x, z, dir_x, dir_z


def frenet_to_latlng(frame: FrenetFrame, s: float, d: float) -> Tuple[float, float]:
    x, z, _, _ = frenet_to_local_xz(frame, s, d)
    return local_to_latlng(x, z, frame.origin_lat, frame.origin_lng)
