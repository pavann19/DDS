"""Pinned real-road routes, usable deterministically without a network request.

Coordinates are recorded OSRM/OpenStreetMap road geometry, never translated or
procedurally generated. Attribution and source URL live beside the coordinates.
"""
import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

ROUTE_DATA = Path(__file__).resolve().parents[1] / "data" / "real_road_routes.json"


@lru_cache(maxsize=2)
def _profile(name):
    data = json.loads(ROUTE_DATA.read_text(encoding="utf-8"))
    profile = data["profiles"][name]
    points = profile["waypoints"]
    digest = hashlib.sha256(json.dumps(points, separators=(",", ":")).encode()).hexdigest()
    if digest != profile["geometry_sha256"] or len(points) < 2:
        raise ValueError("Recorded road geometry failed integrity check")
    if not all(len(p) == 2 and math.isfinite(p[0]) and math.isfinite(p[1])
               and abs(p[0]) <= 90 and abs(p[1]) <= 180 for p in points):
        raise ValueError("Invalid recorded road coordinates")
    return profile["name"], tuple(tuple(p) for p in points)


def real_road_route(name="highway"):
    """Return a fresh route list; missing/corrupt data fails without fake fallback."""
    return list(_profile(name)[1])


def real_road_name(name="highway"):
    return _profile(name)[0]


@lru_cache(maxsize=2)
def real_road_continuation(name="highway"):
    profile = json.loads(ROUTE_DATA.read_text(encoding="utf-8"))["profiles"][name]
    points = profile.get("road_continuation", [])
    digest = hashlib.sha256(json.dumps(points, separators=(",", ":")).encode()).hexdigest()
    if digest != profile.get("continuation_sha256"):
        raise ValueError("Recorded road continuation failed integrity check")
    if not all(len(p) == 2 and all(math.isfinite(v) for v in p)
               and abs(p[0]) <= 90 and abs(p[1]) <= 180 for p in points):
        raise ValueError("Invalid road continuation coordinates")
    return tuple(tuple(p) for p in points)
