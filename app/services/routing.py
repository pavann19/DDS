"""
Road-following route fetching for the physics simulation.

Uses the free OSRM public demo API (router.project-osrm.org) to turn an
origin/destination pair into a real sequence of road-following waypoints,
instead of the straight-line bearing the physics engine used before P3-1.
This is a public, rate-limited demo service (not meant for production
traffic) -- acceptable for this project's scope, but any failure (network
error, timeout, non-OK route response) must fail soft: the caller falls
back to the old straight-line behavior rather than breaking the drive.
"""
import logging
from typing import List, Optional, Tuple

import httpx
import asyncio
from collections import OrderedDict

logger = logging.getLogger(__name__)

OSRM_BASE_URL = "http://router.project-osrm.org/route/v1/driving"
REQUEST_TIMEOUT_SECONDS = 5.0

_route_cache = OrderedDict()
_CACHE_MAX_SIZE = 100


def clear_route_cache():
    """Test-only hook: this is a module-level, process-lifetime cache with
    no TTL, so tests that populate it with mocked data must clear it
    afterward -- otherwise a later test hitting the same rounded lat/lng
    (e.g. the default SF coordinates used throughout this test suite) gets
    a cache hit on stale mocked data instead of a real fetch, breaking in a
    way that only reproduces when the full suite runs in one process."""
    _route_cache.clear()

async def get_route(
    origin_lat: float, origin_lng: float, dest_lat: float, dest_lng: float
) -> Optional[Tuple[List[Tuple[float, float]], List[dict]]]:
    """
    Returns a tuple of:
      - waypoints: list of (lat, lng) tracing a real road-following route
      - steps: list of maneuver dicts (type, modifier, instruction, location)
    or None if the routing service is unavailable/errored.
    """
    url = f"{OSRM_BASE_URL}/{origin_lng},{origin_lat};{dest_lng},{dest_lat}"
    params = {"overview": "full", "geometries": "geojson", "steps": "true"}

    cache_key = f"{round(origin_lat, 4)},{round(origin_lng, 4)}_{round(dest_lat, 4)},{round(dest_lng, 4)}"
    if cache_key in _route_cache:
        logger.debug(f"Route cache hit for {cache_key}")
        # Move to end to show it was recently used (LRU)
        _route_cache.move_to_end(cache_key)
        return _route_cache[cache_key]

    try:
        async with httpx.AsyncClient(timeout=REQUEST_TIMEOUT_SECONDS) as client:
            response = await client.get(url, params=params)
            response.raise_for_status()
            data = response.json()
    except Exception as e:
        logger.warning(f"Route fetch failed, falling back to straight-line navigation: {e}")
        return None

    if data.get("code") != "Ok" or not data.get("routes"):
        logger.warning(f"OSRM returned no usable route (code={data.get('code')}); falling back to straight-line.")
        return None

    # GeoJSON coordinates are [lng, lat] -- flip to (lat, lng) to match the
    # rest of this codebase's convention (PhysicsEngine, NavState, etc.).
    route_data = data["routes"][0]
    coordinates = route_data["geometry"]["coordinates"]
    waypoints = [(lat, lng) for lng, lat in coordinates]
    
    # Defensive: OSRM's response shape is not something this service
    # controls, and a missing/empty "legs"/"steps" must not crash route
    # fetching (this function's whole contract is "fail soft, fall back to
    # straight-line" -- see the module docstring). Waypoints alone are
    # enough for the car to drive; turn-by-turn steps are a bonus.
    parsed_steps = []
    legs = route_data.get("legs") or []
    steps_data = legs[0].get("steps", []) if legs else []
    for step in steps_data:
        maneuver = step.get("maneuver", {})
        location = maneuver.get("location") or [0, 0]
        parsed_steps.append({
            "type": maneuver.get("type", ""),
            "modifier": maneuver.get("modifier", ""),
            "instruction": step.get("name", ""), # Using name as instruction placeholder if maneuver doesn't have it, or we could just use name
            "location": (location[1], location[0]),
            "distance": step.get("distance", 0.0)
        })

    result = (waypoints, parsed_steps)
    
    _route_cache[cache_key] = result
    if len(_route_cache) > _CACHE_MAX_SIZE:
        _route_cache.popitem(last=False)
        
    return result
