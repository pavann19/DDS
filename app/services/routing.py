"""
Road-following route fetching for the physics simulation.

Uses the free OSRM public demo API (router.project-osrm.org) to turn an
origin/destination pair into a real sequence of road-following waypoints,
instead of the straight-line bearing the physics engine used before .
This is a public, rate-limited demo service (not meant for production
traffic). On failure the session retains its current road route and remains
paused; no generated or straight-line route is substituted.
"""
import logging
from typing import List, Optional, Tuple

import httpx
import asyncio
import copy
import math
from collections import OrderedDict

logger = logging.getLogger(__name__)

OSRM_BASE_URL = "https://router.project-osrm.org/route/v1/driving"
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

    values = (origin_lat, origin_lng, dest_lat, dest_lng)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return None
    if not (-90 <= origin_lat <= 90 and -90 <= dest_lat <= 90 and -180 <= origin_lng <= 180 and -180 <= dest_lng <= 180):
        return None
    cache_key = values
    if cache_key in _route_cache:
        logger.debug(f"Route cache hit for {cache_key}")
        # Move to end to show it was recently used (LRU)
        _route_cache.move_to_end(cache_key)
        return copy.deepcopy(_route_cache[cache_key])

    try:
        async with httpx.AsyncClient(timeout=REQUEST_TIMEOUT_SECONDS) as client:
            response = await client.get(url, params=params)
            response.raise_for_status()
            data = response.json()
    except Exception as e:
        logger.warning(f"Route fetch failed; no route returned: {e}")
        return None

    if not isinstance(data, dict) or data.get("code") != "Ok" or not isinstance(data.get("routes"), list) or not data["routes"]:
        logger.warning("OSRM returned no usable route; no route returned.")
        return None

    # GeoJSON coordinates are [lng, lat] -- flip to (lat, lng) to match the
    # rest of this codebase's convention (PhysicsEngine, NavState, etc.).
    route_data = data["routes"][0]
    if not isinstance(route_data, dict):
        return None
    geometry = route_data.get("geometry")
    coordinates = geometry.get("coordinates") if isinstance(geometry, dict) else None
    def valid_coordinate(value):
        return (isinstance(value, (list, tuple)) and len(value) == 2
                and all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in value)
                and -180 <= value[0] <= 180 and -90 <= value[1] <= 90)
    if not isinstance(coordinates, list) or not 2 <= len(coordinates) <= 100_000 or not all(valid_coordinate(p) for p in coordinates):
        logger.warning("OSRM returned invalid route coordinates")
        return None
    waypoints = [(lat, lng) for lng, lat in coordinates]
    
    # Defensive: OSRM's response shape is not something this service
    # controls, and a missing/empty "legs"/"steps" must not crash route
    # fetching. The caller rejects an unavailable route. Waypoints alone are
    # enough for the car to drive; turn-by-turn steps are a bonus.
    parsed_steps = []
    legs = route_data.get("legs") or []
    steps_data = legs[0].get("steps", []) if isinstance(legs, list) and legs and isinstance(legs[0], dict) else []
    if not isinstance(steps_data, list):
        steps_data = []
    for step in steps_data:
        if not isinstance(step, dict):
            continue
        maneuver = step.get("maneuver", {})
        if not isinstance(maneuver, dict):
            continue
        location = maneuver.get("location")
        if not valid_coordinate(location):
            continue
        distance = step.get("distance", 0.0)
        if (isinstance(distance, bool) or not isinstance(distance, (int, float))
                or not math.isfinite(distance) or distance < 0
                or not all(isinstance(v, str) for v in (maneuver.get("type", ""), maneuver.get("modifier", ""), step.get("name", "")))):
            continue
        parsed_steps.append({
            "type": maneuver.get("type", ""),
            "modifier": maneuver.get("modifier", ""),
            "instruction": step.get("name", ""), # Using name as instruction placeholder if maneuver doesn't have it, or we could just use name
            "location": (location[1], location[0]),
            "distance": distance
        })

    result = (waypoints, parsed_steps)
    
    _route_cache[cache_key] = copy.deepcopy(result)
    if len(_route_cache) > _CACHE_MAX_SIZE:
        _route_cache.popitem(last=False)
        
    return result
