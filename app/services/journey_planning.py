"""Trusted, expiring road-route previews; no client-provided route is executed."""
import asyncio
import hashlib
import json
import math
import os
import secrets
import time
from collections import OrderedDict
from dataclasses import dataclass
import httpx
from app.services.routing import get_route

@dataclass(frozen=True)
class JourneyPlan:
    plan_id: str
    route_hash: str
    start: tuple
    destination: tuple
    waypoints: tuple
    steps: tuple
    expires_at: float

class JourneyPlans:
    def __init__(self, capacity=32, ttl_s=300., clock=time.monotonic):
        self.capacity, self.ttl_s, self.clock = capacity, ttl_s, clock
        self.plans = OrderedDict()

    def _prune(self):
        for key, plan in list(self.plans.items()):
            if plan.expires_at <= self.clock(): self.plans.pop(key, None)

    async def preview(self, start, destination):
        start, destination = coordinate(start), coordinate(destination)
        if start == destination: raise ValueError('Start and destination must differ')
        data = await get_route(*start, *destination)
        if not isinstance(data, (list, tuple)) or len(data) != 2: raise ValueError('Road routing unavailable; no substitute route created')
        points, steps = data
        if not isinstance(points, (list, tuple)) or not 2 <= len(points) <= 50000: raise ValueError('Invalid road-route geometry')
        points = tuple(coordinate(p) for p in points)
        if len(set(points)) < 2: raise ValueError('Road route has no travel distance')
        # Freeze maneuvers through JSON, preventing shared route-cache mutation.
        if not isinstance(steps, (list, tuple)) or any(not isinstance(s, dict) for s in steps): raise ValueError("Invalid route maneuvers")
        frozen_steps = tuple(json.dumps(s, allow_nan=False, sort_keys=True) for s in steps)
        digest = hashlib.sha256(json.dumps([points, frozen_steps], separators=(',', ':')).encode()).hexdigest()
        plan = JourneyPlan(secrets.token_urlsafe(24), digest, points[0], points[-1], points, frozen_steps, self.clock()+self.ttl_s)
        self._prune()
        self.plans[plan.plan_id] = plan
        while len(self.plans) > self.capacity: self.plans.popitem(last=False)
        return plan

    def consume(self, plan_id):
        self._prune()
        if not isinstance(plan_id, str) or plan_id not in self.plans:
            raise ValueError('Route preview expired or unknown; preview again')
        return self.plans.pop(plan_id)

    def get(self, plan_id):
        self._prune()
        if not isinstance(plan_id, str) or plan_id not in self.plans: raise ValueError('Route preview expired or unknown')
        return self.plans[plan_id]


def coordinate(value):
    if not isinstance(value, (tuple, list)) or len(value) != 2: raise ValueError('Expected latitude, longitude')
    if any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in value):
        raise ValueError('Coordinates must be finite numbers')
    lat,lng=value
    if not -90<=lat<=90 or not -180<=lng<=180: raise ValueError('Coordinates outside geographic bounds')
    return float(lat), float(lng)

journey_plans = JourneyPlans()
_geocode_lock = asyncio.Lock()
_geocode_cache = OrderedDict()
_last_search = 0.

async def geocode(query):
    global _last_search
    query = ' '.join(query.split())
    if not 2 <= len(query) <= 200: raise ValueError('Enter 2–200 characters')
    endpoint = os.getenv('DDS_GEOCODER_URL', '')
    if not endpoint: raise ValueError('Address search is not configured. Pick on the map or enter latitude, longitude.')
    key = (endpoint, query.casefold())
    async with _geocode_lock:
        if key in _geocode_cache: return _geocode_cache[key]
        await asyncio.sleep(max(0., 1. - (time.monotonic()-_last_search)))
        _last_search = time.monotonic()
        async with httpx.AsyncClient(timeout=8.) as client:
            response = await client.get(endpoint, params={'q':query,'format':'jsonv2','limit':5},
                headers={'User-Agent':os.getenv('DDS_GEOCODER_USER_AGENT','DDSRoadSimulator/1.0 (https://github.com/pavann19/DDS)')})
            response.raise_for_status()
            data = response.json()
        results=[]
        for item in data[:5]:
            try:
                lat,lng=coordinate((float(item['lat']),float(item['lon'])))
                results.append({'lat':lat,'lng':lng,'label':str(item['display_name'])[:500]})
            except (ValueError, KeyError, TypeError): continue
        _geocode_cache[key]=results
        while len(_geocode_cache)>128: _geocode_cache.popitem(last=False)
        return results


