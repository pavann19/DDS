"""Recorded OSM road tags with conservative direction-aware map matching.

Map tags are observations from a dated map, not verified live road conditions.
Unknown/ambiguous geometry returns no rule rather than an invented speed limit.
"""
from dataclasses import dataclass, replace
from functools import lru_cache
import json
import math
from pathlib import Path
import re

import numpy as np
from app.services.map_evidence import parse_width

DATA = Path(__file__).resolve().parents[1] / "data" / "road_semantics.json"


def parse_speed(value):
    match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(mph|km/h)?", value or "")
    if not match:
        return None
    speed = float(match[1]) * (1.609344 if match[2] == "mph" else 1.)
    return speed if 0 < speed <= 160 else None


@dataclass(frozen=True)
class RoadRules:
    way_id: int
    speed_limit_kmh: float | None
    lane_count: int | None
    oneway: str | None
    highway: str
    map_timestamp: str
    recorded_width_m: float | None = None
    width_tag: str | None = None
    map_version: int | None = None


class RoadMap:
    ORIGIN_LAT = 37.6
    ORIGIN_LNG = -122.38
    SCALE_LAT = 111320.
    SCALE_LNG = SCALE_LAT * math.cos(math.radians(ORIGIN_LAT))
    CELL = 100.

    def __init__(self, data):
        self.features = tuple(data["features"])
        # Static node metadata is never a live signal phase or permission
        # to cross a junction. Keep coverage separate from implemented rules.
        self.recorded_control_count = len(data.get("controls", ()))
        self.controls = tuple(data.get("controls", ()))
        starts, ends, rules = [], [], []
        self.index = {}
        for feature in self.features:
            tags = feature["tags"]
            if "highway" not in tags or tags["highway"] not in {
                "motorway", "motorway_link", "trunk", "trunk_link", "primary",
                "secondary", "tertiary", "residential", "unclassified", "service"}:
                continue
            lanes = tags.get("lanes", "")
            rule = RoadRules(feature["id"], parse_speed(tags.get("maxspeed")),
                             int(lanes) if lanes.isdigit() and 0 < int(lanes) < 20 else None,
                             tags.get("oneway"), tags["highway"], feature["timestamp"],
                             parse_width(tags.get('width')), tags.get('width'), feature.get('version'))
            for a, b in zip(feature["geometry"], feature["geometry"][1:]):
                p, q = self.xy(*a), self.xy(*b)
                if math.dist(p, q) < .01:
                    continue
                i = len(starts)
                starts.append(p); ends.append(q); rules.append(rule)
                for x in range(math.floor((min(p[0], q[0]) - 8) / self.CELL),
                               math.floor((max(p[0], q[0]) + 8) / self.CELL) + 1):
                    for y in range(math.floor((min(p[1], q[1]) - 8) / self.CELL),
                                   math.floor((max(p[1], q[1]) + 8) / self.CELL) + 1):
                        self.index.setdefault((x, y), []).append(i)
        self.starts = np.asarray(starts).reshape(-1, 2)
        self.delta = np.asarray(ends).reshape(-1, 2) - self.starts
        self.length_sq = np.sum(self.delta ** 2, axis=1)
        for array in (self.starts, self.delta, self.length_sq):
            array.setflags(write=False)
        self.rules = tuple(rules)

    @classmethod
    def xy(cls, lat, lng):
        return ((lng - cls.ORIGIN_LNG) * cls.SCALE_LNG,
                (lat - cls.ORIGIN_LAT) * cls.SCALE_LAT)

    def match(self, lat, lng, heading_deg):
        if not all(math.isfinite(v) for v in (lat, lng, heading_deg)):
            return None
        p = np.asarray(self.xy(lat, lng))
        ids = self.index.get(tuple(np.floor(p / self.CELL).astype(int)), ())
        if not ids:
            return None
        starts, delta = self.starts[ids], self.delta[ids]
        t = np.clip(np.sum((p - starts) * delta, axis=1) / self.length_sq[ids], 0, 1)
        distances = np.linalg.norm(starts + t[:, None] * delta - p, axis=1)
        heading = math.radians(heading_deg)
        alignment = (delta @ np.array([math.sin(heading), math.cos(heading)])) / np.sqrt(self.length_sq[ids])
        candidates = []
        for row in np.argsort(distances):
            if distances[row] > 8.:
                break
            rule = self.rules[ids[row]]
            dot = float(alignment[row])
            if rule.oneway == "-1":
                dot = -dot
            elif rule.oneway not in ("yes", "1", "true"):
                dot = abs(dot)
            if dot >= math.cos(math.radians(35)):
                candidates.append((float(distances[row]), rule))
        if not candidates:
            return None
        best_distance, best = candidates[0]
        # Parallel/stacked ways with incompatible rules are ambiguous.
        for distance, other in candidates[1:]:
            if distance > best_distance + 1.:
                break
            if other.speed_limit_kmh != best.speed_limit_kmh:
                return None
            if other.lane_count != best.lane_count:
                best = replace(best, lane_count=None)
            if other.recorded_width_m != best.recorded_width_m or other.width_tag != best.width_tag:
                best = replace(best, recorded_width_m=None, width_tag=None)
        return best


@lru_cache(maxsize=1)
def recorded_map():
    return RoadMap(json.loads(DATA.read_text(encoding="utf-8")))
