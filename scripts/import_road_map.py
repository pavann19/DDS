"""Import a small recorded OSM extract; never downloads or invents map tags."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

TAGS = {"highway", "service", "amenity", "parking", "access", "motor_vehicle",
        "oneway", "lanes", "width", "maxspeed", "name", "ref", "layer", "bridge",
        "tunnel", "surface", "orientation", "capacity", "access:conditional",
        "vehicle", "motorcar", "vehicle:conditional", "motor_vehicle:conditional",
        "motorcar:conditional", "oneway:conditional", "lanes:forward", "lanes:backward", "lanes:both_ways"}
CONTROL_TAGS = {"highway", "direction", "traffic_signals", "traffic_signals:direction",
                "traffic_signals:conditional", "stop", "stop:direction", "give_way", "crossing",
                "layer", "level", "traffic_signals:arrow"}


def import_map(paths):
    nodes, ways, sources, controls = {}, {}, [], {}
    for path in paths:
        raw = path.read_bytes()
        root = ET.fromstring(raw)
        sources.append({"file": path.name, "sha256": hashlib.sha256(raw).hexdigest(),
                        "bounds": root.find("bounds").attrib,
                        "database_timestamp": root.find("meta").get("osm_base")
                        if root.find("meta") is not None else None})
        for n in root.findall("node"):
            nodes[n.get("id")] = (float(n.get("lat")), float(n.get("lon")))
            tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
            if tags.get("highway") in {"stop", "give_way", "traffic_signals"}:
                controls[n.get("id")] = {
                    "id": int(n.get("id")), "version": int(n.get("version", "1")),
                    "timestamp": n.get("timestamp"),
                    "geometry": nodes[n.get("id")],
                    "tags": {k: v for k, v in tags.items() if k in CONTROL_TAGS}}
        for w in root.findall("way"):
            tags = {t.get("k"): t.get("v") for t in w.findall("tag") if t.get("k") in TAGS}
            if "highway" in tags or tags.get("amenity") in ("parking", "parking_space"):
                ways[w.get("id")] = {"id": int(w.get("id")), "version": int(w.get("version")),
                    "timestamp": w.get("timestamp"), "tags": tags,
                    "nodes": [n.get("ref") for n in w.findall("nd")]}
    features = []
    for w in ways.values():
        if all(n in nodes for n in w["nodes"]):
            features.append({**w, "geometry": [nodes[n] for n in w["nodes"]]})
    memberships = {node: [] for node in controls}
    for w in features:
        for node in set(w["nodes"]) & memberships.keys():
            memberships[node].append(w["id"])
    for node, control in controls.items():
        control["way_ids"] = sorted(memberships[node])
    return {"schema_version": 2, "source": "OpenStreetMap contributors",
            "source_api": "https://api.openstreetmap.org/api/0.6/map",
            "license": "ODbL-1.0", "downloaded_date": "2026-10-11", "sources": sources,
            "features": sorted(features, key=lambda w: w["id"]),
            "controls": sorted(controls.values(), key=lambda n: n["id"])}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(import_map(args.inputs), separators=(",", ":")), encoding="utf-8")
