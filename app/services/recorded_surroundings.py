"""Integrity-pinned real OSM records for the default mission corridor only.
Recorded maps supply static context, never proof of current free space.
"""
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re

DATA_DIR=Path(__file__).resolve().parents[1]/"data"/"recorded_surroundings"
MANIFEST_SHA256="e129bf09570f3a4783089b0a2c00b93e33350f3fde2dad0a8740e8d5d10b077a"
MAX_RECORD_BYTES=4_000_000
MAX_BUNDLE_BYTES=8_000_000
MAX_RECORDS=96

def _bounded_bytes(path,limit):
    with path.open("rb") as stream:raw=stream.read(limit+1)
    if len(raw)>limit:raise ValueError("Recorded asset exceeds byte bound")
    return raw

def _number(value,low,high):
    return not isinstance(value,bool) and isinstance(value,(int,float)) and math.isfinite(value) and low<=value<=high

def _distance(a,b):
    return math.hypot((a[0]-b[0])*111320,(a[1]-b[1])*111320*math.cos(math.radians(a[0])))

def _point(p):
    return isinstance(p,list) and len(p)==2 and _number(p[0],-85,85) and _number(p[1],-180,180)

def validate_record(payload):
    if not isinstance(payload,dict) or payload.get("status")!="available" or payload.get("schema_version")!=2 or payload.get("source")!="OpenStreetMap_geometry" or payload.get("live_verified") is not False:return False
    if payload.get("data_provider") not in {"Overpass_API","OpenStreetMap_official_map_API"} or payload.get("attribution")!="OpenStreetMap contributors (ODbL)":return False
    center=payload.get("center")
    if not _point(center) or payload.get("radius_m")!=300:return False
    try:
        retrieved=datetime.fromisoformat(payload["retrieved_at"].replace("Z","+00:00"))
        if retrieved.tzinfo is None:return False
    except (ValueError,TypeError,KeyError,AttributeError):return False
    features=payload.get("features");roads=payload.get("infrastructure")
    if not isinstance(features,list) or not isinstance(roads,list) or len(features)>160 or len(roads)>160:return False
    for f,is_feature in [(f,True) for f in features]+[(f,False) for f in roads]:
        if not isinstance(f,dict) or not _number(f.get("osm_id"),1,10**13):return False
        geometry=f.get("geometry")
        if not isinstance(geometry,list) or not 1<=len(geometry)<=128 or not all(_point(p) and _distance(center,p)<=600 for p in geometry):return False
        if is_feature:
            if f.get("kind") not in {"building","tree","park"} or not _number(f.get("height_m"),2,100):return False
            if f.get("height_source") not in {"recorded_OSM_height_tag","modeled_display_height","modeled_from_recorded_levels"}:return False
            if f["kind"]=="tree":
                if len(geometry)!=1:return False
            elif len(geometry)<4 or geometry[0]!=geometry[-1]:return False
        else:
            if f.get("kind") not in {"road","railway"} or len(geometry)<2 or not _number(f.get("width_m"),1,60):return False
            if f.get("width_source") not in {"recorded_OSM_width_tag","modeled_from_lanes","modeled_display_width"}:return False
            if not isinstance(f.get("bridge"),bool) or not isinstance(f.get("tunnel"),bool):return False
            if f.get("layer") is not None and not _number(f["layer"],-10,10):return False
        if not isinstance(f.get("timestamp"),str) or not isinstance(f.get("version"),(str,int)):return False
    return True

def load_recorded(lat,lng):
    """Return no geometry outside the validated mission corridor/record radius.
    Asset timestamps and all source geometry/provenance are kept unchanged.
    """
    if not _point([lat,lng]):return None
    from app.services.parking import parking_plan
    try:
        if min(_distance([lat,lng],p) for p in parking_plan().waypoints)>150:return None
        raw=_bounded_bytes(DATA_DIR/"manifest.json",100_000)
        if len(raw)>100_000 or hashlib.sha256(raw).hexdigest()!=MANIFEST_SHA256:return None
        manifest=json.loads(raw)
        entries=manifest["records"]
        if manifest.get("attribution")!="OpenStreetMap contributors (ODbL)" or manifest.get("coverage")!="partial_default_mission_corridor" or not isinstance(entries,list) or not 1<=len(entries)<=MAX_RECORDS:return None
        if sum(e["bytes"] for e in entries)>MAX_BUNDLE_BYTES:return None
        nearest=None;distance=151.
        for entry in entries:
            if not re.fullmatch(r"-?\d+\.\d{3}_-?\d+\.\d{3}\.json",entry["file"]) or not _point(entry["center"]):return None
            d=_distance([lat,lng],entry["center"])
            if d<distance:nearest=entry;distance=d
        if nearest is None:return None
        raw=_bounded_bytes(DATA_DIR/nearest["file"],MAX_RECORD_BYTES)
        if not 1<=len(raw)<=MAX_RECORD_BYTES or len(raw)!=nearest["bytes"] or hashlib.sha256(raw).hexdigest()!=nearest["sha256"]:return None
        payload=json.loads(raw)
        if not validate_record(payload) or payload["center"]!=nearest["center"]:return None
        return dict(payload,recorded_fallback=True,coverage="partial_default_mission_corridor",recorded_asset_sha256=nearest["sha256"])
    except (OSError,ValueError,TypeError,KeyError,OverflowError):return None
