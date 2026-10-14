import asyncio
import copy
import json
import time
import httpx
from app.services import recorded_surroundings as rs
from app.services import environment_context as ec


def test_real_recorded_provenance_geometry_and_timestamp_unchanged():
    result=rs.load_recorded(37.616,-122.398)
    assert result and result["recorded_fallback"] and not result["live_verified"]
    source=json.loads((rs.DATA_DIR/"37.616_-122.398.json").read_text(encoding="utf-8"))
    assert result["retrieved_at"]==source["retrieved_at"]
    assert result["features"]==source["features"] and result["infrastructure"]==source["infrastructure"]
    assert len(result["features"])==16 and len(result["infrastructure"])==42
    assert result["coverage"]=="partial_default_mission_corridor"


def test_outside_default_corridor_has_no_recorded_substitute():
    assert rs.load_recorded(40.,-74.) is None
    assert rs.load_recorded(float("nan"),-122.) is None


def test_tampered_asset_and_manifest_fail_closed(monkeypatch,tmp_path):
    original=rs.DATA_DIR
    (tmp_path/"manifest.json").write_bytes((original/"manifest.json").read_bytes())
    record="37.616_-122.398.json"
    (tmp_path/record).write_bytes((original/record).read_bytes()+b" ")
    monkeypatch.setattr(rs,"DATA_DIR",tmp_path)
    assert rs.load_recorded(37.616,-122.398) is None
    (tmp_path/"manifest.json").write_text('{"records":[]}',encoding="utf-8")
    assert rs.load_recorded(37.616,-122.398) is None


def test_untrusted_provenance_and_malformed_geometry_rejected():
    p=rs.load_recorded(37.616,-122.398)
    for changes in [{"data_provider":"invented"},{"retrieved_at":"yesterday"},{"live_verified":True},{"source":"mock"}]:
        assert not rs.validate_record(dict(p,**changes))
    for field,value in [("height_m",float("nan")),("height_m",True),("geometry",[[90,0]])]:
        broken=copy.deepcopy(p);broken["features"][0][field]=value
        assert not rs.validate_record(broken)
    broken=copy.deepcopy(p);broken["infrastructure"][0]["width_m"]=True
    assert not rs.validate_record(broken)


def test_network509_uses_recorded_and_retries_after60s(monkeypatch,tmp_path):
    ec.CACHE.clear();monkeypatch.setattr(ec,"CACHE_DIR",tmp_path)
    calls=[];clock=[time.monotonic()]
    monkeypatch.setattr(ec.time,"monotonic",lambda:clock[0])
    async def failed(*args):
        calls.append(args)
        request=httpx.Request("GET","https://api.openstreetmap.org/api/0.6/map")
        response=httpx.Response(509,request=request)
        raise httpx.HTTPStatusError("bandwidth",request=request,response=response)
    monkeypatch.setattr(ec,"fetch_osm",failed)
    result=asyncio.run(ec.surroundings(37.616,-122.398))
    assert result["recorded_fallback"] and result["refresh_error"]=="HTTPStatusError"
    assert result["retrieved_at"]==rs.load_recorded(37.616,-122.398)["retrieved_at"]
    asyncio.run(ec.surroundings(37.616,-122.398));assert len(calls)==1
    clock[0]+=61
    assert ec.cached_environment_at(37.616,-122.398)["status"]=="unavailable"
    asyncio.run(ec.surroundings(37.616,-122.398));assert len(calls)==2
    assert not list(tmp_path.glob("*.json")),"recorded fallback must not overwrite live disk cache"
    outside=asyncio.run(ec.surroundings(40.,-74.))
    assert outside["status"]=="unavailable" and not outside.get("recorded_fallback")
    ec.CACHE.clear()
