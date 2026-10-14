"""Geometry/cache contract checks; fixtures are tests, not claimed live OSM."""
import asyncio
import math
import time
import httpx
import pytest
from app.services import environment_context as ec


def building(i=1):
    return {'type':'way','id':i,'timestamp':'2026-10-13','tags':{'building':'yes'},'geometry':[
        {'lat':37.,'lon':-122.},{'lat':37.0001,'lon':-122.},
        {'lat':37.0001,'lon':-121.9999},{'lat':37.,'lon':-122.}]}


def test_actual_position_footprint_contract_with_declared_modeled_height():
    b=building();b2=building(2);b2['tags']['height']='12 m'
    t={'type':'node','id':3,'lat':37.,'lon':-122.,'tags':{'natural':'tree'}}
    out=ec.normalize({'elements':[b,b2,t]},37.,-122.)
    assert len(out['features'])==3
    assert out['features'][0]['geometry']==[[p['lat'],p['lon']] for p in b['geometry']]
    assert out['features'][0]['height_source']=='modeled_display_height'
    assert out['features'][1]['height_m']==12.
    assert out['features'][1]['height_source']=='recorded_OSM_height_tag'
    assert out['features'][2]['geometry']==[[37.,-122.]]
    assert not out['live_verified']


def test_missing_features_stay_absent_and_malformed_geometry_is_not_invented():
    far=building();far['geometry'][0]['lat']=40.
    open_ring=building();open_ring['geometry'].pop()
    huge=building();huge['geometry']*=100
    assert ec.normalize({'elements':[far,open_ring,huge]},37.,-122.)['features']==[]
    assert ec.normalize({'elements':[]},37.,-122.)['features']==[]


def test_feature_count_is_bounded():
    out=ec.normalize({'elements':[building(i) for i in range(200)]},37.,-122.)
    assert len(out['features'])==ec.MAX_FEATURES


@pytest.mark.parametrize('lat,lng',[(float('nan'),0),(90,0),(0,181)])
def test_invalid_query_rejected(lat,lng):
    with pytest.raises(ValueError):ec.cell(lat,lng)


def test_memory_and_disk_cache_avoid_repeated_network(monkeypatch,tmp_path):
    ec.CACHE.clear();monkeypatch.setattr(ec,'CACHE_DIR',tmp_path);calls=[]
    async def fetch(lat,lng):calls.append((lat,lng));return {'elements':[building()]}
    monkeypatch.setattr(ec,'fetch_osm',fetch)
    async def run():
        a=await ec.surroundings(37.,-122.);b=await ec.surroundings(37.0001,-122.0001)
        assert a['status']=='available' and b['cache_hit'] and len(calls)==1
        ec.CACHE.clear();c=await ec.surroundings(37.,-122.)
        assert c['cache_hit'] and len(calls)==1
    asyncio.run(run())


def test_network_failure_is_explicit_empty_and_cached_lookup_nonblocking(monkeypatch,tmp_path):
    ec.CACHE.clear();monkeypatch.setattr(ec,'CACHE_DIR',tmp_path)
    async def fail(*args):raise httpx.ReadTimeout('test timeout')
    monkeypatch.setattr(ec,'fetch_osm',fail)
    out=asyncio.run(ec.surroundings(37.,-122.))
    assert out['status']=='unavailable' and out['features']==[] and out['error']=='ReadTimeout'
    assert ec.cached_environment_at(37.,-122.)['status']=='unavailable'


def test_static_lookup_requires_fresh_coverage():
    ec.CACHE.clear();ec.CACHE['37.000_-122.000']=(time.monotonic(),ec.normalize({'elements':[building()]},37.,-122.))
    assert ec.cached_environment_at(37.,-122.)['status']=='available'
    assert ec.cached_environment_at(38.,-122.)['status']=='unavailable'
    ec.CACHE['37.000_-122.000']=(time.monotonic()-90000,ec.CACHE['37.000_-122.000'][1])
    assert ec.cached_environment_at(37.,-122.)['status']=='unavailable'



def test_static_lookup_reuses_nearby_real_center_but_not_distant_queries():
    ec.CACHE.clear();payload=ec.normalize({'elements':[building()]},37.,-122.)
    ec.CACHE['37.000_-122.000']=(time.monotonic(),payload)
    nearby=ec.cached_environment_at(37.001,-122.)
    assert nearby is payload and nearby['center']==[37.,-122.]
    assert ec.cached_environment_at(37.002,-122.)['status']=='unavailable'

def test_mapped_infrastructure_width_and_bridge_metadata_are_honest():
    road={'id':4,'tags':{'highway':'residential','width':'9 m','bridge':'yes','layer':'1'},'geometry':[{'lat':37.,'lon':-122.},{'lat':37.0001,'lon':-122.}]}
    rail=dict(road,id=5,tags={'railway':'rail'})
    out=ec.normalize({'elements':[road,rail]},37.,-122.)
    assert out['features']==[]
    a,b=out['infrastructure']
    assert a['width_m']==9 and a['width_source']=='recorded_OSM_width_tag'
    assert a['bridge'] and a['layer']==1 and a['elevation_source']=='unavailable_ground_projection'
    assert b['kind']=='railway' and b['width_source']=='modeled_display_width'


def test_infrastructure_malformed_and_bounds():
    road={'tags':{'highway':'residential','width':'nan','lanes':'4'},'geometry':[{'lat':37.,'lon':-122.},{'lat':37.0001,'lon':-122.}]}
    out=ec.normalize({'elements':[road]*200},37.,-122.)
    assert len(out['infrastructure'])==ec.MAX_FEATURES
    assert out['infrastructure'][0]['width_m']==12.8
    road['geometry'][0]['lat']=float('nan')
    assert ec.normalize({'elements':[road]},37.,-122.)['infrastructure']==[]

def test_recorded_levels_are_used_with_modeled_height_provenance():
    b=building();b['tags']['building:levels']='5'
    f=ec.normalize({'elements':[b]},37.,-122.)['features'][0]
    assert f['height_m']==15 and f['height_source']=='modeled_from_recorded_levels'

def test_malformed_infrastructure_tags_do_not_crash():
    road={'tags':{'highway':{},'bridge':{}},'geometry':[]}
    assert ec.normalize({'elements':[road],'osm3s':None},37.,-122.)['infrastructure']==[]


def test_building_budget_does_not_drop_roads_later_in_query():
    road={'id':400,'tags':{'highway':'residential'},'geometry':[{'lat':37.,'lon':-122.},{'lat':37.0001,'lon':-122.}]}
    out=ec.normalize({'elements':[building(i) for i in range(200)]+[road]},37.,-122.)
    assert len(out['features'])==ec.MAX_FEATURES
    assert len(out['infrastructure'])==1


def test_building_no_and_numeric_height_are_handled():
    b=building();b['tags']['height']=12
    assert ec.normalize({'elements':[b]},37.,-122.)['features'][0]['height_m']==12
    b['tags']['building']='no'
    assert ec.normalize({'elements':[b]},37.,-122.)['features']==[]

def road_xy(coords):
    return {'type':'way','id':991,'tags':{'highway':'residential','width':'7 m'},'geometry':[{'lat':37+y/111320,'lon':-122+x/(111320*math.cos(math.radians(37)))} for x,y in coords]}


def test_long_crossing_way_clips_even_both_endpoints_outside():
    road=road_xy([(-1000,0),(1000,0)])
    out=ec.normalize({'elements':[road]},37.,-122.)
    f=out['infrastructure'][0]
    assert f['osm_id']==991 and f['width_m']==7
    assert f['geometry_source']=='OSM_segments_clipped_boundary_interpolation'
    xs=[(p[1]+122)*111320*math.cos(math.radians(37)) for p in f['geometry']]
    assert xs==pytest.approx([-600,600])


def test_multiple_entries_split_without_outside_shortcut():
    road=road_xy([(-1000,0),(0,0),(1000,0),(1000,1000),(0,0),(-1000,0)])
    fs=ec.normalize({'elements':[road]},37.,-122.)['infrastructure']
    assert len(fs)==2 and len({f['geometry_id'] for f in fs})==2
    assert fs[0]['geometry'][-1]!=fs[1]['geometry'][0]
    for f in fs:
        for lat,lng in f['geometry']:
            assert math.hypot((lat-37)*111320,(lng+122)*111320*math.cos(math.radians(37)))<=600.00001


def test_invalid_way_coordinate_rejects_whole_way_and_input_bound():
    road=road_xy([(-1000,0),(0,0),(1000,0)])
    road['geometry'][1]['lat']=float('nan')
    assert ec.normalize({'elements':[road]},37.,-122.)['infrastructure']==[]
    road['geometry']=road_xy([(0,0),(1,0)])['geometry']*(ec.MAX_INPUT_VERTICES//2+1)
    assert ec.normalize({'elements':[road]},37.,-122.)['infrastructure']==[]


def test_large_valid_local_way_split_respects_output_bounds():
    road=road_xy([(i,0) for i in range(300)])
    fs=ec.normalize({'elements':[road]},37.,-122.)['infrastructure']
    assert len(fs)==3
    assert all(2<=len(f['geometry'])<=ec.MAX_VERTICES for f in fs)
    assert fs[0]['geometry'][-1]==fs[1]['geometry'][0]


def test_old_cache_schema_refetched_and_new_clips_survive_disk(monkeypatch,tmp_path):
    ec.CACHE.clear();monkeypatch.setattr(ec,'CACHE_DIR',tmp_path);calls=[]
    old=ec.normalize({'elements':[]},37.,-122.);old['schema_version']=2
    (tmp_path/'37.000_-122.000.json').write_text(__import__('json').dumps(old))
    ec.CACHE['37.000_-122.000']=(time.monotonic(),old)
    async def fetch(*args):calls.append(1);return {'elements':[road_xy([(-1000,0),(1000,0)])]}
    monkeypatch.setattr(ec,'fetch_osm',fetch)
    a=asyncio.run(ec.surroundings(37.,-122.));ec.CACHE.clear()
    b=asyncio.run(ec.surroundings(37.,-122.))
    assert len(calls)==1 and b['cache_hit']
    assert a['infrastructure']==b['infrastructure'] and a['schema_version']==ec.SCHEMA_VERSION


def test_network_failure_negative_cache_keeps_current_schema(monkeypatch,tmp_path):
    ec.CACHE.clear();monkeypatch.setattr(ec,'CACHE_DIR',tmp_path);calls=[]
    async def fail(*args):calls.append(1);raise httpx.ReadTimeout('fixture failure')
    monkeypatch.setattr(ec,'fetch_osm',fail)
    async def run():
        a=await ec.surroundings(37.,-122.)
        b=await ec.surroundings(37.,-122.)
        assert a['status']=='unavailable' and a['schema_version']==ec.SCHEMA_VERSION
        assert b['cache_hit'] and len(calls)==1
        assert ec.cached_environment_at(37.,-122.)['status']=='unavailable'
    asyncio.run(run())
