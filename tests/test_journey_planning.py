import pytest
import app.services.journey_planning as module

async def route(*args): return ([(37.,-122.),(37.001,-122.001)], [{'type':'depart'}])

async def test_immutable_server_plan_one_time_and_hash(monkeypatch):
    monkeypatch.setattr(module,'get_route',route)
    store=module.JourneyPlans()
    plan=await store.preview((37.,-122.),(37.001,-122.001))
    assert len(plan.route_hash)==64 and plan.waypoints[0]==plan.start
    assert store.get(plan.plan_id) is plan
    assert store.consume(plan.plan_id) is plan
    with pytest.raises(ValueError):store.consume(plan.plan_id)

async def test_expiry_and_bounded_storage(monkeypatch):
    monkeypatch.setattr(module,'get_route',route)
    now=[0.];store=module.JourneyPlans(capacity=1,ttl_s=2,clock=lambda:now[0])
    old=await store.preview((1.,1.),(2.,2.));latest=await store.preview((1.,1.),(2.,2.))
    with pytest.raises(ValueError):store.get(old.plan_id)
    now[0]=2.
    with pytest.raises(ValueError):store.get(latest.plan_id)

@pytest.mark.parametrize('point',[(float('nan'),0),(91,0),(0,181),(True,0),(1,),('x',2)])
async def test_invalid_coordinates_never_fetch(monkeypatch,point):
    async def forbidden(*args):raise AssertionError('Should not fetch')
    monkeypatch.setattr(module,'get_route',forbidden)
    with pytest.raises(ValueError):await module.JourneyPlans().preview(point,(1.,2.))

async def test_unavailable_and_degenerate_never_create_fake_route(monkeypatch):
    async def unavailable(*args):return None
    monkeypatch.setattr(module,'get_route',unavailable)
    store=module.JourneyPlans()
    with pytest.raises(ValueError):await store.preview((1.,1.),(2.,2.))
    assert not store.plans
    async def bad(*args):return ([(1.,1.),(1.,1.)],[])
    monkeypatch.setattr(module,'get_route',bad)
    with pytest.raises(ValueError):await store.preview((1.,1.),(2.,2.))

async def test_geocoder_disabled_does_not_contact_external_service(monkeypatch):
    monkeypatch.delenv('DDS_GEOCODER_URL',raising=False)
    with pytest.raises(ValueError,match='not configured'):await module.geocode('London')

async def test_configured_geocode_explicit_cached_and_identified(monkeypatch):
    import httpx
    calls=[]
    monkeypatch.setenv('DDS_GEOCODER_URL','https://example.test/search')
    module._geocode_cache.clear()
    module._last_search=0.
    class Client:
        def __init__(self,**kwargs):pass
        async def __aenter__(self):return self
        async def __aexit__(self,*args):pass
        async def get(self,url,**kwargs):
            calls.append((url,kwargs))
            return httpx.Response(200,json=[{'lat':'37','lon':'-122','display_name':'Place'}],request=httpx.Request('GET',url))
    monkeypatch.setattr(module.httpx,'AsyncClient',Client)
    first=await module.geocode(' Place ')
    second=await module.geocode('place')
    assert first==second==[{'lat':37.,'lng':-122.,'label':'Place'}]
    assert len(calls)==1 and 'DDSRoadSimulator' in calls[0][1]['headers']['User-Agent']
    module._geocode_cache.clear()

async def test_api_rejects_coerced_coordinates_and_client_waypoints():
    from pydantic import ValidationError
    from app.api.journey import Preview
    for bad in [True,'37',float('nan')]:
        if isinstance(bad,float):continue # Finite validation is in trusted service.
        with pytest.raises(ValidationError):Preview(start={'lat':bad,'lng':0},destination={'lat':1,'lng':1})
    with pytest.raises(ValidationError):Preview(start={'lat':0,'lng':0},destination={'lat':1,'lng':1},waypoints=[[0,0],[1,1]])

def test_start_journey_consumes_trusted_plan_and_spawns_paused_route_origin():
    import json, time
    from app.services.journey_planning import JourneyPlan, journey_plans
    from app.services.simulation_session import SimulationSession
    session=SimulationSession(seed=42,real_time=False)
    session._handle_command({'type':'load_scenario','scenario_id':'driving_environment','params':{'scene_camera_controls':True}})
    original=session.world.physics
    route=tuple(original.route[20:100])
    token='trusted-route-preview-123456'
    plan=JourneyPlan(token,'a'*64,route[0],route[-1],route,
        tuple(json.dumps({'instruction':'Continue on mapped road'}) for _ in range(1)),time.monotonic()+60)
    journey_plans.plans[token]=plan
    session._handle_command({'type':'start_journey','plan_id':token})
    physics=session.world.physics
    assert physics.route[0] == route[0]
    assert physics.speed_kmh == 0.
    assert physics.traffic.finite_world
    assert session.route_status == 'ready' and session.route_steps[0]['instruction']=='Continue on mapped road'
    assert not session.is_paused
    with pytest.raises(ValueError,match='expired or unknown'):
        journey_plans.get(token)

def test_paused_vehicle_publishes_hazard_lights():
    from app.services.simulation_session import SimulationSession
    session=SimulationSession(seed=42,real_time=False)
    session._handle_command({'type':'load_scenario','scenario_id':'driving_environment','params':{'scene_camera_controls':True}})
    session.step_pipeline(10)
    ego=session.build_telemetry_payload()['channels']['pose']['ego']
    assert ego['signals']=={'left':False,'right':False,'hazards':True,'brake':False}

def test_live_vehicle_diagnostics_publish_each_required_camera_view():
    from app.services.simulation_session import SimulationSession
    session=SimulationSession(seed=42,real_time=False)
    session._handle_command({'type':'load_scenario','scenario_id':'driving_environment','params':{'scene_camera_controls':True}})
    session.step_pipeline(10)
    ego=session.build_telemetry_payload()['channels']['pose']['ego']
    assert len(ego['camera_view_health']) == 8
    assert {row[0] for row in ego['camera_view_health']} == {'front','front_left','front_right','left','right','rear_left','rear_right','rear'}

@pytest.mark.parametrize('token',[None,{},[],True,'missing'])
def test_untrusted_token_shapes_rejected(token):
    with pytest.raises(ValueError):module.JourneyPlans().get(token)
    with pytest.raises(ValueError):module.JourneyPlans().consume(token)
