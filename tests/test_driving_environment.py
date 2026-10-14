"""Combined world configuration tests; not a driving-success benchmark."""
import dataclasses
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services.scenario_engine import ScenarioEngine
from app.services.parking import parking_plan
from app.services.traffic import DENSITY_NPC_COUNTS, EGO_LANE_OFFSET_M


def setup(**kwargs):
    p=PhysicsEngine();e=ScenarioEngine();e.load_scenario('driving_environment',p,**kwargs)
    return e,p


def snapshot(e,p):
    return (e.active_scenario_id,e.active_density,dict(e.active_params),p.current_station_m,
            p.speed_kmh,tuple(p.route),p.scene_camera_controls,p.camera_controls,
            p.camera_required_for_drive,p.friction_mu,
            tuple(tuple(dataclasses.asdict(n).items()) for n in p.traffic.npcs))


def test_default_combined_world_preserves_full_recorded_mission_and_camera():
    e,p=setup()
    assert p.route[0]==parking_plan().waypoints[0]
    assert p.route[-1]==parking_plan().waypoints[-1]  # Existing plant resamples recorded input.
    assert p.frenet_frame.total_length_m>4000
    assert p.speed_kmh==0 and p.current_station_m==0
    assert p.scene_camera_controls and p.camera_controls and p.camera_required_for_drive
    assert e.active_params['scene_camera_controls'] is True
    assert len(p.route_controls)>=2
    assert p.parking_mission['goal']==parking_plan().goal
    assert e.get_active_scenario().tick_fn is None
    for _ in range(50):assert e.update(p,.1) is None
    assert len(e.events_log)==1
    env=e.get_state()['environment']
    assert env['scripted_tick_events'] is False
    assert env['lane_source']=='single_same_direction_modeled_lane_not_surveyed'
    assert env['traffic_source']=='seeded_TrafficModel_IDM'


@pytest.mark.parametrize('density',list(DENSITY_NPC_COUNTS))
def test_density_has_actual_ordinary_actors_outside_parking(density):
    e,p=setup(density=density)
    assert len(p.traffic.npcs)==DENSITY_NPC_COUNTS[density]
    assert p.traffic.density==density
    assert p.traffic.total_length_m==p.parking_mission['approach_start_m']
    assert p.traffic.total_length_m<p.parking_mission['maneuver_start_m']
    for actor in p.traffic.npcs:
        assert actor.id.startswith('npc-') and not actor.prevent_recycle
        assert 0<=actor.station_m<=p.traffic.total_length_m
        assert actor.lane_offset==actor.base_lane_offset==EGO_LANE_OFFSET_M
        assert actor.wander_amplitude_m==0 and actor.speed_kmh>0
    p.traffic.update(.1,ego_station_m=0.)
    assert all(0<=a.station_m<=p.traffic.total_length_m for a in p.traffic.npcs)


def test_reset_reproduces_world_and_surface_parameter():
    e,p=setup(density='high',params={'friction_mu':.5,'sensor_noise_enabled':True})
    before=snapshot(e,p)
    p.traffic.npcs[0].station_m+=123
    p.current_station_m=300
    p.friction_mu=1
    e.reset(p)
    assert snapshot(e,p)==before
    assert p.friction_mu==.5 and p.traffic.sensor_noise_enabled
    assert e.get_state()['environment']['weather_source']=='friction_parameter_only_no_weather_sensor'


@pytest.mark.parametrize('params,scene,cpu',[
    ({'scene_camera_controls':False},False,False),
    ({'camera_controls':True},False,True),
    ({'camera_controls':False},False,False),
    ({'scene_camera_controls':True,'camera_controls':False},True,True)])
def test_explicit_advanced_camera_override(params,scene,cpu):
    e,p=setup(params=params)
    assert p.scene_camera_controls is scene and p.camera_controls is cpu
    assert p.camera_required_for_drive is scene
    e.reset(p)
    assert p.scene_camera_controls is scene and p.camera_required_for_drive is scene


@pytest.mark.parametrize('kwargs',[
    {'density':'fake'}, {'initial_speed_kmh':20},
    {'params':{'friction_mu':float('nan')}}, {'params':{'scene_camera_controls':'true'}},
    {'params':{'unknown':True}}, {'params':{'friction_mu':.01}}])
def test_invalid_config_is_rejected_before_world_mutation(kwargs):
    e,p=setup();before=snapshot(e,p)
    with pytest.raises(ValueError):e.load_scenario('driving_environment',p,**kwargs)
    assert snapshot(e,p)==before


def test_previous_fixture_remains_unchanged_and_clears_requirement():
    e,p=setup();e.load_scenario('mapped_parking',p)
    assert p.traffic.npcs==[]
    assert not p.scene_camera_controls and not p.camera_required_for_drive
    assert p.frenet_frame.total_length_m<1000



def test_finite_world_has_no_ego_relative_teleports_and_actors_leave():
    e,p=setup();t=p.traffic
    assert t.finite_world
    a=t.npcs[0];a.station_m=500.;a.speed_kmh=20.
    t.update(.1,ego_station_m=4000.)
    assert 500.<=a.station_m<502.  # No teleport near ego.
    a.station_m=t.total_length_m-.01;a.speed_kmh=36.
    t.update(.1,ego_station_m=0.)
    assert a not in t.npcs and a.station_m>t.total_length_m


def test_background_actor_obeys_actual_world_red_then_green():
    from app.services.junctions import RouteControl
    from app.services.traffic import TrafficModel,NpcVehicle
    c=RouteControl(0,'traffic_signals',100.,0.,-100.,'recorded',True)
    a=NpcVehicle('world',1.75,18.,70.,desired_speed_kmh=36.)
    t=TrafficModel(200.,npcs=[a],finite_world=True,world_controls=(c,))
    for _ in range(1100):t.update(.01,ego_station_m=9999.)
    assert a.station_m<94. and a.speed_kmh==0.
    before=a.station_m
    for _ in range(400):t.update(.01,ego_station_m=-9999.)
    assert a.station_m>before+1 and a.speed_kmh>1.


def test_background_speed_target_uses_mapped_limit():
    from app.services.traffic import TrafficModel,NpcVehicle
    a=NpcVehicle('world',1.75,0.,20.,desired_speed_kmh=60.)
    t=TrafficModel(200.,npcs=[a],finite_world=True,speed_limit_at_station=lambda station:15.)
    for _ in range(1000):t.update(.01,ego_station_m=0.)
    assert 0<a.speed_kmh<=15.



def test_background_stop_requires_two_seconds_of_exact_standstill():
    from app.services.junctions import RouteControl
    from app.services.traffic import TrafficModel,NpcVehicle
    c=RouteControl(1,'stop',100.,0.,-100.,'recorded',True)
    a=NpcVehicle('world',1.75,18.,70.,desired_speed_kmh=36.)
    t=TrafficModel(200.,npcs=[a],finite_world=True,world_controls=(c,))
    held=0.
    for _ in range(1400):
        t.update(.01,ego_station_m=0.)
        if a.speed_kmh==0.:held+=.01
        if ('world',1) in t._control_released:
            break
    assert held>=1.99 and ('world',1) in t._control_released
    assert a.station_m<94.
    for _ in range(200):t.update(.01,ego_station_m=0.)
    assert a.station_m>94. and a.speed_kmh>1.



def test_copied_world_limit_does_not_capture_mutable_physics_route():
    import copy
    e,p=setup()
    cloned=copy.deepcopy(p.traffic)
    before=cloned.speed_limit_at_station(100.)
    p.frenet_frame=None
    assert cloned.speed_limit_at_station(100.)==before


def test_existing_scenario_names_keep_unicode_separator():
    e=ScenarioEngine()
    assert e.scenarios['recorded_signal'].name=='East Millbrae '+chr(183)+' Simulated Signal'
    assert e.scenarios['recorded_stop'].name=='Ray Drive '+chr(183)+' Recorded Stop'
    assert e.scenarios['recorded_yield_conflict'].name=='Recorded Yield '+chr(183)+' Simulated Conflict'



def test_destination_helper_preserves_finite_world_over_entire_new_route():
    from app.services.scenario_engine import configure_driving_environment_traffic
    from app.services.road_routes import real_road_route
    e,p=setup()
    p.set_route(real_road_route('wet_curve'))
    t=configure_driving_environment_traffic(p,'high')
    assert p.traffic is t and t.finite_world
    assert len(t.npcs)==14
    assert t.total_length_m==p.frenet_frame.total_length_m
    assert t.world_controls==p.route_controls
    assert t.speed_limit_at_station is not None
    assert all(a.lane_offset==1.75 and a.speed_kmh>0 for a in t.npcs)


@pytest.mark.parametrize('density,end',[('fake',None),('low',float('nan')),('low',-1),('low',10000)])
def test_destination_helper_bad_config_keeps_current_traffic(density,end):
    from app.services.scenario_engine import configure_driving_environment_traffic
    e,p=setup();old=p.traffic
    with pytest.raises(ValueError):configure_driving_environment_traffic(p,density,end)
    assert p.traffic is old
