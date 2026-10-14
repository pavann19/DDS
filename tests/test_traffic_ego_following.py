"""Background traffic follows the actual ego body through the world boundary."""
import math
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services.traffic import TrafficModel,NpcVehicle
from app.services.frenet import frenet_to_local_xz,latlng_to_local
from app.services.safety_geometry import Footprint,intersects_during
from app.services.vehicle_geometry import lateral_half_extent,CHASSIS_HALF_LENGTH_M
from app.services.perception.entities import EntityClass,dimensions_for


@pytest.mark.parametrize('count',[3,8,16,17,64])
def test_short_and_long_body_paths_match_vectorized_mapping(count):
    import numpy as np
    from app.services.parking import parking_plan
    from app.services.frenet import frenet_to_local_xz_batch
    p=PhysicsEngine(seed=42);p.set_route(parking_plan().waypoints)
    for stations in (np.linspace(4320.,4450.,count),
                     np.linspace(p.frenet_frame.total_length_m-1.,p.frenet_frame.total_length_m+1.,count),
                     np.linspace(-1.,1.,count)):
        for offset in (1.75,5.25):
            actual=TrafficModel._path_samples(p.frenet_frame,stations,offset)
            expected=frenet_to_local_xz_batch(p.frenet_frame,stations,[offset]*count)
            np.testing.assert_allclose(actual,expected,rtol=1e-12,atol=1e-12)


def flat_world(ego_speed=0.):
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.01,-122.)])
    p.reset_state(station_m=200.,lateral_offset_m=1.75,speed_kmh=ego_speed)
    p.traffic=TrafficModel(total_length_m=p.frenet_frame.total_length_m,seed=42,finite_world=True)
    p.traffic.npcs=[NpcVehicle('rear',1.75,45.,140.,desired_speed_kmh=50.,prevent_recycle=True)]
    return p


def bodies(p,npc):
    frame=p.frenet_frame
    ex,ez=latlng_to_local(p.lat,p.lng,frame.origin_lat,frame.origin_lng)
    width=2*lateral_half_extent(0.,p.steering_angle_rad,p.WHEELBASE_M)
    ego=Footprint(ex,ez,math.radians(p.heading),2*CHASSIS_HALF_LENGTH_M,width)
    x,z,tx,tz=frenet_to_local_xz(frame,npc.station_m,npc.lane_offset)
    length,width,_=dimensions_for(EntityClass(npc.entity_class))
    other=Footprint(x,z,math.atan2(tx,-tz),length,width)
    return ego,other


def test_rear_actor_stops_behind_actual_stationary_ego_through_physics_update():
    p=flat_world();npc=p.traffic.npcs[0]
    for _ in range(1500):
        p.step_world_sensors(.02,perceive=False,predict=False)
        assert not intersects_during(*bodies(p,npc),0.)
    assert npc.speed_kmh<.1
    assert 6.5<p.current_station_m-npc.station_m<8.
    assert p.current_station_m==pytest.approx(200.)


def test_same_rear_closure_fixture_detects_original_ego_omission(monkeypatch):
    p=flat_world();npc=p.traffic.npcs[0]
    # Exactly the old single-NPC world input: ego station for recycling, no
    # body/velocity for NPC following. Keep the new physical oracle unchanged.
    update=p.traffic.update
    monkeypatch.setattr(p.traffic,'update',lambda dt,station,**_:update(dt,station))
    overlap=False
    for _ in range(500):
        p.step_world_sensors(.02,perceive=False,predict=False)
        overlap |= intersects_during(*bodies(p,npc),0.)
    assert overlap


@pytest.mark.parametrize('braking',[False,True])
def test_rear_actor_follows_actual_moving_or_braking_ego(braking):
    p=flat_world(20.);npc=p.traffic.npcs[0]
    for tick in range(2000):
        p.step_world_sensors(.02,perceive=False,predict=False)
        p.step_dynamics(0.,-2. if braking and tick>=250 and p.speed_kmh>0. else 0.,.02)
        assert not intersects_during(*bodies(p,npc),0.)
        assert npc.station_m<p.current_station_m
    if braking:
        assert p.speed_kmh<.1 and npc.speed_kmh<.1
    else:
        assert abs(npc.speed_kmh-p.speed_kmh)<1.


def test_clear_adjacent_lane_actor_does_not_brake_for_ego_lane_id():
    p=flat_world();npc=p.traffic.npcs[0]
    npc.lane_offset=npc.base_lane_offset=5.25
    for _ in range(500):p.step_world_sensors(.02,perceive=False,predict=False)
    assert npc.station_m>p.current_station_m+50.
    assert npc.speed_kmh>45.
    assert p.traffic.sense_lane_clear(p.current_station_m,5.25)


@pytest.mark.parametrize('entity_class',['SEDAN','TRUCK'])
def test_recorded_curved_signal_queue_rear_actor_respects_actual_ego_footprint(entity_class):
    from app.services.parking import parking_plan
    p=PhysicsEngine(seed=42);p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4521.20597152823,lateral_offset_m=1.8261026564901217,
                  speed_kmh=0.,heading_deg=43.68746026907053)
    p.steering_angle_rad=-.01239150231883529
    p.traffic=TrafficModel(total_length_m=p.frenet_frame.total_length_m,seed=42,finite_world=True)
    npc=NpcVehicle('rear',1.75,36.,4460.,desired_speed_kmh=36.,prevent_recycle=True,entity_class=entity_class)
    p.traffic.npcs=[npc]
    for _ in range(2000):
        p.step_world_sensors(.02,perceive=False,predict=False)
        assert not intersects_during(*bodies(p,npc),0.)
    assert npc.speed_kmh<.1
    assert npc.station_m<p.current_station_m-6.


@pytest.mark.parametrize('legacy_center_gap',[False,True])
def test_multi_npc_stopped_queue_uses_physical_bumper_gaps(monkeypatch,legacy_center_gap):
    from app.services.parking import parking_plan
    p=PhysicsEngine(seed=42);p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4400.,lateral_offset_m=5.25,speed_kmh=0.)
    p.traffic=TrafficModel(total_length_m=p.frenet_frame.total_length_m,seed=42,finite_world=True)
    p.traffic.npcs=[
        NpcVehicle('stopped-at-signal',1.75,0.,4523.,desired_speed_kmh=0.,prevent_recycle=True),
        NpcVehicle('middle',1.75,36.,4470.,desired_speed_kmh=36.,prevent_recycle=True),
        NpcVehicle('rear',1.75,36.,4410.,desired_speed_kmh=36.,prevent_recycle=True)]
    if legacy_center_gap:
        # Restore the two old world-following omissions, not the collision
        # oracle. This counterexample must exhibit the original queue overlap.
        monkeypatch.setattr(p.traffic,'_npc_lead_gap',lambda npc,lead:lead.station_m-npc.station_m)
        monkeypatch.setattr(p.traffic,'_body_following_constraint',lambda *_:None)
    overlap=False
    for _ in range(2000):
        p.step_world_sensors(.02,perceive=False,predict=False)
        boxes=[bodies(p,npc)[1] for npc in p.traffic.npcs]
        overlap |= any(intersects_during(a,b,0.) for i,a in enumerate(boxes) for b in boxes[i+1:])
    assert overlap is legacy_center_gap
    if not legacy_center_gap:
        assert max(n.speed_kmh for n in p.traffic.npcs)<.1
