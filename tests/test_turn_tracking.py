import math
import pytest
from app.services.driver.turn_tracking import center_steer_for_curvature, center_tracking_steer, offset_path_curvature
from app.services.vehicle_kinematics import sideslip

@pytest.mark.parametrize('radius',[8.,12.,25.,60.])
@pytest.mark.parametrize('direction',[-1.,1.])
def test_center_bicycle_inverse_roundtrip(radius,direction):
    k=direction/radius
    delta=center_steer_for_curvature(k,2.8)
    assert math.cos(sideslip(delta))*math.tan(delta)/2.8 == pytest.approx(k,abs=1e-12)

@pytest.mark.parametrize('radius',[12.,25.,60.])
@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('speed',[2.,4.])
def test_center_tracker_converges_on_real_integrated_circle(radius,direction,speed):
    # Synthetic isolated kinematic fixture, no path-position snapping.
    x,z=0.,-radius-0.4
    heading=direction*math.pi/2
    steer=0.
    errors=[]
    for i in range(1500):
        r=math.hypot(x,z)
        tangent=math.atan2(direction*(-z),-direction*x)
        d=direction*(r-radius)
        command=center_tracking_steer(curvature_1pm=direction/radius,wheelbase_m=2.8,
            heading_error_rad=tangent-heading,cross_track_m=d,speed_mps=speed,
            steer_limit_rad=.61,actual_steer_rad=steer)
        steer+=max(-.012,min(.012,command-steer))
        yaw=speed*math.cos(sideslip(steer))*math.tan(steer)/2.8
        x+=speed*.02*math.sin(heading+yaw*.01+sideslip(steer))
        z-=speed*.02*math.cos(heading+yaw*.01+sideslip(steer))
        heading+=yaw*.02
        if i>1200:errors.append(abs(math.hypot(x,z)-radius))
    assert max(errors)<.1


def test_parallel_offset_curvature_matches_left_positive_convention():
    assert offset_path_curvature(.1,2.)==pytest.approx(1/12)
    assert offset_path_curvature(-.1,2.)==pytest.approx(-1/8)
    with pytest.raises(ValueError):offset_path_curvature(-.1,10.)


def test_impossible_center_curvature_rejected():
    with pytest.raises(ValueError):center_steer_for_curvature(1.,2.8)
@pytest.mark.parametrize('direction',[-1.,1.])
def test_actual_driver_control_tracks_mapped_frame_circle(direction):
    from app.services.physics_engine import PhysicsEngine
    from app.services.frenet import local_to_latlng
    from app.services.interfaces import SensorObservation,EgoProprioception
    p=PhysicsEngine(seed=42)
    points=[local_to_latlng(25*math.sin(direction*a),-25*math.cos(a),37.,-122.)
            for a in [i*.02 for i in range(260)]]
    p.set_destination(*points[-1]);p.set_route(points)
    p.reset_state(station_m=10.,speed_kmh=7.2,lateral_offset_m=.3)
    p._driver.lateral_target_d_m=0.
    errors=[]
    for i in range(1000):
        p.step_world_sensors(.02,perceive=False,predict=False)
        obs=SensorObservation(clock=p.clock,ego=EgoProprioception(
            speed_mps=p.speed_kmh/3.6,heading_deg=p.heading,accel_mps2=0.,
            steer_rad=p.steering_angle_rad,station_m=p.current_station_m,
            lateral_offset_m=p.current_lateral_offset_m),frenet_frame=p.frenet_frame,
            safety_detections=())
        command=p._driver.control(obs,p.build_driver_context(.02))
        p.step_dynamics(steer_rad=command.steer_rad,accel_mps2=0.,dt=.02)
        if i>500:errors.append(abs(p.current_lateral_offset_m))
    assert max(errors)<.5


def test_curvature_grip_cap_can_fall_below_fifteen_kmh():
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorObservation,EgoProprioception
    p=PhysicsEngine(seed=42)
    p.step_world_sensors(0.,perceive=False,predict=False)
    ctx=replace(p.build_driver_context(.02),max_curvature_1pm=.125,friction_mu=.1,
                dist_to_dest_m=1000.,waypoint_dist_m=100.,has_arrived=False,speed_kmh=0.)
    obs=SensorObservation(clock=p.clock,ego=EgoProprioception(0.,0.,0.,0.,0.,0.),safety_detections=())
    p._driver.decide_longitudinal(obs,ctx)
    expected=math.sqrt(p.A_LAT_MAX_MPS2*.1/.125)
    assert p._driver.target_speed_mps == pytest.approx(expected)
    assert expected*3.6<15

def test_invalid_turn_reference_returns_braking_command(monkeypatch):
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorObservation,EgoProprioception,SOURCE_SAFETY_OVERRIDE
    from app.services.driver import driver as module
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.001,-122.)])
    p.step_world_sensors(0.,perceive=False,predict=False)
    obs=SensorObservation(clock=p.clock,ego=EgoProprioception(0.,0.,0.,0.,0.,0.),frenet_frame=p.frenet_frame)
    def invalid(*args,**kwargs):raise ValueError('folded reference')
    monkeypatch.setattr(module,'offset_path_curvature',invalid)
    command=p._driver.control(obs,p.build_driver_context(.02))
    assert command.accel_mps2 == -p.A_MAX_BRAKE_MPS2
    assert command.source == SOURCE_SAFETY_OVERRIDE
    assert command.speed_limit_reason == 'turn_reference_infeasible'
    assert p._driver.target_speed_mps == 0.

@pytest.mark.parametrize('field',['heading_error_rad','cross_track_m','speed_mps','steer_limit_rad','actual_steer_rad'])
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_tracking_inputs_fail_closed(field,bad):
    inputs=dict(curvature_1pm=.01,wheelbase_m=2.8,heading_error_rad=0.,
                cross_track_m=0.,speed_mps=2.,steer_limit_rad=.61,actual_steer_rad=0.)
    inputs[field]=bad
    with pytest.raises(ValueError):center_tracking_steer(**inputs)


def test_distant_scenario_reset_seeds_projection_at_actual_station():
    from app.services.physics_engine import PhysicsEngine
    from app.services.parking import parking_plan
    p = PhysicsEngine(seed=42)
    p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4250., speed_kmh=0., lateral_offset_m=1.75)
    p.step_world_sensors(.02, perceive=False, predict=False)
    assert p.current_station_m == pytest.approx(4250., abs=.01)
    assert p.current_lateral_offset_m == pytest.approx(1.75, abs=.01)


def test_camera_outage_does_not_adopt_lane_tracking_error_as_target():
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorHealth, SensorObservation, EgoProprioception
    p = PhysicsEngine(seed=42)
    p.set_route([(37.,-122.), (37.001,-122.)])
    p.reset_state(station_m=20., lateral_offset_m=2.7)
    p._driver.lateral_target_d_m = 1.75
    obs = SensorObservation(clock=p.clock, ego=EgoProprioception(0.,0.,0.,0.,20.,2.7),
        frenet_frame=p.frenet_frame, sensor_health=SensorHealth(0.,0.,True,None))
    p.step_world_sensors(.02,perceive=False,predict=False)
    ctx = p.build_driver_context(.02)
    p._driver.decide_lateral(obs,ctx)
    p._driver.control(obs,ctx)
    assert p._driver.lateral_target_d_m == 1.75
    p._driver.control(obs,replace(ctx,lateral_offset_m=3.5))
    assert p._driver.lateral_target_d_m == 1.75


def test_camera_outage_latches_active_lane_change_hold_once():
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorHealth, SensorObservation, EgoProprioception
    p = PhysicsEngine(seed=42)
    p.set_route([(37.,-122.), (37.001,-122.)])
    p.reset_state(station_m=20., lateral_offset_m=.4)
    from app.services.planner.state_machine import LaneChangeState
    p._driver._lc_sm.state = LaneChangeState.EXECUTE_LANE_CHANGE
    p._driver._changing_lane_now = True
    obs = SensorObservation(clock=p.clock,ego=EgoProprioception(0.,0.,0.,0.,20.,.4),
        frenet_frame=p.frenet_frame,sensor_health=SensorHealth(0.,0.,True,None))
    p.step_world_sensors(.02,perceive=False,predict=False)
    ctx = p.build_driver_context(.02)
    p._driver.decide_lateral(obs,ctx)
    assert p._driver.lateral_target_d_m == pytest.approx(.4)
    p._driver.control(obs,replace(ctx,lateral_offset_m=.9))
    assert p._driver.lateral_target_d_m == pytest.approx(.4)


def test_recorded_fault_corner_keeps_lane_reference_during_camera_recovery():
    # Actual recorded parking mission near the live station4367 failure;
    # RGB availability is fault injected, vehicle pose is physically integrated.
    # Fixed low speed isolates lateral tracking from junction/traffic decisions.
    from app.services.physics_engine import PhysicsEngine
    from app.services.parking import parking_plan
    from app.services.interfaces import SensorHealth, SensorObservation, EgoProprioception
    p = PhysicsEngine(seed=42)
    p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4330.,speed_kmh=14.4,lateral_offset_m=1.75)
    p._driver.lateral_target_d_m=1.75
    errors=[]
    for i in range(1000):
        p.clock=p.clock.advance()
        p.step_world_sensors(.02,perceive=False,predict=False)
        now=p.clock.sim_time_s
        camera_stamp=None if i%80<20 else now
        obs=SensorObservation(clock=p.clock,ego=EgoProprioception(
            p.speed_kmh/3.6,p.heading,0.,p.steering_angle_rad,
            p.current_station_m,p.current_lateral_offset_m),frenet_frame=p.frenet_frame,
            sensor_health=SensorHealth(now,now,True,camera_stamp),safety_detections=())
        command=p._driver.control(obs,p.build_driver_context(.02))
        p.step_dynamics(steer_rad=command.steer_rad,accel_mps2=0.,dt=.02)
        assert p._driver.lateral_target_d_m == 1.75
        errors.append(abs(p.current_lateral_offset_m-1.75))
    assert p.current_station_m>4400.
    assert max(errors)<.8


def test_recorded_fault_corner_actual_multirate_pipeline_keeps_body_contained():
    from app.services.simulation_session import SimulationSession
    from scripts.evaluate_driving_acceptance import physical_sample, summarize
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.reset_state(station_m=4300.,speed_kmh=30.,lateral_offset_m=1.75)
    # Isolate controller geometry from other actors, without altering the road,
    # physics, actuator limits, freshness guard, or containment supervisor.
    p.traffic.npcs=[]
    rows=[]
    for _ in range(1200):
        session.step_pipeline(2)
        rows.append(physical_sample(session))
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'], report['first_failures']
    assert p.current_station_m>4430.
    assert report['max_road_center_tracking_error_m']<.6
    assert report['minimum_modeled_road_margin_m']>2.


def test_camera_outage_preserves_parking_centerline_reference():
    from app.services.physics_engine import PhysicsEngine
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.001,-122.)])
    p.reset_state(station_m=20.,lateral_offset_m=.3)
    p.step_world_sensors(.02,perceive=False,predict=False)
    p._driver.lateral_target_d_m=0.
    p._driver._hold_lateral_during_sensor_loss(p.build_driver_context(.02))
    assert p._driver.lateral_target_d_m == 0.


def test_recorded_inner_overtaking_lane_requests_checked_return_before_turn():
    from app.services.simulation_session import SimulationSession
    from app.services.planner.state_machine import LaneChangeState
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.reset_state(station_m=4230.,speed_kmh=36.,lateral_offset_m=5.25)
    p.traffic.npcs=[]
    driver=session.driver
    driver._lc_sm.origin_lane_d_m=5.25
    driver._lc_sm.adjacent_lane_d_m=1.75
    driver.lateral_target_d_m=5.25
    saw_return=False
    for _ in range(1200):
        session.step_pipeline(2)
        if driver.spatiotemporal_traj is not None:
            saw_return=True
            assert driver.spatiotemporal_traj.sampled_aggregate_grip_verified
            assert driver.spatiotemporal_traj.continuous_modeled_corridor_verified
        assert driver.shield_verdict.body_contained
    assert saw_return
    assert driver._lc_sm.origin_lane_d_m == 1.75
    assert driver._lc_sm.state is LaneChangeState.LANE_KEEP
    assert p.current_station_m>4360.
    assert abs(p.current_lateral_offset_m-1.75)<.8


def test_occupied_adjacent_lane_prevents_geometry_requested_return():
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.planner.state_machine import LaneChangeState
    p=PhysicsEngine(seed=42)
    from app.services.parking import parking_plan
    p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4250.,speed_kmh=36.,lateral_offset_m=5.25)
    p.step_world_sensors(.02,perceive=False,predict=False)
    driver=p._driver
    driver._lc_sm.origin_lane_d_m=5.25
    driver._lc_sm.adjacent_lane_d_m=1.75
    driver.lateral_target_d_m=5.25
    ctx=replace(p.build_driver_context(.1),lane_clear_fn=lambda *_:False)
    assert not driver._modeled_lane_turn_feasible(p.frenet_frame,4250.,5.25,120.)
    assert driver._modeled_lane_turn_feasible(p.frenet_frame,4250.,1.75,120.)
    for _ in range(10):driver.decide_lateral(p.observe(),ctx)
    assert driver._lc_sm.state is LaneChangeState.LANE_KEEP
    assert not driver._changing_lane_now
    assert driver.lateral_target_d_m == 5.25
