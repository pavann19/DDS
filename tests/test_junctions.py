import dataclasses
import math
import pytest
from app.services.frenet import build_frenet_frame
from app.services.interfaces import SensorObservation, EgoProprioception, SimClock, SafetyDetection
from app.services.junctions import (JunctionBehavior, RouteControl, ControlObservation,
                                   associate_controls, simulated_signal_phase)


def observation(kind='stop', time=0., station=94., speed=0., phase=None, detections=()):
    control = RouteControl(1, kind, 100., 0., -100., '2026-10-11', True)
    return SensorObservation(clock=SimClock(sim_time_s=time),
        ego=EgoProprioception(speed_mps=speed, heading_deg=0., accel_mps2=0.,
            steer_rad=0., station_m=station, lateral_offset_m=0.),
        safety_detections=detections,
        traffic_controls=(ControlObservation(control, phase, time),))


def test_stop_requires_full_stop_dwell_and_fresh_clearance():
    driver = JunctionBehavior()
    assert driver.speed_cap(observation(speed=1.))[0] == 0.
    assert driver.speed_cap(observation(time=1.))[0] == 0.
    assert driver.speed_cap(observation(time=2.9))[0] == 0.
    assert driver.speed_cap(observation(time=3.1)) is None


def test_crossing_actor_vetoes_release_then_clear_dwell_restarts():
    driver = JunctionBehavior()
    crossing = SafetyDetection(-30., -100., 10., 0., 4.8, 1.8)
    driver.speed_cap(observation(time=0.))
    assert driver.speed_cap(observation(time=3., detections=(crossing,)))[0] == 0.
    assert driver.speed_cap(observation(time=4.))[0] == 0.
    assert driver.speed_cap(observation(time=5.1)) is None


def test_red_yellow_unknown_and_stale_signal_never_authorize_crossing():
    for phase in ('RED', 'YELLOW', None):
        driver = JunctionBehavior()
        driver.speed_cap(observation('traffic_signals', phase=phase))
        assert driver.speed_cap(observation('traffic_signals', time=3., phase=phase))[0] == 0.
    driver = JunctionBehavior()
    obs = observation('traffic_signals', phase='GREEN')
    driver.speed_cap(obs)
    assert driver.speed_cap(dataclasses.replace(obs, clock=SimClock(sim_time_s=3.)))[0] == 0.
    assert driver.speed_cap(observation('traffic_signals', time=3.1, phase='GREEN')) is None


def test_distant_green_is_rechecked_before_crossing():
    driver = JunctionBehavior()
    driver.speed_cap(observation('traffic_signals', station=10., phase='GREEN'))
    assert driver.speed_cap(observation('traffic_signals', time=2., station=20., phase='GREEN'))[0]>0.
    assert 1 not in driver.released
    assert driver.speed_cap(observation('traffic_signals', time=3., station=94., phase='RED'))[0] == 0.


@pytest.mark.parametrize('source', ['synthetic_RGB_camera_classical_CV', 'scene_RGB_camera_classical_CV'])
def test_camera_green_commits_only_after_crossing_then_survives_lost_view(source):
    def camera_obs(time, station, phase='GREEN', observed=True):
        obs = observation('traffic_signals', time=time, station=station, phase=phase)
        reading = dataclasses.replace(obs.traffic_controls[0], source=source,
                                      identity_observed=observed)
        return dataclasses.replace(obs, traffic_controls=(reading,))
    driver = JunctionBehavior()
    driver.speed_cap(camera_obs(0., 93.))
    assert driver.speed_cap(camera_obs(2., 93.))[0]>0.
    assert 1 not in driver.released  # Still a meter before modeled line.
    assert driver.speed_cap(camera_obs(2.1, 93.5, None, False))[0] == 0.
    assert 1 not in driver.released
    assert driver.speed_cap(camera_obs(2.2, 94.01)) is None
    assert 1 in driver.released
    assert driver.speed_cap(camera_obs(2.6, 95., None, False)) is None


@pytest.mark.parametrize('source', ['synthetic_RGB_camera_classical_CV', 'scene_RGB_camera_classical_CV'])
def test_camera_unknown_after_line_never_creates_crossing_permission(source):
    obs = observation('traffic_signals', time=2., station=95., phase=None)
    reading = dataclasses.replace(obs.traffic_controls[0], source=source,
                                  identity_observed=False)
    driver = JunctionBehavior()
    assert driver.speed_cap(dataclasses.replace(obs, traffic_controls=(reading,)))[0] == 0.
    assert 1 not in driver.released


def test_missing_or_stale_surround_sensor_holds_stop():
    for obs in (observation(detections=None), dataclasses.replace(observation(), safety_measurement_age_s=.2)):
        driver = JunctionBehavior()
        driver.speed_cap(obs)
        assert driver.speed_cap(dataclasses.replace(obs, clock=SimClock(sim_time_s=3.)))[0] == 0.


def test_direction_and_way_membership_avoid_adjacent_controls():
    frame = build_frenet_frame([(37.6, -122.38), (37.601, -122.38)])
    way = {'id': 10, 'tags': {'highway': 'residential'}, 'nodes': ['1','2','3'],
           'geometry': [[37.6,-122.38],[37.6005,-122.38],[37.601,-122.38]]}
    node = {'id': 2, 'geometry': [37.6005,-122.38], 'way_ids': [10],
            'tags': {'highway': 'stop', 'stop:direction': 'backward'}, 'timestamp':'2026-10-11'}
    assert associate_controls({'features':[way], 'controls':[node]}, frame) == ()
    node['tags']['stop:direction'] = 'forward'
    controls = associate_controls({'features':[way], 'controls':[node]}, frame)
    assert len(controls) == 1 and controls[0].direction_verified
    node['way_ids'] = []
    assert associate_controls({'features':[way], 'controls':[node]}, frame) == ()


def test_signal_schedule_uses_simulation_time_only():
    assert simulated_signal_phase(0,0) == 'RED'
    assert simulated_signal_phase(0,12) == 'GREEN'
    assert simulated_signal_phase(0,22) == 'YELLOW'
    assert simulated_signal_phase(0,25) == 'RED'


def camera_observation(*,time,station,speed=0.,acceleration=0.,phase='GREEN',observed=True):
    obs=observation('traffic_signals',time=time,station=station,speed=speed,phase=phase)
    reading=dataclasses.replace(obs.traffic_controls[0],source='scene_RGB_camera_classical_CV',identity_observed=observed)
    return dataclasses.replace(obs,ego=dataclasses.replace(obs.ego,accel_mps2=acceleration),traffic_controls=(reading,))


def test_jerk_limited_entry_commit_survives_green_pixels_leaving_at_line():
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=90.,speed=1.,phase='RED'))
    # At recorded failure motion, positive acceleration makes instantaneous
    # braking assumptions invalid. This is inside the bounded modeled entry.
    assert behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921)) is None
    assert 1 in behavior.committed
    assert behavior.committed[1]['entry_boundary_m']<=3.
    assert behavior.speed_cap(camera_observation(time=2.1,station=94.01,speed=1.7,phase=None,observed=False)) is None
    assert 1 in behavior.committed
    assert behavior.speed_cap(camera_observation(time=2.2,station=95.,speed=1.7,phase='RED')) is None
    behavior.speed_cap(camera_observation(time=3.,station=112.1,phase=None,observed=False))
    assert not behavior.committed
    assert 1 in behavior.released # Passage history until a route/reset invalidates it.


@pytest.mark.parametrize('phase',['RED','YELLOW',None])
def test_near_line_unknown_or_red_never_creates_commitment(phase):
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=90.,phase=phase,observed=phase is not None))
    result=behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921,
        phase=phase,observed=phase is not None))
    assert result[0] == 0.
    assert not behavior.committed and not behavior.released


def test_green_loss_before_bounded_commit_zone_requires_stop():
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=84.,speed=3.))
    assert behavior.speed_cap(camera_observation(time=2.,station=85.,speed=3.,acceleration=1.))[0]>0.
    assert not behavior.committed
    assert behavior.speed_cap(camera_observation(time=2.1,station=86.,speed=3.,phase=None,observed=False))[0]>0.
    assert not behavior.committed and not behavior.released


def test_commit_reset_and_clock_rollback_invalidate_old_green_permission():
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=90.,phase='RED'))
    behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921))
    assert behavior.committed
    assert behavior.speed_cap(camera_observation(time=.1,station=93.,phase=None,observed=False))[0] == 0.
    assert not behavior.committed and not behavior.released
    behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921))
    assert behavior.committed
    behavior.reset()
    assert not behavior.committed and not behavior.released


@pytest.mark.parametrize('speed,acceleration,braking',[(0.,0.,4.5),(1.3,.9,4.5),(8.,3.,4.5),(8.,-1.,2.),(3.,-.5,.5)])
def test_stop_envelope_matches_independent_jerk_integrator(speed,acceleration,braking):
    from app.services.junctions import jerk_limited_stopping_distance
    predicted=jerk_limited_stopping_distance(speed,acceleration,braking,2.5)
    velocity=speed;accel=max(-braking,acceleration);distance=0.;dt=1e-4
    for _ in range(500000):
        if velocity<=0. and accel<=0.:break
        next_accel=max(-braking,accel-2.5*dt)
        next_velocity=max(0.,velocity+(accel+next_accel)*.5*dt)
        distance+=(velocity+next_velocity)*.5*dt
        velocity=next_velocity;accel=next_accel
    assert distance == pytest.approx(predicted,abs=.001)


@pytest.mark.parametrize('mode',['red','unknown','green_then_view_loss'])
def test_recorded_signal_motion_stops_or_commits_without_phase_fallback(monkeypatch,mode):
    # Actual recorded road/vehicle/controller pipeline, controlled phase-view
    # injection. It is not a replacement for live rendered pixel validation.
    from app.services.simulation_session import SimulationSession
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.reset_state(station_m=4470.,speed_kmh=30.,lateral_offset_m=1.75)
    p.traffic.npcs=[]
    control=next(c for c in p.route_controls if c.node_id==1447264315)
    stop_station=control.station_m-JunctionBehavior.STOP_OFFSET_M
    original=session.world.observe
    green_station=None
    def observed():
        nonlocal green_station
        obs=original();distance=stop_station-p.current_station_m
        phase=None if mode=='unknown' else 'RED'
        if mode=='green_then_view_loss':
            if distance<=1.8 and green_station is None:green_station=p.current_station_m
            if green_station is not None:
                phase='GREEN' if distance>.13 else None
        reading=ControlObservation(control,phase,p.clock.sim_time_s,
            source='scene_RGB_camera_classical_CV',identity_observed=phase is not None)
        return dataclasses.replace(obs,traffic_controls=(reading,))
    monkeypatch.setattr(session.world,'observe',observed)
    rows=[]
    for i in range(2500):
        session.step_pipeline(2);rows.append(physical_sample(session))
        if (mode=='green_then_view_loss' and p.current_station_m>control.station_m+12.
                and control.node_id not in session.driver.junctions.committed):break
        if mode!='green_then_view_loss' and i>500 and p.speed_kmh==0.:break
    if mode=='green_then_view_loss':
        assert green_station is not None
        assert p.current_station_m>control.station_m+12.
        assert control.node_id in session.driver.junctions.released
        assert control.node_id not in session.driver.junctions.committed
    else:
        assert p.speed_kmh == 0.
        assert p.current_station_m<stop_station
        assert control.node_id not in session.driver.junctions.released
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']


@pytest.mark.parametrize('phase',['RED',None])
def test_preline_green_commit_is_revoked_after_stopped_sensor_hold(phase):
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=90.,phase='RED'))
    behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921))
    assert 1 in behavior.committed
    result=behavior.speed_cap(camera_observation(time=60.,station=93.5,speed=0.,phase=phase,observed=phase is not None))
    assert result[0] == 0.
    assert not behavior.committed and not behavior.released


def test_postline_standstill_does_not_revoke_permitted_passage():
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=90.,phase='RED'))
    behavior.speed_cap(camera_observation(time=2.,station=93.,speed=1.289,acceleration=.921))
    assert behavior.speed_cap(camera_observation(time=2.1,station=94.01,speed=.5,phase=None,observed=False)) is None
    assert behavior.speed_cap(camera_observation(time=60.,station=95.,speed=0.,phase=None,observed=False)) is None
    assert behavior.committed[1]['crossed_line']


def test_high_acceleration_second_signal_commits_at_physical_entry_not_arbitrary_three_meters():
    behavior=JunctionBehavior()
    behavior.speed_cap(camera_observation(time=0.,station=87.,speed=2.,phase='RED'))
    # Actual second-signal failure: fresh GREEN at~5.2m with positive1.8m/s²
    # could not physically stop before the line if pixels disappeared next.
    assert behavior.speed_cap(camera_observation(time=2.,station=88.8,speed=2.384,acceleration=1.8)) is None
    assert 1 in behavior.committed
    boundary=behavior.committed[1]['entry_boundary_m']
    assert 5.2<=boundary<=behavior.MAX_ENTRY_COMMIT_DISTANCE_M
    assert behavior.speed_cap(camera_observation(time=2.1,station=90.7,speed=3.6,phase=None,observed=False)) is None


@pytest.mark.parametrize('braking',[4.5,1.35,.45])
def test_green_outside_entry_zone_has_dynamic_stopability_cap(braking):
    from app.services.junctions import approach_stop_distance
    behavior=JunctionBehavior()
    obs=camera_observation(time=0.,station=75.,speed=5.,acceleration=1.5)
    behavior.speed_cap(obs,braking_mps2=braking)
    obs=camera_observation(time=2.,station=78.,speed=5.,acceleration=1.5)
    cap,reason=behavior.speed_cap(obs,braking_mps2=braking)
    assert reason=='signal_green_approach'
    assert not behavior.committed
    assert approach_stop_distance(cap/3.6,1.5,braking,2.5,.1)<=16.-behavior.GREEN_APPROACH_MARGIN_M+1e-7


@pytest.mark.parametrize('friction,speed,acceleration',[(1.,40.,1.5),(.3,30.,.6)])
def test_fast_green_loss_outside_entry_zone_stops_actual_plant_before_line(monkeypatch,friction,speed,acceleration):
    # Isolated straight-road fixture; actual multirate controller/plant. It
    # tests stopability under lost GREEN, not physical infrastructure/perception.
    from app.services.simulation_session import SimulationSession
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.set_route([(37.,-122.),(37.01,-122.)])
    p.parking_mission=None
    p.reset_state(station_m=194.,speed_kmh=speed,lateral_offset_m=1.75)
    p.friction_mu=friction;p.acceleration_mps2=acceleration;p.traffic.npcs=[]
    control=RouteControl(91,'traffic_signals',300.,0.,-300.,'explicit_test_fixture',False)
    stop_station=294.
    original=session.world.observe
    loss_station=None
    def observed():
        nonlocal loss_station
        obs=original();distance=stop_station-p.current_station_m
        if distance<=10. and loss_station is None:loss_station=p.current_station_m
        phase='GREEN' if loss_station is None else None
        reading=ControlObservation(control,phase,p.clock.sim_time_s,
            source='scene_RGB_camera_classical_CV',identity_observed=phase is not None)
        return dataclasses.replace(obs,traffic_controls=(reading,))
    monkeypatch.setattr(session.world,'observe',observed)
    rows=[]
    for i in range(2500):
        session.step_pipeline(2);rows.append(physical_sample(session))
        assert not session.driver.junctions.committed
        if loss_station is not None and p.speed_kmh==0.:break
    assert loss_station is not None
    assert stop_station-loss_station>JunctionBehavior.MAX_ENTRY_COMMIT_DISTANCE_M
    assert p.speed_kmh==0.
    assert p.current_station_m<stop_station
    assert not session.driver.junctions.released
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']
