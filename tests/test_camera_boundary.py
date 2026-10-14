"""Control camera boundary regressions; synthetic RGB, not field accuracy claims."""
from dataclasses import replace
import math
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services.frenet import frenet_to_latlng
from app.services.interfaces import SensorObservation,EgoProprioception,SimClock
from app.services.junctions import RouteControl,JunctionBehavior
from app.services import control_camera_bridge as bridge


def world(kind="stop",station=94.,p=None):
    p=p if p is not None else PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.003,-122.)])
    p.route_controls=(RouteControl(1,kind,100.,0.,-100.,"2026-10-13",True),)
    p.current_station_m=station
    p.current_lateral_offset_m=0.
    p.lat,p.lng=frenet_to_latlng(p.frenet_frame,station,0.)
    p.heading=0.
    p.camera_controls=True
    p.control_camera=bridge.ControlCameraPipeline()
    return p


def capture(p,time):
    p.clock=replace(p.clock,sim_time_s=time)
    p.control_camera.acquire(p)
    return p.observe_traffic_controls()[0]


def acquire_confirmed(p,start=0.):
    for index in range(3):reading=capture(p,start+index*.05)
    assert reading.identity_observed, p.control_camera.state(p.clock.sim_time_s)
    return reading


def observation(p,time,reading):
    return SensorObservation(clock=SimClock(sim_time_s=time),
        ego=EgoProprioception(0.,0.,0.,0.,94.,0.),safety_detections=(),
        safety_measurement_age_s=0.,traffic_controls=(reading,))


def test_baseline_remains_explicitly_labeled_and_schedule_driven():
    p=world("traffic_signals")
    p.camera_controls=False
    reading=p.observe_traffic_controls()[0]
    assert reading.source=="simulated_signal_sensor"
    assert reading.identity_observed
    assert reading.phase==bridge.simulated_signal_phase(1,p.clock.sim_time_s)
    status=p.get_navigation_state()["control_perception"]
    assert status["mode"]=="map_schedule_baseline"
    assert status["source"]=="predefined_map_and_signal_schedule"


def test_camera_observation_never_reads_hidden_phase_without_new_pixels(monkeypatch):
    p=world("traffic_signals")
    monkeypatch.setattr(bridge,"simulated_signal_phase",lambda *args:"GREEN")
    before=acquire_confirmed(p)
    assert before.phase=="GREEN"
    def forbidden(*args):raise AssertionError("observation read hidden schedule")
    monkeypatch.setattr(bridge,"simulated_signal_phase",forbidden)
    assert p.observe_traffic_controls()[0]==before
    assert p.control_camera.fault is None


@pytest.mark.parametrize("fault",["initial_missing","dropout","occluded"])
def test_expected_map_control_without_camera_evidence_is_unknown(fault):
    p=world()
    if fault!="initial_missing":
        acquire_confirmed(p)
        setattr(p,"camera_dropout" if fault=="dropout" else "camera_occluded",True)
        capture(p,.15)
    reading=p.observe_traffic_controls()[0]
    assert reading.control.node_id==1
    assert not reading.identity_observed and reading.phase is None
    behavior=JunctionBehavior()
    behavior.speed_cap(observation(p,0.,reading))
    assert behavior.speed_cap(observation(p,3.,replace(reading,observed_time_s=3.)))[0]==0.
    assert 1 not in behavior.released


@pytest.mark.parametrize("age",[.151,-.001,math.inf])
def test_stale_or_future_capture_cannot_release_stop(age):
    p=world()
    reading=acquire_confirmed(p)
    behavior=JunctionBehavior()
    behavior.speed_cap(observation(p,0.,replace(reading,observed_time_s=0.)))
    bad=replace(reading,observed_time_s=3.-age)
    assert behavior.speed_cap(observation(p,3.,bad))[0]==0.
    assert 1 not in behavior.released
    if math.isfinite(age):
        p.clock=replace(p.clock,sim_time_s=reading.observed_time_s+age)
        assert not p.observe_traffic_controls()[0].identity_observed


def test_fresh_stop_pixels_are_required_after_dwell_before_release():
    p=world()
    behavior=JunctionBehavior()
    missing=p.observe_traffic_controls()[0]
    behavior.speed_cap(observation(p,0.,missing))
    assert behavior.speed_cap(observation(p,3.,replace(missing,observed_time_s=3.)))[0]==0.
    reading=acquire_confirmed(p,3.)
    assert reading.source=="synthetic_RGB_camera_classical_CV"
    assert behavior.speed_cap(observation(p,3.1,reading)) is None
    assert 1 in behavior.released


def test_reset_clears_camera_mode_faults_and_prior_frame():
    p=world()
    acquire_confirmed(p)
    old=p.control_camera
    assert old.frame is not None and old.readings
    p.camera_dropout=True
    p.camera_occluded=True
    p.reset_state()
    assert p.control_camera is None
    assert not p.camera_controls and not p.camera_dropout and not p.camera_occluded
    p.camera_controls=True
    reading=p.observe_traffic_controls()[0]
    assert p.control_camera is not old
    assert p.control_camera.frame is None and not p.control_camera.readings
    assert not reading.identity_observed


def test_actual_session_occluded_camera_cannot_blindly_release_mapped_stop():
    from app.services.simulation_session import SimulationSession
    session=SimulationSession(seed=42,real_time=False)
    p=world(p=session.world.physics)
    p.speed_kmh=0.
    p.camera_occluded=True
    for _ in range(160):
        session.step_pipeline(2)
        readings=p.observe_traffic_controls()
        assert len(readings)==1 and not readings[0].identity_observed
        assert 1 not in session.driver.junctions.released
    assert p.clock.sim_time_s>=3.
    assert p.speed_kmh==0.
    assert session.driver.junctions.status["reason"]=="stop_sign"


def test_failed_acquisition_clears_previous_evidence_and_reports_fault(monkeypatch):
    p=world()
    acquire_confirmed(p)
    def failed_renderer(*args,**kwargs):raise ValueError("invalid frame")
    monkeypatch.setattr(bridge,"render_control_camera",failed_renderer)
    reading=capture(p,.15)
    assert p.control_camera.fault=="ValueError"
    assert p.control_camera.frame is None and not p.control_camera.readings
    assert not reading.identity_observed

def test_camera_junction_near_stop_brake_hold_removes_residual_creep():
    p=world()
    p.speed_kmh=.08
    behavior=JunctionBehavior()
    behavior.status={'distance_to_stop_m':.8,'phase_source':'synthetic_RGB_camera_classical_CV'}
    for _ in range(100):
        accel=behavior.stopping_accel(p.speed_kmh/3.6)
        p.step_dynamics(0.,accel,.02)
    assert p.speed_kmh==0.
