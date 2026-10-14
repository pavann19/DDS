from dataclasses import replace
import math
import pytest
from app.services.interfaces import SensorHealth
from app.services.driver.sensor_freshness import SensorFreshnessGuard
from app.services.physics_engine import PhysicsEngine
from app.services.simulation_session import SimulationSession


def health(t, camera=None):
    return SensorHealth(t, t, camera_required=True, camera_capture_time_s=camera)


def ready():
    g=SensorFreshnessGuard()
    for t in (0., .1, .2):
        g.evaluate(t, health(t,t))
    assert g.status.ready
    return g


@pytest.mark.parametrize('stamp,reason',[(None,'sensor_camera_missing'),(math.nan,'sensor_camera_timestamp_invalid'),(math.inf,'sensor_camera_timestamp_invalid'),(True,'sensor_camera_timestamp_invalid'),(-1.,'sensor_camera_timestamp_invalid'),(.4,'sensor_camera_timestamp_invalid'),(0.,'sensor_camera_timestamp_invalid')])
def test_missing_invalid_stale_camera_stops(stamp,reason):
    g=ready()
    result=g.evaluate(.36,health(.36,stamp))
    assert not result.ready and result.reason==reason
    assert result.camera_required and result.camera_recovery_samples==0


def test_replay_does_not_restore_confirmation_and_regression_rejected():
    g=ready()
    assert not g.evaluate(.56,health(.56,.2)).ready
    assert not g.evaluate(.57,health(.57,.19)).ready
    assert g.status.reason=='sensor_camera_timestamp_invalid'
    for t in (.6,.61,.62):
        assert not g.evaluate(t,health(t,.6)).ready
    assert g.status.camera_recovery_samples==1
    assert not g.evaluate(.7,health(.7,.7)).ready
    assert g.evaluate(.8,health(.8,.8)).ready


def test_gap_and_reset_require_three_new_distinct_frames():
    g=ready()
    assert not g.evaluate(.7,health(.7,.7)).ready
    assert g.status.camera_recovery_samples==1
    g.reset()
    assert not g.evaluate(0.,health(0.,0.)).ready
    assert not g.evaluate(.1,health(.1,.1)).ready
    assert g.evaluate(.2,health(.2,.2)).ready


def test_fresh_camera_cannot_override_stale_range():
    g=ready()
    r=g.evaluate(.4,SensorHealth(.2,.2,True,.4))
    assert not r.ready and r.reason=='sensor_stale'
    assert r.camera_age_s==0.


def test_producer_uses_captured_frame_not_observation_time():
    from app.services.control_camera import CameraCalibration, CameraFrame
    from app.services.control_camera_bridge import ControlCameraPipeline
    p=PhysicsEngine(seed=42)
    p.camera_required_for_drive=True
    p.scene_camera_controls=True
    assert p.observe().sensor_health.camera_capture_time_s is None
    p.control_camera=ControlCameraPipeline(source='scene_RGB_camera_classical_CV')
    p.control_camera.frame=CameraFrame(rgb=bytes(640*360*3),capture_time_s=.123,calibration=CameraCalibration())
    assert p.observe().sensor_health.camera_capture_time_s==.123
    p.reset_state()
    assert not p.observe().sensor_health.camera_required
    assert p.observe().sensor_health.camera_capture_time_s is None


def test_camera_missing_blocks_actual_pipeline_without_any_mapped_control():
    s=SimulationSession(seed=42,real_time=False)
    s.scenario_engine.load_scenario('normal_cruising',s.world.physics)
    p=s.world.physics
    p.camera_required_for_drive=True
    p.scene_camera_controls=True
    p.camera_controls=True
    p.route_controls=[]
    for _ in range(5):
        s.step_pipeline(10)
    assert s.driver.target_speed_mps==0.
    assert s.driver.sensor_guard.status.reason=='sensor_camera_missing'
    assert p.acceleration_mps2<=0.


def test_unchanged_camera_capture_ages_despite_fresh_ranges():
    g=ready()
    r=g.evaluate(.56,health(.56,.2))
    assert not r.ready and r.reason=='sensor_camera_stale'
    assert r.camera_age_s==pytest.approx(.36)


def test_legacy_controller_cannot_accelerate_with_missing_camera():
    from app.services.road_routes import real_road_route
    p=PhysicsEngine(seed=42)
    p.set_route(real_road_route('highway'))
    p.step_world_sensors(.02)
    p.camera_required_for_drive=True
    p.scene_camera_controls=True
    p.camera_controls=True
    p.speed_kmh=20.
    ctx=replace(p.build_driver_context(.02),controller='legacy',dist_to_dest_m=1000.,has_arrived=False,corner_turn_deg=0.)
    decision=p._driver.decide_longitudinal(p.observe(),ctx)
    assert decision.speed_kmh_override < 20.
    assert decision.accel_mps2 < 0.
    assert p._driver.target_speed_mps==0.
