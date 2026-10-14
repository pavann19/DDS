from types import SimpleNamespace
from app.services.physics_engine import PhysicsEngine
from app.services.interfaces import SimClock
from app.services.junctions import RouteControl
from app.services.control_camera import CameraFrame,CameraCalibration,CameraDetection
from app.services.control_camera_bridge import ControlCameraPipeline
from app.services.scene_camera_ingress import CaptureIdentity,PendingSceneFrame


def fixture(monkeypatch):
    p=PhysicsEngine(seed=42)
    p.camera_controls=True;p.scene_camera_controls=True
    p.control_camera=ControlCameraPipeline(source='scene_RGB_camera_classical_CV')
    p.route_controls=(RouteControl(1,'traffic_signals',30.,0.,0.,'dated',True),)
    monkeypatch.setattr(p.control_camera,'projected_control',lambda p,c:(12.,0.))
    return p


def pending(t,phase='GREEN',seq=1):
    frame=CameraFrame(bytes(640*360*3),t,CameraCalibration(640,360,140.))
    hit=CameraDetection('signal',phase.lower(),(300,130,330,180),0.,12.,2.,.85,t)
    return PendingSceneFrame(frame,(hit,),CaptureIdentity('session',0,int(t*10+1e-8),t,seq,'browser'))


def test_scene_perception_never_invokes_cpu_truth_renderer(monkeypatch):
    p=fixture(monkeypatch)
    def forbidden(*args,**kwargs):raise AssertionError('CPU camera fallback')
    monkeypatch.setattr(p.control_camera,'acquire',forbidden)
    monkeypatch.setattr('app.services.control_camera_bridge.render_control_camera',forbidden)
    p.step_perception(.05)
    observations=p.observe_traffic_controls()
    assert observations and all(not o.identity_observed and o.phase is None for o in observations)


def test_side_camera_control_evidence_uses_calibrated_bearing_not_world_phase(monkeypatch):
    from dataclasses import replace
    import math
    p=fixture(monkeypatch)
    for i,t in enumerate((1.,1.1,1.2)):
        sample=pending(t,seq=i+1)
        side_frame=CameraFrame(bytes(320*180*3),t,CameraCalibration(320,180,100.))
        side_hit=replace(sample.detections[0],bearing_deg=-45.,range_m=12.*math.cos(math.pi/4),range_uncertainty_m=1.)
        p.clock=SimClock(sim_time_s=t)
        p.scene_camera_pending=replace(sample,detections=(),auxiliary_frames=(('front_right',side_frame),),auxiliary_detections=(('front_right',(side_hit,)),))
        p.control_camera.acquire_scene(p)
    reading=p.observe_traffic_controls()[0]
    assert reading.identity_observed and reading.phase=='GREEN'
    assert p.control_camera.confirmations[(1,'GREEN')]==3


def test_conflicting_camera_phases_never_release_green(monkeypatch):
    from dataclasses import replace
    import math
    p=fixture(monkeypatch)
    for i,t in enumerate((1.,1.1,1.2)):
        sample=pending(t,phase='RED',seq=i+1)
        side_frame=CameraFrame(bytes(320*180*3),t,CameraCalibration(320,180,100.))
        side_hit=replace(sample.detections[0],phase='green',bearing_deg=-45.,range_m=12.*math.cos(math.pi/4),range_uncertainty_m=1.)
        p.clock=SimClock(sim_time_s=t)
        p.scene_camera_pending=replace(sample,auxiliary_frames=(('front_right',side_frame),),auxiliary_detections=(('front_right',(side_hit,)),))
        p.control_camera.acquire_scene(p)
    assert not p.observe_traffic_controls()[0].identity_observed


def test_repeated_same_frame_never_accumulates_green_confirmations(monkeypatch):
    p=fixture(monkeypatch)
    for _ in range(5):
        p.clock=SimClock(sim_time_s=1.)
        p.scene_camera_pending=pending(1.)
        p.control_camera.acquire_scene(p)
    assert not p.observe_traffic_controls()[0].identity_observed
    assert p.control_camera.confirmations[(1,'GREEN')]==1


def test_distinct_frames_confirm_pixels_then_expire_without_schedule_fallback(monkeypatch):
    p=fixture(monkeypatch)
    monkeypatch.setattr('app.services.junctions.simulated_signal_phase',lambda *args: (_ for _ in ()).throw(AssertionError('truth schedule')))
    for i,t in enumerate((1.,1.1,1.2)):
        p.clock=SimClock(sim_time_s=t);p.scene_camera_pending=pending(t,seq=i+1)
        p.control_camera.acquire_scene(p)
    assert p.observe_traffic_controls()[0].phase=='GREEN'
    p.clock=SimClock(sim_time_s=1.56)
    assert not p.observe_traffic_controls()[0].identity_observed


def test_long_capture_gap_restarts_confirmation(monkeypatch):
    p=fixture(monkeypatch)
    for i,t in enumerate((1.,1.1,1.7)):
        p.clock=SimClock(sim_time_s=t);p.scene_camera_pending=pending(t,seq=i+1)
        p.control_camera.acquire_scene(p)
    assert p.control_camera.confirmations[(1,'GREEN')]==1
    assert not p.observe_traffic_controls()[0].identity_observed


def test_stale_boundary_capture_is_never_confirmed_as_observed(monkeypatch):
    p=fixture(monkeypatch)
    p.clock=SimClock(sim_time_s=2.)
    p.scene_camera_pending=pending(1.)
    p.control_camera.acquire_scene(p)
    assert not p.observe_traffic_controls()[0].identity_observed


def test_dropout_does_not_refresh_scene_frame_or_confirmations(monkeypatch):
    p=fixture(monkeypatch)
    p.clock=SimClock(sim_time_s=1.)
    p.scene_camera_pending=pending(1.)
    p.control_camera.acquire_scene(p)
    old=p.control_camera.frame
    p.camera_dropout=True
    p.clock=SimClock(sim_time_s=1.1);p.scene_camera_pending=pending(1.1,seq=2)
    p.control_camera.acquire_scene(p)
    assert p.control_camera.frame is old
    assert p.control_camera.confirmations[(1,'GREEN')]==1
    assert not p.observe_traffic_controls()[0].identity_observed


def test_changing_detected_phase_requires_three_new_distinct_captures(monkeypatch):
    p=fixture(monkeypatch)
    for i,(t,phase) in enumerate(((1.,'RED'),(1.1,'RED'),(1.2,'RED'),(1.3,'GREEN'))):
        p.clock=SimClock(sim_time_s=t);p.scene_camera_pending=pending(t,phase,seq=i+1)
        p.control_camera.acquire_scene(p)
    observation=p.observe_traffic_controls()[0]
    assert not observation.identity_observed and observation.phase is None


def test_baseline_mode_retains_explicit_predefined_signal_source(monkeypatch):
    p=fixture(monkeypatch);p.camera_controls=False;p.scene_camera_controls=False
    monkeypatch.setattr('app.services.junctions.simulated_signal_phase',lambda *args:'RED')
    observation=p.observe_traffic_controls()[0]
    assert observation.phase=='RED' and observation.identity_observed
    assert observation.source=='simulated_signal_sensor'

def test_pending_frame_that_aged_before_boundary_cannot_confirm(monkeypatch):
    p=fixture(monkeypatch)
    p.clock=SimClock(sim_time_s=2.)
    p.scene_camera_pending=pending(1.)
    p.control_camera.acquire_scene(p)
    assert not p.control_camera.confirmations
    assert not p.control_camera.readings
    assert p.control_camera.fault=='stale_scene_frame'


def test_captured_curved_junction_pixels_associate_with_rendered_roadside():
    # Actual browser RGB captured2026-10-14 at the formerly blocked junction.
    # A single frozen fixture checks optical geometry, NOT temporal validation.
    from pathlib import Path
    from PIL import Image
    from app.services.control_camera import detect_controls
    from app.services.camera_rig import CAMERA_CALIBRATIONS
    from app.services.parking import parking_plan
    p=PhysicsEngine(seed=42)
    p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4522.377608308,lateral_offset_m=1.7970345256858888,
                  heading_deg=42.797454690488735)
    p.control_camera=ControlCameraPipeline(source='scene_RGB_camera_classical_CV')
    views={}
    for name in ('front','front_right'):
        path=Path(__file__).parent/'fixtures'/'scene_camera'/f'curved_junction_amber_{name}.png'
        views[name]=CameraFrame(Image.open(path).convert('RGB').tobytes(),1.,CAMERA_CALIBRATIONS[name])
    front=views.pop('front')
    p.clock=SimClock(sim_time_s=1.)
    p.scene_camera_pending=PendingSceneFrame(front,detect_controls(front),
        CaptureIdentity('geometry-test',0,10,1.,1,'fixture'),tuple(views.items()),
        tuple((name,detect_controls(frame)) for name,frame in views.items()))
    p.control_camera.acquire_scene(p)
    assert p.control_camera.confirmations[(1447264315,'YELLOW')]==1
    assert not p.control_camera.readings # Two cameras still count as one time sample.
    control=next(c for c in p.route_controls if c.node_id==1447264315)
    forward,right=p.control_camera.projected_control(p,control)
    import math
    assert abs(math.degrees(math.atan2(right,forward))-30.8928)<.01
