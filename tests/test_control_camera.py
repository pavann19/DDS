from dataclasses import FrozenInstanceError
import numpy as np
import pytest
from app.services.control_camera import (CameraCalibration,CameraFrame,ControlPrimitive,
    render_control_camera,detect_controls)


def test_components_preserve_four_neighbour_connectivity_and_evidence_floor():
    from app.services.control_camera import _components
    mask=np.zeros((5,7),dtype=bool)
    mask[:2,:4]=True
    mask[2,4]=True  # Diagonal contact alone must not join the component.
    components=list(_components(mask))
    assert len(components)==1 and len(components[0])==8
    assert {tuple(point) for point in components[0]}=={(y,x) for y in range(2) for x in range(4)}
    mask[2,3]=True
    assert len(list(_components(mask))[0])==10


@pytest.mark.parametrize('kind,phase',[('stop','unknown'),('yield','unknown'),
    ('signal','red'),('signal','amber'),('signal','green'),('signal','unknown')])
def test_projected_geometry_is_detected_from_rgb(kind,phase):
    frame=render_control_camera([ControlPrimitive(kind,12,2,phase=phase)],3.5)
    detections=detect_controls(frame)
    assert len(detections)==1
    hit=detections[0]
    assert hit.kind==kind
    assert hit.phase==(phase if kind=='signal' else None)
    assert hit.capture_time_s==3.5
    assert hit.bearing_deg==pytest.approx(9.46,abs=.3)
    assert abs(hit.range_m-12)<hit.range_uncertainty_m
    assert not hasattr(frame,'primitives') and not hasattr(hit,'node_id')


def test_mutating_pixels_changes_phase_without_any_truth_metadata():
    frame=render_control_camera([ControlPrimitive('signal',12,phase='red')],1.)
    pixels=np.array(frame.image())
    red=(pixels[:,:,0]>170)&(pixels[:,:,1]<90)&(pixels[:,:,2]<90)
    pixels[red]=(45,45,45)
    # The lower lens is inferred from pixels' housing bbox, not primitive truth.
    hit=detect_controls(frame)[0]
    x0,y0,x1,y1=hit.bbox
    cx=(x0+x1)//2;cy=round((y0+y1)/2+(y1-y0+1)*.30)
    pixels[cy-2:cy+3,cx-2:cx+3]=(20,235,50)
    mutated=CameraFrame(pixels.tobytes(),1.,frame.calibration)
    assert detect_controls(mutated)[0].phase=='green'


def test_blank_occluded_and_dropped_frames_do_not_invent_controls():
    assert detect_controls(None)==()
    assert render_control_camera([],0,drop=True) is None
    assert detect_controls(render_control_camera([],0))==()
    assert detect_controls(render_control_camera([ControlPrimitive('stop',12)],0,
        occlusions=((280,120,360,200),)))==()


def test_partial_sign_is_unknown_and_has_no_size_based_range():
    frame=render_control_camera([ControlPrimitive('stop',12)],0,
        occlusions=((320,130,350,190),))
    hits=detect_controls(frame)
    assert hits and all(h.kind=='unknown' and h.range_m is None for h in hits)


def test_far_out_of_fov_and_behind_controls_are_not_guaranteed_visible():
    for primitive in (ControlPrimitive('stop',150),ControlPrimitive('stop',10,100),
                      ControlPrimitive('stop',-10)):
        assert detect_controls(render_control_camera([primitive],0))==()


def test_frame_is_immutable_and_calibration_validated():
    frame=render_control_camera([],0)
    with pytest.raises(FrozenInstanceError):frame.capture_time_s=2
    with pytest.raises(ValueError):CameraFrame(b'',0)
    with pytest.raises(ValueError):CameraCalibration(hfov_deg=float('nan'))


def test_nearer_geometry_occludes_farther_signal():
    frame=render_control_camera([ControlPrimitive('signal',20,phase='green'),
        ControlPrimitive('stop',5)],0)
    assert all(hit.phase!='green' for hit in detect_controls(frame))


def test_conflicting_lens_pixels_do_not_yield_green_permission():
    frame=render_control_camera([ControlPrimitive('signal',12,phase='green')],0)
    pixels=np.array(frame.image())
    x0,y0,x1,y1=detect_controls(frame)[0].bbox
    cx=(x0+x1)//2;cy=round((y0+y1)/2-(y1-y0+1)*.30)
    pixels[cy-2:cy+3,cx-2:cx+3]=(245,25,25)
    hits=detect_controls(CameraFrame(pixels.tobytes(),0,frame.calibration))
    assert all(hit.phase!='green' for hit in hits)
    assert any(hit.phase=='unknown' for hit in hits)


def test_occluded_signal_housing_never_retains_green_from_hidden_truth():
    frame=render_control_camera([ControlPrimitive('signal',12,phase='green')],0,
        occlusions=((300,130,340,153),))
    assert all(hit.phase!='green' for hit in detect_controls(frame))


def test_arbitrary_red_rectangle_is_not_a_stop_sign():
    from PIL import ImageDraw
    frame=render_control_camera([],0)
    image=frame.image()
    ImageDraw.Draw(image).rectangle((300,140,330,170),fill=(240,25,25))
    hits=detect_controls(CameraFrame(image.tobytes(),0,frame.calibration))
    assert hits and all(hit.kind=='unknown' for hit in hits)



def _captured_scene_frame(phase):
    from pathlib import Path
    from PIL import Image
    image=Image.open(Path(__file__).parent/'fixtures'/'scene_camera'/f'{phase}.png').convert('RGB')
    return CameraFrame(image.tobytes(),7.,CameraCalibration(hfov_deg=140.))


@pytest.mark.parametrize('phase',['red','green'])
def test_captured_three_scene_housing_and_pole_from_pixels(phase):
    frame=_captured_scene_frame(phase)
    hits=detect_controls(frame)
    assert len(hits)==1 and hits[0].kind=='signal' and hits[0].phase==phase
    hit=hits[0]
    # Broad head only: the long narrow support pole is not a depth cue.
    assert hit.bbox==(507,139,522,163)
    lower=frame.calibration.focal_px/25
    upper=frame.calibration.focal_px*1.5/25
    assert hit.range_m-hit.range_uncertainty_m<=lower
    assert hit.range_m+hit.range_uncertainty_m>=upper


@pytest.mark.parametrize('band',[(139,146),(147,154),(155,164)])
def test_captured_scene_partial_lens_occlusion_never_permits_green(band):
    frame=_captured_scene_frame('green')
    pixels=np.array(frame.image())
    pixels[band[0]:band[1],507:523]=(100,105,110)
    assert all(hit.phase!='green' for hit in detect_controls(CameraFrame(pixels.tobytes(),7.,frame.calibration)))


def test_captured_scene_conflicting_lamps_reject_green():
    frame=_captured_scene_frame('green')
    pixels=np.array(frame.image())
    # Illuminate the existing top lens pixels; no world phase enters perception.
    pixels[141:145,517:521]=(245,25,25)
    hits=detect_controls(CameraFrame(pixels.tobytes(),7.,frame.calibration))
    assert all(hit.phase!='green' for hit in hits)
    assert any(hit.kind=='signal' and hit.phase=='unknown' for hit in hits)


def test_captured_scene_missing_head_leaves_pole_without_control():
    frame=_captured_scene_frame('green')
    pixels=np.array(frame.image());pixels[138:165,506:524]=(130,160,180)
    assert detect_controls(CameraFrame(pixels.tobytes(),7.,frame.calibration))==()



def _captured_oblique_signal():
    from pathlib import Path
    from PIL import Image
    p=Path(__file__).parent/'fixtures'/'scene_camera'/'signal_1447264315_oblique_green_front.png'
    image=Image.open(p).convert('RGB')
    return CameraFrame(image.tobytes(),979.2,CameraCalibration(hfov_deg=90.))


def test_captured_complete_three_oblique_lenses_are_pixel_observable():
    hits=detect_controls(_captured_oblique_signal())
    assert len(hits)==1 and hits[0].kind=='signal' and hits[0].phase=='green'
    assert hits[0].bbox==(505,120,516,156)
    assert hits[0].confidence_heuristic==.75  # Shape-quality heuristic, not accuracy.


@pytest.mark.parametrize('band',[(123,131),(134,142),(145,153)])
def test_oblique_actual_any_lens_occluded_never_permits_green(band):
    f=_captured_oblique_signal();pixels=np.array(f.image())
    pixels[band[0]:band[1],514:517]=(100,105,110)
    assert all(h.phase!='green' for h in detect_controls(CameraFrame(pixels.tobytes(),979.2,f.calibration)))


def test_oblique_actual_conflicting_lit_lenses_are_unknown():
    f=_captured_oblique_signal();pixels=np.array(f.image())
    pixels[123:131,514:517]=(245,25,25)
    hits=detect_controls(CameraFrame(pixels.tobytes(),979.2,f.calibration))
    assert hits and all(h.phase!='green' for h in hits)
    assert any(h.phase=='unknown' for h in hits)


def _oblique_pixel_fixture(phase='green',width=4,clip=False):
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(640,360),(130,160,180));draw=ImageDraw.Draw(image)
    x=0 if clip else 300
    draw.rectangle((x,100,x+15,143),fill=(18,18,18))
    for i,color in enumerate(('red','amber','green')):
        active={'red':(245,25,25),'amber':(245,185,15),'green':(20,235,50)}
        draw.ellipse((x+9,103+i*13,x+9+width-1,112+i*13),fill=active[color] if color==phase else (45,45,45))
    return CameraFrame(image.tobytes(),0.)


@pytest.mark.parametrize('phase',['red','amber','green','unknown'])
@pytest.mark.parametrize('width',[3,4])
def test_pixel_oblique_other_geometry_preserves_all_phases(phase,width):
    hits=detect_controls(_oblique_pixel_fixture(phase,width=width))
    assert len(hits)==1 and hits[0].phase==phase


@pytest.mark.parametrize('width',[1,2])
def test_unresolved_foreshortened_lenses_do_not_permit_green(width):
    assert all(h.phase!='green' for h in detect_controls(_oblique_pixel_fixture(width=width)))


def test_clipped_complete_signal_has_no_permission():
    assert all(h.phase!='green' for h in detect_controls(_oblique_pixel_fixture(clip=True)))


def test_arbitrary_dark_rectangle_and_green_pole_do_not_become_signal():
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(640,360),(130,160,180));d=ImageDraw.Draw(image)
    d.rectangle((300,100,315,143),fill=(18,18,18))
    d.rectangle((309,129,312,138),fill=(20,235,50))
    assert all(h.phase!='green' for h in detect_controls(CameraFrame(image.tobytes(),0.)))



def test_captured_oblique_pixels_confirm_correct_recorded_control_through_pipeline(monkeypatch):
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.scenario_engine import ScenarioEngine
    from app.services.control_camera_bridge import ControlCameraPipeline
    from app.services.scene_camera_ingress import PendingSceneFrame,CaptureIdentity
    from app.services.interfaces import SimClock
    p=PhysicsEngine();e=ScenarioEngine();e.load_scenario('driving_environment',p)
    # Actual stopped-mission pose, recorded separately from detector pixels.
    p.current_station_m=4522.452364466207;p.current_lateral_offset_m=1.7824786075884287
    p.lat=37.59115610517239;p.lng=-122.36081625291811;p.heading=42.97892234985278
    p.control_camera=ControlCameraPipeline(source='scene_RGB_camera_classical_CV')
    def forbidden(*args):raise AssertionError('World phase must not enter observation')
    monkeypatch.setattr('app.services.control_camera_bridge.simulated_signal_phase',forbidden)
    frame=_captured_oblique_signal()
    # Replayed static RGB fixture at three synthetic acquisition times tests
    # temporal confirmation mechanics; it does not claim three live captures.
    for seq,t in enumerate((979.2,979.3,979.4),1):
        sample=replace(frame,capture_time_s=t);p.clock=SimClock(sim_time_s=t)
        p.scene_camera_pending=PendingSceneFrame(sample,detect_controls(sample),CaptureIdentity('unit',0,int(t*10+1e-8),t,seq,'test'))
        p.control_camera.acquire_scene(p)
        readings={o.control.node_id:o for o in p.control_camera.observations(p)}
        if seq<3:assert not readings[1447264315].identity_observed
    reading=readings[1447264315]
    assert reading.identity_observed and reading.phase=='GREEN'
    assert reading.source=='scene_RGB_camera_classical_CV'
    assert p.control_camera.confirmations[(1447264315,'GREEN')]==3



def _captured_projected_boundary_signal():
    from pathlib import Path
    from PIL import Image
    image=Image.open(Path(__file__).parent/'fixtures'/'scene_camera'/'signal_4946419558_projected_boundary_front_right.png').convert('RGB')
    return CameraFrame(image.tobytes(),531.9,CameraCalibration(320,180,60.))


def test_captured_projected_triplet_includes_quantized_point_five_boundary():
    hits=detect_controls(_captured_projected_boundary_signal())
    assert len(hits)==1 and hits[0].phase=='green' and hits[0].confidence_heuristic==.75


def test_actual_second_signal_pixels_associate_with_correct_recorded_node(monkeypatch):
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.scenario_engine import ScenarioEngine
    from app.services.control_camera_bridge import ControlCameraPipeline
    from app.services.scene_camera_ingress import PendingSceneFrame,CaptureIdentity
    from app.services.interfaces import SimClock
    p=PhysicsEngine();ScenarioEngine().load_scenario('driving_environment',p)
    p.current_station_m=4545.543963856089;p.current_lateral_offset_m=1.725846057860235
    p.lat=37.5913133244878;p.lng=-122.36087072435357;p.heading=317.1249142773586
    p.control_camera=ControlCameraPipeline(source='scene_RGB_camera_classical_CV')
    def forbidden(*args):raise AssertionError('No world phase in driver observation')
    monkeypatch.setattr('app.services.control_camera_bridge.simulated_signal_phase',forbidden)
    captured=_captured_projected_boundary_signal()
    for seq,t in enumerate((531.9,532.,532.1),1):
        auxiliary=replace(captured,capture_time_s=t)
        front=CameraFrame(bytes(640*360*3),t,CameraCalibration(640,360,90.))
        p.clock=SimClock(sim_time_s=t)
        p.scene_camera_pending=PendingSceneFrame(front,(),CaptureIdentity('test',0,int(t*10+1e-8),t,seq,'unit'),
            auxiliary_frames=(('front_right',auxiliary),),auxiliary_detections=(('front_right',detect_controls(auxiliary)),))
        p.control_camera.acquire_scene(p)
        reading=next(o for o in p.control_camera.observations(p) if o.control.node_id==4946419558)
        if seq<3:assert not reading.identity_observed
    assert reading.identity_observed and reading.phase=='GREEN'
    assert p.control_camera.confirmations[(4946419558,'GREEN')]==3


@pytest.mark.parametrize('height,allowed',[(6,True),(5,False)])
def test_projected_lens_quantization_boundary_remains_bounded(height,allowed):
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(640,360),(130,160,180));d=ImageDraw.Draw(image)
    d.rectangle((300,100,314,137),fill=(18,18,18))
    for i in range(3):
        h=7 if i<2 else height
        d.rectangle((309,104+i*11,311,104+i*11+h-1),fill=(20,235,50) if i==2 else (45,45,45))
    hits=detect_controls(CameraFrame(image.tobytes(),0.))
    assert any(h.phase=='green' for h in hits) is allowed


def test_inconsistent_projected_lenses_never_authorize_green():
    frame=_captured_projected_boundary_signal();pixels=np.array(frame.image())
    # Narrow the middle resolved lens to one pixel; other lenses remain clear.
    pixels[51:58,215:217]=(18,18,18)
    assert all(h.phase!='green' for h in detect_controls(CameraFrame(pixels.tobytes(),531.9,frame.calibration)))



@pytest.mark.parametrize('last_width,allowed',[(25,True),(26,False)])
def test_projected_aspect_just_above_half_rejected_even_when_other_gates_pass(last_width,allowed):
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(640,360),(130,160,180));d=ImageDraw.Draw(image)
    d.rectangle((300,80,359,259),fill=(18,18,18))
    for i in range(3):
        width=25 if i<2 else last_width
        d.ellipse((318,98+i*55,318+width-1,148+i*55),fill=(20,235,50) if i==2 else (45,45,45))
    # 25/51=.4902 is supported; 26/51=.5098 is outside the declared range.
    hits=detect_controls(CameraFrame(image.tobytes(),0.))
    assert any(h.phase=='green' for h in hits) is allowed
