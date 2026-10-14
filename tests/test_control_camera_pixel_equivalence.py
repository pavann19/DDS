"""Byte comparisons preserve the original detector observations."""
import inspect
import numpy as np
from app.services import control_camera as camera


def test_unsigned_pixels_match_signed_reference_on_thresholds_and_scenes():
    source=inspect.getsource(camera.detect_controls)
    source=source.replace('.reshape(c.height,c.width,3)', '.reshape(c.height,c.width,3).astype(np.int16)')
    source=source.replace('dark=(rgb[:,:,0]<65)&(rgb[:,:,1]<65)&(rgb[:,:,2]<65)', 'dark=np.max(rgb,axis=2)<65')
    scope=dict(vars(camera));exec(source,scope)
    reference=scope['detect_controls']
    calibration=camera.CameraCalibration(96,64,90.)
    rng=np.random.default_rng(17)
    frames=[camera.CameraFrame(rng.integers(0,256,(64,96,3),dtype=np.uint8).tobytes(),1.,calibration) for _ in range(12)]
    for phase in ['red','amber','green','unknown']:
        frames.append(camera.render_control_camera([camera.ControlPrimitive('signal',5.,phase=phase)],1.,calibration))
    for kind in ['stop','yield']:
        frames.append(camera.render_control_camera([camera.ControlPrimitive(kind,3.)],1.,calibration))
    for value in [0,37,38,60,64,65,89,90,99,100,149,150,169,170,171,219,220,255]:
        pixels=np.full((64,96,3),value,dtype=np.uint8)
        pixels[12:36,40:52]=[value,20,20]
        frames.append(camera.CameraFrame(pixels.tobytes(),1.,calibration))
    for frame in frames:assert camera.detect_controls(frame)==reference(frame)
