"""Modeled center-mounted camera rig contract, not surveyed physical extrinsics."""
from app.services.control_camera import CameraCalibration
CAMERA_IDS=('front','front_left','front_right','left','right','rear_left','rear_right','rear')
CAMERA_YAWS_DEG=dict(zip(CAMERA_IDS,(0.,-45.,45.,-90.,90.,-135.,135.,180.)))
CALIBRATION=CameraCalibration(640,360,90.)
AUX_CALIBRATION=CameraCalibration(320,180,100.)
CAMERA_CALIBRATIONS={name: (CALIBRATION if name=='front' else CameraCalibration(320,180,60.) if name in {'front_left','front_right'} else AUX_CALIBRATION) for name in CAMERA_IDS}
FRAME_BYTES=640*360*3
AUX_FRAME_BYTES=320*180*3
RIG_BYTES=FRAME_BYTES+7*AUX_FRAME_BYTES
