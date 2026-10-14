"""Bounded browser RGB ingress; accepted observations await a session boundary."""
import asyncio
from dataclasses import dataclass
import math
import time
from time import perf_counter
from concurrent.futures import ThreadPoolExecutor, wait
from app.services.control_camera import CameraCalibration,CameraFrame,detect_controls

from app.services.camera_rig import CALIBRATION,FRAME_BYTES,CAMERA_IDS,CAMERA_CALIBRATIONS,AUX_FRAME_BYTES,RIG_BYTES

# Bounded CPU fan-out; frames are immutable and detection is stateless.
_detector_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix='dds-camera')


class IngressRejected(Exception):
    def __init__(self,reason,status=409):
        self.reason,self.status=reason,status


@dataclass(frozen=True)
class CaptureIdentity:
    simulation_id:str
    epoch:int
    tick:int  # Published telemetry tick: 10 Hz, not the internal 100 Hz tick.
    capture_time_s:float
    seq:int
    publisher:str


@dataclass(frozen=True)
class PendingSceneFrame:
    frame:CameraFrame
    detections:tuple
    identity:CaptureIdentity
    auxiliary_frames:tuple=()
    auxiliary_detections:tuple=()
    detection_ms:float=0.


class SceneCameraIngress:
    PUBLISHER_LEASE_S = 2.0
    def __init__(self):
        self.busy=False
        self.key=None
        self.publisher=None
        self.seq=-1
        self.capture=-math.inf
        self.last_wall=-math.inf

    def validate(self,session,identity):
        p=session.world.physics
        if not getattr(p,'scene_camera_controls',False):raise IngressRejected('scene_camera_disabled')
        if identity.simulation_id!=session.session_id or identity.epoch!=session.epoch:
            raise IngressRejected('session_or_epoch_changed')
        if identity.seq<0 or identity.tick<0 or not identity.publisher or len(identity.publisher)>128:
            raise IngressRejected('invalid_identity',422)
        capture=identity.capture_time_s
        now=session.executor.clock.sim_time_s
        if not math.isfinite(capture) or capture<0:raise IngressRejected('invalid_capture_time',422)
        if math.floor((capture+1e-8)*10)!=identity.tick:
            raise IngressRejected('tick_time_mismatch',422)
        if capture>now+.02:raise IngressRejected('future_capture')
        if now-capture>.35:raise IngressRejected('stale_capture')
        key=(session.session_id,session.epoch)
        if self.key==key:
            same_publisher=identity.publisher==self.publisher
            if not same_publisher and time.monotonic()-self.last_wall <= self.PUBLISHER_LEASE_S:
                raise IngressRejected('publisher_conflict')
            # Sequence numbers belong to each tab. Capture time stays a global
            # high-water mark even when an expired owner is replaced.
            if (same_publisher and identity.seq<=self.seq) or capture<=self.capture:
                raise IngressRejected('duplicate_or_out_of_order')

    async def accept(self,session,identity,rgb,*,rig=False):
        self.validate(session,identity)
        if len(rgb)!=(RIG_BYTES if rig else FRAME_BYTES):raise IngressRejected('invalid_rgb_length',422)
        if self.busy:raise IngressRejected('frame_inflight',429)
        wall=time.monotonic()
        # Telemetry is 10 Hz; wall arrival jitter must not reject a distinct
        # next tick merely because it arrived slightly before 100 ms.
        # Keep a bounded wall-rate guard and strictly increasing capture IDs.
        if self.key==(session.session_id,session.epoch) and wall-self.last_wall<.075:
            raise IngressRejected('publisher_rate_limit',429)
        self.busy=True
        try:
            frame=CameraFrame(bytes(rgb[:FRAME_BYTES]),identity.capture_time_s,CALIBRATION)
            auxiliary=tuple((name,CameraFrame(bytes(rgb[FRAME_BYTES+i*AUX_FRAME_BYTES:FRAME_BYTES+(i+1)*AUX_FRAME_BYTES]),
                identity.capture_time_s,CAMERA_CALIBRATIONS[name])) for i,name in enumerate(CAMERA_IDS[1:])) if rig else ()
            # Shield worker lifetime so cancelled requests cannot spawn parallel CV.
            def detect_bundle():
                frames=(frame,)+tuple(view for _,view in auxiliary)
                futures=tuple(_detector_pool.submit(detect_controls,view) for view in frames)
                # Do not release ingress busy on one view's exception while
                # other workers are still processing the same bundle.
                wait(futures)
                results=tuple(tuple(future.result()) for future in futures)
                return results[0],tuple((name,results[i+1]) for i,(name,_) in enumerate(auxiliary))
            detection_start=perf_counter()
            task=asyncio.create_task(asyncio.to_thread(detect_bundle))
            try:
                detections,auxiliary_detections=await asyncio.shield(task)
            except asyncio.CancelledError:
                await task
                raise
            self.validate(session,identity)
            pending=PendingSceneFrame(frame,tuple(detections),identity,auxiliary,auxiliary_detections,
                (perf_counter()-detection_start)*1000)
            session.world.physics.scene_camera_pending=pending
            self.key=(session.session_id,session.epoch)
            self.publisher=identity.publisher
            self.seq=identity.seq
            self.capture=identity.capture_time_s
            self.last_wall=wall
            return pending
        finally:
            self.busy=False
