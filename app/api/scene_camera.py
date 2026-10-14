"""Raw RGB transport only; no control commands or world labels accepted."""
from fastapi import APIRouter,Request,Header,HTTPException
from starlette.requests import ClientDisconnect
from app.services.scene_camera_ingress import SceneCameraIngress,CaptureIdentity,IngressRejected,FRAME_BYTES,RIG_BYTES
from app.services.simulation_session import get_simulation_session

router=APIRouter()
ingress=SceneCameraIngress()


@router.post('/camera/scene')
@router.post('/camera/scene/rig')
async def scene_camera(request:Request,
    x_simulation_id:str=Header(),x_camera_epoch:int=Header(),x_camera_tick:int=Header(),
    x_camera_capture_time:float=Header(),x_camera_seq:int=Header(),x_camera_publisher:str=Header()):
    if request.headers.get('content-type','').split(';')[0]!='application/octet-stream':
        raise HTTPException(415,detail='raw_rgb_octet_stream_required')
    rig=request.url.path.endswith('/rig')
    expected_bytes=RIG_BYTES if rig else FRAME_BYTES
    length=request.headers.get('content-length')
    if length is not None and length!=str(expected_bytes):
        raise HTTPException(413,detail='invalid_rgb_length')
    identity=CaptureIdentity(x_simulation_id,x_camera_epoch,x_camera_tick,x_camera_capture_time,x_camera_seq,x_camera_publisher)
    session=get_simulation_session()
    try:
        ingress.validate(session,identity)
        body=bytearray()
        async for chunk in request.stream():
            if len(body)+len(chunk)>expected_bytes:raise IngressRejected('body_too_large',413)
            body.extend(chunk)
        accepted=await ingress.accept(session,identity,body,rig=rig)
    except IngressRejected as error:
        raise HTTPException(error.status,detail={'accepted':False,'stale_reason':error.reason}) from error
    except ClientDisconnect as error:
        # An aborted upload supplies no observation. Classify the transport
        # interruption without turning a normal disconnect into an ASGI fault.
        raise HTTPException(400,detail={'accepted':False,'stale_reason':'upload_disconnected'}) from error
    return {'accepted':True,'seq':accepted.identity.seq,'stale_reason':None,
        'detection_ms':accepted.detection_ms,
        'capture_age_s':session.executor.clock.sim_time_s-accepted.identity.capture_time_s}


@router.get('/camera/scene/{camera_id}.png')
async def scene_camera_view(camera_id:str):
    from io import BytesIO
    from fastapi.responses import Response
    from app.services.scene_camera_ingress import CAMERA_IDS
    if camera_id not in CAMERA_IDS:raise HTTPException(404,detail='unknown_camera')
    camera=get_simulation_session().world.physics.control_camera
    frame=getattr(camera,'camera_frames',{}).get(camera_id)
    if frame is None and camera_id=='front':frame=getattr(camera,'frame',None)
    if frame is None:raise HTTPException(404,detail='camera_pending')
    output=BytesIO();frame.image().save(output,format='PNG')
    return Response(output.getvalue(),media_type='image/png',headers={'Cache-Control':'no-store',
        'X-Capture-Time-S':str(frame.capture_time_s),'X-Camera-Id':camera_id})
