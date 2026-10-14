"""Read-only camera preview, acquired frames only; never synthesizes on request."""
from io import BytesIO
from fastapi import APIRouter, HTTPException
from fastapi.responses import Response
from app.services.simulation_session import get_simulation_session
router=APIRouter()

@router.get('/camera/control.png')
def control_camera_frame():
    p=get_simulation_session().world.physics
    camera=p.control_camera if p.camera_controls else None
    if camera is None or camera.frame is None:
        raise HTTPException(404,'Camera frame unavailable')
    output=BytesIO();camera.frame.image().save(output,format='PNG')
    return Response(output.getvalue(),media_type='image/png',headers={'Cache-Control':'no-store','X-Capture-Time-S':str(camera.frame.capture_time_s)})
