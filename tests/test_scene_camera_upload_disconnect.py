from types import SimpleNamespace
import pytest
from fastapi import HTTPException
from starlette.requests import ClientDisconnect
from app.api import scene_camera as endpoint


@pytest.mark.asyncio
async def test_aborted_rgb_upload_never_becomes_a_camera_observation(monkeypatch):
    class Request:
        headers={'content-type':'application/octet-stream'}
        url=SimpleNamespace(path='/api/camera/scene')
        async def stream(self):
            yield b'partial RGB'
            raise ClientDisconnect()
    monkeypatch.setattr(endpoint,'get_simulation_session',lambda:object())
    monkeypatch.setattr(endpoint.ingress,'validate',lambda *args:None)
    async def forbidden(*args,**kwargs):
        raise AssertionError('Incomplete camera frame was accepted')
    monkeypatch.setattr(endpoint.ingress,'accept',forbidden)
    with pytest.raises(HTTPException) as failure:
        await endpoint.scene_camera(Request(),x_simulation_id='test',x_camera_epoch=0,
                                    x_camera_tick=1,x_camera_capture_time=.1,
                                    x_camera_seq=1,x_camera_publisher='test')
    assert failure.value.status_code==400
    assert failure.value.detail=={'accepted':False,'stale_reason':'upload_disconnected'}
