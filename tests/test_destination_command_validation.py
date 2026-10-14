import pytest
from pydantic import ValidationError
from app.api.websockets import SetDestinationCommand
from app.api.websockets import websocket_endpoint


@pytest.mark.parametrize('value', [True, False, '37.5', float('nan'), float('inf'), 91])
def test_destination_latitude_rejects_coercion_and_invalid_values(value):
    with pytest.raises(ValidationError):
        SetDestinationCommand(type='set_destination', lat=value, lng=0)


def test_integer_coordinates_remain_valid():
    assert SetDestinationCommand(type='set_destination', lat=37, lng=-122).lat == 37


@pytest.mark.asyncio
async def test_untrusted_browser_origin_rejected_before_session_access():
    class Socket:
        headers={'origin':'https://unrelated.example'}
        code=None
        async def close(self,code):self.code=code
    socket=Socket()
    await websocket_endpoint(socket)
    assert socket.code==1008
