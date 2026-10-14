"""Black-box checks against the running sandbox; no world-state mutation."""
import asyncio
import json
import time
from pathlib import Path
import httpx
import websockets
from websockets.exceptions import InvalidStatus

async def main():
    checks=[]
    try:
        async with websockets.connect('ws://127.0.0.1:8001/ws/telemetry', origin='https://unrelated.example'):
            raise AssertionError('Untrusted browser origin accepted')
    except InvalidStatus as error:
        assert error.response.status_code==403
        checks.append({'check':'untrusted_websocket_origin_rejected','status':403})
    async with httpx.AsyncClient(base_url='http://127.0.0.1:8001',timeout=20.) as client:
        for body in ({'start':{'lat':True,'lng':0},'destination':{'lat':1,'lng':1}},
                     {'start':{'lat':91,'lng':0},'destination':{'lat':1,'lng':1}}):
            response=await client.post('/api/journey/preview',json=body)
            assert response.status_code==422
            checks.append({'check':'invalid_journey_rejected','status':response.status_code})
        response=await client.post('/api/camera/scene',json={},headers={
            'x-simulation-id':'invalid','x-camera-epoch':'0','x-camera-tick':'0',
            'x-camera-capture-time':'0','x-camera-seq':'0','x-camera-publisher':'blackbox'})
        assert response.status_code==415
        checks.append({'check':'non_rgb_camera_transport_rejected','status':response.status_code})
        response=await client.get('/api/camera/scene/unknown.png')
        assert response.status_code==404
        checks.append({'check':'unknown_camera_rejected','status':response.status_code})
    identities=[]
    for attempt in range(2):
        async with websockets.connect('ws://127.0.0.1:8001/ws/telemetry') as socket:
            await socket.send(json.dumps({'type':'unsupported_blackbox_command','command_id':f'blackbox-{attempt}'}))
            state=reply=None
            deadline=time.monotonic()+10
            while (state is None or reply is None) and time.monotonic()<deadline:
                message=json.loads(await asyncio.wait_for(socket.recv(),5))
                if message.get('type')=='state':state=message
                if message.get('command_id')==f'blackbox-{attempt}':reply=message
            assert reply and reply['status']=='rejected'
            assert state and isinstance(state['channels']['pose']['ego']['sensor_health']['monitored'], bool)
            identities.append((state['simulation_id'],state['epoch']))
            await socket.send(json.dumps({'type':'set_destination','lat':True,'lng':0,'command_id':f'invalid-coordinate-{attempt}'}))
            while True:
                message=json.loads(await asyncio.wait_for(socket.recv(),5))
                if message.get('command_id')==f'invalid-coordinate-{attempt}':
                    assert message['status']=='rejected'
                    break
    assert identities[0]==identities[1]
    checks.append({'check':'websocket_invalid_command_and_reconnect','same_session':True})
    checks.append({'check':'websocket_boolean_destination_rejected','status':'rejected'})
    Path('output/sdlc-round-blackbox.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
    print(json.dumps(checks,indent=2))

if __name__=='__main__':asyncio.run(main())
