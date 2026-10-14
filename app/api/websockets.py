import asyncio
import json
import logging
from typing import List, Optional
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ValidationError, Field
from starlette.websockets import WebSocketState

from app.services.simulation_session import get_simulation_session
from app.core.config import settings

logger = logging.getLogger(__name__)

router = APIRouter()

# Telemetry stream / simulation tick rate.
STREAM_HZ = 10.0

# Safety cap on the streaming loop for tests.
MAX_STREAM_TICKS: Optional[int] = None


class SetDestinationCommand(BaseModel):
    type: str
    lat: float = Field(ge=-90, le=90, allow_inf_nan=False, strict=True)
    lng: float = Field(ge=-180, le=180, allow_inf_nan=False, strict=True)


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)


manager = ConnectionManager()


@router.websocket("/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    # CORS middleware does not protect WebSocket upgrades. Browser commands
    # must originate from the configured cockpit, not an unrelated website.
    origin = websocket.headers.get('origin')
    if origin is not None and origin not in settings.ALLOWED_ORIGINS:
        await websocket.close(code=1008)
        return
    await manager.connect(websocket)

    session = get_simulation_session()
    session.start()

    sub_id, sub_queue = session.subscribe()
    disconnected = False
    ticks_sent = 0

    async def listen_for_commands():
        nonlocal disconnected
        try:
            while not disconnected:
                data = await websocket.receive_text()
                try:
                    payload = json.loads(data)
                    if not isinstance(payload, dict):
                        raise ValueError("Command must be an object")
                    cmd_type = payload.get("type")
                    if cmd_type == "set_destination":
                        cmd = SetDestinationCommand(**payload)
                        session.enqueue_command({**payload, "lat": cmd.lat, "lng": cmd.lng}, subscriber_id=sub_id)
                        logger.info(f"New destination queued: {cmd.lat}, {cmd.lng}")
                    elif cmd_type in {
                        "load_scenario",
                        "pause_simulation",
                        "resume_simulation",
                        "reset_simulation",
                        "step_simulation",
                        "set_cruise_speed",
                        "start_journey",
                    }:
                        session.enqueue_command(payload, subscriber_id=sub_id)
                        logger.info(f"Simulation command queued: {cmd_type}")
                    else:
                        raise ValueError("Unsupported command type")
                except json.JSONDecodeError:
                    session.command_reply(sub_id, None, "rejected", "Invalid JSON")
                except ValidationError as e:
                    session.command_reply(sub_id, payload.get("command_id"), "rejected", str(e))
                except ValueError as e:
                    session.command_reply(sub_id, payload.get("command_id") if isinstance(payload, dict) else None, "rejected", str(e))
                except Exception as e:
                    logger.error(f"Error processing incoming message: {e}", exc_info=True)
        except WebSocketDisconnect:
            disconnected = True
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error in websocket listener: {e}", exc_info=True)
            disconnected = True

    async def stream():
        nonlocal ticks_sent
        while not disconnected:
            if MAX_STREAM_TICKS is not None and ticks_sent >= MAX_STREAM_TICKS:
                break
            msg = await sub_queue.get()
            sub_queue.task_done()
            if msg.get("type") == "subscriber_overflow":
                await websocket.close(code=1013, reason="Reliable subscriber queue overflow; reconnect")
                return
            if websocket.client_state != WebSocketState.CONNECTED:
                return
            await websocket.send_json(msg)
            if msg.get("type") == "state":
                ticks_sent += 1

    listener_task = asyncio.create_task(listen_for_commands())
    sender_task = asyncio.create_task(stream())
    try:
        done, _ = await asyncio.wait((listener_task, sender_task), return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
    except WebSocketDisconnect:
        logger.info(f"Websocket disconnected for subscriber {sub_id}")
    except RuntimeError as e:
        # The transport can observe peer-close before receive() wakes up.
        # This specific ASGI close race is a disconnect, not a simulation fault.
        if "after sending 'websocket.close'" not in str(e) and "close message has been sent" not in str(e):
            raise
    except Exception as e:
        if not disconnected:
            logger.error(f"Error in websocket streaming loop: {e}", exc_info=True)
    finally:
        disconnected = True
        for task in (listener_task, sender_task):
            task.cancel()
        await asyncio.gather(listener_task, sender_task, return_exceptions=True)
        session.unsubscribe(sub_id)
        manager.disconnect(websocket)
        if websocket.application_state == WebSocketState.CONNECTED:
            try:
                await websocket.close()
            except (RuntimeError, WebSocketDisconnect):
                pass
