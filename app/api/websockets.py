import json
import time
import uuid
import asyncio
import logging
from typing import List
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ValidationError

from app.core.database import AsyncSessionLocal, SessionRecord, TelemetryLog, DriverScoreLog
from app.services.inference import pipeline
from app.services.physics_engine import PhysicsEngine
from app.services.driver_scoring import DriverScorer
from app.services.routing import get_route
from sqlalchemy.future import select
from datetime import datetime

logger = logging.getLogger(__name__)

router = APIRouter()

class SetDestinationCommand(BaseModel):
    type: str
    lat: float
    lng: float

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

async def create_stream_session(session_id: str):
    try:
        async with AsyncSessionLocal() as db:
            db.add(SessionRecord(id=session_id))
            await db.commit()
    except Exception as e:
        logger.error(f"Failed to create session {session_id}: {e}", exc_info=True)

async def persist_stream_batch(log_entries: list, score_entries: list):
    if not log_entries and not score_entries:
        return
    try:
        async with AsyncSessionLocal() as db:
            db.add_all([*log_entries, *score_entries])
            await db.commit()
    except Exception as e:
        logger.error(f"Failed to persist stream batch: {e}", exc_info=True)

async def finalize_stream_session(session_id: str, status: str, total_predictions: int, total_score: float):
    try:
        async with AsyncSessionLocal() as db:
            result = await db.execute(select(SessionRecord).where(SessionRecord.id == session_id))
            session = result.scalars().first()
            if session:
                session.end_time = datetime.utcnow()
                session.status = status
                session.total_predictions = total_predictions
                session.avg_score = total_score / max(1, total_predictions)
                await db.commit()
    except Exception as e:
        logger.error(f"Failed to finalize session {session_id}: {e}", exc_info=True)

@router.websocket("/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)

    session_id = str(uuid.uuid4())
    scorer = DriverScorer(window_size=60)
    physics = PhysicsEngine(start_lat=37.7749, start_lng=-122.4194)
    physics.set_destination(37.8199, -122.4783) # Default: Golden Gate Bridge
    
    disconnected = False
    final_status = "completed"
    predictions_count = 0
    total_score = 0
    pending_logs: List[TelemetryLog] = []
    pending_scores: List[DriverScoreLog] = []

    await create_stream_session(session_id)

    # P3-1: fetch a real road-following route in the background (the
    # routing call can take 1-2s -- the car keeps driving on the
    # straight-line fallback in the meantime, then snaps onto the real
    # route once this resolves). Pushed to the client as a one-time
    # "route" message rather than resent every tick; the frontend renders
    # the full path and slices it locally using navigation.route_index.
    async def fetch_and_apply_route(origin_lat, origin_lng, dest_lat, dest_lng):
        route_data = await get_route(origin_lat, origin_lng, dest_lat, dest_lng)
        if not route_data:
            return  # routing service unavailable -- straight-line fallback already active
        waypoints, steps = route_data
        
        if (physics.target_lat, physics.target_lng) != (dest_lat, dest_lng):
            # A newer set_destination happened while this fetch was in
            # flight -- discard this now-stale route rather than
            # overwriting the route for the current destination.
            return
        physics.set_route(waypoints)
        if disconnected:
            return
        try:
            await websocket.send_json({
                "type": "route",
                # P6-1d: send physics.route (the spline-smoothed, uniformly
                # resampled path) rather than the raw OSRM waypoints, so the
                # rendered road is exactly the path the car is actually
                # driving. Sending the raw polyline here would reintroduce the
                # backend/frontend world-disagreement that P6-1b had to fix.
                "waypoints": [[lat, lng] for lat, lng in physics.route],
                "steps": steps
            })
        except Exception as e:
            logger.warning(f"Failed to send route to client (likely disconnected): {e}")

    asyncio.create_task(fetch_and_apply_route(physics.lat, physics.lng, physics.target_lat, physics.target_lng))

    # Background task to listen for commands
    async def listen_for_commands():
        nonlocal disconnected
        try:
            while not disconnected:
                data = await websocket.receive_text()
                try:
                    payload = json.loads(data)
                    if payload.get("type") == "set_destination":
                        cmd = SetDestinationCommand(**payload)
                        physics.set_destination(cmd.lat, cmd.lng)
                        logger.info(f"New destination set: {cmd.lat}, {cmd.lng}")
                        asyncio.create_task(fetch_and_apply_route(physics.lat, physics.lng, cmd.lat, cmd.lng))
                except json.JSONDecodeError:
                    logger.warning("Invalid JSON received on websocket")
                except ValidationError as e:
                    logger.warning(f"Validation error for incoming message: {e}")
                except Exception as e:
                    logger.error(f"Error processing incoming message: {e}", exc_info=True)
        except WebSocketDisconnect:
            disconnected = True
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error in websocket listener: {e}", exc_info=True)
            disconnected = True

    listener_task = asyncio.create_task(listen_for_commands())
    
    current_action = "Maintain Speed"

    try:
        if not pipeline.is_ready():
            final_status = "failed"
            errors = pipeline.get_errors()
            await websocket.send_json({"error": "Backend is not ready for streaming.", "details": errors})
            await websocket.close(code=1011)
            return

        while not disconnected:
            # Update Physics Engine
            physics.update(current_action)
            nav_state = physics.get_navigation_state()
            input_dict = physics.get_ml_features()
            
            # Run ML pipeline in background thread
            result = await asyncio.to_thread(pipeline.predict, input_dict)
            
            current_action = result["prediction"]
            clean_input = result["clean_input"]

            # Score the driver
            scorer.add_reading(clean_input, current_action, result["confidence"], result["anomaly_result"])
            score_data = scorer.calculate_score()

            # Prepare Payload
            current_time = time.time()
            payload = {
                "timestamp": current_time,
                "telemetry": clean_input,
                "navigation": nav_state,
                "predicted_decision": current_action,
                "confidence": result["confidence_dict"],
                "confidence_override": result["confidence_override"],
                "shap": result["shap_result"],
                "anomaly": result["anomaly_result"],
                "driver_score": score_data,
                # P6-1b: server-side NPC traffic state, for the frontend to
                # RENDER (P6-1c). Distinct from navigation.sensed_lead_* above,
                # which is the ego's own sensor output used for control -- the
                # HMI is allowed to see the full scene, the planner is not.
                "npcs": physics.get_npc_states(),
                # P6-2: the local planner's last scored lateral-candidate set
                # (dimmed alternatives + the chosen path), for the HMI (P6-5)
                # to visualise and for the P6-6 evaluation.
                "planner_candidates": physics.get_planner_candidates(),
            }

            if not disconnected:
                await websocket.send_json(payload)

            # DB Persistence buffering
            pending_logs.append(TelemetryLog(
                session_id=session_id,
                timestamp=current_time,
                features=clean_input,
                prediction=current_action,
                confidence=result["confidence"],
                shap_values=result["shap_result"],
                is_anomaly=result["anomaly_result"]["is_anomaly"],
                anomaly_type=result["anomaly_result"]["type"]
            ))

            if predictions_count % 10 == 0:
                pending_scores.append(DriverScoreLog(
                    session_id=session_id,
                    timestamp=current_time,
                    score=score_data["score"],
                    rating=score_data["rating"],
                    breakdown=score_data["breakdown"]
                ))

            predictions_count += 1
            total_score += score_data["score"]

            if len(pending_logs) >= 10:
                await persist_stream_batch(pending_logs, pending_scores)
                pending_logs = []
                pending_scores = []

            # Tick rate (10 Hz)
            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        logger.info(f"Websocket disconnected for session {session_id}")
    except Exception as e:
        if not disconnected:
            final_status = "failed"
            logger.error(f"Error in websocket loop: {e}", exc_info=True)
    finally:
        disconnected = True
        listener_task.cancel()
        manager.disconnect(websocket)
        await persist_stream_batch(pending_logs, pending_scores)
        await finalize_stream_session(session_id, final_status, predictions_count, total_score)
