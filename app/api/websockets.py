import json
import time
import uuid
import asyncio
import logging
from typing import List
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ValidationError

from app.core.database import AsyncSessionLocal, SessionRecord, TelemetryLog, DriverScoreLog
from app.services.executor import MultiRateExecutor
from app.services.inference import pipeline
from app.services.physics_engine import PhysicsEngine
from app.services.driver_scoring import DriverScorer
from app.services.routing import get_route
from app.services.scenario_engine import ScenarioEngine
from sqlalchemy.future import select
from datetime import datetime

logger = logging.getLogger(__name__)

router = APIRouter()

# Telemetry stream / simulation tick rate.
STREAM_HZ = 10.0

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
    scenario_engine = ScenarioEngine()
    
    disconnected = False
    final_status = "completed"
    predictions_count = 0
    total_score = 0
    pending_logs: List[TelemetryLog] = []
    pending_scores: List[DriverScoreLog] = []

    await create_stream_session(session_id)

    #  fetch a real road-following route in the background (the
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
                #  send physics.route (the spline-smoothed, uniformly
                # resampled path) rather than the raw OSRM waypoints, so the
                # rendered road is exactly the path the car is actually
                # driving. Sending the raw polyline here would reintroduce the
                # backend/frontend world-disagreement that the previous had to fix.
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
                    cmd_type = payload.get("type")
                    if cmd_type == "set_destination":
                        cmd = SetDestinationCommand(**payload)
                        physics.set_destination(cmd.lat, cmd.lng)
                        logger.info(f"New destination set: {cmd.lat}, {cmd.lng}")
                        asyncio.create_task(fetch_and_apply_route(physics.lat, physics.lng, cmd.lat, cmd.lng))
                    elif cmd_type == "load_scenario":
                        scenario_id = payload.get("scenario_id")
                        density = payload.get("traffic_density")
                        initial_speed = payload.get("initial_speed_kmh")
                        if scenario_id:
                            evt = scenario_engine.load_scenario(
                                scenario_id,
                                physics,
                                density=density,
                                initial_speed_kmh=initial_speed,
                            )
                            logger.info(f"Scenario loaded: {scenario_id}")
                            await websocket.send_json(evt)
                    elif cmd_type == "pause_simulation":
                        scenario_engine.pause(physics)
                        logger.info("Simulation paused")
                    elif cmd_type == "resume_simulation":
                        scenario_engine.resume(physics)
                        logger.info("Simulation resumed")
                    elif cmd_type == "reset_simulation":
                        evt = scenario_engine.reset(physics)
                        logger.info("Simulation reset")
                        if evt and isinstance(evt, dict) and "type" in evt:
                            await websocket.send_json(evt)
                    elif cmd_type == "step_simulation":
                        physics.is_paused = False
                        physics.update(current_action)
                        physics.is_paused = True
                        logger.info("Simulation stepped 1 tick")
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

        # The MultiRateExecutor owns the authoritative fixed-step SimClock
        # (ADR-001 item 4). Scenario + physics run as ONE stage registered
        # at the stream rate -- gate 6.5.3 is "the executor drives the
        # tick", with the 50/20/10 Hz perception/planner/control split
        # documented as wired-but-single-stage until the Phase 11 deep
        # decouple gives each stage its own registration. Stepping both off
        # executor.clock.dt_s keeps the scenario/physics desync closed.
        executor = MultiRateExecutor(base_hz=STREAM_HZ)
        _tick_ctx = {"event": None}

        def _sim_stage(clock):
            _tick_ctx["event"] = scenario_engine.update(physics, clock.dt_s)
            physics.update(current_action, dt=clock.dt_s)

        executor.add_stage("sim", STREAM_HZ, _sim_stage)

        while not disconnected:
            executor.step()
            scenario_event = _tick_ctx["event"]
            if scenario_event and not disconnected:
                await websocket.send_json(scenario_event)

            nav_state = physics.get_navigation_state()
            input_dict = physics.get_ml_features()
            
            # Run ML pipeline in background thread
            result = await asyncio.to_thread(pipeline.predict, input_dict)
            
            current_action = result["prediction"]
            clean_input = result["clean_input"]

            # Score the driver
            scorer.add_reading(clean_input, current_action, result["confidence"], result["anomaly_result"])
            score_data = scorer.calculate_score()

            # Prepare Payload (V2 Protocol)
            current_time = time.time()
            iso_time = datetime.fromtimestamp(current_time).isoformat() + "Z"
            
            # Extract NPC and perception data
            npc_states = physics.get_npc_states()
            traffic_data = []
            for npc in npc_states:
                speed = npc.get('speed_kmh', 0)
                is_oncoming = speed < 0
                traffic_data.append({
                    "id": f"npc_{npc['id']}",
                    "pose": {"x": npc.get('lane_offset', 0), "y": 0, "z": -npc.get('station_m', 0)},
                    "yaw": 180 if is_oncoming else 0,
                    "velocity": abs(speed) / 3.6,
                    "acceleration": 0,
                    "frenet": {"s": npc.get('station_m', 0), "d": npc.get('lane_offset', 0)}
                })
                
            perception_data = []
            sensed_gap = nav_state.get("sensed_lead_gap_m")
            if sensed_gap is not None and sensed_gap < 100.0:
                lead_speed_mps = (nav_state.get("sensed_lead_speed_kmh") or 0.0) / 3.6
                ego_speed_mps = physics.speed_kmh / 3.6
                rel_v = lead_speed_mps - ego_speed_mps
                lead_station = nav_state.get("station_m", 0) + sensed_gap
                lead_lane = nav_state.get("lateral_offset_m", 0)

                perception_data.append({
                    "id": "sensed_lead_vehicle",
                    "type": "VEHICLE",
                    "pose": {"x": lead_lane, "y": 0, "z": -lead_station},
                    "yaw": 0,
                    "velocity": lead_speed_mps,
                    "acceleration": 0,
                    "frenet": {"s": lead_station, "d": lead_lane},
                    "distance": round(sensed_gap, 1),
                    "rel_velocity": round(rel_v, 1),
                    "confidence": 0.98
                })

            # Generate 50m Planned Trajectory Corridor (Tesla / Waymo Path of Intent)
            curr_s = nav_state.get("station_m", 0)
            target_d = nav_state.get("lateral_target_m", 0)
            curr_d = nav_state.get("lateral_offset_m", 0)
            trajectory = []
            for i in range(12):
                progress = i / 11.0
                interp_d = curr_d + (target_d - curr_d) * progress
                trajectory.append({
                    "x": round(interp_d, 2),
                    "y": 0.02,
                    "z": round(-(curr_s + i * 4.5), 2)
                })

            # Regression fix: SHAP/anomaly/driver-score/planner-candidates
            # were being computed every tick and passed to the DB-logging
            # calls below, but never placed in the WS payload -- so they
            # reached SQLite but no connected client, live or otherwise.
            # Restored as additive `data` siblings (protocol_version stays
            # "2.0"; nothing existing changes shape) so the frontend's
            # explainability/safety panels have something real to render.
            planner_candidates = physics.get_planner_candidates()
            chosen_candidate = next((c for c in planner_candidates if c["is_chosen"]), None)
            is_changing_lane = bool(chosen_candidate and chosen_candidate["is_lane_change"])

            ego_state = {
                "id": "ego_1",
                "pose": {"x": nav_state.get("lateral_offset_m", 0), "y": 0, "z": -nav_state.get("station_m", 0)},
                "yaw": getattr(physics, 'steering_angle_rad', 0) * 15,  # subtle wheel angle
                "velocity": physics.speed_kmh / 3.6,
                "acceleration": getattr(physics, 'acceleration_mps2', 0),
                "frenet": {"s": nav_state.get("station_m", 0), "d": nav_state.get("lateral_offset_m", 0)},
                # Retained on ego for HMI compatibility; since ADR-001 item 5
                # the decision does NOT drive the vehicle -- the authoritative
                # copy lives in channels.semantic.driver_analytics.
                "decision": current_action,
                "confidence": result["confidence"],
                # Which physical constraint is currently binding the speed
                # controller -- "car_following" is IDM braking for traffic,
                # "predictive_cut_in" is the Phase 7 proactive response, etc.
                "speed_limit_reason": nav_state.get("speed_limit_reason"),
                "target_velocity": 50.0 / 3.6,
                "steering_angle": getattr(physics, 'steering_angle_rad', 0),
                "throttle": max(0.0, getattr(physics, 'acceleration_mps2', 0) / 3.0),
                "brake": max(0.0, -getattr(physics, 'acceleration_mps2', 0) / 4.5),
            }

            # Protocol v3 (ADR-001 item 7): one message, layered channels.
            #   pose      -- small, would-be-high-rate ego kinematics
            #   semantic  -- everything the HMI needs to explain a decision
            #   heavy     -- large payloads (surround tracks, per-agent
            #                predictions); a later phase can gate these
            #                on-demand / delta-encode without reshaping the
            #                envelope.
            payload = {
                "protocol_version": "3.0",
                "simulation_id": session_id,
                "run_id": "live",
                "tick": predictions_count,
                "simulation_time_s": round(predictions_count * 0.1, 2),
                "timestamp": iso_time,
                "coordinate_frame": "dds_world_v1",
                "units": "SI",
                "type": "state",
                "channels": {
                    "pose": {
                        "ego": ego_state,
                    },
                    "semantic": {
                        "traffic": traffic_data,
                        "perception": perception_data,
                        "planner": {
                            "trajectory": trajectory,
                            "lookahead_point": trajectory[-1] if trajectory else {"x": 0, "y": 0, "z": 0},
                            "lane_center": nav_state.get("lateral_offset_m", 0),
                            "curvature": getattr(physics, 'path_curvature', 0),
                            "candidates": planner_candidates,
                            "is_changing_lane": is_changing_lane,
                        },
                        # Independent safety layer, evaluated AFTER the
                        # planner/IDM decision -- see safety_shield.py.
                        "safety_shield": physics.get_safety_shield_state(),
                        "scenario": scenario_engine.get_state(),
                        # ADR-001 item 5: the learned model as a
                        # driver-behaviour / eco-efficiency analytics
                        # channel, NOT a driving policy.
                        "driver_analytics": {
                            "decision": current_action,
                            "confidence": result["confidence"],
                            "shap": result["shap_result"],
                            "anomaly": result["anomaly_result"],
                            "driver_score": score_data,
                        },
                    },
                    "heavy": {
                        # Phase 6: 360-degree surround perception -- confirmed
                        # tracks only.
                        "surround_perception": physics.get_surround_perception_state(),
                        # Phase 7: per-agent forecasts (3 s / 0.1 s trails),
                        # intent distributions, proactive cut-in response.
                        "prediction": physics.get_prediction_state(),
                    },
                },
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
