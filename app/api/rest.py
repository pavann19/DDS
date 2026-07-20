import os
import json
from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy.future import select
from sqlalchemy import func, desc
from app.core.database import get_db, SessionRecord, TelemetryLog
from app.core.config import settings
from app.services.inference import pipeline

router = APIRouter()

def session_to_dict(session: SessionRecord) -> dict:
    return {
        "id": session.id,
        "start_time": session.start_time.isoformat() if session.start_time else None,
        "end_time": session.end_time.isoformat() if session.end_time else None,
        "total_predictions": session.total_predictions,
        "avg_score": session.avg_score,
        "status": session.status,
    }

def telemetry_to_dict(log: TelemetryLog) -> dict:
    return {
        "id": log.id,
        "session_id": log.session_id,
        "timestamp": log.timestamp,
        "features": log.features,
        "prediction": log.prediction,
        "confidence": log.confidence,
        "shap_values": log.shap_values,
        "is_anomaly": log.is_anomaly,
        "anomaly_type": log.anomaly_type,
    }

@router.get("/health")
async def health_check():
    errors = pipeline.get_errors()
    return {
        "status": "ok" if not errors else "degraded",
        "model_loaded": pipeline.model is not None,
        "ready_for_streaming": pipeline.is_ready(),
        "errors": errors,
    }

@router.get("/metrics")
async def get_metrics():
    metrics_file = os.path.join(settings.BASE_DIR, "metrics.json")
    try:
        with open(metrics_file, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"error": "metrics.json not found. Train the model first."}

@router.get("/sessions")
async def list_sessions(
    skip: int = Query(0, ge=0), 
    limit: int = Query(50, ge=1, le=100), 
    db = Depends(get_db)
):
    result = await db.execute(select(SessionRecord).order_by(desc(SessionRecord.start_time)).offset(skip).limit(limit))
    sessions = result.scalars().all()
    return [session_to_dict(session) for session in sessions]

@router.get("/sessions/{session_id}")
async def get_session(
    session_id: str, 
    skip: int = Query(0, ge=0), 
    limit: int = Query(1000, ge=1, le=5000), 
    db = Depends(get_db)
):
    result = await db.execute(select(SessionRecord).where(SessionRecord.id == session_id))
    session = result.scalars().first()
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
        
    logs = await db.execute(
        select(TelemetryLog)
        .where(TelemetryLog.session_id == session_id)
        .order_by(TelemetryLog.timestamp)
        .offset(skip)
        .limit(limit)
    )
    return {
        "session": session_to_dict(session),
        "logs": [telemetry_to_dict(log) for log in logs.scalars().all()]
    }

@router.get("/analytics/summary")
async def get_analytics_summary(db = Depends(get_db)):
    total_sessions = await db.scalar(select(func.count(SessionRecord.id)))
    avg_score = await db.scalar(select(func.avg(SessionRecord.avg_score)))
    total_predictions = await db.scalar(select(func.sum(SessionRecord.total_predictions)))
    
    anomalies = await db.execute(
        select(TelemetryLog)
        .where(TelemetryLog.is_anomaly == True)
        .order_by(desc(TelemetryLog.timestamp))
        .limit(50)
    )
    
    return {
        "total_sessions": total_sessions or 0,
        "avg_driver_score": avg_score or 100.0,
        "total_predictions": total_predictions or 0,
        "recent_anomalies": [telemetry_to_dict(log) for log in anomalies.scalars().all()]
    }
