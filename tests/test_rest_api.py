"""
Integration tests for app/api/rest.py, run against the real FastAPI app
with the DB dependency swapped for an isolated in-memory SQLite instance
(see tests/conftest.py) so nothing touches dds_telemetry.db.
"""
import pytest
from datetime import datetime

from app.core.database import SessionRecord, TelemetryLog


async def _seed_session(test_db_session, session_id="test-session-1", **overrides):
    session_maker, _ = test_db_session
    async with session_maker() as db:
        record = SessionRecord(
            id=session_id,
            start_time=datetime.utcnow(),
            total_predictions=overrides.get("total_predictions", 5),
            avg_score=overrides.get("avg_score", 87.5),
            status=overrides.get("status", "completed"),
        )
        db.add(record)
        await db.commit()
    return session_id


async def test_health_endpoint_reports_model_loaded(api_client):
    resp = await api_client.get("/api/health")
    assert resp.status_code == 200
    body = resp.json()
    assert body["model_loaded"] is True
    assert body["ready_for_streaming"] is True
    assert body["errors"] == []


async def test_metrics_endpoint_returns_json_or_documented_error(api_client):
    resp = await api_client.get("/api/metrics")
    assert resp.status_code == 200
    body = resp.json()
    assert isinstance(body, dict)


async def test_list_sessions_empty_by_default(api_client):
    resp = await api_client.get("/api/sessions")
    assert resp.status_code == 200
    assert resp.json() == []


async def test_list_sessions_returns_seeded_session(api_client, test_db_session):
    await _seed_session(test_db_session, session_id="s1")
    resp = await api_client.get("/api/sessions")
    assert resp.status_code == 200
    body = resp.json()
    assert len(body) == 1
    assert body[0]["id"] == "s1"
    assert body[0]["avg_score"] == 87.5


async def test_get_session_not_found_returns_404(api_client):
    resp = await api_client.get("/api/sessions/does-not-exist")
    assert resp.status_code == 404


async def test_get_session_returns_session_and_logs(api_client, test_db_session):
    session_id = await _seed_session(test_db_session, session_id="s2")
    session_maker, _ = test_db_session
    async with session_maker() as db:
        db.add(TelemetryLog(
            session_id=session_id, timestamp=1.0, features={"RPM": 2000},
            prediction="Maintain Speed", confidence=0.9, is_anomaly=False,
        ))
        await db.commit()

    resp = await api_client.get(f"/api/sessions/{session_id}")
    assert resp.status_code == 200
    body = resp.json()
    assert body["session"]["id"] == session_id
    assert len(body["logs"]) == 1
    assert body["logs"][0]["prediction"] == "Maintain Speed"


async def test_analytics_summary_with_no_data_returns_sane_defaults(api_client):
    resp = await api_client.get("/api/analytics/summary")
    assert resp.status_code == 200
    body = resp.json()
    assert body["total_sessions"] == 0
    assert body["total_predictions"] == 0
    assert body["recent_anomalies"] == []


async def test_analytics_summary_aggregates_seeded_sessions(api_client, test_db_session):
    await _seed_session(test_db_session, session_id="s3", total_predictions=10, avg_score=80.0)
    await _seed_session(test_db_session, session_id="s4", total_predictions=20, avg_score=90.0)

    resp = await api_client.get("/api/analytics/summary")
    body = resp.json()
    assert body["total_sessions"] == 2
    assert body["total_predictions"] == 30
    assert body["avg_driver_score"] == pytest.approx(85.0)


async def test_sessions_pagination_limit_is_enforced(api_client):
    resp = await api_client.get("/api/sessions?limit=101")
    assert resp.status_code == 422
