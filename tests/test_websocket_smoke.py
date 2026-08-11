"""
Smoke test for the /ws/telemetry streaming endpoint (app/api/websockets.py).

This is deliberately a single "does the real streaming loop wire up
end-to-end" check, not a full behavioral suite -- the loop runs on a live
10Hz `asyncio.sleep` tick against the real inference pipeline and DB layer,
which is expensive and timing-sensitive to test exhaustively. It uses the
real trained ML artifacts (best_model.pkl etc.) and an isolated in-memory
DB, matching the rest of the suite's "test against the real pipeline, not
a mock" approach.
"""
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker

from app.core import database as db_module


def test_websocket_streams_a_valid_telemetry_payload(tmp_path):
    db_path = tmp_path / "ws_test.db"
    test_engine = create_async_engine(f"sqlite+aiosqlite:///{db_path}", echo=False)
    test_session_maker = sessionmaker(test_engine, class_=db_module.AsyncSession, expire_on_commit=False)

    import asyncio
    asyncio.run(_create_schema(test_engine))

    # websockets.py imports AsyncSessionLocal directly (not via FastAPI DI),
    # so redirect the module-level session factory for the duration of the test.
    original_session_local = db_module.AsyncSessionLocal
    db_module.AsyncSessionLocal = test_session_maker
    import app.api.websockets as ws_module
    ws_module.AsyncSessionLocal = test_session_maker

    from app.main import app

    try:
        with TestClient(app) as client:
            with client.websocket_connect("/ws/telemetry") as websocket:
                payload = websocket.receive_json()

                assert "telemetry" in payload
                assert "navigation" in payload
                assert payload["predicted_decision"] in {"Accelerate", "Decelerate", "Maintain Speed"}
                assert "confidence" in payload
                assert "driver_score" in payload
                assert payload["driver_score"]["rating"] in {"A+", "A", "B", "C", "D", "F"}

                websocket.send_json({"type": "set_destination", "lat": 37.8, "lng": -122.4})
                second_payload = websocket.receive_json()
                assert "navigation" in second_payload
    finally:
        db_module.AsyncSessionLocal = original_session_local
        ws_module.AsyncSessionLocal = original_session_local
        asyncio.run(test_engine.dispose())


async def _create_schema(engine):
    async with engine.begin() as conn:
        await conn.run_sync(db_module.Base.metadata.create_all)
