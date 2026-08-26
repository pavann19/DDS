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
                # The route-fetch task and the first tick are both scheduled
                # immediately on connect and race; a "route" message can
                # legitimately arrive before, after, or (if OSRM is
                # unreachable in this environment) never. Drain messages
                # until a real "state" tick shows up rather than assuming
                # it's always first.
                payload = _receive_until_state(websocket)

                # V2 protocol (app/api/websockets.py) -- see
                # frontend/src/types/protocol.ts for the TypeScript mirror
                # of this shape.
                assert payload["protocol_version"] == "2.0"
                assert payload["type"] == "state"
                ego = payload["data"]["ego"]
                assert ego["decision"] in {"Accelerate", "Decelerate", "Maintain Speed"}
                assert "confidence" in ego
                assert "frenet" in ego

                # Regression check: SHAP/anomaly/driver-score/planner-
                # candidates were being computed every tick and logged to
                # the DB, but never placed in the WS payload itself -- so
                # they reached SQLite but no connected client. Restored as
                # `data` siblings; this pins that they stay there.
                assert "shap" in payload["data"]
                assert "anomaly" in payload["data"]
                assert payload["data"]["driver_score"]["rating"] in {"A+", "A", "B", "C", "D", "F"}
                # Not asserted non-empty: candidates only exist once a real
                # route has been fetched (async, races the first tick), so
                # an empty list on an early message is legitimate, not a bug.
                assert isinstance(payload["data"]["planner"]["candidates"], list)

                # Safety Shield: an independent verdict, distinct from the
                # planner/IDM decision above.
                shield = payload["data"]["safety_shield"]
                assert isinstance(shield["approved"], bool)
                assert shield["risk_level"] in {"NONE", "LOW", "MEDIUM", "HIGH", "CRITICAL"}

                websocket.send_json({"type": "set_destination", "lat": 37.8, "lng": -122.4})
                second_payload = _receive_until_state(websocket)
                assert second_payload["type"] == "state"
    finally:
        db_module.AsyncSessionLocal = original_session_local
        ws_module.AsyncSessionLocal = original_session_local
        asyncio.run(test_engine.dispose())


async def _create_schema(engine):
    async with engine.begin() as conn:
        await conn.run_sync(db_module.Base.metadata.create_all)


def _receive_until_state(websocket, max_messages=5):
    """The route-fetch task and the first tick race on connect, so the
    next message off the socket is not always a "state" tick -- it can be
    a one-time "route" message instead. Drain until a real tick shows up."""
    for _ in range(max_messages):
        payload = websocket.receive_json()
        if payload.get("type") == "state":
            return payload
    raise AssertionError(f"no 'state' message received in {max_messages} messages")
