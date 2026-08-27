"""
Shared pytest fixtures.

REST/DB tests use an isolated in-memory SQLite database via a
`get_db` dependency override, so they never touch the real
`dds_telemetry.db` file the running app uses.

This module also pins the math-library thread pools to 1 (below, before
numpy/sklearn are imported anywhere). SHAP's TreeExplainer runs via
`asyncio.to_thread` inside the websocket streaming loop; with the default
pools it oversubscribes and can deadlock in the worker thread, wedging the
TestClient portal on teardown (observed on CI and on a 1.9.x-sklearn venv).
One thread per pool removes the oversubscription and the test suite is not
perf-sensitive to it.

The ML artifacts (`best_model.pkl`, `scaler.pkl`, `label_encoder.pkl`,
`optimal_features.json`, `anomaly_model.pkl`, `anomaly_feature_bounds.json`)
are committed to the repo -- `test_inference.py` / `test_anomaly_detector.py`
exercise the *real* trained pipeline, not a mock, on every machine and on
CI. (An earlier `setup_ml_artifacts` fixture generated a 4-feature synthetic
mock and deleted the real files on teardown; it never matched what those
tests assert and has been removed.)
"""
import os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import pytest
import pytest_asyncio


def _testclient_ws_portal_hangs() -> bool:
    """Starlette 1.x's TestClient tears a long-lived websocket endpoint down
    through `anyio.start_blocking_portal` -> `thread.join()`, which never
    returns here (reproduced across fastapi / anyio / pytest-asyncio
    versions -- the only thing that fixes it is starlette < 1.0). The two
    websocket *streaming-loop* tests are skipped on that toolchain: it is a
    harness incompatibility, not an app bug. The app-side robustness they
    would exercise -- the loop is bounded by ``websockets.MAX_STREAM_TICKS``
    and every spawned task is cancelled+awaited in ``finally`` -- is in the
    code regardless, and the tests still run on a starlette 0.x env."""
    try:
        from importlib.metadata import version
        return int(version("starlette").split(".")[0]) >= 1
    except Exception:
        return False


ws_portal_skip = pytest.mark.skipif(
    _testclient_ws_portal_hangs(),
    reason="Starlette 1.x TestClient websocket portal teardown deadlocks (harness bug, not app)",
)
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
from httpx import AsyncClient, ASGITransport

from app.core import database as db_module


@pytest.fixture(autouse=True)
def _stub_live_routing(monkeypatch):
    """Neutralise the background OSRM route fetch in the websocket
    streaming tests.

    `app/api/websockets.py` kicks off `fetch_and_apply_route()` on connect,
    which calls the public OSRM demo API (router.project-osrm.org). On CI
    that request stalls and, with no historical per-test timeout, hung the
    whole job (a prior run sat 1h18m on the pytest step). Stub it to the
    documented "routing unavailable" branch (returns None -> straight-line
    fallback). `app/services/routing.py` itself stays covered directly by
    tests/test_routing.py, which patches httpx, not this symbol.
    """
    async def _no_route(*_args, **_kwargs):
        return None

    try:
        import app.api.websockets as _ws
        monkeypatch.setattr(_ws, "get_route", _no_route, raising=False)
    except Exception:
        pass


@pytest.fixture
def stub_inference(monkeypatch):
    """Make the websocket *streaming-loop* tests fast and self-terminating.

    Those tests check the loop plumbing + protocol v3 payload shape +
    command handling -- NOT the ML pipeline (covered by test_inference.py /
    test_explainability.py / test_anomaly_detector.py) and NOT open-ended
    streaming. This fixture:

    * swaps `pipeline.predict` for a fast deterministic stub (the real one
      runs SHAP's TreeExplainer every 10 Hz tick via asyncio.to_thread),
      and
    * caps `websockets.MAX_STREAM_TICKS` so the endpoint coroutine returns
      on its own -- the TestClient portal's teardown of a never-ending
      streaming loop deadlocks on some starlette/anyio builds.
    """
    from app.services import inference as _inf
    import app.api.websockets as _ws

    def _fast_predict(input_dict):
        clean = {f: float(input_dict.get(f, 0.0)) for f in (_inf.pipeline.features or [])}
        return {
            "prediction": "Maintain Speed",
            "confidence": 0.92,
            "confidence_dict": {"Accelerate": 0.04, "Maintain Speed": 0.92, "Decelerate": 0.04},
            "confidence_override": False,
            "shap_result": {"base_value": 0.0, "contributions": []},
            "anomaly_result": {"is_anomaly": False, "type": "NONE", "severity": "NONE", "message": ""},
            "clean_input": clean,
        }

    monkeypatch.setattr(_inf.pipeline, "predict", _fast_predict)
    monkeypatch.setattr(_ws, "MAX_STREAM_TICKS", 40, raising=False)
    return _fast_predict


@pytest_asyncio.fixture
async def test_db_session():
    engine = create_async_engine("sqlite+aiosqlite:///:memory:", echo=False)
    async with engine.begin() as conn:
        await conn.run_sync(db_module.Base.metadata.create_all)

    session_maker = sessionmaker(engine, class_=db_module.AsyncSession, expire_on_commit=False)

    async def override_get_db():
        async with session_maker() as session:
            yield session

    yield session_maker, override_get_db
    await engine.dispose()


@pytest_asyncio.fixture
async def api_client(test_db_session):
    """An httpx AsyncClient wired to the real FastAPI app, with the DB
    dependency swapped for an isolated in-memory SQLite instance."""
    from app.main import app
    from app.core.database import get_db

    _, override_get_db = test_db_session
    app.dependency_overrides[get_db] = override_get_db

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client

    app.dependency_overrides.clear()
