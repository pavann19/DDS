"""
Shared pytest fixtures.

REST/DB tests use an isolated in-memory SQLite database via a
`get_db` dependency override, so they never touch the real
`dds_telemetry.db` file the running app uses.

The ML artifacts (`best_model.pkl`, `scaler.pkl`, `label_encoder.pkl`,
`optimal_features.json`, `anomaly_model.pkl`, `anomaly_feature_bounds.json`)
are committed to the repo -- `test_inference.py` / `test_anomaly_detector.py`
exercise the *real* trained pipeline, not a mock, on every machine and on
CI. (An earlier `setup_ml_artifacts` fixture generated a 4-feature synthetic
mock and deleted the real files on teardown; it never matched what those
tests assert and has been removed.)
"""
import pytest
import pytest_asyncio
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
