"""
Shared pytest fixtures.

REST/DB tests use an isolated in-memory SQLite database via a
`get_db` dependency override, so they never touch the real
`dds_telemetry.db` file the running app uses.
"""
import pytest
import pytest_asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
from httpx import AsyncClient, ASGITransport

from app.core import database as db_module


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
