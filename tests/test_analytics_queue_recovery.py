import asyncio
import pytest
from app.services.simulation_session import SimulationSession, pipeline


@pytest.mark.asyncio
async def test_failed_analytics_reading_is_acknowledged_and_worker_continues(monkeypatch):
    session = SimulationSession(real_time=False)
    session.analytics_queue = asyncio.Queue()
    session._running = True
    calls = []
    monkeypatch.setattr(pipeline, 'is_ready', lambda: True)
    def fail(features):
        calls.append(features)
        raise RuntimeError('injected inference failure')
    monkeypatch.setattr(pipeline, 'predict', fail)
    await session.analytics_queue.put(({'sample':1},0))
    await session.analytics_queue.put(({'sample':2},1))
    worker = asyncio.create_task(session._analytics_worker())
    try:
        await asyncio.wait_for(session.analytics_queue.join(), 2)
        assert calls == [{'sample':1}, {'sample':2}]
    finally:
        session._running = False
        worker.cancel()
        await worker
