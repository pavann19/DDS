import asyncio
import pytest
from sqlalchemy import select,func
from app.core import database
from app.services.simulation_session import SimulationSession


async def test_sqlite_missing_scores_remain_unknown_and_average_uses_rows(test_db_session,monkeypatch):
    maker,_=test_db_session
    monkeypatch.setattr(database,'AsyncSessionLocal',maker)
    session=SimulationSession(seed=42,real_time=False)
    session._running=False
    session.persistence_queue=asyncio.Queue()
    await database.create_stream_session(session.session_id)
    for score in ({},{'score':0.,'rating':'F'},{'score':80.,'rating':'B'}):
        session.persistence_queue.put_nowait((session.session_id,1.,{},
            {'decision':'Unavailable','confidence':0.,'driver_score':score}))
    await session._persistence_worker()
    await database.finalize_stream_session(session.session_id,'completed',999,99900.)
    async with maker() as db:
        row=await db.get(database.SessionRecord,session.session_id)
        assert row.avg_score==40.
        assert row.total_predictions==3
        assert await db.scalar(select(func.count()).select_from(database.DriverScoreLog))==2


async def test_batch_failure_propagates_from_actual_sqlite(test_db_session,monkeypatch):
    maker,_=test_db_session
    monkeypatch.setattr(database,'AsyncSessionLocal',maker)
    await database.create_stream_session('duplicate')
    with pytest.raises(Exception):
        await database.persist_stream_batch([database.SessionRecord(id='duplicate')],[])


async def test_failure_is_retained_and_session_finalizes_failed(monkeypatch):
    session=SimulationSession(seed=42,real_time=False)
    entered=asyncio.Event()
    finalized=[]
    async def create(*args):pass
    async def fail(*args):
        entered.set()
        raise RuntimeError('injected disk failure')
    async def finalize(*args):finalized.append(args[1])
    monkeypatch.setattr(database,'create_stream_session',create)
    monkeypatch.setattr(database,'persist_stream_batch',fail)
    monkeypatch.setattr(database,'finalize_stream_session',finalize)
    session.start()
    await asyncio.wait_for(entered.wait(),2.)
    session.stop()
    await asyncio.wait_for(session._main_task,2.)
    assert session.persistence_failed
    assert finalized==['persistence_failed']


async def test_sqlite_worker_write_failure_records_failed_session(test_db_session,monkeypatch):
    maker,_=test_db_session
    monkeypatch.setattr(database,'AsyncSessionLocal',maker)
    session=SimulationSession(seed=42,real_time=False)
    session._running=False
    session.persistence_queue=asyncio.Queue()
    await database.create_stream_session(session.session_id)
    session.persistence_queue.put_nowait((session.session_id,1.,{},{}))
    actual=database.persist_stream_batch
    async def violate_unique(logs,scores):
        await actual([database.SessionRecord(id=session.session_id)],[])
    monkeypatch.setattr(database,'persist_stream_batch',violate_unique)
    await session._persistence_worker()
    assert session.persistence_failed
    assert await database.finalize_stream_session(session.session_id,'persistence_failed')
    async with maker() as db:
        row=await db.get(database.SessionRecord,session.session_id)
        assert row.status=='persistence_failed'
        assert row.avg_score is None and row.total_predictions==0
