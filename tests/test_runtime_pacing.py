import asyncio
from types import SimpleNamespace

import pytest
import app.services.simulation_session as module


def test_worker_rechecks_deadline_after_dispatch_delay(monkeypatch):
    sleeps=[]
    monkeypatch.setattr(module,'time',SimpleNamespace(perf_counter=lambda:12.,sleep=sleeps.append))
    module.SimulationSession._wait_until(11.)
    assert sleeps==[]
    module.SimulationSession._wait_until(12.02)
    assert sleeps==[pytest.approx(.02)]


async def test_windows_wait_does_not_block_event_loop_and_is_cancellable(monkeypatch):
    import threading
    entered,release=threading.Event(),threading.Event()
    def wait(deadline):
        entered.set()
        assert release.wait(1.)
    session=module.SimulationSession(seed=42,real_time=False)
    monkeypatch.setattr(module,'sys',SimpleNamespace(platform='win32'))
    monkeypatch.setattr(session,'_wait_until',wait)
    task=asyncio.create_task(session._pace_until(0.))
    try:
        await asyncio.wait_for(asyncio.to_thread(entered.wait,1.),1.5)
        # Worker remains blocked, but coroutine dispatch and cancellation work.
        assert not task.done()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    finally:
        release.set()


async def test_other_platform_retains_async_deadline_wait(monkeypatch):
    session=module.SimulationSession(seed=42,real_time=False)
    monkeypatch.setattr(module,'sys',SimpleNamespace(platform='linux'))
    def forbidden(deadline):
        raise AssertionError('native Windows worker must not run')
    monkeypatch.setattr(session,'_wait_until',forbidden)
    await session._pace_until(module.time.perf_counter()-1.)


async def test_shutdown_finishes_writer_before_session_finalization(monkeypatch):
    import app.core.database as database
    entered,release=asyncio.Event(),asyncio.Event()
    order=[]
    async def create(session_id):
        pass
    async def finalize(*args):
        order.append('finalize')
    session=module.SimulationSession(seed=42,real_time=False)
    async def writer():
        entered.set()
        await release.wait()
        order.append('committed')
    monkeypatch.setattr(database,'create_stream_session',create)
    monkeypatch.setattr(database,'finalize_stream_session',finalize)
    monkeypatch.setattr(session,'_persistence_worker',writer)
    session.start()
    await asyncio.wait_for(entered.wait(),1.)
    session.stop()
    await asyncio.sleep(0)
    assert not session._main_task.done()
    release.set()
    await asyncio.wait_for(session._main_task,1.)
    assert order==['committed','finalize']


async def test_stopped_persistence_worker_drains_queued_records(monkeypatch):
    import app.core.database as database
    session=module.SimulationSession(seed=42,real_time=False)
    session.persistence_queue=asyncio.Queue()
    session._running=False
    for tick in range(3):
        session.persistence_queue.put_nowait((session.session_id,tick,{'tick':tick},
            {'decision':'Unavailable','confidence':0.,'driver_score':{}}))
    batches=[]
    async def persist(logs,scores):
        batches.append(([row.features['tick'] for row in logs],len(scores)))
    monkeypatch.setattr(database,'persist_stream_batch',persist)
    await asyncio.wait_for(session._persistence_worker(),1.)
    await asyncio.wait_for(session.persistence_queue.join(),.1)
    assert batches==[([0,1,2],0)]
    assert session.persistence_queue.empty()


async def test_actual_sqlite_shutdown_backlog_is_persisted_and_finalized(monkeypatch,tmp_path):
    import app.core.database as database
    from sqlalchemy.ext.asyncio import create_async_engine,async_sessionmaker
    from sqlalchemy import select,func
    engine=create_async_engine('sqlite+aiosqlite:///'+(tmp_path/'shutdown.db').as_posix())
    factory=async_sessionmaker(engine,expire_on_commit=False)
    monkeypatch.setattr(database,'AsyncSessionLocal',factory)
    try:
        async with engine.begin() as connection:
            await connection.run_sync(database.Base.metadata.create_all)
        session=module.SimulationSession(seed=42,real_time=False)
        await database.create_stream_session(session.session_id)
        session._running=False
        session.persistence_queue=asyncio.Queue()
        for tick in range(3):
            session.persistence_queue.put_nowait((session.session_id,tick,{'tick':tick},
                {'decision':'Maintain Speed','confidence':.7,
                 'driver_score':{'score':80.,'rating':'B','breakdown':{}}}))
        await session._persistence_worker()
        await database.finalize_stream_session(session.session_id,'completed',3,240.)
        async with factory() as connection:
            assert await connection.scalar(select(func.count()).select_from(database.TelemetryLog))==3
            assert await connection.scalar(select(func.count()).select_from(database.DriverScoreLog))==3
            assert (await connection.get(database.SessionRecord,session.session_id)).status=='completed'
    finally:
        await engine.dispose()
