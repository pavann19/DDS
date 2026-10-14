"""Actual realtime session + analytics/database workers, with JSON consumer load.

No browser, TCP transport or painted-frame measurement is inferred from this run.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import time
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(name, "1")


def distribution(values):
    values = sorted(values)
    return {"count": len(values), **{key: values[min(len(values)-1, int(len(values)*q))] if values else 0.
        for key, q in (("p50_ms", .5), ("p95_ms", .95), ("p99_ms", .99), ("max_ms", 1.))}}


async def evaluate(seconds, consumers, pacing='native', database_scope='application_configured'):
    from app.core.database import init_db, engine, AsyncSessionLocal, SessionRecord, TelemetryLog, DriverScoreLog
    from sqlalchemy import select, func
    from app.services.simulation_session import SimulationSession, pipeline
    await init_db()
    session = SimulationSession(seed=42, real_time=True)
    if pacing == 'legacy_asyncio':
        async def legacy_pace(deadline):
            await asyncio.sleep(max(0., deadline - time.perf_counter()))
        session._pace_until = legacy_pace
    session.scenario_engine.load_scenario("mapped_parking", session.world.physics)
    serialization, lateness, lateness_samples = [], [], []
    received = [0] * consumers
    async def consume(index, queue):
        while True:
            message = await queue.get()
            started = time.perf_counter()
            json.dumps(message, allow_nan=False)
            serialization.append((time.perf_counter()-started)*1000)
            received[index] += 1
            queue.task_done()
    clients = []
    for index in range(consumers):
        _, queue = session.subscribe(str(index))
        clients.append(asyncio.create_task(consume(index, queue)))
    session.start()
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        await asyncio.sleep(.02)
        lateness.append(session.timing.scheduling_lateness_ms)
        lateness_samples.append({"sim_time_s": session.executor.clock.sim_time_s,
            "lateness_ms": session.timing.scheduling_lateness_ms})
    session.stop()
    await session._main_task
    for task in clients:
        task.cancel()
    await asyncio.gather(*clients, return_exceptions=True)
    async with AsyncSessionLocal() as database:
        persisted = {}
        for name, model in (('telemetry_rows',TelemetryLog),('score_rows',DriverScoreLog)):
            persisted[name] = await database.scalar(select(func.count()).select_from(model).where(model.session_id==session.session_id))
        record = await database.get(SessionRecord,session.session_id)
        persisted['session_status'] = record.status if record else None
    result = {"scope": "actual realtime mapped session; simulated vehicle; in-process JSON consumers",
        "seconds_requested": seconds, "consumers": consumers, "messages_received": received,
        "pacing": pacing,
        "database_scope": database_scope,
        "persistence": persisted,
        "ml_ready": pipeline.is_ready(), "sim_time_s": session.executor.clock.sim_time_s,
        "real_time_factor": session.timing.real_time_factor,
        "stages": session.executor.timing_summary(),
        "serialization": distribution(serialization), "scheduling_lateness": distribution(lateness),
        "largest_lateness_samples": sorted(lateness_samples, key=lambda row: row["lateness_ms"], reverse=True)[:10]}
    await engine.dispose()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--consumers", type=int, default=4)
    parser.add_argument("--output", required=True)
    parser.add_argument('--pacing', choices=('native','legacy_asyncio'), default='native')
    args = parser.parse_args()
    # Configure before importing application settings/engine: never write the
    # live server or another worktree's database during a benchmark.
    with tempfile.TemporaryDirectory(prefix='dds-runtime-evaluation-') as directory:
        database = (Path(directory) / 'telemetry.db').as_posix()
        os.environ['DATABASE_URL'] = 'sqlite+aiosqlite:///' + database
        result = asyncio.run(evaluate(args.seconds, args.consumers, args.pacing,
            'isolated_temporary_SQLite_for_this_evaluator_process'))
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
