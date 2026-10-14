from app.services.executor import MultiRateExecutor


def test_stage_distribution_explicitly_labels_rolling_window():
    executor = MultiRateExecutor()
    executor.add_stage("control", 50, lambda clock: None)
    metric = executor.metrics["control"]
    metric["count"] = 3000
    metric["deadline_misses"] = 2
    metric["durations_ms"].extend(range(3000))
    summary = executor.timing_summary()["control"]
    assert summary["count"] == 3000
    assert summary["sample_count"] == summary["window_capacity"] == 2000
    assert summary["percentile_scope"] == "last_2000_stage_calls"
    assert summary["p50_ms"] == 2000
    assert summary["p95_ms"] == 2900
    assert summary["p99_ms"] == 2980
    assert summary["max_ms"] == 2999
    assert summary["deadline_misses"] == 2


async def test_worker_queue_overflow_accounts_for_evicted_items():
    import asyncio
    from app.services.simulation_session import SimulationSession
    session = SimulationSession(seed=42, real_time=False)
    session.analytics_queue = asyncio.Queue(maxsize=1)
    session.persistence_queue = asyncio.Queue(maxsize=1)
    session._emit_telemetry_snapshot()
    session.executor.step()
    session._emit_telemetry_snapshot()
    for queue in (session.analytics_queue, session.persistence_queue):
        assert queue.qsize() == 1
        queue.get_nowait()
        queue.task_done()
        await asyncio.wait_for(queue.join(), timeout=.1)
    summary = session.executor.timing_summary()["telemetry_build_dispatch"]
    assert summary["count"] == 2 and summary["sample_count"] == 2
