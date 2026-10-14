# Windows runtime pacing and writer shutdown

Baseline `51f76b74e60a973aa7217317e5602b54b5b9f54d`.
Sandbox `DDS_Next_Sandbox`, local-only changes. Fixed-step simulation,
controller rates, physical dynamics and analytic-only ML roles remain intact.

Windows deadline pacing now runs native `time.sleep` in a worker, rechecking
the absolute deadline after dispatch. Other platforms retain asynchronous
sleep. No busy spinning, process-wide timer resolution change or physics
threading is introduced. This uses the default asyncio thread pool: heavy
worker contention can still cause lateness. Cancellation does not interrupt
a running native wait, ordinarily at most the next 20 ms control deadline.

Each actual realtime chunk records scheduler lateness. The original evaluator
polled the latest reading through another asyncio timer, giving biased and
unequal samples. Exact per-chunk samples replace that comparison oracle;
polling diagnostics remain present but must not be substituted for it.

The evaluator CLI creates a unique temporary SQLite database before importing
application settings. It reports persisted row counts and final session
status. It supports `--pacing legacy_asyncio` for a same-source comparison.

An isolated legacy run exposed shutdown SQLite lock and Windows open-file
cleanup errors. Session teardown formerly cancelled a possible in-flight
commit, then retried pending writes/finalization. Teardown now waits for the
single persistence worker to finish and flush before finalization. Its idle
queue timeout is one second. A regression holds the writer through stop and
asserts commit finishes before finalization. Shutdown drains already-queued
records; an isolated actual SQLite regression verifies the three queued
telemetry/score records and completed session. No blanket database retries or
error suppression were added. Contaminated/failed runs were excluded.

Measured clean sequential 20 s runs, four in-process JSON consumers, actual
mapped session plus analytics and persistence workers, Python 3.13/Windows:

| Per-chunk lateness | Legacy asyncio | Native wait |
|---|---:|---:|
| samples | 1000 | 1001 |
| p50 ms | 8.3755 | 0.7475 |
| p95 ms | 19.1737 | 15.4917 |
| p99 ms | 72.3057 | 28.9477 |
| maximum ms | 147.3786 | 156.1550 |
| samples exceeding 20 ms | 42 | 20 |

A further clean legacy repeat measured 1001 chunks, p50 7.8387 ms,
p95 16.6358 ms, p99 22.4548 ms, maximum 121.6212 ms, 24 lateness misses.
Thus native p50/p95 improved in these observations, while its p99 and maximum
were worse than this repeat. Tail reduction is not consistent or guaranteed.
The misses above are **scheduler lateness over 20 ms**, not control-stage
execution-duration budget overruns.

Both delivered 201 messages per consumer with real-time factor approximately
1.000. Native verified 200 telemetry rows, 200 score rows, completed session,
clean exit and temporary database removal. These are observations of two
runs, not a general performance guarantee. Worst latency remains substantial;
this is not hard realtime, TCP/browser frame timing or production certification.
Artifacts: `output/runtime-clean-legacy20.json`,
`output/runtime-clean-native20.json`. Further final legacy repeat includes
post-shutdown persistence row verification: 200 telemetry, 200 score rows,
completed session, clean exit and temporary database removal.

48 focused executor/session/runtime tests passed after the additive
`grip_budget` telemetry passthrough requested for combined integration; the
subsequent actual SQLite regression also passed (six runtime pacing tests).
That passthrough preserves absence as null. Combined physical/browser
acceptance remains a separate integration check.

Discovered and reported separately: before analytics readiness, persistence
currently substitutes score 100/rating A+ for an empty driver score and final
average defaults to 100. Normal initial confidence is explicitly 0; the .85
fallback applies only to absent confidence keys. This is not fixed in the
pacing/lifecycle increment; no model-output validity claim follows from row
counts. Root owns the reviewed follow-up.

Primary documentation:
[Python time.sleep](https://docs.python.org/3.13/library/time.html#time.sleep),
[asyncio.to_thread](https://docs.python.org/3.13/library/asyncio-task.html#asyncio.to_thread).
