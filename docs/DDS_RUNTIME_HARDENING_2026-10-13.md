# Runtime hardening evidence

Baseline: `42aff5ef6d4d59aca635b4ed1cc7ed54da952fcf`. Isolated `DDS_Runtime_Sandbox`, local branch `fix/runtime-observability-20261013`.

The recorded backend log contains `ConnectionResetError: [WinError 10054]` in Python 3.13 `asyncio.proactor_events._ProactorBasePipeTransport._call_connection_lost`, at socket shutdown. The installed implementation then misses close/server-detachment. A Windows Proactor lifespan handler completes only that exact callback/source-operation/error case. It does not repeat protocol notification, hides no chained protocol exception, delegates unrelated exceptions, and restores the previous handler if it still owns the slot. This deliberately depends on the verified private stdlib implementation; a changed callback/source line delegates to normal error reporting rather than guessing. Protocol, cleanup and other socket failures stay visible.

Telemetry worker queue eviction now calls task_done for each removed item. Bounded latest-item behavior is unchanged. Regression tests show join completes after the retained item is processed.

Timing metrics expose p50/p95, retained sample count and a last-2000-call scope. Existing count and deadline_misses remain lifetime counters. Added telemetry_build_dispatch includes payload building and queue publication, excluding socket serialization/transmission. Measurement cannot influence driving commands or ML authority.

## Measured load

Actual seeded mapped_parking Session at realtime pacing, real analytics (ML ready) and database workers, simulated vehicle/sensors, with concurrent in-process JSON consumers. Two 30-second runs. Raw distributions are the neighboring JSON artifacts. These are neither browser painted FPS nor TCP/network latency measurements. JSON serialization is measured separately from build/dispatch. Scheduling lateness is polled every 20ms and may repeat the latest chunk sample.

| Consumers | Control p99 / max ms | Telemetry p99 / max ms | Scheduling p99 / max ms | RTF |
|---|---|---|---|---|
|4|12.270 /15.751|1.054 /2.955|20.250 /105.070|1.00060|
|8|13.100 /19.597|1.393 /3.493|20.094 /26.039|1.00020|

Both runs: 1501 control calls, 300 planning calls, 300 telemetry builds; no observed stage-budget misses. This finite observation is not a success-rate or hard realtime guarantee. The 8-consumer control maximum approaches its 20ms budget. Good average RTF does not eliminate scheduling jitter. The 105ms outlier in the first run was not timestamp-localized; its cause remains unverified. Later evaluator records ten worst sample simulation times.

## Validation

51 tests passed: socket cleanup, runtime distributions/overflow, executor, SimulationSession, engineering runtime. Actual Windows Proactor event-loop callback recovery is tested with an injected shutdown failure, not a claim of reproducing an external TCP reset in a browser. Existing actual stdlib callback reproduces missing close/detach before recovery. Parent integration will run full regression and browser observation separately.

Reproduce: main venv Python, sandbox cwd, `scripts/evaluate_runtime_load.py --seconds 30 --consumers 8 --output output/runtime-load-8.json`. Database is sandbox-local, not main. No public service, push, PR or deployment performed.
