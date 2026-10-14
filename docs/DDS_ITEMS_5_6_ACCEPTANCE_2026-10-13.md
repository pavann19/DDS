# DDS items 5–6 acceptance — 13 October 2026

Scope: measured UI/runtime smoothness and Windows socket cleanup (#5); better evidence-aware road/parking understanding (#6). The project remains a real-road simulator with explicitly simulated vehicle, sensors, traffic and vacancy. No ML driving policy was introduced; internal-reading analytics remain separate from Driver/planner/Shield authority.

## Preserved version and local provenance

Baseline and original59-commit review head: `42aff5ef6d4d59aca635b4ed1cc7ed54da952fcf`. Preserved tag `dds-local-items5-6-baseline-20261013` and verified complete-history bundle `output/dds-items5-6-baseline-20261013.bundle`.

Integrated code: `6e40b78ee37c04e7be0f2ffe160a190640f645be`. All work was first implemented/tested in separate Runtime, Map and UI Runtime worktrees, then locally fast-forwarded. Independent review found narrow follow-ups; those were fixed before acceptance. No push, PR or deployment occurred.

Original59 summaries and full SHA ledger: `output/DDS_UNPUSHED_59_2026-10-13.md`. Later #5/#6 commits are separate from that publishing-review snapshot.

Full backend collection began at `483c313ec9b2d43e4e8414835b91919f23b5dad3`; its app/scripts/tests Git content is identical to the integrated code. Subsequent commits changed frontend only. Main's128 tracked source/data files equal the sandbox after CRLF/LF normalization; raw and normalized hashes remain in `output/items56-main-source-manifest.json`.

## Implemented

- A Windows Proactor lifespan handler completes cleanup only for the verified stdlib socket-shutdown WinError10054 failure. Exact callback/class/source-line/error guards, no repeated protocol notification, unrelated/chained-error delegation, once-only cleanup and ownership-safe restoration are tested. This intentionally depends on the inspected private Python implementation; a changed implementation delegates to normal reporting.
- Evicted background queue items now call task_done, preserving latest-item behavior while preventing unfinished-task accounting from accumulating. Join/drain regression passes.
- Stage metrics expose p50/p95/p99, retained sample count, last2000-call scope and lifetime count/deadline counters. Telemetry build/dispatch is measured separately; socket serialization/transmission is outside that stage.
- Optional `?ddsMetrics=1` frontend capture measures worker parsing, worker-to-main queue, store application, first scene callback, callback cadence and supported long tasks. Capacity2048; malformed/negative samples are counted, not clamped into good measurements. Hidden-tab waiting is excluded from scene-latency samples. Diagnostics never supply driving inputs. Normal URLs do not enable collection.
- The telemetry hook subscribes to stable actions instead of the entire store, avoiding its previous page-wide packet subscription. Driving data subscriptions remain in their actual consumers.
- Road/parking telemetry adds dated recorded width/version/tag evidence without deriving lane width or changing driving geometry. Vehicle details show Unknown/Modeled/Not recorded explicitly. No parking bay, capacity, permission or vacancy is invented.
- Future imports retain vehicle/motorcar access and conditional tags, including conditional one-way. Parking assembly blocks restrictive/unresolved access and contrary/unresolved aisle direction. Eligibility is explicitly simulator policy, not actual permission. This is not a full OSM hierarchy or conditional-expression resolver.

## Validation

Combined backend: **872 passed, 3 skipped, 61 warnings,336.91 seconds** (`output/items56-full.txt`). Skips remain unvalidated scope. Focused runtime51 tests and final map44 tests passed before integration, including a complete actual simulated parking mission and held stop. Runtime diagnostics unit check, frontend TypeScript, UI contract, sensor/turn status checks and production builds passed. Main production build passed after the final tab-layout change. Lint:zero errors/five existing warnings.

A temporary Windows decoding regression corrupted two arrow glyphs while preparing instrumentation. Git UTF8 was restored and only narrow hunks reapplied before integration. Review and actual detailed-overlay DOM showed correct ▲/▼. The first Road layout wrapped the fifth tab; the final CSS and browser bounding-box check keep all five tabs in one row.

## Measured runtime workload

Isolated runtime sandbox, actual realtime mapped Session with real analytics/database workers and in-process JSON consumers,30-second runs:

| Consumers | Control p99 / maximum, ms | Telemetry build p99 / maximum, ms | Scheduling p99 / maximum, ms | RTF |
|---|---|---|---|---|
|4|12.270 /15.751|1.054 /2.955|20.250 /105.070|1.00060|
|8|13.100 /19.597|1.393 /3.493|20.094 /26.039|1.00020|

These are isolated runtime-candidate observations before combined map/UI integration, not TCP or display latency. Counts1501 control/300 planning/300 telemetry calls per run. Eight-consumer control approached its20ms budget. The105ms scheduling outlier in the first run was not timestamp-localized; its cause is unverified. No startup attribution is inferred. Raw scope/timing data: `docs/DDS_RUNTIME_LOAD_{4,8}_2026-10-13.json`; details: `docs/DDS_RUNTIME_HARDENING_2026-10-13.md`.

## Real browser and TCP evidence

Headed Chromium, localhost3000/8000. Warmed10-second `traffic_overtake`, high density yielding15 actual simulated actors, five auxiliary real WebSockets plus the UI socket. Tracing captures compositor DrawFrame and Chrome frame-reporter states, alongside animation callback intervals. Native window state matters; document.visibilityState alone was insufficient.

| Foreground sample,1601×812 | Compositor draw cadence, draws/s | Animation interval p95 / maximum, ms |
|---|---|---|
|Baseline frontend, normal|53.76|33.4 /50.2|
|Candidate, normal, diagnostics enabled|58.92|16.8 /33.6|
|Baseline frontend, detailed overlays|39.21|50.1 /83.4|
|Candidate, detailed overlays, diagnostics enabled|45.23|50.0 /66.8|

The matched-viewport baseline frontend used the same candidate backend and scenario workload. These are single finite sequential runs, with cache/load/order effects not fully controlled; no guaranteed or causal FPS gain is inferred. Original smaller-viewport baseline was59.64draws/s. Detailed mode retains a performance gap. Chrome-reported presentation states and compositor cadence do not independently certify physical display/GPU completion.

Two earlier candidate samples recorded ~1Hz callbacks/all dropped Chrome frame reports. CDP then confirmed the native window was minimized although document.visibilityState reported visible. Those raw samples are retained as minimized-window evidence and excluded from foreground UI acceptance. The benchmark now rejects an initially minimized window. The restored foreground samples are recorded separately; results were not silently replaced.

Foreground normal instrumented sample: worker parse p95/max0.400/0.500ms; worker-to-main queue3.000/5.900ms; main store0.200/0.300ms; worker receipt to first scene callback35.400/40.600ms. Scene callback p95/max39.600/47.900ms. Same-host server-stamp-to-worker wall comparison p95/max10/24ms; this requires the declared shared host clock and is not calibrated cross-host one-way network latency. No invalid metric samples or superseded pose states in this sample. The callback does not establish completed interpolation or GPU presentation.

Detailed mode: one58ms main-thread long task; scene callback p95/max59.5/79.2ms; worker-to-scene callback60.5/79.1ms. This is not continuous60FPS or a universal smoothness claim. A separate earlier live capture recorded one control-budget miss at20.3533ms during concurrent regression work; it remains in evidence despite passing tests and good average RTF.

Artifacts: `output/items56-{candidate-foreground,candidate-detail,matched-baseline-normal,matched-baseline-detail}-browser-audit.txt`. Minimized captures: `items56-candidate-{default,instrumented}-browser-audit.txt`. Native bounds restoration was observed in the tool result of `browser-items56-window-check.js`.

An actual target Proactor recovery was logged during sandbox preview reconnect (`output/items56-candidate-backend.stderr.log`,10:58:07). Eight subsequent raw WebSocket handshakes used Windows zero-linger TCP resets; a healthy subscription and ready sensors followed (`output/items56-tcp-reset.json`). Do not attribute the earlier recovery log to those later resets. Isolated real Proactor event-loop tests inject the exact shutdown failure and verify close/detach/delegation; they are not substituted for the live TCP observations.

Main restart: actual Road/parking panel and metadata verified, with unknown width, modeled lanes, absent recorded bays/capacity/access and simulated sensing source. All five tabs fit one row. Offline browser hid current readings/disabled destination controls, then recovered to the paused drive. `output/items56-main-map-audit.txt`, `items56-main-map-layout-log.txt`, `items56-main-reconnect-audit.txt`; screenshots `output/playwright/items56-road-parking-refined.png`, `items56-main-offline.png`. Browser observed zero application console errors and one existing Three.Clock warning in these checks.

## Road/parking data boundary and remaining work

The retained snapshot has3127 features,471 lane tags, one explicit21m road-width tag and zero retained individual parking-space features. Original XML is absent: filtered legacy tags and any unretained bay features cannot be recovered or claimed inspected. The selected lot/aisle have no width/capacity/access tags; selected aisle has no one-way tag and the reverse traversal remains eligible only within simulator policy. Recorded contributor tags are not surveyed geometry, live permission or live vacancy. A general restriction with a narrower allowance remains conservatively blocked; complete precedence/conditions are unresolved. See `docs/DDS_MAP_EVIDENCE_2026-10-13.md` and its primary OSM documentation links.

Detailed-overlay rendering, longer/repeated latency trials, hard scheduling deadlines, independently measured server-to-physical-display delay and broader hardware/load testing remain open. Road/lane widths and parking vacancy remain modeled or unverified; no recorded bay was invented. Prior aggregate friction/tire-force and continuous Cartesian swept-path limitations remain unchanged. Production readiness and Level5 are not established.

Backend/frontend run from the main workspace; mapped drive remains paused for inspection. Diagnostics are disabled in the normal final URL. No publishing action follows this record without user confirmation.

## Local implementation commits

- `a869c600723b5f7a26e63df53bd6eb5e5a6cb770` — feat(map): expose recorded road and parking evidence with conservative access checks
- `e419011e10a0eb948c43ec6e26e80e712977ab1a` — Harden Proactor peer-reset cleanup and expose bounded runtime timing
- `483c313ec9b2d43e4e8414835b91919f23b5dad3` — fix(map): reject unresolved conditional parking aisle directions
- `9771ab6c71774ecf23c468b22175a2d793762049` — feat(diagnostics): measure opt-in telemetry to scene timing and avoid root render subscriptions
- `d42b87d5d847ae24b89195e22a260412bac35c39` — feat(hmi): expose recorded road and parking evidence without inferred bays
- `6e40b78ee37c04e7be0f2ffe160a190640f645be` — fix(hmi): keep all vehicle detail tabs on one row
