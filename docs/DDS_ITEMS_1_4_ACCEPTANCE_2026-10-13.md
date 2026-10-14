# DDS items 1–4 acceptance — 12–13 October 2026

Latest critical turn, camera, mapped-world and signal fixes: [14 October local acceptance](DDS_CRITICAL_FIX_ACCEPTANCE_2026-10-14.md). Historical results below remain scoped to their recorded revisions.

Later integration: [Unified driving environment and required camera acceptance](DDS_UNIFIED_DRIVING_ACCEPTANCE_2026-10-13.md). This historical batch's measurements remain scoped to its recorded revision.

This batch advances the real-road simulator with simulated vehicle, sensors, traffic and infrastructure. It does not establish Level 5 or physical-car readiness. ML remains internal powertrain analytics; the deterministic Driver, planner and independent Shield command driving.

## Local provenance and preserved baseline

- Baseline: `be37a4092a025d6ececc29f51c1c45e24265ffd0`.
- Preserved tag: `dds-local-items1-4-baseline-20261012`; verified complete-history bundle: `output/dds-items1-4-baseline-20261012.bundle`.
- Final backend, evaluator and regression source: `fdd72d3928c90d169968d0e16c30d70395832edd`.
- Integrated feature/UI commit: `4e757c34b2116bec0eb150e18d58447aaef57e49`, branch `phase-7.5-ui-design`.
- The final UI commit changes only the unavailable lead-distance label. Backend, scripts and tests are identical Git content to the full-suite revision.
- Main was fast-forwarded only after sandbox acceptance. Separate sensor/combined, precision, corridor, validation and boundary worktrees remain preserved. No push, PR or deployment was performed.
- Cloud commit `7b124e6` was not available locally and was not fetched or applied.

The final evaluator stores raw source SHA256 values, revision and trace digests. Main's 71 backend/evaluator files equal the accepted sandbox after LF normalization; 24 raw hashes differ solely because of checkout CRLF conversion. `output/items-main-source-manifest.json` preserves both raw hashes and the normalized hash. Do not call them byte-identical checkouts.

## Implemented scope

| Item | Result |
|---|---|
| 1. Sensor freshness and degraded operation | Producer capture timestamps distinguish fresh empty from missing/stale detections. Both feeds must advance through three fresh captures before recovery. Missing metadata after monitoring starts, bad clocks, producer failures and lane-check exceptions fail closed. Simulated outages stop new maneuvers and request jerk-limited braking; emergency Shield precedence remains. HMI shows unavailable/checking sensors and unavailable lead distance, without a false clear-path status. |
| 2. Precise turning | Front-axle reference follows the center-referenced bicycle geometry. Heading and curvature use matching support. Desired steering demand informs a friction-related speed cap and bounded restoration brake; actual steering retains grip and slew limits. Existing Ackermann front-wheel steering and distance-integrated rolling remain. |
| 3. Trajectory and occupied-space safety | Outward-rounded polynomial bounds check the continuous modeled Frenet corridor and station interval. Initial actual steering is checked through the first planning interval, including endpoint rate. Sampled body checks include chassis and tires. Driver collision and road-boundary Shield rectangles cover the steered tire envelope; body heading is separate from center motion heading. Telemetry explicitly leaves full Cartesian swept-corridor verification false. |
| 4. Broader driving validation and recovery | Independent 50Hz ego chassis/tire SAT oracle, traffic and adversarial-abort fixtures, recorded stop/yield behavior, invalid-state, reset and ML-isolation tests. Recorded control scenarios are selectable through the actual command queue and WebSocket API. Early rejected lane changes request a validated abort; late rejection retains the validation latch. Arrival and unresolved joint rejection use bounded braking to eliminate near-zero creep. |

Recorded stop: OSM way130716952/node65424265, Ray Drive. Recorded yield: way370279592/node6383816258, unnamed approach. The yield fixture uses an explicitly simulated stationary actor, cleared at20 seconds of authoritative simulation time. Stop-line offsets and lane widths remain modeled. The scenarios are validation fixtures, not proof of arbitrary mapped-road behavior.

## Regression and physical mission evidence

Full backend suite on the frozen backend source: **836 passed, 3 skipped, 61 warnings, 300.83 seconds**. Evidence: `output/items-accepted-full.txt`. Skips remain unvalidated scope, not passes. Final UI production build, TypeScript and focused HMI lint passed. Whole frontend lint: zero errors and five existing warnings. UI channel/claim, lint dependency, sensor-status and turn-status checks passed.

Final matrix: **15 cases, 39,763 actual 50Hz samples**, all passing the declared sampled physical envelope oracle. Ordinary scenarios ran35 seconds; mapped parking ran until exact standstill. Behavioral gates are separate: mapped parking, seeded erratic recovery, the explicit adversarial abort return, and recorded-control fixtures. Cases without a behavioral oracle are not counted as behavioral completion successes. No success percentage was inferred. Evidence: `output/items-accepted-driving.json`.

| Final mapped mission measurement, 50Hz | Dry | Wet, μ=0.3 |
|---|---:|---:|
| Samples | 8414 | 8849 |
| Simulation time to exact parked standstill, s | 168.26 | 176.96 |
| Maximum road center-target error, m | 0.465968 | 0.461391 |
| Maximum raw-polyline body-heading error, degrees | 45.611335 | 45.103872 |
| Maximum 5m-chord motion-heading error, degrees | 21.649092 | 21.368155 |
| Maximum normal actuator jerk, m/s³ | 2.5 | 2.5 |
| Maximum steering rate, rad/s | 0.6 | 0.6 |
| Minimum active red-stop margin, m | 0.776808 | 28.659890 |
| Minimum modeled road-envelope margin, m | 2.376439 | 2.404133 |
| Final speed, m/s | 0 | 0 |

The earlier accepted report recorded0.829368/0.813777m maximum target error at100ms sampling. The final combined measurements above use20ms sampling; these are distinct measurement cadences, not a fabricated matched-rate improvement percentage. Raw polyline heading errors remain substantial and must not be replaced by the smaller smoothed-reference metric. The precision sandbox's 6m-support heading metric is also different from this evaluator's 5m-chord metric.

Exact25-second seeded erratic case:109 rejected-plan samples/1250, longest continuous hold1.2s, final speed13.819056m/s and183.091639m station progress. The pre-recovery combined sandbox had1045 rejected-plan samples and ended atzero speed. The checked-in regression requires observed rejection, ABORT, return to LANE_KEEP and >6m progress in the final3 seconds, as well as the independent physical oracle. Evidence: `output/items-accepted-erratic25.json`. This is fixture recovery, not general liveness certification.

Emergency collision mitigation deliberately bypasses normal comfort jerk limits; the oracle still checks bounded emergency deceleration. Do not claim the2.5m/s³ normal limit for emergency transitions.

## Actual browser and live telemetry

A headed browser displayed real OpenStreetMap roads with explicit simulated-vehicle/sensor labels, visible Start/Destination pins and the preserved clean cockpit. Final checks used localhost3000/8000. Later screenshots are at1601×812; earlier checks also used1037×787.

- Sensor outage2–14s: exact speed0 observed; pause did not age readings; recovery required3 distinct captures. Primary/secondary status and lead reading remained unavailable while unready. No status/readings overlap. `output/items-final-sensor-hmi-audit.txt`, `output/playwright/items-sensor-stopped.png` and `items-sensor-recovered.png`.
- Dry left/right and wet left turns: actual scene position, body heading, front Ackermann steering, rear zero yaw and tire roll checked against telemetry. Agreement passed1e-5m/rad tolerances. Dry left position error1.104589e-6m and roll error4.251159e-6rad after900ms pause settling; a first1e-6 assertion was too strict for the remaining interpolation and is not claimed as passed. `output/items-final-turns-audit.txt`, `items-final-wet-audit.txt`, scene-verification JSON files and `output/playwright/items-final-{left,right,wet-left}.png`.
- Final dry parking: exact speed0, parked flag and modeled containment observed live, with scene agreement. `output/items-final-parked-audit.txt`, `items-parked-scene-verification.json`, `output/playwright/items-final-parked.png`.
- Erratic cut-in: observed rejected-plan status followed by resumed motion through25s. `output/items-final-erratic-audit.txt`, `output/playwright/items-final-erratic.png`.
- Selectable recorded stop: live stopped samples18.1–20.1s before release; vehicle subsequently crossed. Ray Drive label shown. `output/items-final-stop-audit.txt`, `output/playwright/items-final-stop.png`.
- Selectable yield: stopped/near-standstill while conflict remained, then resumed after authoritative20s actor clearance and fresh clearance dwell. `output/items-accepted-yield-audit.txt`, `output/playwright/items-yield-{settled,released}.png`. This is a different fixture from the evaluator's standalone recorded-control helper; do not combine their release times.
- Repeated actual loads: identical zero-time physical/planner snapshots apart from epoch; steering and wheel phases reset tozero. `output/items-final-reset-audit.txt`.
- Offline browser: waiting-for-connection status, disabled destination controls, then recovery to paused drive. `output/items-final-reconnect-audit.txt`, `output/playwright/items-final-disconnected.png`.
- Restarted main: live left-turn scene verified again; position error2.945328e-7m and wheel-roll error1.103459e-6rad. `output/items-main-left-audit.txt`, `items-main-scene-verification.json`, `output/playwright/items-main-left.png`.

A first right-turn observation expired because the route's later turn was beyond its55-second observation window. The staged approach/turn check reached the actual turn and passed. This timeout was not counted as a driving failure or a passing check. The browser retained one existing Three.Clock deprecation warning; no application console errors were observed in these checks.

Eight-second visible-tab cadence sample at1601×812:471 animation-callback intervals, median16.7ms/p9516.8ms/max33.4ms. The final normal-cruising reset segment had82 advancing telemetry intervals, median99.35ms/p95115.4ms/max120.3ms. Evidence: `output/items-cadence-audit.txt`, `items-cadence-summary.json`. These are callback and receipt intervals, not measured painted FPS, GPU completion, server-to-screen latency or a guarantee under all loads. Filtering excludes previous scenarios/pauses and command snapshots that did not advance the clock.

## Defects found and addressed during this batch

Missing timestamps previously appeared fresh; lane exceptions could appear clear; delayed delivery hid original acquisition age; producer exceptions interrupted the pipeline; empty worlds lacked real capture stamps; stalled lane-change FSMs never abandoned rejected early plans; collision/road-boundary widths omitted rendered tires; translating collision/boundary checks used body instead of motion heading; arrival and late rejection crept asymptotically; sensor status text overlapped readings; secondary status/clearance/lead labels could imply healthy sensing; actuator acceleration was mislabeled as measured motion; recorded-control fixtures inherited an invented route label. These findings were reported during work and covered by the relevant regressions or live checks. Sources and historical evidence remain preserved.

## Remaining limits

The continuous proof is for polynomial modeled Frenet coordinates, not the continuous Cartesian swept body, tracking error, surveyed lane geometry or arbitrary vertex transitions. Steering reachability checks the initial planning interval; it does not certify every later actual actuator state. The short Shield rectangle sweep assumes constant translation/orientation. NPC dimensions remain perception-class assumptions, not independently verified rendered/surveyed dimensions.

The restoration brake respects its additional nominal budget, but aggregate PID/IDM/junction/Shield/plant tire-force commands are not constrained by a single friction ellipse. No real tire-slip dynamics or hardware sensors/actuators are validated. Late infeasible maneuvers intentionally remain stopped pending a validated plan; universal stationary-start recovery and all lane-completion lateral settling are not established.

Maps retain partial tags, modeled widths/stop offsets, simulated signals and vacancy. Parking is a selected contained pose in a recorded area, not a surveyed bay or physical vacancy. Wider routes, uncertainty/occlusion combinations, sustained stage/deadline and server-to-screen latency, physical plant fidelity and deployment hardening remain future work. The passing best-of-batch planner timing gate does not establish a hard real-time budget for every mapped call.

Backend and frontend are left running from the main workspace with the mapped drive paused. Test cruise preference was explicitly50km/h; the user's80km/h preference was restored afterward. No further feature work is started by this acceptance record.

## Exact local implementation commits

- `5cf4d41cb033191a35ef65e3ab493885148b5248` — feat(sensors): stop on stale feeds and confirm producer-clock recovery
- `cbf346a1fbd550479cb08da9d51b49091aee7c2b` — feat(steering): align front axle reference and restore friction-limited turning
- `05f6ad101e997af3aec976a1d048124950c819c3` — feat(planner): prove modeled corridor bounds and check steering reachability
- `811e932904c64a06f4e1d5e706490eb7342e5fed` — test(driving): audit traffic missions and recorded stop-yield behavior
- `3a8c6c189d394bc2a53303d4ddc41a7ab7baa35d` — fix(safety): align collision footprint with rendered vehicle and expose proof scope
- `6ef42cb88b3af69775efd40145d9651330206803` — feat(scenarios): expose recorded stop and yield missions
- `88ec88ce30a659f2f7ebc78d08afda2d6f21ea2e` — fix(hmi): label actuator acceleration and type planner proof scope
- `9d371cc486e96c2024895fdba018ba62021447af` — fix(parking): hold bounded braking after arrival to eliminate creep
- `39dac75564d49231acca73ae1f03788462ec79f4` — fix(planner): recover rejected early lane changes through validated aborts
- `4ac2badd9189637e54aa477333ae980b65a4e67d` — test(driving): require fixture recovery and exact parked standstill
- `00c702afd5de1a11e6b07db093caa128277ebe75` — fix(safety): check steered tire envelope and center motion at road boundary
- `fdd72d3928c90d169968d0e16c30d70395832edd` — fix(planner): hold bounded braking while joint trajectories remain rejected
- `4e757c34b2116bec0eb150e18d58447aaef57e49` — fix(hmi): show unavailable lead distance while sensor feeds are unready


## 13 October 2026 — runtime smoothness and map/parking evidence

Local code `6e40b78ee37c04e7be0f2ffe160a190640f645be`:872 backend tests passed,3 skipped; final main frontend build passed. Narrow Proactor cleanup and queue accounting fixes, bounded timing capture, stable action subscriptions, recorded map provenance and conservative parking access/direction checks were sandboxed and reviewed before integration. Foreground browser measurements and live TCP/map/reconnect evidence are in [items5–6 acceptance](DDS_ITEMS_5_6_ACCEPTANCE_2026-10-13.md). Detailed overlays and scheduling outliers remain measured limitations; no universal60FPS, surveyed bays, physical access or Level5 claim. No push/PR/deployment. Original59-commit review stays separate from this new batch.
