# DDS fixes 1–4: local engineering evidence

Date: 2026-10-10. Local branch `phase-7.5-ui-design`, HEAD `a4c9072`.
The cloud branch `fix/blueprint-p0` and object `7b124e6` are absent locally.
No fetch, push, PR, deployment or commit was performed. Existing working-tree changes were preserved.
The local `DDS_Engineering_Blueprint.md` was reviewed as a design/audit document, not as proof of implementation.

## Changes

1. **WebSocket lifecycle:** sender and receiver tasks are cancelled and awaited together; close races are handled as disconnects. A subscriber disconnect no longer stops the shared simulation. Application shutdown owns session shutdown. The browser worker reconnects with backoff and marks stale/disconnected readings unavailable.
2. **Scheduling:** the session runs tracking at 20 Hz, prediction/planning at 10 Hz and control/safety/physics at 50 Hz on a 100 Hz simulation clock. Control no longer invokes planning again. Session, world and scenario clocks share the executor clock. Deadline pacing prevents relative sleep overshoot from accumulating. Stage costs and deadline misses are exposed; fixed simulation dt is retained.
3. **Route transitions:** routes are prepared separately and committed atomically at a command boundary. Older fetch generations cannot replace a newer route or scenario. Incompatible routes are rejected without teleporting the car. Geographic pose is preserved on accepted transitions; Frenet coordinates may change. Route versions and epochs accompany geometry. Scenario geometry replacements are published too. Route preparation pauses the run; resume requires an explicit operator command. Results report pending/applied/rejected/superseded.
4. **HMI truthfulness:** target speed comes from the driving controller. Planned corridors use committed backend polynomial samples; nominal driving without a committed polynomial publishes no trajectory. Commands have acknowledgements, loss of connection before acknowledgement reports an unknown result, startup analytics are unavailable, and the configured simulation telemetry rate is distinguished from measured wall-clock performance. Low-friction status and paused status are explicit.

ML remains internal vehicle telemetry analytics. Driving commands come from the deterministic intelligent driving stack: perception/tracking, prediction, behavior planning, trajectory planning, controllers and independent safety checks. A runtime regression replaces analytics with conflicting labels/scores and asserts identical actuator commands and vehicle traces. This is not a claim of learned driving or general autonomy.

## Validation

The actual API and Next.js application were run locally. Playwright operated the browser, captured screenshots and observed actual WebSocket frames. Evidence is local and ignored by Git under `output/playwright/fixes-1-4/`.

- Final focused regressions: **55 passed** (`targeted-final.log`, `targeted-final.xml`).
- Final full suite: **416 passed, 3 skipped, 7 warnings** in 236.07 seconds (`full-final.log`, `full-final.xml`). The earlier snapshot passed 413 tests; the final run includes three added regressions.
- Final TypeScript check and UI contract check passed. UI contract: 7 panels, 23 live files.
- Browser pause: telemetry tick remained 31. Single-step advanced to 32 and displayed `Command: applied`. Reset advanced the epoch; the sampled final reset frame was already at 0.5 simulated seconds, so that screenshot is not an assertion of an exact zero-time frame.
- Final emergency capture: 81 frames over 8 simulated seconds, minimum acceleration **−4.5 m/s²**, 20 captured frames below −4 m/s², shield emergency override and controller target zero. All captured session/world/scenario clocks matched exactly. Reported real-time factor was **1.002**.
- Normal cruising: 51 frames over 5 seconds; overtaking: 81 over 8 seconds; stop-and-go: 61 over 6 seconds; low friction: 41 over 4 seconds. Maximum jerk calculated from the 10 Hz telemetry samples was approximately **2.5 m/s³** in each. This sampling does not prove every 50 Hz peak. Real-time factors were **1.000–1.003** in these short captures.
- Overtaking supplied a nonempty committed trajectory in 19 captured frames. Normal cruising supplied none. Low friction changed route version from 1 to 2 and rendered the replacement curve.
- A localhost proxy forced three transport closes. The real browser worker reconnected three times, preserved one simulation ID and advanced from tick 1183 to 1276. Offline HMI showed no target, `Telemetry unavailable`, `Shield unavailable` and `Path unknown`. This occurred before the final pacing/geometry additions; the WebSocket lifecycle implementation was unchanged afterward.
- A real OSRM HTTP 200 route was accepted in an earlier live capture (version 1→2). The final real OSRM request, from the synthetic low-friction road, was rejected as outside the current road corridor. Final geographic position remained exactly `{lat:37.77491573812684,lng:-122.41869310671888}`; route version stayed 2 and the run stayed paused. No successful final route adoption is claimed for that request. Accepted-route pose preservation and A→B→A stale-fetch rejection are also covered by focused regressions.

Relevant artifacts: `browser-final.log`, `additional-scenarios.log`, `route-browser-final.log`, `proxy-browser.log`, `telemetry.json`, `emergency.png`, `low-friction-final.png`, `paused-final.png`, `reconnect-offline.png`, backend logs and reproducible browser/proxy scripts.

## Additional defects found and reported during the work

- Planning duplicated at the control rate; nominal perception/prediction rates did not represent actual work. Corrected.
- Synthetic frontend planned corridor unrelated to the committed backend plan. Corrected.
- Subscriber queues silently discarded reliable route/command messages. State is now coalesced independently; reliable overflow explicitly disconnects with code 1013.
- Invented startup 85% confidence/A+ score and optimistic destination highlighting. Removed.
- Superseded destinations lacked final results; completed route tasks accumulated. Corrected.
- Paused heartbeats resampled stateful feature deltas at an unchanged tick. Deduplicated by epoch/version/tick.
- Low-friction scenario changed backend geometry without publishing it. Corrected and regression-tested.
- Hardcoded “10 Hz” operational wording and low-friction “Cruising” status were misleading. Corrected.
- Top status overlapped lead distance at the observed 1244×944 viewport. Compact breakpoints and text clipping were adjusted and visually rechecked. Crowded 3D labels around nearby vehicles remain a visual limitation; comprehensive responsive-layout repair is not claimed.

## Limits

These are bounded local simulator observations, not a success rate, safety certification, collision-free guarantee or production deployment. Before deadline pacing, live real-time factor was 0.649–0.666. Later short captures were near 1.0, but a longer startup capture still recorded intermittent stage deadline misses; hard real-time behavior is not established. Metrics retain cumulative miss counts and a rolling window of at most 2,000 stage durations.

Three legacy Starlette TestClient WebSocket smoke tests remain skipped by their existing environment guard; real-browser and focused lifecycle checks provide separate evidence, not a conversion of skips to passes. Dependency warnings remain (SHAP color APIs, Python deepcopy of itertools, and THREE.Clock deprecation). No model was retrained and no physical vehicle/CARLA validation was performed.
