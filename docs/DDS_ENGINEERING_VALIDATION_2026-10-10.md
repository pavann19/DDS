# DDS local engineering validation — 10 October 2026

## Checkout and scope

Inspected `phase-7.5-ui-design` at `a4c9072`. Neither the reported cloud
branch `fix/blueprint-p0` nor commit `7b124e6` exists in local Git refs/objects.
No remote fetch, push, PR, deployment or commit was performed. File inspection
found the reported SHAP and emergency-actuation fixes absent.

The checkout already contained uncommitted changes to WebSockets, database,
physics, deterministic backend and tests, plus an untracked SimulationSession.
Those changes were preserved. The initial tracked diff and session source were
saved under `output/playwright/dds-20261009/` before this work.

Read `DDS_Engineering_Blueprint.md` (modified 10 October; audit dated 7 October,
pinned to an older revision). Its historical measurements were not treated as
current results. This work addresses parts of R01/R02/R04/R05/R06/R13; it does
not complete that blueprint or qualify production readiness.

## Changes

- Shield vetoes NaN/Inf inputs; dynamics/control/reset reject invalid values.
- Emergency mitigation bypasses the 2.5 m/s³ comfort limiter and applies the
  declared 4.5 m/s² brake demand. This is a kinematic simulation policy, not a
  calibrated real brake actuator or collision-avoidance guarantee.
- SHAP receives the same standardized, ordered input as prediction; displayed
  feature values retain sensor units. Regression reconstructs model margin.
- Scenario reset clears driver/RNG/powertrain handoffs and refreshes sensors
  after traffic replacement. The refresh fixes a live-observed first-tick
  braking spike caused by the previous scenario's lead measurement.
- Single-step resumes scenario progression for exactly 0.10 s, then pauses and
  publishes the paused state. Scenario triggers retain their 10 Hz tick units.
- Feature deltas are sampled once per published snapshot for analytics/storage.
- Worker retries connections with capped backoff and a 3-second stale timeout.
  Missing telemetry clears operational state; HMI shows unavailable safety/path
  status rather than default approval. Compact offline labels avoid overlap.
- Database session creation resumes an existing ID on reconnect.
- Removed an autouse fixture's stub of the routing function under direct test;
  network routing remains stubbed at the session consumer boundary.

## Live browser evidence

Ran FastAPI with the local venv and Next.js development server; exercised the
actual browser UI using Playwright. Evidence is local and ignored by Git under
`output/playwright/dds-20261009/`.

`final-fixed-browser.log` contains raw published frames for cruise, emergency
cut-in and queue scenarios plus UI text. Non-emergency sampled jerk maxima are
2.5 m/s³ (floating-point tolerance). Emergency override first appears at 1.3 s
with applied acceleration -4.5 m/s²; subsequent samples reduce velocity by
0.45 m/s per 0.1 s. Before the fix, browser capture showed emergency acceleration
ramping from about -0.10 to -1.85 m/s² over 0.7 s.

`final-transport.log`: pause holds simulation time at 0.18 s for the observation
interval; step advances to 0.28 s and remains paused; reset emits a zero-time
snapshot with scenario tick/time zero and unpaused state. Scenario elapsed time
is quantized to 10 Hz (0.2 s at session time 0.28 s), so full clock alignment is
not established. Browser reconnect after a real backend stop/restart was
observed; `disconnected.png` captures unavailable HMI state. Later compact labels
were shortened. Scenario screenshots are `1-scenario.png` through `3-scenario.png`.

Earlier browser attempts timed out and their logs remain retained. The initial
43.5 m/s³ transition spike was investigated and fixed, not discarded as noise.
The final short queue capture does not establish complete stop-and-go behavior
or collision freedom. Telemetry is sampled at publication rate, not an oracle
for every internal control substep.

## Verification and remaining gaps

Full suite at the intermediate working tree: **398 passed, 4 failed, 3 skipped**
in 415.59 s (`full-suite.log/xml`). Two failures were the routing fixture issue
subsequently corrected; two were performance thresholds (session cycle 107.8 ms
against 50 ms, planner 4.82 ms against 4 ms). Keep those failures as evidence;
concurrent browser/backend load was present. A later targeted run passed the
session timing assertion without changing its threshold. Full suite has not
been rerun after final fixes. Final targeted run: **54 passed**, 3 warnings
(`final-targeted.xml`).
TypeScript and `verify-ui-contract.mjs` pass. No production build/CI claim.

Open blueprint work includes route generations/atomic transitions, explicit
epochs and command acknowledgements, full core 50/20/10 Hz separation, actual
planner trajectory/target publication, invalid payload schemas at all ingress,
capture-loss semantics, and independent swept collision/road oracle. The
session still invokes combined driver/world work in the 50 Hz stage; registered
stage names do not prove complete multirate separation. Reconnect teardown also
logged an ASGI send-after-close race; lifecycle robustness needs further work.
