# Supervised DDS development — 12 October 2026

The preserved working version is local checkpoint **ea6b4bbc18d17d100050cc407959caa9a5e092d9** on `phase-7.5-ui-design`. Its parent is `a4c9072c4c9328c3d60790fb0ffa66adf479532b`; the checkpoint captures the earlier uncommitted software, recorded maps and engineering reports. Local tag: `dds-local-checkpoint-20261012`. A verified local Git bundle and tracked-file SHA-256 manifest are retained in `output/dds-working-checkpoint-20261012.bundle` and `output/advanced-checkpoint-manifest.json`.

New executable changes are developed first in the isolated worktree `D:\college\Major Project\DDS_Turning_Sandbox` on `feat/turning-feasibility-20261012`. The main runtime remains at the checkpoint until sandbox tests and independent review pass. No GitHub push, PR or deployment is authorized. ML continues to analyze simulated internal readings; driving decisions belong to the planning, control and safety systems.

## Baseline reproduced during this session

The recorded mapped-parking mission completed with 1671 sampled frames. Worst road-approach target deviation: **1.186573 m**; worst instantaneous road-heading error: **48.173121 degrees**. Maximum sampled steering rate: **0.600000 rad/s**; minimum active red-stop distance: **0.951990 m**. Maneuver samples remained contained. Backend world position matched integrated lat/lng projection exactly. `output/advanced-baseline-turning.json` records this run against the clean checkpoint SHA.

## Batch scope and acceptance

This batch targets model-consistent turn tracking and physical curvature/friction speed limits. It does not claim general Level 5 autonomy or a surveyed intersection corridor. The current 15 km/h curvature-speed floor can override a lower grip limit; route tangents also change discontinuously, and center-reference dynamics need center-compatible steering feedforward. These defects are tested before integration.

A separate next batch must check combined road curvature plus lateral-maneuver acceleration; the current lattice treats these contributions separately. Other existing limitations remain documented in the steering report.

Acceptance requires sandbox unit/physical-invariant tests, mirrored turn and wet-road cases, the existing mission's speed/red-stop/containment checks, full backend regression, and actual main-server browser/WebSocket validation after integration. Evidence and precise local commit SHAs will be appended only after those operations succeed. Rollback uses a reversible revert of the accepted feature commit; the checkpoint worktree and tag remain available.


## Accepted sandbox batch

The active tracking controller now uses the exact center-bicycle curvature inverse, measured sideslip compensation, parallel-offset path curvature and a local chord tangent. The preview distinguishes the physical front axle from future center-velocity displacement. Nominal cross-track gain and physical steering/rate limits were retained. The curvature/grip speed cap can fall below 15 km/h. Folded/non-finite/out-of-domain turn-reference helpers reject input; detected invalid references produce bounded braking with a zero speed target and explicit HMI status. This guard does not certify arbitrary physical turn feasibility or corrupted ego-state recovery.

Independent measured recorded-road missions:

| Metric | Checkpoint dry | Candidate dry | Checkpoint wet mu=0.3 | Candidate wet mu=0.3 |
|---|---:|---:|---:|---:|
| Worst road-approach deviation (m) | 1.186573 | 0.831585 | 0.944212 | 0.855880 |
| Worst instantaneous road-heading error (degrees) | 48.173121 | 47.601894 | 46.810873 | 46.746588 |
| Minimum active red-stop distance (m) | 0.951990 | 0.752372 | 11.820930 | 15.745992 |
| Maximum sampled road lateral acceleration (m/s2) | not captured in first dry summary | 2.957609 | 0.899279 | 0.899249 |

Both candidate missions parked with all maneuver samples contained. Steering rate remained at or below 0.6 rad/s; measured pose-projection error was zero. Artifacts: `output/advanced-baseline-turning.json`, `advanced-baseline-wet.json`, `advanced-candidate-turning-final.json`, `advanced-candidate-wet-final.json`. Samples are spaced 0.1 simulated seconds; these are bounded mission measurements, not general-road success rates.

Sandbox full regression: **535 passed, 3 known harness skips, 38 warnings**, 250.73 seconds (`output/advanced-sandbox-full-tests.txt`). Final focused steering tests: 41 passed, including mirrored ideal-circle integrations, actual Driver.control plus physics circle fixtures, the below-15 wet curvature target, malformed helper input, and invalid-reference braking. The isolated frontend production build/TypeScript passed; lint has zero errors and five existing unused-symbol warnings. The status regression checks moving/stopped invalid references, parking precedence, emergency precedence and normal cruising. Independent review accepted this as a scoped controller upgrade.

A shared dependency junction initially failed Turbopack's filesystem-root rule. Isolated `npm ci` resolved the sandbox build. A UTF-8 text corruption introduced during the status edit was caught in review and restored from the checkpoint before acceptance; only the intended five-line status branch remains. Main-runtime validation and exact feature commit provenance are recorded after integration below.


## Integrated local provenance and live verification

Accepted feature commit: **5294885ad094a18d5e5d3b23d15942804d158d82** (`feat(steering): align turn tracking with center bicycle and grip limits`). The main branch fast-forwarded to this same SHA after sandbox checks; local tag `dds-local-turning-model-20261012` identifies it. Checkpoint **ea6b4bbc18d17d100050cc407959caa9a5e092d9** remains preserved. No history rewrite, remote push, PR or deployment occurred.

The restarted main backend and actual browser completed the mapped mission with **1686 distinct snapshots**, **0.831585 m** worst road-approach deviation, **47.601894 degrees** worst instantaneous road-heading error, and **0.752372 m** minimum active red-stop distance. The car parked, and all parking-maneuver samples were contained. `output/advanced-live-turning.json` and `output/advanced-live-summary.json` record the integrated runtime, not an injected browser trace. Main integration tests: **45 passed**, 3 existing SHAP warnings (`output/advanced-integrated-targeted.txt`). HMI status and UI channel contracts passed.

A separate browser-controlled run loaded the actual mapped scenario, waited for the left-turn steering event, paused through the real command channel, and inspected the actual rendered Three.js scene. Rendered wheel steering matched telemetry within **1.03e-9 radians**; phase change during a subsequent one-second pause check was below **4.70e-8 radians** (interpolation rounding). Screenshot `output/playwright/advanced-left-controlled.png` was opened and inspected. `output/advanced-browser-left-controlled.txt` contains the exact state/transforms. The completed parked view was also inspected in `output/playwright/advanced-parked-paused.png`; its wheel phases held unchanged during a one-second pause. The earlier automatic pause screenshot attempt missed its time window, so that image is not used as evidence of a paused right turn. No browser errors were reported; the existing Three.Clock deprecation warning remains.

Both local servers remain running, with the car paused at the left turn for manual review. Click Resume drive to continue. The frontend layout and all prior features were retained.

### Remaining work

The 0.5 m sharp-junction target is still unmet. Curvature averaging/clipping, sparse geometric tangents, unverified real lane boundaries, the compatibility controller difference and combined lane-change/road acceleration need further development. This batch establishes model-consistent control and corrected grip targets, not a full curvature/steering-rate-feasible junction trajectory or Level 5 capability. The next implementation should create and verify such trajectories in the sandbox before changing the active planner.

For a reversible feature rollback, pause the runtime and use `git revert 5294885ad094a18d5e5d3b23d15942804d158d82`, then restart and validate. Do not use a destructive reset: the working checkpoint and prior engineering features must remain preserved. Exact checkpoint/feature/evidence SHAs are also retained in the local output commit ledger.


## 12 October 2026 — joint maneuver and anticipatory turn batch

Accepted local feature `c2c68725be38ea95359d6484896a4241b803f8fd`. Full combined suite:668 passed,3 skipped; frontend build passed; actual browser/live steering, parking, reset and reconnect checks recorded. See [measured acceptance and remaining limits](DDS_JOINT_TURN_ACCEPTANCE_2026-10-12.md). The user-authorized continuation finishes this batch; no next feature was started.


## 13 October 2026 — sensor, turning, corridor and driving validation batch

Accepted local feature/UI `4e757c34b2116bec0eb150e18d58447aaef57e49`; frozen backend/evaluator `fdd72d3928c90d169968d0e16c30d70395832edd`:836 tests passed,3 skipped. Final frontend production build and live sensor, dry/wet steering, exact-stop parking, traffic recovery, recorded stop/yield, reset and reconnect checks passed within the documented simulator scope. See [the new acceptance and provenance record](DDS_ITEMS_1_4_ACCEPTANCE_2026-10-13.md). This supersedes the earlier pending items for these scoped implementations; broader Cartesian/physical/friction and timing certification remain unproved. All commits remain local.
