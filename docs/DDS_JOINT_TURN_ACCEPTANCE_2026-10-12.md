# DDS joint maneuver and anticipatory turn acceptance — 12 October 2026

The user authorized finishing the current batch and stopping immediately afterward. This record supersedes the unfinished status in the earlier autonomous-window handoff. No next feature was started during the continuation. The initial two-hour deadline was exceeded as recorded in that handoff; this acceptance occurred under subsequent explicit authorization, not within the original window.

## Local provenance

- Preserved starting source: `6766cc26f0c83c3db4d573f0cee5d03544a40bc8`; tag `dds-local-autonomous-window-20261012`, verified bundle `output/dds-window-baseline-20261012.bundle`.
- Isolated combined sandbox source: `6662283bfed1744d55e7c74e5bac32a210f7181b`, branch `feat/joint-trajectory-physics-20261012` in sibling `DDS_Planner_Sandbox`.
- Accepted application feature: `c2c68725be38ea95359d6484896a4241b803f8fd` on `phase-7.5-ui-design`.
- `git diff 6662283bfed1744d55e7c74e5bac32a210f7181b c2c68725be38ea95359d6484896a4241b803f8fd -- app frontend tests` produced no differences. The accepted application, UI and tests are byte-equivalent to the tested candidate. Both services were then restarted from the main workspace and its actual turn was inspected again.
- All commits are local. No push, PR or deployment. Reverting the feature commit provides a reversible code rollback; earlier bundles and checkpoints remain intact.

## Resulting behavior

The joint planner evaluates actual Cartesian speed and longitudinal/lateral acceleration over the complete maneuver horizon rather than separately accepting road curvature and lateral motion. It rejects route-range violations, reversal, incompatible lateral movement at longitudinal standstill and sampled quasi-static steering angle/rate violations. Terminal lateral position is held and terminal longitudinal velocity continues correctly. Initial overspeed is allowed only through explicit causal monotone recovery, rather than reporting an already-satisfied speed cap. Straight-lattice NumPy batching preserves scalar constraints, with seeded parity tests.

The driver replans from estimated current model motion, converting station and physical speed consistently. Remaining committed feasibility is rechecked at the planning rate, using actual configured geometry and limits. Rejected joint maneuvers hold measured offset and request a stop instead of falling through to an unchecked independent lane change. Full maneuver telemetry and explicit unavailable-plan HMI status accompany this behavior.

The turn governor anticipates the upcoming reference geometry, offset-path physical arc distance, steering-rate limits and friction. Its braking envelope accounts for the actual current acceleration and jerk ramp. Front-axle preview follows the exact constant-actual-steer bicycle arc and predicted body heading. Only isolated short fully overlapping collinear backtracking reference artifacts are filtered, preserving the existing artifact regression; recorded map geometry is unchanged. Ambiguous half-turn heading domains and out-of-recorded-route queries fail closed.

ML remains restricted to simulated internal-reading analytics. Deterministic planning, controller and independent safety systems command vehicle motion; no learned driving policy was enabled. Runtime attribution tests remain included in the full suite.

## Validation on the combined source

- Full backend regression: **668 passed, 3 skipped**, 38 warnings, 298.54s. Evidence: `output/window-combined-full-tests.txt`.
- Combined focused tests: **138 passed**, 3 SHAP warnings, 16.90s. Evidence: `output/window-combined-focused-tests.txt`.
- Final frontend production build passed: `output/window-combined-final-build.txt`.
- Lint: **0 errors, 5 existing unused-symbol warnings**: `output/window-combined-lint.txt`.
- `node --experimental-strip-types scripts/verify-turn-status.mjs` passed for moving/stopped infeasible turns, unavailable maneuvers, turn preparation, emergency precedence and ordinary cruising.
- An earlier separate turn-only suite had one unchanged perception timing failure (2.276ms vs2ms) under concurrent load. Its isolated rerun and the final combined suite passed. Historical failures were retained; no threshold was relaxed.

The recorded-road mission evaluator ran the actual simulation session pipeline, not a synthetic replacement of the route. Vehicle and sensor measurements are simulated.

| Measured metric | Combined dry | Combined wet, mu=0.3 |
|---|---:|---:|
| Samples to parked state | 1678 | 1752 |
| Maximum road target error, m | 0.829368 | 0.813777 |
| Maximum instantaneous road heading error, degrees | 43.120796 | 44.308549 |
| Maximum road lateral acceleration, m/s2 | 1.694707 | 0.898998 |
| Maximum sampled steering rate, rad/s | 0.600000 | 0.600000 |
| Minimum active modeled red-stop margin, m | 0.771342 | 27.154160 |
| Maximum reported world-pose projection error, m | 0 | 0 |
| Parked / modeled maneuver contained | Yes / Yes | Yes / Yes |

Evidence: `output/window-combined-dry.json`, `output/window-combined-wet.json`, stamped with the clean sandbox SHA. These measurements do not imply a success percentage or arbitrary route capability. Worst target error still exceeds0.5m; instantaneous heading errors remain substantial.

Isolated planner timing over50 calls: straight mean3.104ms, max4.609ms, p994.530ms; mapped synthetic fixture mean21.484ms, max29.666ms, p9928.059ms. The unchanged best-of-batch4ms regression gate passed. This does not establish that every call meets4ms or any hard real-time deadline. Evidence: `output/window-joint-planner-timing.txt`.

## Actual browser and live WebSocket observations

A headed browser at1037x787 displayed real OpenStreetMap roads and clearly labeled simulated vehicle/sensors. Start and Destination pins were visible. Dry left and right turns, a wet left turn, and final parking were observed with live WebSocket telemetry and actual Three scene transforms. Front wheels have distinct Ackermann angles; rear wheel yaw remains zero. Rendered world position and front steering matched backend telemetry within1e-6m/rad. Paused interpolation drift was below1e-6m and1e-6rad. The restarted main-workspace left turn was verified again on the accepted feature source with zero browser application errors and one existing Three.Clock deprecation warning.

- Screenshots: `output/playwright/window-left-accepted.png`, `window-right-accepted.png`, `window-wet-left-accepted.png`, `window-parked-accepted.png`, `window-main-left-accepted.png`.
- Browser audits: `output/window-browser-{left,right,wet-left,parked,main-left}-audit.txt`.
- Partial dry live capture:922 frames, stations290.660–747.612m, peak sampled jerk2.5m/s3 and road target error0.829368m. It contains pauses and does NOT alone prove completion. `output/window-live-accepted.json`, `output/window-live-summary.json`.
- Separate final live parking capture:284 frames, ended parked and modeled-footprint contained. `output/window-live-parking.json`.
- Offline/recovery browser check: waiting-for-connection status, hidden stale speed/safety readings, disabled destination controls, then recovery to the paused drive. `output/window-browser-reconnect-audit.txt`, `output/playwright/window-disconnected.png`.
- Repeated actual WebSocket scenario loads produced matching zero-simulation-time snapshots apart from epoch, including zero velocity, steer and all wheel rotations. `output/window-browser-reset-audit.txt`, `output/playwright/window-reset-accepted.png`.

A first right-turn observation timed out before the later curve and was retried on current telemetry; it was not treated as a vehicle failure. The initial temporary port3001 preview encountered the existing REST origin allowlist; final checks used supported port3000 without widening it. A Windows Proactor socket-close `ConnectionResetError/WinError10054` appeared during preview transitions. Reconnection passed afterward, but this exception remains a known runtime issue, not an error-free production claim.

## Remaining work, not implemented in this continuation

1. Explicit sensor freshness/degraded controlled-stop behavior: distinguish fresh empty detections from missing feeds, use simulation time and test recovery. First reproduce current behavior in a sandbox.
2. Continuous corridor and actuator reachability checks: sampled quasi-static limits do not certify initial steering reachability, tire transients, every continuous-time point or surveyed lane containment.
3. Validate larger traffic/abort/cut-in and route-intersection matrices with complete browser/live traces. Existing passing/abort tests are regression evidence, not a real-world driving certificate.
4. Measure control, telemetry and UI frame latency under controlled load; no FPS or UI smoothness guarantee was inferred from screenshots.
5. Strengthen map semantics and physical plant/sensor realism. Recorded roads currently retain modeled lane widths and simulated vacancy. Parking is a selected pose inside a mapped area, not a verified physical bay.

This advances the real-road simulator; it does not establish production readiness or Level5 autonomy. Backend and frontend are left running locally with the drive paused for manual inspection. No additional improvement work follows this record.


## 13 October 2026 — sensor, turning, corridor and driving validation batch

Accepted local feature/UI `4e757c34b2116bec0eb150e18d58447aaef57e49`; frozen backend/evaluator `fdd72d3928c90d169968d0e16c30d70395832edd`:836 tests passed,3 skipped. Final frontend production build and live sensor, dry/wet steering, exact-stop parking, traffic recovery, recorded stop/yield, reset and reconnect checks passed within the documented simulator scope. See [the new acceptance and provenance record](DDS_ITEMS_1_4_ACCEPTANCE_2026-10-13.md). This supersedes the earlier pending items for these scoped implementations; broader Cartesian/physical/friction and timing certification remain unproved. All commits remain local.
