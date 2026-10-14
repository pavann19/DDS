# DDS autonomous-window handoff — 2026-10-12

Historical status at the original deadline: sandbox candidate preserved; acceptance incomplete.

Current status: the user subsequently authorized finishing the current batch. See [accepted validation record](DDS_JOINT_TURN_ACCEPTANCE_2026-10-12.md) for completed integration, exact feature SHA, live observations and remaining limitations. The sections below preserve the original handoff history.

Baseline and running application source: `6766cc26f0c83c3db4d573f0cee5d03544a40bc8`.
Local baseline tag: `dds-local-autonomous-window-20261012`.
Verified baseline bundle: `output/dds-window-baseline-20261012.bundle`.
Sandbox branch: `feat/joint-trajectory-physics-20261012`, at sibling `DDS_Planner_Sandbox`.
Sandbox WIP commit: `6662283bfed1744d55e7c74e5bac32a210f7181b`. This is not an accepted application release.

The authorized window started 10:26:14 UTC with deadline 12:26:14 UTC. The later clock check returned 12:44:54 UTC, already 18m40s beyond the deadline; implementation then stopped. Do not describe this as completion within two hours.

## Implemented only in the sandbox

- Joint lateral/longitudinal Cartesian feasibility checks across the complete maneuver horizon, including physical speed, acceleration, sampled quasi-static steering angle/rate, route bounds and reversal rejection.
- Scalar-equivalent straight-lattice vector batching; explicit causal initial overspeed recovery.
- Current-motion Frenet conversion and remaining committed-plan revalidation. Rejected joint candidates request a stop instead of unchecked lateral fallback.
- Maneuver telemetry retains the full lateral/longitudinal horizon and reports joint-plan availability.
- Anticipatory turn speed governor with friction, actual jerk ramp, offset arc distance and steering-rate limits; exact constant-steer front-axle preview.
- HMI distinguishes unavailable maneuvers and infeasible turns. ML remains analytics-only.

## Evidence and outstanding gates

Combined focused backend tests: **138 passed**, 3 existing SHAP warnings, 16.90s (`DDS_Planner_Sandbox/output/combined-focused-tests.txt`). Frontend production build passed (`DDS_Rebuilt/output/window-frontend-build.txt`). Additional HMI reason handling was added afterward; final frontend verification remains required.

Planner-only focused acceptance: 66 passed. Straight timing: mean3.104ms/max4.609ms; mapped synthetic timing: mean21.484ms/max29.666ms. These are measurements, not scheduling guarantees.

Turn-only frozen full suite: 620 passed, 3 skipped, one unchanged perception timing gate failed at2.276ms against2ms during concurrent work. An isolated rerun passed. Retain both results; do not claim all final checks passed.

Separate turn candidate simulated mapped missions: dry road error0.829368m, wet0.813777m; both parked and modeled maneuvers contained. These do not validate the combined committed source or real hardware. Combined full regression was launched (`DDS_Planner_Sandbox/output/combined-final-full-tests.txt`); at handoff the session was still running and no final output was available. Final actual browser/WebSocket acceptance of this candidate has NOT been performed. The existing running application remains on the baseline.

## Next sequence

1. Resolve the final combined regression result without relaxing existing assertions; rerun timing gates under controlled load and record distributions.
2. Verify combined frontend build, lint and status checks on the exact WIP SHA.
3. Run dry/wet mapped missions plus traffic passing and abort cases on combined source; verify physical/station-speed conversion and shield precedence.
4. Observe actual browser turns, front-wheel angles, body motion, route pins, terminal parking, resets and WebSocket reconnection using live telemetry. Record screenshots and numeric limits on the same SHA.
5. Only after acceptance, integrate a local commit into the application branch and restart the backend. No push, PR or deployment is authorized.
6. Next capability: explicit sensor freshness/degraded controlled-stop handling using simulation time, distinguishing fresh empty detections from a missing feed; first reproduce current behavior in a sandbox.
7. Then strengthen corridor feasibility and actuator reachability, add an operational-domain and adversarial scenario matrix, and measure control/telemetry/UI latency under load. Dependency: real mapped roads still have modeled widths and simulated sensors, not surveyed lane geometry or physical tire dynamics.

This work advances a real-road simulator. Sampled constraints, estimated motion derivatives and quasi-static steering are not continuous feasibility certification or Level5 autonomy. No success percentage is inferred.
