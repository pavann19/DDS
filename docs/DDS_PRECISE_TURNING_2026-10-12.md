# DDS steering and wheel validation â€” 12 October 2026

Scope: real-road simulator with simulated vehicle and sensors. These changes do not establish physical-car autonomy. Driving remains deterministic planning/control/safety; the ML analytics channel does not command actuators.

## Implemented

- The active 50 Hz controller projects a center-referenced front axle with a bounded 200 ms preview (at most 0.7 m) onto the same route frame. Heading and signed-curvature feedforward use that preview station. Existing steering angle, steering-rate and friction constraints remain.
- Vehicle pose is the body center. Ideal bicycle motion applies beta = atan(0.5 tan(delta)), yaw rate v cos(beta) tan(delta)/L, and midpoint-heading displacement. The compatibility decision path uses the same center-based yaw formula.
- Ackermann front wheels have separate inner/outer angles. Four cumulative rolling angles integrate traveled distance and each wheel's turning-radius factor. They reset with the scenario and do not advance while the simulation is paused.
- The visible wheelbase is 2.8 m; track width is 1.96 m and tire radius 0.34 m. Front steering pivots and tire rolling axles are separate. Visible spokes reveal rolling. Wheel bottoms meet the rendered ground.
- Rendering uses telemetry wheel states, the correct compass-to-Three.js rotation sign, and frame-rate-independent interpolation. Scenario epoch/session changes snap the ego and reset the chase camera. The camera anticipates toward the turn, and visual chassis roll leans outward.
- Finite authoritative world position is used even if it departs from the locally reconstructed route. Backend world position now projects the integrated lat/lng directly; it cannot snap to the end of the route. A dedicated endpoint regression checks this.

## Defects reported during work

Both front wheels had equal angles with the wrong rotation sign; tires did not roll. Center/body rendering and collision footprints disagreed with rear-reference integration. The chase camera looked away from turns, and the acceleration-linked chassis roll leaned inward. Finally, route-projected telemetry could hide integrated endpoint overshoot. These were reported during the task and corrected.

## Measured evidence

`output/audit-turning.py` runs the actual deterministic session pipeline, loads the recorded mapped-parking scenario, and samples every 0.1 simulated seconds. The road-approach comparison excludes the parking phase, whose lateral target changes from the road offset to the aisle center. It measures abs(actual Frenet d - committed target d), not distance from decorative road markings.

| Same recorded mission | Before | Selected implementation |
|---|---:|---:|
| Worst road-approach lateral tracking error | 2.8967 m | 1.1866 m |
| Worst road-approach heading error | 49.9101 degrees | 48.1731 degrees |
| Samples to parked condition | 1692 | 1671 |

Artifacts: `output/turning-before.json`, `output/turning-final.json`. Rejected controller trials are retained under `output/turning-*.json`; their results are not delivery claims. The final mission regression also bounds road-approach tracking error below 1.3 m and retains parking speed, containment, red-stop and completion checks.

Browser validation used the actual localhost frontend and WebSocket backend, not an injected vehicle trace. The first live trip captured 1689 distinct tick snapshots and reached the mapped parking area with full-footprint containment. Minimum active red-stop distance was 0.9520 m. `output/turning-live.json` contains this trip, including left/right turn pauses.

- Left pause: rendered front-left angle 0.33719 rad and front-right 0.27440 rad in Three.js; equal magnitude/opposite sign to the sampled backend Ackermann angles. The left inner wheel steers farther.
- Right pause: rendered front-left -0.27755 rad and front-right -0.34190 rad. The right inner wheel steers farther. Over 1.5 seconds the rendered pose, heading and all wheel phases stayed equal within floating-point rounding.
- During motion, 60 consecutive browser animation-frame samples had a median interval of 16.7 ms and wheel rotation changed on all 59 intervals. This short desktop sample is not a sustained performance guarantee.
- Screenshots were opened and inspected: `output/playwright/turning-left.png`, `turning-right.png`, and `turning-reset.png`. Actual rendered Three.js transforms are recorded in `output/turning-browser-left.txt`, `turning-browser-right-paused.txt`, and `turning-browser-motion.txt`. Development builds expose the actual scene through the canvas for this inspection; production builds do not.

Final validation after the endpoint pose correction:

- Full backend suite: **494 passed, 3 known harness skips, 38 warnings**, 293.29 seconds. `output/turning-full-tests.txt`. The earlier 493-test result is retained separately and is superseded.
- Frontend production build and TypeScript passed; lint has **0 errors and 5 existing unused-symbol warnings**. `output/turning-build-final.txt`, `output/turning-lint.txt`. UI channel contract and lint dependency checks passed.
- Second actual running-server trip: **1689 snapshots**, parked and contained. Maximum active approach speed 10.06399 km/h, maximum maneuver speed 5.00086 km/h; minimum active red-stop distance 0.95199 m. `output/turning-live-final.json`, `output/turning-live-summary.json`.
- Browser-driven scenario reset: applied acknowledgement, epoch 1 to 2, sampled rendered old/new poses with maximum distance from either endpoint **0.0044 m**. No animated traversal across the map. New wheel phases reset to the new scenario's traveled distance. `output/turning-browser-reset-audit.txt`.
- Browser remained connected through backend restart and received the final scenario state. No browser errors were reported; the existing Three.Clock deprecation warning remains.

Both servers are running locally; the mapped mission is reset and paused at its start for manual inspection.

## Limits and next engineering step

The remaining 1.19 m worst approach error and 48 degree instantaneous road-heading error show that sharp junction turns are not yet precision lane tracking. Sparse polyline tangents and a fixed offset are not surveyed intersection lane boundaries. The next step is a curvature/steering-rate-feasible junction trajectory with measured corridor containment and left/right, wet-road and emergency-stop coverage. No general-road success rate is inferred from these missions.

This is an ideal no-slip kinematic model. Tire-force slip, physical suspension, steering-rack calibration, reverse parking and live parking vacancy/access are not validated. Body pitch/roll remain acceleration-linked visual effects. The compatibility `PhysicsEngine.update` path retains its existing lateral planner; the browser's active session uses the updated `Driver.control` tracking path.

Geometry reference: [ROS control Ackermann kinematics](https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html). Controller reference: [CMU steering-methods report](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf). The DDS bounded-preview extension and its metrics are local implementation evidence, not claims from those references.

All edits are local and uncommitted. No push, PR or deployment was performed.
