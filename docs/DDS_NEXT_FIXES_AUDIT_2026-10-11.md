# Next fixes and improvements — inspected 2026-10-11

Investigation only: no application code changed during this pass. Inspected
the current dirty worktree at HEAD `a4c9072`, the engineering blueprint, recent
validation report, live telemetry and an isolated browser. The live drive was
not reset or reconfigured. Reproductions that alter safety/scenario inputs
ran in separate in-process sessions, not against the user's drive.

## Prioritized fixes

| Order | Finding | Evidence | Acceptance check |
|---|---|---|---|
| 1 | Safety verdict misses existing overlap and vehicle-body road departure | `safety_shield.evaluate` approved a -1 m lead gap at equal 10 m/s speeds, and at equal zero speeds. It also approved a center 0.2 m inside the 7 m road edge; vehicle dimensions are absent from its boundary API. | Detect occupied-space overlap independently of closing speed; check the full body and swept footprint against road bounds. Verify applied response and explicit collision/containment status, including stopped/side/rear cases. |
| 2 | Rejected scenario commands can still change the world | Isolated session at tick 10 received low_friction_curve with friction_mu='invalid'. Conversion raised ValueError, but scenario became low_friction_curve, station reset to 0 and speed became 60 km/h; epoch stayed 0 and session clock stayed 10. Parameter validation happens after setup mutates the world. | Validate all command fields before mutation; prepare a candidate state and commit atomically. Rejected commands must leave geometry, ego, traffic, scenario, epoch and clock unchanged. |
| 3 | Arrival is missing from the HMI contract | Isolated arrived session had DrivingState.ARRIVED and has_arrived=True. Ego telemetry contained velocity=0, target_velocity=0 and speed_limit_reason='cruise', without arrival/driving-state fields. Frontend derives Holding from speed rather than an arrival event. Live browser showed a paused zero-speed vehicle and generic road guidance; that paused display alone is not an arrival reproduction. | Publish approaching/arriving/arrived and remaining distance; display destination reached distinctly from pause, traffic stop and offline state. Confirm in a real browser at route completion. |
| 4 | Map error remains after connectivity recovers | In an isolated browser, aborted one OSM tile, then allowed requests and zoomed. Four tiles loaded but the map still showed Map tiles unavailable. Component only sets unavailable=true. Screenshot: output/playwright/cockpit-redesign/audit-map-recovery.png. | Track current tile failures, clear stale errors after successful retry, offer retry and re-center/follow controls. Verify blocked/offline/recovered states in the browser. |
| 5 | Runtime headroom remains load-sensitive | Prior full run: 424 passed, 2 timing failures, 3 skipped. Those timing checks passed reduced-load. This pass's live snapshot at sim time 297.28 s recorded perception/planning/control deadline-miss counts 6/3/16 and maxima 70.97/59.27/89.71 ms. This is one accumulated session snapshot, not a new controlled benchmark. | Profile allocations and per-stage CPU under a fixed workload. Measure scheduling lateness separately from stage execution time. Retain existing thresholds and validate with UI, analytics and recording active; do not infer hard real-time operation from real-time factor near 1. |

## Next functional improvement

Real road coordinates currently supply the centerline, not a semantic road map.
Road width is still fixed at 7 m per side, lane offsets are predefined, and
cruise policy uses configured speeds. Add road direction, lane count/width,
posted limits and supported junction/control metadata before expanding from
highway driving into city destinations. Unknown semantics need an explicit
unsupported/limited-driving state, not an implicit assumption of a clear road.
ML remains internal telemetry analytics; road-rule decisions belong to the
intelligent driving and independent safety systems.

## Checks that did not establish a new bug

Isolated sessions fetched HTTPS OSRM routes from the recorded highway origin
to all three UI choices: Burlingame (109 points), Redwood City (312 points),
Daly City (544 points). All three passed the atomic route installer. This
checks route fetch/install at that origin; it does not validate complete trips,
traffic rules or every possible reroute position.

## Recommended execution order

Implement safety geometry and atomic scenario validation first, followed by
arrival UX and map recovery. Keep the runtime profiling work measurable and
separate from a semantic-map expansion. No success percentage is inferred.

Nothing was pushed, committed, deployed, or sent to an external collaborator.
