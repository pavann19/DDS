# Camera delivery fix — local sandbox

Parent sandbox commit: `b833c6f53024b630c1655c31d19e62635a6c4d95`. Main remains `390f62300c1e7a030fe2717e1f90e9d6834cb523`. No public push or main integration.

Changes:
- Capture at most once per new telemetry tick rather than applying another exact 100 ms wall gate that can skip the next tick.
- Permit at most two pending browser uploads; epoch changes/unmount abort all pending requests. Ingress keeps one processing bundle, rejects stale/future/replayed identities, and revalidates after processing.
- Ingress wall-rate guard is 75 ms instead of 100 ms to tolerate bounded arrival jitter. Freshness remains 350 ms. A regression accepts the next valid tick at 95 ms while existing excessive-rate and replay tests remain intact.
- Four bounded CPU workers recognize the eight immutable view frames. Results retain camera ordering; ingress remains busy until every worker finishes even on an exception. A regression verifies failed-view worker cleanup.
- Diagnostics include bounded visibility history and maximum render gap.

Validation: 81 targeted ingress, rig, camera-boundary, combined-health, lifecycle and freshness tests passed. Frontend build passed; lint has six warnings and no errors. Local five-run detector benchmark: sequential 24.08, 25.87, 24.55, 36.36, 31.70 ms; four-worker execution 17.80, 12.77, 12.51, 10.67, 12.41 ms. This benchmark excludes rendering, transport and scheduling.

Live browser and telemetry evidence:
1. Tick scheduling with sequential detector: 201 samples over ~20 seconds: 119 ready, 72 recovering, 10 stale; speed 18.83–47.97 km/h. Not accepted as smooth.
2. Parallel detector, single upload: 251 samples over ~25 seconds: 213 ready, 26 recovering, 12 stale; speed 31.84–48.97 km/h. Not accepted as smooth.
3. Two-upload pipeline: 250 samples over ~25 seconds: all ready, max camera age .30 seconds; speed 42.36–43.21 km/h following a lead. No sample had acceleration below -.1 m/s².
4. Longer follow-up: 401 samples over ~40 seconds: 360 ready, 14 recovering, 27 stale; max age .90 seconds. This prevents claiming sustained universal acceptance. Driving stayed between 37.60 and 44.29 km/h, but camera-triggered braking recurred.
5. Latest diagnostic build: 201 samples over ~20 seconds: all ready. Browser diagnostics recorded maximum render gap 165.6 ms and one initial visible-state entry. Subsequent browser state showed Checking roadside controls with healthy sensor status: missing/uncertain recognition is a separate legitimate hold, not the camera-delivery failure.

Raw local captures: output/camera-tick-scheduled.json, output/camera-tick-parallel.json, output/camera-pipeline-two.json, output/camera-pipeline-two-long.json, output/camera-render-gap-run.json. Different segments and durations are not a matched performance benchmark. No success percentage is claimed.

Remaining acceptance: investigate occasional long-run stalls with the new visibility/render diagnostics; validate roadside-control recognition and turning over the full route; complete timestamped camera/range observation fusion. The current change improves delivery, but does not prove complete smooth-driving acceptance or Level 5 autonomy. Keep fail-safe camera and junction checks enabled.
