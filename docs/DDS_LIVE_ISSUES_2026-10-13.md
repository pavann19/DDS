# Live issue investigation — 2026-10-13

Main checkout remains at `390f62300c1e7a030fe2717e1f90e9d6834cb523`. Investigation used sandbox frontend 3001 and backend 8001. No push, deployment, or main integration.

## Confirmed observations

1. The sandbox production frontend process served an earlier build. Restarting the frontend process made the latest eight-camera gallery and safety-hold wording visible. Previous reload-only checks did not validate those changes.
2. Opening a replacement browser tab produced repeated camera-ingress HTTP 409 responses. `SceneCameraIngress.validate` binds the publisher to the session/epoch with no expiry. The old tab was absent from the browser inventory. Restart current drive reset the epoch and restored publishing. The single-publisher guard is intentional, but abandoned-publisher recovery requires an explicit mechanism.
3. Even after reset, live camera health oscillated. At ticks 110–130, capture ages ranged from 0.10 to 0.40 simulation seconds; readiness alternated between ready, recovery and stale. Recovery frequently returned to 1/3. The 0.35-second freshness guard remains enabled. Hidden-tab behavior exists in source, but it does not fully explain this foreground-run observation. GPU capture, transport, CV processing and telemetry timing still need measurement.
4. Eight actual rendered-scene preview images and per-view health entries are now visible in Vehicle > Road. They are simulated camera views. Side/rear views are diagnostic captures, not object-level traffic fusion inputs.
5. Vehicle > Safety continued to show Approved and Monitoring your drive when Sensor readings was Unavailable and Camera view recovery was 0/8. Shield approval is distinct from sensor readiness, but the current presentation conflates them.
6. Vehicle motion was observed at approximately 5 km/h with a zero target speed during recovery. A controlled stop has duration; the new cockpit's unconditional Stopped wording for unavailable camera data is misleading while moving.

## Corrections to prior investigation statements

The frontend store already reads `payload.channels.pose.ego`. The earlier claim that it read the wrong outer telemetry object was incorrect. The diagnostic probe had read the wrong path. The new gallery's absence in prior browser observations was attributable to the stale running frontend build, not proof that its JSX branch was unreachable.

## Still unfinished

Pause freezes the simulation and publishes hazards; no verified shoulder pull-over exists. Eight-view traffic fusion, sustained camera capture acceptance and real mapped environment coverage remain unfinished. No success percentage or end-to-end autonomy acceptance is claimed.

## Next engineering work

Measure end-to-end camera cadence before changing producer rate or image size. Add explicit abandoned-publisher recovery without allowing competing tabs. Make overall HMI readiness depend on sensor health as well as shield verdict, and use measured speed for Slowing versus Stopped. Validate those fixes in the sandbox browser before integration.
