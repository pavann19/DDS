# Smooth-drive fix batch — sandbox only

Baseline main SHA: `390f62300c1e7a030fe2717e1f90e9d6834cb523`. Parent sandbox SHA: `c2b10d8575bcaff066eb1988cba4f4aee30b5586`. This batch preserves prior sandbox integration alongside the following fixes. It is not approved for main integration: sustained smooth motion failed live acceptance.

- Legacy longitudinal path now brakes instead of returning an accelerating speed override while required health is blocked. Regression reproduces the original defect at 20 km/h.
- Invalid current time clears each camera's reported age, recovery and current flag. Overall guard continues failing closed.
- Scene capture minimum interval is 100 ms instead of 200 ms. One request at a time, unique telemetry ticks, ingress rate limits and 350 ms freshness remain enforced.
- Canvas data-camera-audit exposes capture/transport timings and accepted/rejected counters for local diagnostics.
- Safety panel reports Sensor safety hold and Blocked when sensor readiness is false, even if the independent shield approves its own check. Verified in the actual browser after restarting the production frontend.
- Cockpit distinguishes Slowing from Stopped using measured velocity during unavailable sensor readings.

Validation: 21 targeted camera/combined-health tests passed. Frontend production build passed. Lint results reported separately in tool output. Main app and branch were not changed.

Live failure evidence: output/camera-cadence-100ms.json records 151 telemetry samples over approximately 15 wall seconds: 121 stale, 30 recovering, no ready samples, speed 0 throughout, max camera age 1.10 simulation seconds. Subsequent browser observations showed intermittent recovery and motion at 6 km/h, not sustained acceptance. Browser reported document.visibilityState=visible. Individual sampled capture durations were approximately 31–39 ms and HTTP/CV roundtrip approximately 59–62 ms. These samples do not measure the gaps between render-loop invocations; that remains a key diagnostic gap.

Unfinished: bounded publisher recovery, stable rendered-scene sensor production, measured render cadence, timestamped camera/range observation fusion, separate calibrated camera mount transforms, verified shoulder pull-over. Do not describe this batch as completion of smooth driving or full sensor fusion. No public push or deployment.
