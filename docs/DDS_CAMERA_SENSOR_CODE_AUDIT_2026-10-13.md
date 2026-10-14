# Camera and sensor code audit

Inspection: 2026-10-13. Sandbox HEAD `c2b10d8575bcaff066eb1988cba4f4aee30b5586` plus existing uncommitted integration changes. Main checkout unchanged. This is an investigation, not completion of sensor fusion.

## Findings

### High: camera timing repeatedly interrupts motion

`SceneControlCamera.tsx:63-64` suppresses hidden-tab capture, caps production at 5 Hz, and allows only one pending request. Frames carry the most recent 10 Hz telemetry timestamp, not a new independent sensor clock. Each bundle must finish transport, all eight detector calls, ingress revalidation, and the next perception boundary. `sensor_freshness.py:158` declares the camera stale after 350 ms and requires three distinct captures for recovery. `driver.py:747` requests braking whenever combined health is blocked. Recent live evidence shows 0.40-second age and recurring 1/3 recovery. This explains repeated command switching; GPU/transport/telemetry contributions have not yet been individually measured.

No freshness threshold was relaxed. No live-world perception claim is supported.

### High: health gating exists, observation fusion does not

`physics_engine.py:1258` requires all eight camera views. `control_camera_bridge.py:64-65` stores auxiliary detections but uses only front detections for mapped-control association. Traffic tracks come from simulated range sensors. There is no cross-camera object association, camera/range measurement association, fused object covariance, or unified observation freshness contract. A diagnostic-view failure currently blocks the same way as loss of a driving-critical observation.

### High: legacy controller bypasses sensor-stop policy

`driver.py:548` returns a speed override before the sensor-health braking block at line 747. Reproduction with a recorded highway route, 20 km/h and blocked sensor health requested 21.2 km/h with reason cruise. This is a compatibility-path defect; current browser use of that branch has not been demonstrated.

### Medium: abandoned camera publisher cannot recover within epoch

`scene_camera_ingress.py:60` rejects a new publisher for the same session/epoch indefinitely. Live replacement-tab requests returned 409; Restart current drive restored accepted frames. Add explicit ownership transfer/recovery without allowing simultaneous publishers or sequence replay.

### Medium: invalid-clock diagnostics retain healthy camera rows

After warming all eight views with stamps 0.0, 0.1 and 0.2, evaluating with NaN current time correctly returns ready=false, but all eight per-view rows remain current=true with recovery=3. The invalid-time branch resets counters without rebuilding the reported rows. Controller fails closed; diagnostics are misleading.

### Medium: HMI shield approval conflicts with sensor readiness

Live Safety panel retained Approved and Monitoring your drive while sensor readings became Unavailable and camera recovery became 0/8. Overall readiness must include sensor health. Cockpit Stopped wording must also distinguish a vehicle still slowing from a measured standstill.

### Architecture gap: camera mounting geometry

`SceneControlCamera.tsx:69` positions every camera at the same vehicle-center location and changes yaw only. These are eight directional scene views, not eight separately mounted cameras around the body. Distinct modeled mount transforms and calibration are required for realistic surrounding coverage and fusion.

## Measured detector sample

Fetched all eight accepted PNG previews from sandbox API and decoded to RGB. Five sequential detector-bundle runs: 21.82, 25.32, 24.93, 19.73 and 20.11 ms. All had zero recognized controls in that captured scene. Payload is 1,900,800 RGB bytes per bundle. These timings exclude GPU rendering/readback, HTTP transfer, scheduling and boundary consumption; they do not prove an end-to-end frame rate.

## Required integration sequence

1. Instrument capture, ingress and consumption latency plus rejection reasons using one coherent clock; reproduce foreground and background behavior.
2. Correct controller bypass, invalid-clock health rows, ownership recovery and contradictory HMI states.
3. Define a timestamped environment-observation interface with sensor provenance, validity and uncertainty. Associate camera evidence with simulated range tracks in calibrated coordinates. Never let fresh ranges imply an observed green signal or let stale image evidence authorize a junction crossing.
4. Add explicit per-capability degraded behavior and bounded stopping policy. Smoothness must come from timely observations and a stable controller, not hiding failures or substituting predefined signal phases.
5. Validate sustained motion, camera dropout, range dropout, contradictory evidence, turns, junction holds, resets and reconnection in the browser with live telemetry before integrating into main.

ML remains restricted to internal vehicle analytics under the user's requirement. Deterministic perception, fusion, planning and safety algorithms can implement this integration.
