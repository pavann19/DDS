# Sensor-driven roadside-control prototype — 2026-10-13

Baseline39e82935fd4e82a918aa76545f37d066e162c3d1 preserved by tag dds-camera-redesign-baseline-20261013. Changes developed in DDS_Next_Sandbox. No push, PR or deployment authorized.

## Architecture and actual scope

The previous map/schedule mode remains as an explicitly labeled baseline. New optional scenario parameter camera_controls=true selects a synthetic RGB camera prototype. Settings exposes Signs & signals; switching restarts the current scenario. The driving-view label and Road details distinguish the modes. Camera mode remains opt-in; this increment does not replace all sensing or establish general autonomous vision.

World control primitives own simulated signal cycles. A CPU pinhole renderer creates immutable RGB bytes, capture time and calibration. Classical pixel processing consumes only that frame, not map IDs, schedules, world object labels or visibility flags. It detects simplified red octagons/triangles and colored lenses within dark signal housings. Confidence is a shape heuristic, not probability. Range uses nominal modeled sizes and heuristic uncertainty, not surveyed depth accuracy.

Map expectations retain control locations/kinds for conservative approach planning. Camera detections associate to expectations by bearing/range/type; ambiguous associations are rejected. Three consecutive acquisition results are required. Missing/stale/future/occluded/uncertain evidence cannot visually confirm or release a mapped control. Missing detections do not remove map expectations. Source timestamps are not rewritten during observation. Conflict clearance also requires a finite nonnegative age. Signal camera mode does not permanently release a light just before the line; it continues checking until crossing. Brake hold eliminates tiny residual creep near camera-controlled stops.

The camera is an explicitly simplified CPU sensor renderer, NOT the Three.js scene and NOT a physical camera. It renders supported mapped control primitives, with a140-degree modeled field of view and manual occlusion/dropout fault modes. It does not yet render photorealistic roads, natural NPC occlusion, weather/lighting, or detect arbitrary unmapped scene controls. Detector range is limited; map-prior braking is essential. Unassociated detections are not yet handled as general road hazards. This is a tested boundary/prototype, not real-world recognition or complete independent perception.

Trained ML remains internal vehicle analytics only. The new control perception uses classical image processing and deterministic planning/safety. No learned driving policy or scene model was introduced.

## User-visible controls

Settings -> Signs & signals -> Synthetic camera prototype. Vehicle -> Road -> Show camera frame displays the actual acquired frame, with capture-time cache busting and explicit source label. The read-only PNG endpoint returns404 when no camera frame exists; requests never synthesize fresh images. Paused simulation freezes capture age in simulation time, consistent with other simulated sensors.

Recorded signal fixture: East Millbrae Avenue way396939375, control node6386743913. Geometry/control identity are recorded; current colors are simulated world state, observed through pixels in camera mode. The world still needs predefined content; the change isolates that truth from the driver rather than pretending simulation has no truth.

## Validation findings and corrections

15 renderer/detector tests cover pixels-only phase mutation, contradictory lenses, partial housing/sign occlusion, blank/drop/FoV/far images and immutable frames. Boundary tests use actual renderer/detector positives, not injected golden detections; malformed timestamps are unit-only negatives. They check hidden-schedule isolation, fresh STOP confirmation, reset, faults and actual Session occlusion hold.

Initial combined suite:942passed,3skipped,1failed. The new scenario used unsupported category safety; corrected to safety_critical without weakening registry validation. Raw output/camera-full-regression.txt retained. Live occlusion also exposed residual tiny creep; camera-only bounded brake hold added and tested. A separate saved camera-view SSR hydration mismatch was reproduced and fixed by restoring saved state after mount, preserving the preference.

Initial live runs at8cadf23, actual headed browser plus WebSocket: STOP387samples, observed low-speed dwell18.1–20.4s then movement; fully occluded STOP315samples, zero confirmed controls and no release, residual velocity2.39e-5m/s; signal317samples, red recognition/hold then proceeding. Those initial captures precede exact-zero brakehold and are retained separately, not overwritten. output/camera-live-summary-initial.json.

Final gate counts, source revisions and live observations are appended after completion. Camera processing is CPU work, initially measured17.6ms/frame for three synthetic objects; no hard real-time or production-performance claim. Camera acquisition runs at perception cadence, not every control call.

## Outstanding development

Shared Three.js camera/sensor scene geometry, independent unmapped-control tracking, natural depth occlusion, robust lighting/weather/shape false-positive evaluation, calibrated depth/covariance and friction-aware stopping prediction remain open. Map coverage and widths remain partial/modeled. Existing full-plant continuous swept-body and Level5 limitations are unchanged.

## Exact local implementation commits

- `b0e4f0bd9cda9b0f3388fcf46f68e54eabfbf11d` — feat(camera): render pinhole RGB controls and detect pixels without truth labels
- `94c25c8f4a7b0dbf6eb4d3806762b73497073857` — Verify RGB control boundary fails closed without fresh positive evidence
- `8cadf239f57aa2654d74ea2c3d848d0f28234bcc` — feat(perception): isolate opt-in RGB control sensing from map schedule baseline
- `9d6f040cfcea610990c7765a4cc602c40be690e3` — fix(hmi): restore saved camera view after initial hydration
- `df22f190b569766fb188d45e901a691b292d7fe9` — fix(perception): hold camera-controlled stops and validate scenario metadata

## Final acceptance

Full combined suite at df22f19: **944 passed,3 skipped**,61 warnings,297.09s (output/camera-final-regression.txt). Exact-zero camera hold and unsupported-category correction are included. Application code remained frozen during final checks. TypeScript, lint and final production build results in parent record; only five existing lint warnings, zero lint errors.

Final actual headed-browser/WS runs: normal camera STOP311 samples, reaches exact0 and resumes; fully occluded STOP317samples, final velocity0, no confirmed controls, remains before line. Recorded signal red preview: actual velocity0, visual control confirmed, phaseRED, acquired640x360 RGB loaded in Road panel. Resumed5.1s then paused on green; saved acquired PNG frames independently reprocessed through pixel-only detector return signal/red and signal/green respectively. Artifacts output/camera-final-live-summary.json, camera-live-pixel-check.json, camera-final-live-cases.txt and output/playwright/camera-{red,green}-preview.png. These are finite controlled synthetic observations, not recognition success rates or guaranteed collision avoidance. Raw initial evidence retained separately.

Fresh browser reload after hydration fix showed zero application console errors and the existing Three.Clock warning. No publishing action follows this acceptance.

The green capture rows include paused heartbeat states; they are not independent camera frames or accuracy samples. RGB detector measurements and freshness use producer capture time, not heartbeat count.
