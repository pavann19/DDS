# Shared 3D scene camera increment — 2026-10-13

Baseline: `834d98df6e72d9ba2f72296d176cf99bfede2360`, preserved by local tag `dds-scene-camera-baseline-20261013`. Developed and tested in `DDS_Next_Sandbox`. No push, PR or deployment.

## Implemented boundary

Settings → Signs & signals → **3D scene camera prototype** selects the optional shared-scene mode. Existing map/schedule and simplified CPU-camera modes remain available. The camera renders the actual Three.js scene into a 640×360 framebuffer at a maximum five captures per wall second. It uses the telemetry vehicle pose, modeled 140-degree horizontal pinhole field of view, depth buffering and an ego-hidden camera mount. It captures GPU scene geometry, not HTML overlays or a generated list of control labels.

World-owned roadside meshes and simulated light colors are now independent of the driver's interpreted junction status. World controls travel through the display-only `heavy.scene_controls` channel. They do not feed driving observations. This prevents a circular camera that sees only the driver's existing belief.

The camera transport contains top-first RGB bytes, calibration contract and capture identity/time. It contains no object IDs, signal colors, map schedules or driving commands. Backend ingress bounds body size, concurrency, publisher cadence, session/epoch, capture age, sequence and telemetry-tick consistency. It revalidates after asynchronous pixel processing and publishes one pending immutable result. The simulation consumes that result at its perception boundary. Scene mode has no fallback to the CPU truth renderer or hidden signal schedule.

Classical image processing recognizes the supported high-contrast sign and three-lens housing geometry. Three distinct consistent captures are required; repeated frames cannot accumulate confirmations. Missing, ambiguous, conflicting or stale observations cannot grant entry. Signal range remains a modeled-size proxy over a supported 1–1.5 m head-height interval, with explicit size ambiguity; it is not calibrated real-world depth. ML remains internal vehicle analytics and does not command driving.

## Defects reproduced and fixed

1. Explicit renderer `setViewport` multiplied the sensor dimensions by display DPR and cropped controls out of the framebuffer. Fixed by retaining the render target's pixel-sized viewport. Actual captured before/after images retained.
2. Actual 3D signal heads were connected to poles and showed a perspective side face; the simplified detector rejected them. Broad-head extraction and three visible lens checks now support captured red/green fixtures, while retaining occlusion and conflicting-lamp rejection. The incomplete intermediate detector failed three existing tests and was not integrated.
3. The previous camera release threshold required continued sight until two meters beyond the modeled stop line. Actual roadside lights can leave the wide camera field of view after valid entry; the recovery run then commanded braking inside the intersection. A fresh visually permitted crossing is now committed when the vehicle center crosses the modeled line. Unknown evidence before the line still holds; unknown evidence after the line cannot itself create permission. Scenario/route resets clear the latch.

## Validation evidence

Application revision for final validation: `5ed40dded14e3e0ec35dad7cdf147de91209ab94`.

At detector revision `3b8a5dd`: full backend suite **973 passed, 3 skipped**, 61 warnings, 314.28 s. Final crossing revision: targeted junction/camera suites **33 passed**. Detector suite **22 passed**, including seven captured-scene cases: red/green, each missing lens, contradictory lamps and head erased with support pole remaining. Boundary fixtures and transport tests are separate from actual pixel end-to-end evidence; injected unit detections are not a vision accuracy evaluation.

TypeScript and lint passed; lint has five existing warnings and no errors. Production frontend build passed. Raw artifacts: `output/scene-camera-resume-build.txt`, `scene-camera-resume-full-regression.txt` and final revision regression log.

Actual headed browser and live WebSocket telemetry at final revision: recorded signal red preview speed exactly **0 m/s**, phase RED, one fresh visually confirmed control. After green, speed **0.820 m/s**; after continuing through the intersection, **12.392 m/s**, reason `cruise`. All 47 captured states beyond the modeled line in that interval retain `cruise`, including loss of the light from the image. This is a finite controlled observation, not a recognition success rate or complete intersection-safety guarantee. Raw `output/scene-camera-crossing-live-raw.txt` and `output/playwright/scene-camera-final-{red,green,crossing}.png`.

Camera starvation trial before the crossing correction: a background-tab trial reduced capture cadence rather than stopping it entirely. In 23 stale telemetry states, maximum capture age was 1.2 s and maximum speed was **0 m/s**. The last state was fresh again with zero confirmed controls and `signal_unknown`; do not describe it as a continuously frozen camera. Raw `output/scene-camera-starvation-{raw.txt,tail.json}`. Native window minimization and paused duplicate-frame 409 responses are retained separately; they are not app hydration failures or accuracy samples.

## Remaining limits

This is an explicitly simulated camera and supported-geometry prototype. It is not physical autonomy, Level 5, a production vision stack or a measured real-world recognition rate. It still uses map location/type priors; arbitrary unmapped controls are not handled. Most surrounding roads are recorded centerline context, not surveyed lane surfaces. Light cycles, signs and modeled stop offsets belong to the simulated world.

The browser is the sensor producer; foreground rendering and transport latency matter. Actor interpolation is the displayed HMI interpolation, not a certified synchronized sensor snapshot. Sensor rendering uses no tone mapping for predictable colors; display rendering uses its existing tone mapping. HTML sign text is not in RGB, and no OCR is implemented. GPU readback and approximately 3.46 MB/s raw RGB at five Hz have no new performance guarantee. Camera range, weather, lighting, false-positive robustness, calibration and independent unmapped-object tracking remain development work.

The crossing latch is route/session control-ID state; released controls no longer run junction-specific clearance checks. The independent safety monitor continues, but full post-entry intersection conflict safety is not established. Crossing uses the modeled vehicle-center stop line, not a surveyed bumper position. No source sensor should be represented as a physical camera or real signal feed.

## Exact local implementation commits

- `432ac3797debe99aede316c00bab385f5f1bd629` — bounded raw RGB ingress and capture identity validation.
- `24f0bd6ddb334751e69c25831856bb94c525643c` — actual Three.js scene camera capture and transport.
- `1ec2baea34a3bf0cbf1b627099667834e3bf1141` — scene boundary tests without CPU fallback.
- `f63ac815ae259d5fcbd3fb1d49bb1a59dcf609ce` — preserve framebuffer viewport across display DPR.
- `387c937a0ed3f8e4c80d050d145d58c60995ad01` — separate world controls from driver observations and add scene mode.
- `3b8a5dd91738125b24d7a7186cc0b90bf0cb795d` — recognize captured signal heads and reject incomplete/conflicting lenses.
- `5ed40dded14e3e0ec35dad7cdf147de91209ab94` — retain fresh visually permitted crossing after entering the intersection.

## Final gate

Full backend suite at `5ed40dd`: **977 passed, 3 skipped**, 61 warnings, 322.35 s. `output/scene-camera-crossing-full-regression.txt`. Source remained fixed during this run. Frontend source was unchanged after the successful TypeScript/lint/production build described above.

Live GPU geometry occlusion: a diagnostic clone of the existing opaque circle mesh, scaled and placed in front of the recorded STOP face, blocks the actual rendered pixels. No golden detections, backend visibility flags or object truth were injected into the driver. At 28.92 simulated seconds: **0 m/s**, 0 detections, 0 confirmed controls, fresh frame, modeled line 0.843 m ahead; 291 telemetry states captured. Actual acquired PNG shows the opaque cover. Artifacts `output/scene-camera-geometry-occlusion-raw.txt`, `scene-camera-geometry-occluded-stop.png` and `output/playwright/scene-camera-geometry-occlusion.png`. The diagnostic cover is removed after the test and is not a project feature or permanent world object.

An initial uncover recovery ran with Chrome minimized and remained held with stale evidence; it is not counted as a successful foreground recovery. After explicitly restoring the native window, fresh captures confirmed the uncovered sign and the vehicle resumed, ending at **11.326 m/s**, reason `cruise`, with no active junction. The raw record identifies `recorded_stop`; `output/scene-camera-uncover-foreground-raw.txt` includes native window bounds showing the prior minimized state. These observations establish the bounded fixture behavior, not broad camera availability or accuracy guarantees.

All implementation commits above and this acceptance record are local. Integration is a fast-forward of the preserved main working branch after these gates, with no public operation.
