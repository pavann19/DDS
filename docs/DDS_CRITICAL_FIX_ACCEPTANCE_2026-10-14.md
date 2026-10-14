# DDS critical fixes — local acceptance, 14 October 2026

Status: candidate accepted by combined regression and an actual rendered eight-camera drive. Local main integration and native production builds passed; no public release is authorized.

This is a real-road simulator with simulated vehicle, traffic, sensors and infrastructure. ML remains internal powertrain analytics. The deterministic Driver, constrained planner and independent Safety Shield command driving. This work does not establish physical autonomy, general autonomous driving or Level 5 capability.

## Provenance

Main checkout baseline: `390f62300c1e7a030fe2717e1f90e9d6834cb523`, retained by local tag `dds-local-baseline-20261014`. Implementation sandbox: `feat/surround-driving-20261013`. Candidate source: `8631d5b5cbe63086f6b864031357017cd17daec3`. Main was fast-forward integrated after acceptance. The baseline tag remains available. No push, PR or deployment is authorized or performed.

| Local commit | Change |
| --- | --- |
| `df0f8934ff2d1e42ec564b94e4863cbd7b906d01` | Turn recovery, timestamped camera delivery and mapped safety presentation. |
| `7ab2de1ff5fe5fb3306a2675b3c4005f5dc93b16` | Passing-lane turn feasibility and repeated-world map coordinate picks. |
| `8e160f78ac9696c20c17cf009d912388199263b1` | Bounded raw camera upload worker. |
| `762621317338ba0126a937ba99fc94640bacf71c` | Exact standstill before parked status; explicit passing-lane replay. |
| `fa9c031d9c8ec4dec1bce9b62a8f4fffd9fd3f48` | Critical safety banner above journey planning. |
| `a9c9f25d6ff9bb40934767a02e5553f299481dfe` | Mandatory lane return survives camera and plan interruptions. |
| `c9fdcf3fb7432bfd1eeeac3a07a1cf542db448e2` | Finite 16 MiB recorder transport bound for long routes. |
| `1a9155a4a75f559881aaa028630d2b7da76f575b` | Bounded nearby fallback road rendering, including dense routes. |
| `da2ad73bd99728b4093167db0af6e8b1338c1650` | Preserve long OSM roads through local segment clipping; cache schema 3. |
| `da7cd796746618e557eda30b6bc29aa2750d125d` | Three-digit cruise values remain visible. |
| `bb2e587c3e1ee7ee754769705246b7f312b92412` | Initial jerk-aware signal-entry commitment; subsequent live replay exposed its insufficient 3 m bound. |
| `4fbd8e24d802cec62abeee14d1111ed75db01a9a` | Asynchronous eight-view GPU readback with identity, context and expiry checks. |
| `8480f337a3683e1c401c22ac3737a75e4ac5a20a` | Stoppable green approaches and conflict/vehicle-derived modeled entry zone; matching HMI state. |
| `e12678187492f851c96dc946163e98f5b1316040` | Single upload credit matches serial detector admission; skipped ticks are never queued. |
| `8631d5b5cbe63086f6b864031357017cd17daec3` | Repeated camera loss during a temporary launch cannot replace a mandatory lane-return destination. |

The intermediate signal commit is superseded by the final candidate. Failed live recordings are retained rather than counted as successful acceptance.

## Implemented root-cause fixes

- Retained passing lane +5.25 m was infeasible on the recorded bend. Anticipation requests a collision-checked return to +1.75 m; mandatory destination survives sensing loss and rejected plans. Low-speed launch retains actual lateral state and checked trajectory duration. The original friction, actuator, body and collision validators remain active.
- Road geometry no longer adds a wide painted navigation ribbon over actual mapped roads. Rendering uses nearby OSM surfaces, bounded fallback coverage, upward-facing caps and mapped building footprints with modeled facade details. Long ways are clipped rather than wholly discarded when an endpoint leaves coverage. Separate exit/re-entry runs are never joined across absent geometry.
- Camera acquisition follows unique telemetry ticks and committed actor/control state. Eight actual calibrated RGB views retain one original capture timestamp. Uploads use a bounded worker; asynchronous GPU reads restore renderer/actor/PBO state before waiting. Undefined, wrong-buffer, lost-context, wrong-epoch, hidden, disconnected and expired completions cannot become fresh observations.
- Critical footprint/swept-body/Shield failures take heading priority over paused, camera and ordinary maneuver states. The fixed global banner remains above the journey planner, including the short viewport.
- Green entry previously became unobservable before a valid passage was latched. The final controller derives a 6.4 m modeled entry zone from its existing conflict radius, vehicle front extent and stop offset. Before commitment it maintains jerk-, reaction- and friction-aware stopping feasibility. Only fresh observed GREEN and sustained sensor conflict clearance can authorize entry; preline standstill revokes old permission. No phase schedule is used as a perception fallback.
- Additional audit fixes cover route-cache identity/copying, malformed OSRM geometry, repeated-world map clicks, strict finite commands and origins, camera disconnect handling, analytics queue accounting, recorder epoch integrity and exact-zero parked status.

## Verification and failed evidence

Earlier full suite on `a9c9f25`: 1,133 passed, 3 skipped. Full suite on `bb2e587`: 1,157 passed, 3 skipped, 75 warnings in 450.73 seconds. These are historical candidate results, not the final-source result.

Full suite on `8480f33`: 1,163 passed, 3 skipped, 75 warnings in 417.61 seconds. Final repeated-outage fix: 73 targeted lane/turn/fallback tests passed. The final combined result is recorded below.

Final frontend: TypeScript and isolated production webpack build passed; all 14 verification scripts passed; lint has zero errors and eight warnings. The acceptance checkout uses a node_modules junction, which Turbopack rejects outside its project root; production webpack is used there. Native main-checkout Turbopack production builds passed after integration and after the map-outage fix.

Raw local artifacts remain in `DDS_Next_Sandbox/output` and `DDS_Signal_Acceptance/output`.

- Earlier interrupted default missions and camera-hold timeouts are failures/incomplete runs. A continued mission reached parking after frontend reconnect, but was not an uninterrupted healthy run from station zero.
- `passing-lane-final-eight-camera.jsonl` from the previous acceptance checkout returned to +1.75 m and passed the turn, then stopped 2.656 m past the first modeled signal line. Valid green disappeared before the old crossing-only latch.
- `DDS_Signal_Acceptance/output/signal-entry-bb2-failed.json`: 5,997 samples, 4,076 sensor-ready samples, zero recorded body/swept-boundary failures; first signal passed, second stopped 4.633 m past its line; 600-second timeout, not parked. The fixed 3 m entry bound clipped the actual stopping envelope.
- Eight PNGs from that stopped pose show only the right camera contains a signal head: 5 by 18 pixels, one-pixel-wide lenses. Detector thresholds were not relaxed to invent recognition. This illustrates the current classical-CV fidelity limit.
- `mandatory-return-8480-failed.json`: 4,498 samples, 3,151 sensor-ready samples, zero recorded boundary failures; 450-second timeout, not parked. A second camera outage classified the temporary launch's held offset as the original +5.25 m lane, despite a required +1.75 m return. The final fix prioritizes the required semantic destination; its regression explicitly interrupts a validated offset launch twice.
- Actual canvas audit during that failed run: one steady asynchronous capture measured 7.5 ms submission, 23.6 ms readback wait, 4 ms conversion, 35.1 ms total, and 65.2 ms upload. Earlier synchronous snapshot measured 91.3 ms capture and 78.7 ms upload. These are individual samples, not a FPS, mean, percentile or success-rate claim. Rejected overlapping uploads motivated the final single-credit flow control.
- Live invalid-footprint fixture at +6.8 m confirmed the critical heading/banner above an open planner at a 900 px viewport, including while paused. This is expected negative-test evidence, not nominal driving success.
- A long OSRM route had 38,898 points and a 1.52 MB initial route message. The recorder now handles it within an explicit finite bound; 51 actual subsequent state samples were captured. This is transport validation, not completed long-route driving acceptance.

Final source `8631d5b`: **1,165 passed, 3 skipped, 75 warnings in 458.11 seconds**. Evidence: `DDS_Next_Sandbox/output/critical-fixes-final-return-pytest.log` and its JUnit XML. The skipped cases remain unverified scope.

Final actual browser replay: **1,615 samples**, of which 1,557 are active rather than paused; 1,438 sensor-ready samples; **zero recorded body or swept-boundary failures**. The retained passing-lane start at station 4,200 m / +5.25 m returned to +1.75 m, passed the sharp turn and both signals, then parked at station 4,749.929616 m / -0.022517 m offset, **exactly 0 m/s**, inside the mapped parking polygon. All eight views were ready at the parked sample. The 206 recorded corner samples had a maximum +1.75 m lane-target error of 0.492393 m. Recorded junction status includes 20 observed GREEN samples and 10 observed RED samples. No success percentage is inferred.

This is an explicit staged approach, not a nominal uninterrupted mission from station zero: the ordinary finite eight-NPC world remains behind the staged vehicle. Earlier offline full-traffic and physical-envelope cases complement it; they do not replace live perception acceptance. Telemetry is 10 Hz while the controller runs at 50 Hz, so the live sample record is not exhaustive continuous swept-volume proof.

Active-sample health counts: 1 unmonitored, 1 sensor recovery, 86 camera recovery, 31 camera stale, 1,438 ready. These residual deadline/recovery holds are retained, not suppressed. The drive completed without weakening freshness or physical guards. This does not prove universally smooth driving or a hard real-time guarantee.

Evidence: `DDS_Signal_Acceptance/output/final-rendered-eight-camera-replay.json`, `.jsonl`, `final-replay-metrics.json`, `final-turn-signal-moving.png`, `final-parked.png`. Browser reload closed/reopened the actual WebSocket while retaining the same session ID, epoch 1, parked pose and zero speed. The separate after-reload record contains 101/101 fresh-sensor samples and all eight views ready. Evidence: `final-browser-reconnect.json`, `final-browser-after-reload.json`.

Local main integration/build outcome: passed. Normal backend 8000 and production frontend 3000 are running, paused at station zero; the original telemetry database is preserved. Actual main-browser DOM and screenshot confirm connection and real-map rendering (`output/critical-fixes-integrated-main.png`). This paused view is startup verification, not another completed drive.

Additional implementation: `f06a1f62e673f07c1fd4f5bab533339d3b7212ed` adds 62 byte-identical attributed OSM records for partial default-mission corridor coverage during provider outages, original retrieval dates, integrity validation, bounded reads, 60-second retry and an explicit recorded-data caption. Identical record hashes preserve geometry-array identity to avoid repeated mesh rebuilds. `29a6a52` disables Git text conversion for integrity-pinned JSON: main checkout tests exposed Windows manifest newline conversion and this was corrected before runtime use.

Final map-context tests in main: 27 passed, 3 warnings; TypeScript, targeted lint and geometry-identity verification passed in sandbox. Native production build passed. These additional tests supplement the 1,165-test driving result above; the whole suite was not rerun after the static-map fallback addition. Main provider requests initially returned Overpass 406 and official OSM 509, then recovered: final real HTTP response is available with 16 features and 52 infrastructure records, retrieved 2026-10-14T11:55:03.118175+00:00. `output/recorded-surroundings-live.json` records the recovered live response, not a fallback response. Outage behavior is verified by failure-injection tests, not claimed as a live-browser outage acceptance. Old records are partial and never represented as current free-space sensing.

## Camera timing continuation — retained changes and failed experiments

Retained main code: `12ab525695a1d88aa406506b459f2e081a6991ae` removes an unnecessary signed RGB copy and dark-mask reduction with unchanged detection thresholds; `e68a482ea6e6736cf234c1f3907871b60ad8b62f` exposes detection duration/capture age, adds a reproducible offline benchmark, and removes per-display-frame audit DOM serialization. `afdef3d` excludes HMI grid/planned-path/risk annotations from optical captures and restores their visibility before display rendering. `f698e335322904299250915c3be45c986207cf8c` clears the maneuvering flag only after verified parking, retaining parking context and all containment/standstill requirements.

The sandbox overlap experiment (`e167e73`) and admission-pacing experiment (`802ce2b`) are **rejected and reverted** (`6dacca0`, `337f996`). The paced run remained almost stationary with repeated stale/recovering states. Completed readbacks waited too long before upload. Neither experiment was integrated into main. All failed/interrupted journals remain under `DDS_Next_Sandbox/output`; compiler load overlapped earlier runs, so they do not provide a clean causal performance comparison.

Detector equivalence and camera ingress/rig/boundary/disconnect tests: **60 passed**, 3 warnings. Parking tests: **4 passed**, including the modeled complete mission and continued zero-speed hold. Frontend camera verification scripts, TypeScript, sandbox webpack build and integrated native Turbopack builds passed. The full 1,165-case suite above belongs to the earlier critical-fix source; it was not rerun for this continuation.

Actual saved eight-camera pixels, 30 alternating serial-bundle benchmark rounds: first run medians 47.80 ms reference / 21.37 ms optimized; reproducible script rerun medians 32.09 / 12.95 ms, with identical detections. These are local sampled CPU measurements under different machine loads, not browser FPS, fleet success or end-to-end latency guarantees. Reproduce with `scripts/benchmark_camera_pixels.py --frames <saved-eight-PNG-directory> --output <JSON>`; reference source is pinned to `ef518b21ce1f6ccaa2790a7ce6855177c5091fd3`.

Clean serial station-zero recording before diagnostics: 1,789 samples, 1,662 active, station 1,338.930621 m, zero recorded boundary failures; 948 ready, 620 camera recovery, 92 camera stale, 1 unmonitored and 1 range recovery. It ended without completing the route. Later diagnostics-only 90-second recording: **888 samples, 742 active**, station 639.418947 m, zero recorded boundary failures; 631 ready, 77 camera recovery, 32 camera stale, 1 unmonitored and 1 range recovery. Sampled peak absolute longitudinal jerk 2.5 m/s3; 378 active samples had acceleration below -0.1 m/s2. Braking includes traffic response and cannot all be attributed to cameras. Different durations/load prevent a direct improvement percentage. One actual successful upload showed 37.2 ms capture, 12.26 ms detection, 46.6 ms transport and 0.16 s capture age.

Actual annotation-free RGB signal replay: staged at 4,470 m, not station-zero full-route acceptance. **1,383 samples, 1,248 active, 1,212 ready, zero recorded boundary failures**. Both signals passed, with 68 RED and 36 GREEN junction-phase samples; parked inside the mapped area at 4,749.929616 m and exactly zero speed. Residual active health counts: 13 camera recovery, 21 stale, 1 unmonitored and 1 range recovery. Actual PNGs show physical signal/road/trees/buildings without HMI annotations. This replay exposed `parked=true` alongside `maneuvering=true`; the subsequent parking correction was tested separately and must not be represented as the source of that earlier replay.

Evidence: `DDS_Next_Sandbox/output/camera-byte-reproducible-benchmark.json`, `camera-diagnostics-final-tests.log`, `parking-terminal-state-tests.log`, `camera-serial-optimized-clean.json`, `camera-diagnostics-runtime.json`, `camera-physical-scene-signals.json`, their journals, and `camera-physical-scene-png/`. Main integrated build log: `output/camera-physical-scene-integrated-native-build.log`.

**Status at this earlier checkpoint, superseded by the completed replay below:** station-zero-to-parking acceptance was pending. Remaining GPU/transport tails, general camera/range object-track fusion, and other targeted geometry/traffic/destination/world improvements were not completed. Freshness limits, serial admission and validators remain unchanged.

## Turn-clearance continuation — current measured result

Local code `8fa14ff` exposes actual lane-state provenance and keeps physical turn constraints ahead of unrelated unknown-junction headings. `452749823c7ce89e0925f0290d344fe12ed8f227` brakes before a blocked geometry-required return enters the bend, preserves the original required lane after a rejected prepare, checks measured initial steering, and constrains stationary prepositioning with the existing actuator and footprint validators. Main source matches sandbox `c6521cb886b2dc788d2211a311712407679fc67e` for app/frontend/scripts/tests. Full sandbox regression: **1,176 passed, 3 skipped, 75 warnings**, 467.59 seconds; integrated native production build passed. Evidence: sandbox `output/turn-clearance-full-pytest.log`, `turn-clearance-full-junit.xml`; main `output/turn-clearance-integrated-native-build.log`.

The earlier station-zero replay on `cdf8a8c` timed out at 4,355.560424 m: 8,989 samples, 8,423 sensor-ready, zero recorded boundary failures, not parked. The controller correctly refused physically infeasible forward launches after reaching that pose; this was not a successful full mission.

The second ordinary station-zero replay on `4527498` passed that bend but **also timed out**, now at 4,522.452364 m: **8,987 samples, 7,378 sensor-ready, zero recorded boundary failures, not parked**. Its 900-second wall-time window contains 883.6 simulated seconds. No reset, staged pose, or forced camera evidence was used during this run. At the terminal sample all eight camera views were fresh, but the first signal remained visually unrecognized (`signal_unknown`, node 1447264315, modeled stop-line distance 0.805436 m). The raw front PNG visibly contains a narrow side-facing signal. The vehicle safely remained stopped; this remains an actionable perception/rendering issue, not a passing journey.

The same replay also exposed prolonged ordinary lane-abort recovery. Investigation identified stationary wheel commands in the fast control loop during camera unavailability; a narrow correction is being tested separately in the sandbox. It is not yet part of the main commit above. Evidence: main `output/turn-clearance-normal-mission.json`, `.jsonl`, `turn-clearance-normal-stop.png`, `turn-clearance-stop-front.png`. These results supersede any suggestion that uninterrupted station-zero-to-parking acceptance is complete.

## Sandbox follow-up — recovery and new live failures

Sandbox `d3c84e389cbd8ac0eae3ee5040c30a2a9e8fe576` holds measured steering below 0.05 m/s during an unavailable-sensor hold. This fixes wheel winding in the fast control loop, retaining moving control, braking, lane intent and the original physical validators. Combined targeted regression: 107 passed. Sandbox `6c3ba8bebba69a6a8b436fd990ca55b037cfbde2` supports three fully resolved, consistently foreshortened signal lenses; unresolved, occluded, clipped or conflicting evidence remains nonpermissive. Detector/boundary tests: 66 passed. Combined source full regression: **1,197 passed, 3 skipped, 75 warnings**, 502.85 seconds (`DDS_Next_Sandbox/output/recovery-oblique-full-pytest.log`). These are sandbox results, not main integration or successful journey evidence.

Live staged abort replay began at the actual recorded 2,394.991289 m / 2.088447 m lateral pose, with measured wheel angle and eight seeded NPCs. The first 90-second record had 886 samples, only 163 sensor-ready, and progressed 0.00693 m. Ready windows had median 0.2 seconds and maximum 0.3 seconds; camera holds repeatedly interrupted checked jerk-limited launches. One audit sample had 417 ms GPU readback. After pausing the archived main simulation, a later 60-second interval advanced from 2,401.430037 to 3,138.335561 m, 601 samples / 461 ready / zero recorded boundary failures. The intervals differ in time and workload; they do not establish a causal improvement percentage.

Continuation reached and visually recognized first signal node 1447264315 as GREEN using live scene RGB, but **the drive failed**. In 3,001 continuation samples, 2,271 were sensor-ready, zero recorded boundary failures and **118 actual occupied-space overlap samples** occurred; it timed out at 4,545.543964 m without parking. The world NPC following implementation did not include ego as a leader, permitting rear NPCs to drive through the stopped ego. That critical simulation bug is under correction. The live recorder also incorrectly permitted a parked outcome despite an earlier overlap; sandbox `a9b513ff4611b2e1de1ff75479f79ed376cfbf1d` now rejects observed overlaps explicitly. This recorder correction does not retroactively make earlier runs successful.

The second signal also revealed an inclusive endpoint error in the new oblique-lens branch: one complete actual lens had aspect exactly 0.5, rejected by `>= 0.5` despite the documented supported interval. A captured-pixel boundary regression and correction are pending. Evidence: sandbox `output/recovery-oblique-browser-abort.json`, `recovery-oblique-browser-isolated.json`, `recovery-oblique-browser-to-parking.json`, their journals, `recovery-oblique-overlap-full-state.json` and timestamped camera PNGs. Main remains at `4527498` while these findings are addressed; it is paused at the archived failed signal pose. No public push or deployment occurred.

## Completed station-zero replay — camera and traffic candidate

Sandbox application code `866ba843ec1d276143de48f3fc3e616062b626bd` makes NPCs follow the physical ego and NPC bodies, using body support and mapped offset-path distance. Ego state is supplied only to background-world traffic, never to the ego Driver. Rear closure, moving/braking ego, adjacent clearance, curved sedan/truck queues and multi-NPC queues have independent sampled OBB regressions. Counterfactual fixtures restoring the old omissions reproduce overlaps. Short geometry queries use the existing scalar mapping, cross-checked against the batch mapping, reducing local median 100-update trial means from 2.69 to 0.81 ms at 8 NPCs and 5.36 to 2.56 ms at 14 NPCs. These CPU measurements are not complete scheduler deadline bounds.

Sandbox `9facb2a75920379cf406cec46cec7e3d1f88e4dd` includes the resolved 0.5 lens-aspect endpoint. Targeted camera regression: 73 passed. Traffic/scenario/mapping regression: 52 passed, 1 skipped. Initial combined full regression on `866ba84`: 1,221 passed, 1 failed, 3 skipped. The failure was the adversarial abort's former 20-second window: correcting NPC bumper gaps changed measured return from 10.3 to 20.9 seconds, with unchanged intervention and no sampled physical-envelope failures. `9d4dc4adb7be196b2cd3d5d563722fd00a4deeb9` explicitly uses a 25-second bounded fixture and reports actual return time; collision, braking, jerk and steering limits remain unchanged. Driving/diagnostic targeted tests: 21 passed. The original failed full log is retained, not relabeled green.

**Normal uninterrupted live mission on `9d4dc4a`: completed.** Ordinary app startup, station zero, default eight-NPC medium traffic, 50 km/h preference, actual browser-rendered eight-camera RGB; no staged pose, camera-frame injection, traffic removal, reset or refresh during driving. **5,490 samples, 5,232 active, 5,019 sensor-ready, zero recorded ego occupied-space overlaps, zero predicted ego footprint-risk samples, zero body/swept-body containment failures.** Final station 4,749.929616 m, lateral offset -0.017049 m, speed exactly zero; parked and contained, maneuvering false. Simulation time 523.1 seconds; capture includes paused startup. This is one route/seed/configuration, not a fleet success rate or physical autonomy proof.

First signal 1447264315 had 37 observed RED and 19 GREEN active samples; second 4946419558 had 14 GREEN samples. Both were passed using pixel-derived evidence. Residual active health: 183 camera-recovery, 29 stale-camera and 1 initial unmonitored samples. The successful drive does not establish uniformly smooth motion or eliminate GPU/transport tails. The first staged replay's collision and timeout remain failed evidence.

Actual browser reload after parking retained simulation identity, epoch, pose and zero speed in all 151 reconnect samples; all were sensor-ready, with zero recorded overlap/boundary flags. The DOM returned to Parked. Browser console had Three.Clock deprecation warnings and no captured errors. Evidence: sandbox `output/traffic-signal-final-normal-mission.json`, `.jsonl`, `traffic-signal-final-normal-metrics.json`, `traffic-signal-final-normal-parked.png`, `traffic-signal-final-reconnect.json`, `traffic-signal-final-reconnect-result.json`, `traffic-abort-following-comparison.json`, `traffic-body-timing.json` and `benchmark_traffic_bodies.py`.

Sandbox `127949ca2bb0efb885d331d24626efb40959ccb5` also distinguishes a safety abort from an actual duration timeout. It changes diagnostic classification only; application/frontend source is unchanged from the completed mission. **Final full regression: 1,223 passed, 3 skipped, 75 warnings in 498.95 seconds.** Evidence: sandbox `output/traffic-signal-accepted-full-pytest.log` and `traffic-signal-accepted-full-junit.xml`. Skips are CARLA collection and two Starlette TestClient WebSocket teardown cases; the actual TCP/browser checks above do not establish CARLA or hardware operation.

### Local integration and exact provenance

Preserved pre-integration main-worktree SHA: `bfbe2c3098c935b73b0ca723dc24afe9193d49f3`, local tag `dds-before-camera-traffic-20261014`. Its application source is the previous `4527498` version. Original baseline tags/checkpoints and the original telemetry database remain preserved. No push, PR or deployment occurred.

| Main-worktree local commit | Change |
|---|---|
| `001348834e55f783f4f1e12d8791ad3804ee29cc` | Hold measured stationary steering during sensor loss; retain moving control and lane intent. |
| `71b01f9b32b2aadb8aeadc44a91059f1ad632f0a` | Recognize three resolved oblique lenses from actual scene pixels. |
| `705424f71d40770831eba5f1671638773f3b4bbc` | Reject live acceptance on observed ego occupied-space overlap. |
| `16ad7adacec36cb78f1b881bcfa69b927f6ac759` | Verify parking cannot mask an overlap failure. |
| `5e4f42c46d6a6feb0ea239932755ac50884e2333` | Include the supported resolved 0.5 lens-projection boundary. |
| `b9334de172432fd401784981efa12dbe575c4d62` | Physical ego/NPC following gaps, mapped body constraints and short-query optimization. |
| `f422df68ae6e14b57b09bd858564072ada4cfd3c` | Explicit 25-second abort fixture and measured completion time for corrected traffic. |
| `ef5a47a2be65f3aabf34d01b17adf41df877d119` | Distinguish safety abort from duration timeout in diagnostics. |

Integrated app/frontend/scripts/tests exactly match validated sandbox `127949c` (Git comparison exit 0). Native integrated frontend production build passed: `output/camera-traffic-integrated-native-build.log`. Main API 8000 and frontend 3000 are running updated source, paused at station zero. Browser DOM confirms Drive paused / Resume drive; 92 real TCP startup samples retain station zero with no recorded overlap/boundary flags. Sensor checks are pending while the simulation clock is paused, not claimed healthy without distinct advancing frames. Evidence: main `output/camera-traffic-integrated-startup.json` and `camera-traffic-integrated-main.png`. This main startup check is not a second completed mission.

## Remaining limits

The parking goal is planner-selected inside recorded mapped parking, not a surveyed bay or live vacancy. Stop lines, conflict zones and lane corridors are modeled. Display widths can be OSM-recorded, lane-inferred or modeled; they do not establish surveyed drivable lanes. Bridges/railways retain map metadata but elevation is still ground-projected where unavailable. Buildings use real mapped footprints with inferred/modeled detail, not photorealistic surveyed facades.

The camera detector supports a narrow rendered control-housing prototype, not general traffic/scene recognition. Range sensors observe simulated traffic; there is no demonstrated general camera/range object-track fusion or physical hardware integration. Arbitrary OSRM journeys lack the recorded mission's complete infrastructure/parking validation. NPC traffic is finite and simplified, without general intersection conflict negotiation. CARLA and two TestClient WebSocket cases remain skipped; real TCP/browser checks cover the local transport paths, not CARLA or hardware. No universal success rate, hard real-time guarantee or production-readiness claim is made.
