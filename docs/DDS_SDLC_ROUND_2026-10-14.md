# DDS sandbox engineering and acceptance — 2026-10-14

## Release decision

**BLOCKED for main integration.** Automated regression passes, but live driving produced a vehicle-footprint containment failure. This is a real-road simulator; vehicle motion, cameras, range sensors, traffic and weather remain simulated. It is not a Level 5 autonomous vehicle. No public push, PR or deployment is authorized.

Main baseline: `390f62300c1e7a030fe2717e1f90e9d6834cb523`, branch `phase-7.5-ui-design`. Sandbox baseline: `a45a0c1a84c153689300506eb8ea686bb43a40f9`, branch `feat/surround-driving-20261013`. Main has not received this round. The pre-existing untracked camera fixture is excluded from the commit.

## Requirements and implementation

| Request | Result and remaining limits |
| --- | --- |
| Bugs and SDLC testing | Regression, targeted white-box coverage, live HTTP/WebSocket black-box, production build, static verification, fault injection and measured runtime load executed. Live acceptance is blocked below. |
| Cruise UI | Compact numeric input and slider beside speed. Browser verified typing/Enter, keyboard, wheel and drag. |
| Simple left panel | Removed redundant normal-driving prose and lead/acceleration readings from main cockpit; diagnostics remain available. Desktop and 1024x768 paused views inspected. |
| Mapped surroundings | World-metre road ribbons use OSM widths where available; inferred widths explicitly modeled. Building levels and tree heights used where recorded. Backend route stations now preserved rather than re-splined in the frontend. Exact intersection boundaries and surveyed geometry are not implemented. |
| Indicators and camera | Route maneuver intent, geometric bend fallback and committed lane change intent drive indicators; directional side PNG preview appears in cockpit. Actual left indication and left preview observed during live driving. Both directions tested in code; final responsive preview positioning still needs active-turn visual acceptance. Hazards do not select a side preview. |
| Traffic | Medium retains eight deterministic simulated actors; first four now start 40/70/100/130 metres ahead. Short routes cap count to preserve spacing. No live real-world traffic feed. |
| Bridges and rail | OSM infrastructure geometry, bridge/layer metadata and twin rail lines rendered. Bridges remain ground-projected; grade separation and true bridge elevation are unfinished. Tunnels omitted above ground. |

## Camera and safety boundary

Front camera HFOV is 90 degrees; front-left/right are 60 degrees, remaining auxiliary views 100 degrees. Pixel count is unchanged. The calibrated side-camera bearing and optical depth are transformed into ego coordinates before association with map location/kind priors. Map signal phase never supplies perception evidence. Multiple agreeing views count as one temporal capture; three distinct captures remain required. Conflicting phases and ambiguous observations fail closed.

The detector's Python component traversal was replaced with SciPy four-neighbour labeling, preserving the eight-pixel evidence floor. Captured green signal detection and its bounding box were unchanged. Five offline samples measured front detection 8.645–10.244 ms and right detection 2.631–4.060 ms. These exclude rendering/network and do not prove smoother end-to-end driving.

## Test evidence

- Project virtual environment full regression: **1080 passed, 3 skipped, 73 warnings**, 323.54 seconds. Artifacts: `output/sdlc-release-pytest.log`, `output/sdlc-release-junit.xml`.
- Skips: CARLA smoke collection; two Starlette TestClient WebSocket tests report portal teardown deadlock. Real socket black-box checks executed independently; CARLA validation remains unavailable.
- Targeted white-box suite: **68 passed, 7 warnings**, 14.49 seconds. Selected line coverage: detector 97%, camera bridge 81%, environment context 79%, vehicle signals 94%; combined **87%**. This is selected-module line coverage, not whole-project or branch coverage.
- Black-box against sandbox port 8001: invalid coordinate type/range rejected 422; JSON camera transport rejected 415; unknown camera 404; unsupported WebSocket command rejected and reconnect retains simulation/epoch. Final script fixed an incorrect assumption that monitoring must be active while paused; it now checks the telemetry boolean type. Artifact: `output/sdlc-round-blackbox.json`.
- All nine frontend verification scripts passed, including geometry station preservation, upward winding, roadside clearance, sensor labels and malformed grip reason handling. Production build also passed after the final telemetry-label correction; output is recorded in `output/sdlc-final-ui-build.log`.
- Lint: zero errors, eight warnings (unused symbols and diagnostic raw images). Warnings are not represented as a clean zero-warning result.
- Eight-second in-process runtime load with four consumers: each received 81 messages; real-time factor 1.00157, no measured deadline misses. Perception/planning/control p95: 0.654/0.592/1.743 ms. Artifact: `output/sdlc-runtime-load.json`. This is not browser FPS, TCP throughput or a long-duration soak test.
- Actual browser: cruise interactions, endpoint resolve/preview/start, desktop/tablet layout and directional left preview inspected. Console observation contained THREE.Clock deprecation warnings, no captured console errors. This is not exhaustive browser compatibility or accessibility certification.

An initial run accidentally used system Python and stalled; it was stopped and is not counted as passing. The first project-environment run found one obsolete HFOV140 test expectation (1078 pass/1 fail/3 skip); corrected to90, then the final full run passed. Raw logs retained rather than silently replacing failed evidence.

## Live blocker and next work

`output/sdlc-footprint-fault.json` records station 4367.8558 m, lateral offset 5.2626 m, vehicle heading 114.4707 degrees versus road heading 63.1593 degrees, saturated left steering, zero speed and critical footprint containment failure against the modeled 7 m road half-width. The shield requested `RECOVER_LOW_SPEED`; the observation does not prove successful recovery. Do not weaken the containment guard to make the route pass.

Next: reproduce this route transition deterministically, correct turn/path tracking and recovery arbitration in sandbox, then run continuous route acceptance with body containment, steering, acceleration/jerk, sensor freshness, command acknowledgments and reset/reconnect telemetry. Also finish real intersection geometry and elevation before claiming accurate infrastructure. Final paused UI inspection found an inactive-monitoring label defect; changed the cockpit to show `Sensor checks pending` whenever telemetry monitoring is inactive.

Hardware driving, penetration testing, cross-browser certification, exhaustive performance/soak testing and surveyed map accuracy were not executed. No claim of all possible testing, production readiness or autonomy success rate is made.
