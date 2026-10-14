# Three improvements and reference map acceptance — 2026-10-13

Preserved main baseline: `51f76b74e60a973aa7217317e5602b54b5b9f54d`, tag `dds-local-next-baseline-20261013`, verified complete-history bundle `output/dds-next-baseline-20261013.bundle`. Work tested in DDS_Next_Sandbox before local integration. No push, PR or deployment.

## Implemented

Detailed surroundings badges memoize visible formatted values; projected HTML anchors use a half-pixel threshold. Existing features remain. Windows deadline pacing uses a sleeping executor thread; fixed simulation dt unchanged. Persistence shutdown finishes transactions and drains queued records before finalization. Timing evaluator isolates SQLite and records per-cycle lateness instead of biased polling.

Production bicycle dynamics and sampled planner candidates share a modeled mu*g acceleration envelope. Steering slew and sideslip demand are included; emergency braking bypasses comfort jerk but uses remaining modeled grip. An already infeasible initial state receives explicitly flagged best-effort recovery, not a false feasibility claim. Legacy point-mass controller is excluded. Reset sideslip residue was fixed; original deterministic trajectory digest tests remain unchanged. Fixed-angle kinematic fixtures initialize consistent wheels, with separate exact transient arc tests.

Top-right real street map now uses subdued OSM tiles, solid blue route with white outline, blue Start marker and red Destination flag. Attribution and simulated-vehicle labels remain. Coordinate checks apply to route/recenter actions. Compact layout padding and zoom controls were corrected after real narrow-screen observations.

Offline swept-body oracle checks simultaneous center-linear shortest-yaw rigid-body interpolants, with conservative hull/sagitta inflation and unresolved outcomes. It is evaluation-only, never a privileged driving input. Float guards are engineering margins, not formal interval arithmetic; real plant continuous swept motion and surveyed road containment remain unverified.

## Validation and observations

Full combined application suite at `359cd382c9134c8c8dfcda08e24da44000f5fc26`: **894 passed, 3 skipped**, 61 warnings, 288.45s (`output/next-final-regression.txt`). Subsequent changes add only the offline oracle/docs/tests: **15 oracle tests passed**; application sources unchanged. Initial full run had four failures; raw evidence retained in `output/next-full-regression.txt`. Reset residue fixed and exact assertions preserved.

Frontend TypeScript, production build, UI contract, runtime metrics, sensor/turn/grip status and surroundings-label checks passed. ESLint: zero errors, five existing warnings. Production build covers unchanged frontend through final application revision.

Headed Chrome at1038x633, 10s warmed high-density overtaking,15 simulated actors, five auxiliary actual WebSockets plus UI. Same-backend sequential detailed baseline:37.15 compositor draws/s, RAF interval p95/max66.6/133.3ms; first optimized sample45.85,50/66.8ms. Final combined sample: normal59.51 draws/s with RAF16.8/33.4ms; detailed49.50 with33.4/66.7ms. These finite sequential observations have uncontrolled ordering/cache/load effects; no guaranteed or causal performance gain is asserted. Compositor cadence is not physical display completion. One initial normal capture partly minimized is excluded; benchmark now restores/checks native window state and pauses on rejection. See `output/next-ui-summary.json` and raw named audits.

Actual live wet-curve/emergency cut-in:124 telemetry samples each,0.1–12.28 simulated seconds; sampled initially feasible grip states stayed within modeled budget. Full real-time mapped parking run:1684 samples, final168.3s, exact zero speed, parked=true, no false-feasible grip samples (`output/next-parking-summary.json`). Captured while regression work ran; not an uncontended timing benchmark. Rendered parked chassis position, heading, front/rear steering and wheel rotation match authoritative telemetry within1e-5 tolerance (`output/next-rendered-pose-audit.txt`). Browser offline/reconnect passed with unavailable state and disabled destination control. Desktop/390x844 mobile route markers checked after animation settled; screenshots in output/playwright/next-map-*.png.

Offline50Hz actual-session poses,12s each: traffic_overtake5392 eligible pair intervals clear,8 recycling discontinuities excluded; emergency_cut_in5394 clear,6 discontinuities excluded; zero observed overlaps/unresolved. Final oracle reapplied to saved poses (`output/next-swept-session-audit.json`); fixed conservative ego outer rectangle4.8x2.86m, NPC class-assumed dimensions. This certifies only stated interpolants, not actual between-sample plant paths.

Isolated20s pacing measurements show better median but variable tails: repeated legacy p50/p95/p99/max7.84/16.64/22.45/121.62ms, native0.75/15.49/28.95/156.15ms. Scheduler misses >20ms24 vs20; these are scheduler lateness, not control-duration misses. Earlier legacy42 misses retained. Both clean modes verified200 telemetry+200 score rows/completed/clean exit; row counts do not establish analytics validity. No hard realtime claim.

## Newly discovered open issues

- Persistence substitutes100/A+ before a score exists; final average can default100. Separate correction pending.
- Database batch failures are logged/swallowed, allowing a session to be marked completed after failed writes. Separate correction pending.
- Shield TTC thresholds remain fixed; friction-aware stopping-distance prediction is not established by grip allocation.
- Detailed UI is still below continuous60 draws/s; worst scheduler lateness remains substantial.
- Continuous actual swept-path, surveyed widths/parking bays/live vacancy remain unverified. No Level5 or production-readiness claim.

## Exact local implementation commits

- `655094ff58dde4bff5ddbef513607d34b0a44fd5` — feat(map): refine real-road route with blue guidance and distinct endpoint pins
- `8196e4f016b6b7f0cdf884b7412912f8c288c45e` — perf(ui): isolate stable surrounding object labels
- `334fc658ad70ae58bc18827a0496d448367a5476` — fix(map): validate route controls and soften real street tile styling
- `c06b73eceba990cffaf0e3ca2ac9aed3b4280a4a` — feat(hmi): report active modeled grip recovery without normal cruising status
- `f3307bfdb335f9628ffa2cfd8a958f4abf4942ef` — fix(runtime): improve Windows pacing and drain persistence safely on shutdown
- `2a90513c473ecc9a7946777eca4ba94daa3104f0` — fix(map): keep endpoint guidance visible in compact layouts
- `7ee07bf39e0555dc5283ba2906f0e832fdd2d31d` — Share modeled grip between braking and turning without hiding recovery violations
- `a1230a5a154781af7ac481afbbc980749e0606fd` — Reset bicycle sideslip with steering to restore deterministic replay
- `359cd382c9134c8c8dfcda08e24da44000f5fc26` — Test steady and actuator-limited bicycle arcs with consistent wheel state
- `84473fba25d38850f5651bbb4cc1751e4a2ca535` — test(validation): add offline swept body clearance oracle
- `fe28c67f0704346f3bcf2b83a9de9f175e05c53a` — fix(validation): reject collapsed swept body geometry
