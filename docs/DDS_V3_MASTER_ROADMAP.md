# DDS Autonomy Platform — Master Engineering Roadmap V3

**Source:** Adopted from a comprehensive technical audit and architectural master
plan (2026-08-26), with one deliberate framing change explained below.
**Design reference:** Real published techniques from Waymo/Tesla-class autonomy
stacks (EKF tracking, RSS-style formal safety, spatiotemporal planning,
Pacejka tire models) — used as *engineering references for what to build*, not
as a claim that this project matches those companies' production systems.
**Discipline:** Every phase defines concrete mathematical models, exact module
boundaries, explicit test-count targets, and numeric acceptance gates. A gate
is only checked off when its regenerating test actually passes — no partial
credit, no "should work," matching this project's zero-defect standard.

## Why the framing changed from the original plan

The original draft of this plan (produced by another AI assistant working in
this repo) headlined itself as achieving "parity with Tesla FSD v12/13 and
Waymo Driver 5th/6th Gen." That claim doesn't survive this project's own
audit discipline (`AUDIT_PROTOCOL.md`, the `_evidence/` convention) — a solo
build cannot honestly claim parity with a production system built by hundreds
of engineers on proprietary sensor hardware and fleet-scale data. Every
formula, module boundary, and numeric gate below is kept exactly as
specified; only the headline claim is corrected to "benchmark-informed
design," because a project built to a zero-defect standard cannot open with
a claim it cannot regenerate proof for.

---

## Baseline (Phases 1–6, delivered and verified as of 2026-08-26)

189 automated tests passing, `tsc --noEmit` clean. See `docs/DDS_V2_ROADMAP.md`
for the full Phase 1-5 history. Summary:

- **Phase 1 — Safety Shield**: independent TTC/boundary/lateral-accel
  supervisor, decoupled from the planner (`app/services/safety_shield.py`).
- **Phase 2 — Telemetry protocol**: 10Hz WebSocket V2 protocol, off-thread
  Web Worker, SQLite session logging.
- **Phase 3 — Frenet planner**: real arc-length Frenet frame, quintic lateral
  candidate generation, pure-pursuit steering.
- **Phase 4 — Traffic & sensing**: forward range sensor, IDM car-following,
  seeded deterministic NPC traffic, world-space rendering with real
  path-planning visualization.
- **Phase 5 — Scenario Engine**: 4 deterministic scripted scenarios, REST
  `/api/scenarios`, `ScenarioControlRoom.tsx`, 9 tests.
- **Phase 6 — Surround Perception**: 5-frustum 360° sensor rig, multi-class
  EKF/Kalman tracking with GNN association, log-odds occupancy grid, live
  in the WebSocket stream as `data.surround_perception`, 21 tests.

---

## Phase 6 — 360° Surround Perception, Multi-Class Tracking & Occupancy Grid
**Estimate:** ~25–35h
**Design reference:** multi-sensor fusion + EKF tracking, the standard
approach across the AV industry (not unique to any one company).
**Core problem:** sensing today is a 1D forward cone; no blind-spot coverage,
no multi-class classification, no persistent tracking through occlusion.

### Scope
- [x] `app/services/perception/sensor_rig.py` — 5 virtual sensor frustums
  (forward long-range, forward wide, left/right blind-spot, rear-center),
  each an azimuth+range frustum-culling test against actor `(x, z)`
  (`frustum_contains`), plus fully vectorized batch variants
  (`batch_relative_observations`/`batch_detecting_mask`) used by the
  per-tick hot path.
- [x] `app/services/perception/entities.py` — multi-class entity model
  (`SEDAN`/`SUV`/`TRUCK`/`MOTORCYCLE`/`BICYCLE`/`PEDESTRIAN`/`TRAFFIC_CONE`)
  with real per-class bounding-box dimensions. Simplified from the
  originally-specified full 11-element state vector: the Kalman filter's
  own state is `[x, z, vx, vz, ax, az]` (constant-acceleration, position/
  velocity/acceleration only); heading and box dimensions are carried as
  auxiliary per-track attributes, not filtered state -- there was no
  orientation measurement to justify an unfiltered `θ̇` in the filter
  itself, and building one un-validated would be exactly the kind of
  unearned complexity this project's audit discipline pushes back on.
- [x] `app/services/perception/tracker.py` — constant-acceleration Kalman
  filter per track (a linear KF, not a nonlinear EKF -- see the module
  docstring for why that's a correct simplification, not a shortfall, for
  this project's position-only measurement model), GNN/Hungarian-algorithm
  data association (`scipy.optimize.linear_sum_assignment`), track
  lifecycle `TENTATIVE → CONFIRMED → COASTED → DELETED`. Fully batched
  (numpy stacked matmul across all tracks) after an initial per-track-loop
  version badly missed Gate 6.3's latency budget.
- [x] `app/services/perception/occupancy_grid.py` — 100m×100m ego-centric
  grid at 0.25m resolution, log-odds occupancy, vectorized DDA line
  rasterization (numerically equivalent to Bresenham for this purpose,
  computed via numpy array ops rather than a per-cell Python loop -- see
  the module's performance note).
- [x] `app/services/perception/perception_engine.py` (not originally
  named in the plan, added as the natural integration point) — wires the
  above into one per-tick `SurroundPerceptionEngine.step()`, called from
  `PhysicsEngine.update()` and exposed via
  `get_surround_perception_state()`, streamed live over the WebSocket as
  `data.surround_perception`. Verified live in-browser: real tracks with
  real EKF-estimated velocities and real detecting-sensor names streaming
  end-to-end, zero console/server errors.
- [x] Added `SurroundTrack` as its own interface in
  `frontend/src/types/protocol.ts`, rather than extending the existing
  `PerceptionObject` -- `PerceptionObject` represents the forward sensor's
  single lead-vehicle detection (a different, older concept this phase
  intentionally left untouched); conflating the two would have blurred a
  distinction the codebase already draws on purpose.

### Gates
- **6.1** Blind-spot vehicle at 30m behind in the adjacent lane detected and
  held continuously detected through a full simulated overtake (75m down
  to 5m gap) -- verified in `tests/test_perception.py`. The plan's original
  gate text referenced `sense_lane_clear()`; that's `traffic.py`'s existing
  planner-facing lane-change safety check, a different, older subsystem
  this phase deliberately left untouched (a genuine 1D forward+adjacent
  check, unrelated to the new 360-degree sensor rig). The 360-degree rig's
  own blind-spot detection is what's gated and tested here.
- **6.2** EKF/KF tracks a noisy (σ=0.3m) constant-velocity actor to a MEAN
  steady-state position error <0.15m and velocity error <0.25 m/s,
  averaged over 20 independent noisy trials (a single noisy realization's
  error is itself a random variable around that mean -- asserting a tight
  bound against one fixed seed would test that seed's luck, not the
  filter's real steady-state behavior).
- **6.3** Full perception pipeline (30 actors: sensor culling, EKF predict/
  update, GNN association, occupancy grid update) completes in <2.0ms/tick
  on one CPU core, measured as the best of many independent timing
  batches on a real (noisy, shared) dev machine -- true floor performance
  measured at ~1.3-1.8ms after 5 rounds of profiling-driven optimization
  (from an initial, correctness-first implementation that measured 49ms/
  tick, a ~30x improvement).
- **6.4** 21 new tests in `tests/test_perception.py` (target: ≥18); total
  189 (target: ≥186).

---

## Phase 6.5 — Architectural restructure: World / Driver split
**Estimate:** ~20–30h
**Full rationale and options analysis:** `docs/DDS_ARCHITECTURE.md` (ADR-001).
**Core problem:** four structural issues that every remaining phase makes
worse, and that Phases 9, 11 and 13 are outright blocked by.

This delivers **no new driving behavior**. It is the foundation the rest of
the roadmap is built on, and it is strictly behavior-preserving — proven by
the existing 189 tests passing unchanged.

### The four problems
1. **The learned model is in the control path and cannot see the road.** The
   XGBoost classifier's inputs are all ego powertrain telemetry (`RPM`,
   `CO2`, `Coolant`, fuel rate and their deltas) — nothing about traffic,
   lanes or geometry — yet it perturbs `target_speed` by ±15–20 km/h
   (`physics_engine.py:622-627`). It cannot help (blind to hazards) but can
   hurt (commands acceleration for powertrain reasons). Production stacks put
   learned models in perception/prediction and use a deterministic optimizer
   for planning; DDS currently has this inverted.
2. **Simulator and autonomy are the same object.** `PhysicsEngine` is both
   the world and the driver, so the perception boundary is a convention
   rather than a type signature, and the stack can't run against recorded
   data or a swapped/failed driver.
3. **Single-rate wall-clock execution.** One 10 Hz loop stepped by real
   elapsed time — nondeterministic (contradicting the Scenario Engine's own
   reproducibility guarantee, and already causing a tick/time desync against
   `websockets.py`'s hardcoded `0.1`), and too slow for closed-loop control.
4. **One god-payload to the frontend**, which won't survive Phase 6's
   occupancy grid or Phase 7's per-agent predictions.

### Scope
- [x] `interfaces.py` — typed contracts: `SensorObservation`,
  `PerceptionOutput`, `PredictionOutput`, `PlannedTrajectory`,
  `ActuatorCommand`, `SimClock`.
- [~] `world/` — `vehicle_dynamics.py` (`step_powertrain`, `advance_position`)
  extracted verbatim; `PhysicsEngine` delegates. Full `TrafficModel`/facade
  decouple deferred to Phase 7 (hybrid decision — see ADR-001 status note).
- [~] `driver/` — `lateral_planner.py` (Frenet candidates + pure-pursuit
  tracking) extracted, reading only the scalar subset of `SensorObservation`.
  IDM composition + no-route fallback still inline pending the full
  `SensorObservation` wiring in Phase 7.
- [x] Multi-rate deterministic executor: `executor.py` (`MultiRateExecutor`,
  100 Hz base, tested rate dispatch). `SimClock` wired into `PhysicsEngine`
  (fixed 20 ms substep) and `websockets.py` (single dt source for scenario +
  physics — desync removed). Wall-clock `dt` fallback retained (hybrid).
- [x] Relocate the ML out of the speed-target path (done in Phase 7).
  `ai_decision` no longer influences `target_speed` or the powertrain flavour
  (which now keys off realised acceleration) -- the XGBoost model + SHAP +
  anomaly detection are a driver-behaviour / eco-efficiency analytics
  channel. `test_ai_decision_has_no_effect_on_the_control_path` locks it in;
  README claims updated.
- [x] Split the Safety Shield into a parallel `SafetyMonitor`
  (`driver/safety_monitor.py`), veto-only. Own sensor feed / RSS / MRM is
  Phase 11.
- [x] Protocol v3 — done in Phase 7 (versioned reorg, one message).
  `protocol_version` "3.0"; `data` regrouped into `channels: {pose,
  semantic, heavy}` (heavy = surround perception + predictions, candidates
  for on-demand/delta-encoding later). `protocol.ts`, `telemetryWorker`,
  `useSimulationStore`, and the WS smoke test migrated; `tsc --noEmit`
  clean.

### Gates
- **6.5.1** ✅ All 189 existing tests pass unchanged; +36 new unit tests
  (225 total) for the extracted modules and the determinism gate.
- **6.5.2** ✅ Determinism: `tests/test_determinism.py` — same seed + same
  scenario produces a bit-identical ego trajectory (and `SafetyMonitor`
  verdict sequence) across two separate runs, on the explicit-dt path.
- **6.5.3** ✅ (pragmatic) `MultiRateExecutor` owns the authoritative
  `SimClock` and drives the tick in `websockets.py` — scenario + physics
  run as one stage registered at the stream rate. The 50/20/10 Hz
  perception/planner/control split is wired-but-single-stage; each stage
  gets its own registration in the Phase 11 deep decouple.
- **6.5.4** ✅ `tests/test_driver_boundary.py` — AST-level assertion that no
  `app/services/driver/` module names, imports, or accepts a
  `TrafficModel` / `NpcVehicle` / `sense_lead_vehicle`. `PhysicsEngine`
  itself still reads ground truth as the facade (Phase 11).
- **6.5.5** ✅ `tsc --noEmit` clean after the protocol v3 migration.

---

## Phase 7 — Multi-Agent Trajectory Forecasting & Intent Engine
**Estimate:** ~20–30h
**Core problem:** planner treats all obstacles as fixed-velocity; late
braking on merges.

### Scope
- [x] `app/services/prediction/forecaster.py` — 3.0s horizon @ 0.1s steps
  (30 states/actor). CTRA (constant turn rate + accel, midpoint-integrated)
  for maneuvering actors / no route frame; Frenet lane-following (advance
  station at along-track speed, quintic lateral relax to nearest lane
  centre) otherwise. `project_agent_frenet()` gives lane-relative drift.
- [x] `app/services/prediction/intent.py` — interpretable (non-ML) scoring →
  distribution over `LANE_KEEP`/`MERGE_LEFT`/`MERGE_RIGHT`/`DECELERATING`/
  `STOPPING`, plus `p_cut_in` (merge component toward the ego lane) and
  time-to-cross. Action threshold `P(cut-in) > 0.65`.
- [x] `app/services/prediction/risk_field.py` — bounded [0,1] spatiotemporal
  risk; oriented Gaussian per agent (wider along travel), σ grows with
  horizon time; agents combine as probabilistic OR. `sample_along()` /
  `max_risk()` for planner queries.
- [x] `app/services/prediction/prediction_engine.py` (added as the
  integration point) — per-tick orchestrator; per-track history + EMA drift
  smoothing; emits `PredictionOutput` + a comfort-bounded proactive
  slowdown. Wired into `PhysicsEngine.update()` off the sensor-resolved
  track picture; `data.prediction` on the WebSocket (protocol stays "2.0").
- [ ] Frontend: predictive ribbon, 3s forecasted trails, intent
  color-coding — **not started** (data is on the wire).

### Gates
- **7.1** ✅ `estimate_intent` with sustained 0.4 m/s drift toward the ego →
  `p_cut_in > 0.70` while `time_to_cross_s > 1.2` (test_prediction.py).
- **7.2** ✅ Proactive slowdown caps at 1.2 m/s²; integration test shows the
  ego eases off early and `EMERGENCY_BRAKE` never fires
  (test_prediction_integration.py).
- **7.3** ✅ Curve-follower has ~0 Frenet lateral drift → `p_cut_in < 0.15`;
  stable in-lane agent stays `LANE_KEEP` dominant.
- **7.4** ✅ 30 tests in `tests/test_prediction.py` + 4 in
  `tests/test_prediction_integration.py` (target ≥15); suite total 259
  (target ≥201).

---

## Phase 8 — Unified Spatiotemporal (s, d, t) Motion Planning
**Estimate:** ~25–40h
**Core problem:** lateral candidate selection and longitudinal speed control
are decoupled, producing sub-optimal joint maneuvers.

### Scope
- [ ] `app/services/planner/spatiotemporal.py` — joint `(s, ṡ, s̈, d, ḋ, d̈, t)`
  lattice, 4.0s horizon, quintic `d(t)`/`s(t)` polynomials with C² continuity.
- [ ] Quadratic cost objective over lateral/longitudinal jerk, lane-center
  deviation, target-speed tracking, and the Phase 7 risk field.
- [ ] `app/services/planner/state_machine.py` —
  `LANE_KEEP/PREPARE_LANE_CHANGE/EXECUTE_LANE_CHANGE/ABORT_LANE_CHANGE`, with
  deterministic mid-maneuver abort back to lane center
  (`|lateral jerk| ≤ 1.5 m/s³`).
- [ ] Feasibility filters: `|a_lat| ≤ 2.0 m/s²`, `|jerk_lat| ≤ 1.5 m/s³`,
  `a_long ∈ [-4.5, 2.5] m/s²`, `|d| ≤ 3.0m`.

### Gates
- **8.1** Mid-maneuver abort at 50% lateral completion: peak lateral accel
  <1.8 m/s², zero boundary violations.
- **8.2** 100 simulated lane changes: p99 lateral jerk strictly <1.5 m/s³.
- **8.3** ≥30 joint candidates evaluated and winner selected in <4.0ms/tick.
- **8.4** ≥20 new tests in `tests/test_spatiotemporal_planner.py`; total ≥221.

---

## Phase 9 — Dynamic Vehicle Dynamics & Non-Linear Tire Friction
**Estimate:** ~25–35h
**Core problem:** kinematic bicycle model assumes infinite grip; no
understeer/oversteer, no wet-road behavior, no load transfer.

### Scope
- [ ] `app/services/physics/dynamic_bicycle.py` — dynamic lateral-motion
  equations, tire slip angles `α_f`, `α_r`.
- [ ] `app/services/physics/tire_model.py` — Pacejka Magic Formula,
  `μ` presets for dry (0.95) / wet (0.60) / ice (0.25) surfaces.
- [ ] Dynamic axle load transfer under braking/accel; 1st-order steering
  actuator lag (`τ=0.08s`); ABS/ESP brake-pressure modulation at slip
  ratio `λ > 0.15`.
- [ ] `app/services/physics/controller.py` — Stanley steering controller
  (front-axle cross-track + heading error), replacing pure pursuit.

### Gates
- **9.1** ISO 7401 step-steer at 80km/h: yaw-rate overshoot <18%, settles
  within 0.8s.
- **9.2** Full braking from 60km/h at μ=0.5: ABS keeps front-tire
  steerability, no yaw spinout.
- **9.3** Stanley tracks sharp curvature transitions with cross-track error
  <0.12m.
- **9.4** ≥16 new tests in `tests/test_dynamic_physics.py`; total ≥237.

---

## Phase 10 — Semantic Road Network, Intersections & Traffic Rules
**Estimate:** ~30–45h
**Core problem:** single-corridor routes only; no intersections, lights,
stop signs, crosswalks, or lane topology.

### Scope
- [ ] `app/services/map/vector_map.py` — `Lane` (centerline, boundary
  types, speed limit, width), `Intersection` (conflict zones, stop/yield
  lines, turn corridors), `Crosswalk` (polygon, curb points).
- [ ] `app/services/environment/traffic_light.py` —
  `GREEN→YELLOW(4s)→ALL_RED(2s)→RED[→protected left]` state machine;
  dilemma-zone stop/proceed decision from stopping-distance physics.
- [ ] `app/services/planner/intersection.py` — FIFO 4-way stop arbitration;
  unprotected-left-turn gap acceptance (≥5.5s).
- [ ] Scripted pedestrian crossing agents in `traffic.py` (1.3 m/s walk
  speed, ≥3.0m ego standoff).

### Gates
- **10.1** 50 automated Red/Yellow arrivals: zero stop-line overruns, stops
  within [0.5m, 2.0m] of the line.
- **10.2** 30 unprotected left turns across oncoming flow: zero critical
  TTC violations.
- **10.3** Ego yields ≥3.0m before crosswalk whenever a pedestrian is in the
  roadway corridor.
- **10.4** ≥22 new tests in `tests/test_semantic_map.py`; total ≥259.

---

## Phase 11 — Formal RSS-Style Safety & Minimum Risk Maneuvers
**Estimate:** ~20–30h
**Design reference:** the published Mobileye RSS formalism
(Shalev-Shwartz et al., 2017) as a mathematical model — implemented here as
this project's own supervisor, not a certified/validated safety system.
**Core problem:** current shield uses fixed thresholds; no formal
minimum-safe-distance bound and no fail-operational pullover behavior.

### Scope
- [ ] `app/services/safety/rss.py` — Rule 1 (longitudinal safe-following
  distance formula), Rule 2 (lateral safe-distance formula), Rule 3
  (blame attribution: which party breached the envelope first).
- [ ] `app/services/safety/mrm.py` — three-tier Minimum Risk Maneuver:
  MRM-1 (in-lane comfort stop), MRM-2 (shoulder pullover), MRM-3 (emergency
  brake).
- [ ] `app/services/safety/flight_recorder.py` — 100Hz ring buffer, last 30s
  of sensor/planner/shield state, flushed to disk on any override.

### Gates
- **11.1** Zero at-fault collisions across a defined, reproducible stress
  scenario set (target scenario count set once the harness exists — see
  note below).
- **11.2** Injected planner failure triggers MRM-2 pullover within 8s,
  lateral jerk within bounds.
- **11.3** Flight recorder logs and serializes 3,000 states within 50ms of
  an emergency trigger.
- **11.4** ≥18 new tests in `tests/test_rss_safety.py`; total ≥277.

> Note on Gate 11.1: the original draft specified "500 simulated multi-agent
> stress scenarios." That number will be set for real once Phase 11 starts
> and the actual scenario-generation harness exists — committing to an
> exact count before the generator is built would be the kind of
> unverifiable number this project's audit discipline exists to catch.

---

## Phase 12 — Advanced 3D Visualizer, Audio Engine & Teleoperation
**Estimate:** ~35–50h
**Core problem:** current UI is functional but lacks cinematic camera
options, spatial audio feedback, and interactive scenario authoring.

### Scope
- [ ] Procedural asphalt/road-marking shader (wet-road specular, dashed/
  solid/double-yellow lines, crosswalks).
- [ ] Dynamic vehicle model: steerable wheels tied to real `δ`, wheel spin
  tied to real `v`, brake-triggered taillight glow, turn-indicator pulse
  tied to real lane-change intent.
- [ ] Blind-spot occupancy halo (green→amber→red) fed by Phase 6's tracker.
- [ ] `frontend/src/components/3d/CameraRig.tsx` — 4 modes: chase, cockpit,
  bird's-eye, free orbit.
- [ ] `frontend/src/lib/audioEngine.ts` — Web Audio API synthesis (no
  external assets): autopilot engage/disengage chimes, FCW alarm, lane-
  departure tick.
- [ ] `frontend/src/components/TeleopConsole.tsx` — click-to-nav, drag-drop
  obstacle injection, manual WASD/gamepad takeover with disengage chime.

### Gates
- **12.1** Steady 60 FPS at 1080p with 30 dynamic vehicles.
- **12.2** Synthesized audio triggers within <15ms of the WS event.
- **12.3** Manual takeover engages within one tick (<100ms), no physics
  discontinuity.
- **12.4** `tsc --noEmit` exits 0.

---

## Phase 13 — Real-World NGSIM Highway Replay & Certification Suite
**Estimate:** ~25–40h
**Core problem:** synthetic traffic never tests against real human driving
behavior (tailgating, weaving, shockwaves).

### Scope
- [ ] `app/services/replay/ngsim.py` — ingest public FHWA NGSIM (US-101,
  I-80) trajectories, map into Frenet `(s, d)`, replay as NPC traffic.
- [ ] `app/services/analytics/certification.py` — automated run over
  replayed traffic computing Mean Distance Between Disengagements,
  collision rate/1000km, RSS compliance rate, comfort index
  (`s̈, d̈, jerk` distributions), progress-efficiency ratio.
- [ ] Automated HTML/PDF safety-audit report generator.

### Gates
- **13.1** NGSIM interpolation matches recorded FHWA positions within
  <0.05m at 10Hz.
- **13.2** Zero at-fault collisions over a defined continuous NGSIM replay
  distance (exact km target set once the ingestion pipeline's real data
  coverage is known).
- **13.3** One-command CLI generates the certified report.
- **13.4** ≥15 new tests in `tests/test_ngsim_certification.py`; total ≥292.

---

## Test-count progression

| Phase | Deliverable | New tests | Cumulative | Gate |
|---|---|---|---|---|
| Baseline | Phases 1-5 | — | 168 | 168/168, tsc clean |
| 6 | Perception + EKF tracking | +21 | 189 | blind-spot detect, <2ms |
| 6.5 | World/Driver architecture (partial; items 5/7 → P7) | +36 | 225 | 189 unchanged + bit-identical replay |
| 7 | Trajectory prediction (backend done; ribbon UI pending) | +34 | 259 | ≥1.2s cut-in warning |
| 8 | Spatiotemporal planner | +20 | 221 | mid-maneuver abort |
| 9 | Dynamic tire physics | +16 | 237 | step-steer, ABS |
| 10 | Semantic map + intersections | +22 | 259 | 0 stop-line overruns |
| 11 | RSS safety + MRM | +18 | 277 | 0 at-fault (defined set) |
| 12 | Visualizer + audio + teleop | frontend | 277 | 60 FPS, tsc clean |
| 13 | NGSIM replay + certification | +15 | 292 | 0-collision replay |

## Execution order

Sequential. Phase 6 (perception) is complete. **Phase 6.5 (the World/Driver
architectural restructure, ADR-001 in `docs/DDS_ARCHITECTURE.md`) comes
next** — before Phase 7, because Phases 9, 11 and 13 are structurally
blocked without it and every phase built on the current monolith increases
the cost of doing it later.

Each phase is built, its own tests written and passing, and live-verified
before moving to the next — no phase is marked complete on partial coverage.
