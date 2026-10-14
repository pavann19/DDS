# DDS Progress

2026-10-13: [Unified driving acceptance](DDS_UNIFIED_DRIVING_ACCEPTANCE_2026-10-13.md) adds one combined recorded-road environment, collapsed Advanced testing, Settings-only cruise preference and required scene-camera/range availability checks. Full suite: 1,020 passed, 3 skipped; live camera loss/standstill/recovery and actual WebSocket retry verified. Camera/object fusion and arbitrary unmapped controls remain unfinished.

| Phase | Status | Owner | Evidence |
|------|--------|-------|----------|
| P0 | COMPLETE | Antigravity | 91f70b4 |
| I1 | COMPLETE | Claude | observation-boundary follow-ups landed + Experiment A re-run; see below |
| I2 | REPRESENTATION CHOSEN (Rep A) — model training gated | Claude | `docs/I2_EXPERIMENT_A_FINDINGS.md` Part B; `experiments/i2_experiment_ab.py` |
| I3 | NOT STARTED | Claude | — |
| I4 | NOT STARTED | Claude | — |
| I5 | NOT STARTED | Claude | — |
| I6 | NOT STARTED | Claude | — |
| I7 | NOT STARTED | Claude | — |
| I8 | NOT STARTED | Claude | — |
| I9 | NOT STARTED | Claude | — |
| I10 | NOT STARTED | Claude | — |
| I11 | NOT STARTED | Claude | — |
| I12 | NOT STARTED | Claude | — |
| I13 | NOT STARTED | Claude | — |
| I14 | NOT STARTED | Claude | — |
| I15 | NOT STARTED | Claude | — |
| I16 | NOT STARTED | Claude | — |

## CARLA migration (enabler — must land before I2 training)

| Field | Value |
|------|-------|
| Current phase | Refactor Phase A + **Phase B (driver extraction) COMPLETE** · `mode="external"` integration-split deferred to Phase C · Phase C (CARLA runtime) BLOCKED on hardware |
| Hardware feasibility | **NOT FEASIBLE on this machine** — AMD RX 6500M 4 GB VRAM (CARLA needs NVIDIA ≥ 6 GB), 11.3 GB RAM / 3.8 GB free (needs 16–32 GB), DDS venv Python 3.13 (CARLA client is 3.7–3.10). Details: `docs/CARLA_FEASIBILITY.md` |
| CARLA environment status | Not installed / not runnable here. Recommended: CARLA 0.9.15 (UE4) on an NVIDIA/32 GB/Ubuntu host or cloud GPU box; Python 3.10 client venv; Docker useful only on that host; ROS 2 not used. |
| Completed | Phase A: `PhysicsEngine.observe() -> SensorObservation`; `app/services/carla_adapter/` package (`WorldBackend` ABC, `DeterministicSimBackend`, `CarlaBackend` skeleton, `conversions`, `RouteProvider`, `ScenarioRunner` + scenario-intent map); `docs/CARLA_FEASIBILITY.md`, `docs/CARLA_MIGRATION_PLAN.md`. **Phase B: decision loop extracted into simulator-independent `app/services/driver/driver.py` `Driver` (`decide_lateral` / `decide_longitudinal` / `decide -> ActuatorCommand`) + `app/services/driver/context.py` `DriverContext`. Driver owns lane-change FSM, committed maneuver, lateral smoothing, SafetyMonitor, HMI planner bookkeeping, shield verdict, speed_limit_reason/friction diagnostics; `PhysicsEngine` re-exposes them via forwarding `@property`. `physics_engine.py` 1546 → 1219 lines. ADR-001 Gate 6.5.4 holds (no `traffic` import in `driver/`).** |
| Tests | `tests/test_carla_adapter_conversions.py` (7), `tests/test_world_backend_boundary.py` (6), `tests/test_carla_backend_smoke.py` (`importorskip` — skips). Phase B gate: **375 passed, 1 skipped == pre-refactor baseline**; `test_determinism.py` bit-identical free-drive + scenario replay incl. SafetyMonitor verdict sequence. |
| Commits | b970177 (Phase A), 59b29a7 (progress), **7943460 (Phase B driver extraction)** |
| Blockers | Phase C only: GPU/VRAM, RAM, Python version — all require different hardware. Phase B is done. |
| Exact next action | (1) `mode="external"` integration-split rides with `CarlaBackend` on a CARLA host. (2) Resume the learned-driving roadmap: I1/I2 observation-boundary follow-ups (`AgentSlot.lateral_rate_mps`, role-typed agent set, per-track age/covariance, curvature profile + `dist_to_next_maneuver`) then Experiment A Rep-A vs Rep-B on held-out probes with noise on. Do NOT start I2/I3/I4 model training until the CARLA-backed observe→decide→control→feedback loop works. |

## I1 observation-boundary follow-ups + I2 representation choice

| Field | Value |
|------|-------|
| Status | I1 follow-ups **landed**; I2 representation **chosen by evidence (Rep A)**; **no model trained** (I3 gated on the CARLA loop) |
| Code | `app/services/learning/observation.py` — role-typed `AgentSlot` (`ROLE_SLOTS` = ego/left/right × lead/follow + 2 other), new per-agent `d_rate_mps` / `d_accel_mps2` / `track_age_ticks` / `pos_sigma_m`, `Observation.curvature_profile_1pm` + `dist_to_next_maneuver_m`; `perception_engine.py` — `SurroundTrack.age_ticks` + `pos_sigma_m` (EKF √½·tr P₂ₓ₂); `prediction_engine.py` — `PredictionResult.lateral_rate_by_track` / `long_accel_by_track` (de-biased `drift_ema` / `accel_ema`); `physics_engine.build_learning_observation` emits all of it. All additive / optional — `build_observation` back-compat kept. |
| I1 §4 actions | 1 populate `lateral_rate_mps` ✅ · 2 absorb `lead_gap` into typed set ✅ (partial — `ego_lead` slot; standalone field kept as the distinct longer-range forward corridor sensor) · 3 per-track age + covariance ✅ · 4 curvature profile + `dist_to_next_maneuver` ✅ · 5 role-typed set ✅ · 6 de-emphasise heuristic `p_cut_in` ✅ (now one input; primary temporal substrate = `d_rate` + K-frame history) · 7 targeted low-friction/shield scenarios ❌ **deferred** (needs new scenario defs; coverage still 8 %/5 %) |
| Experiment A re-run (noise ON, 9 360 ticks) | `lateral_rate_mps` dead-fraction 1.00 → **0.078**; slot saturation 0.08 → **0.00**; repeatability **8/8 identical**; leakage unchanged — only permitted localisation/map inputs at R²=1.0, everything a sensor should blur stays 0.32–0.61 → **no privileged runtime input**; 1 constant feature (`n_same_direction_lanes`, Phase-10 map gap, not faked). |
| I2 Rep A vs Rep B probe (`experiments/i2_experiment_ab.py --noise`, held-out `(scenario,seed)`, **no network trained** — closed-form ridge probe; Rep B = fixed-random ReLU set encoder) | lane-choice macro-F1 **Rep A 0.542** / Rep B 0.493 (base 0.958); NPC next-3 s ADE **Rep A 7.38 m** / Rep B 8.07 m / const-velocity baseline 14.63 m; feature-build latency Rep A **19.4 µs** / Rep B 51.3 µs median. |
| I2 decision | **Rep A** (flat, fixed-role substrate + K-frame history). Rep B built + measured + lost both probes; not selected. Re-evaluate a **trained** DeepSets at I3. Reproduce: `python -m experiments.i2_experiment_ab --noise`. |
| Tests | full suite **375 passed, 1 skipped** (unchanged); `tests/test_observation_boundary.py` green (static + API + runtime-invariance). |
| Exact next step | (a) coverage action 7 — add targeted low-friction + Safety-Shield-active scenario/seed definitions (<10 % today). (b) I3 — build the learned head on Rep A + K-frame stack; re-run Rep A vs *trained* Rep B. **Do NOT start I3 model training until the CARLA-backed observe→decide→control→feedback loop is functioning and validated.** |


## 12 October 2026 — joint maneuver and anticipatory turn batch

Accepted local feature `c2c68725be38ea95359d6484896a4241b803f8fd`. Full combined suite:668 passed,3 skipped; frontend build passed; actual browser/live steering, parking, reset and reconnect checks recorded. See [measured acceptance and remaining limits](DDS_JOINT_TURN_ACCEPTANCE_2026-10-12.md). The user-authorized continuation finishes this batch; no next feature was started.


## 13 October 2026 — sensor, turning, corridor and driving validation batch

Accepted local feature/UI `4e757c34b2116bec0eb150e18d58447aaef57e49`; frozen backend/evaluator `fdd72d3928c90d169968d0e16c30d70395832edd`:836 tests passed,3 skipped. Final frontend production build and live sensor, dry/wet steering, exact-stop parking, traffic recovery, recorded stop/yield, reset and reconnect checks passed within the documented simulator scope. See [the new acceptance and provenance record](DDS_ITEMS_1_4_ACCEPTANCE_2026-10-13.md). This supersedes the earlier pending items for these scoped implementations; broader Cartesian/physical/friction and timing certification remain unproved. All commits remain local.


## 13 October 2026 — runtime smoothness and map/parking evidence

Local code `6e40b78ee37c04e7be0f2ffe160a190640f645be`:872 backend tests passed,3 skipped; final main frontend build passed. Narrow Proactor cleanup and queue accounting fixes, bounded timing capture, stable action subscriptions, recorded map provenance and conservative parking access/direction checks were sandboxed and reviewed before integration. Foreground browser measurements and live TCP/map/reconnect evidence are in [items5–6 acceptance](DDS_ITEMS_5_6_ACCEPTANCE_2026-10-13.md). Detailed overlays and scheduling outliers remain measured limitations; no universal60FPS, surveyed bays, physical access or Level5 claim. No push/PR/deployment. Original59-commit review stays separate from this new batch.
