# The Learned-Pipeline Observation Boundary

**Invariant VII of `ADR-003-LEARNED-DRIVING-INTELLIGENCE.md`:** a learned
driving component is trained and evaluated on what a **sensor-derived world
representation** exposes, never on privileged simulator ground truth.

This document is the contract. It is enforced by
`tests/test_observation_boundary.py` (static + runtime) and by
`_reject_privileged()` in `app/services/learning/observation.py`.

## The single choke point

Everything a learned component may condition on is assembled by

```
app.services.learning.observation.build_observation(...) -> Observation
```

and, from the running engine, by

```
PhysicsEngine.build_learning_observation() -> Observation | None
```

There is no other path. The `app/services/learning/` package imports nothing
that carries privileged state (no `traffic`, no `NpcVehicle`, no
`physics_engine`); the static test fails the build if that changes.

## Allowed inputs (sensor-derived / localisation / map)

| Input | Source | Why it is allowed |
|---|---|---|
| ego speed, accel, lateral rate, heading error | ego proprioception | the vehicle's own integrated state; a real car has these from IMU / wheel encoders |
| ego Frenet `d`, distance-to-goal bucket, road curvature | localisation against the route `FrenetFrame` (the map) | a real AV localises against an HD map; exposed coarsely (a bucket, a low-passed curvature), not as exact privileged metres |
| `friction_estimate` | currently `friction_mu` | **labelled as an ESTIMATE.** A real car estimates grip from wheel-slip / IMU / vision. Until a grip-estimation model exists, `friction_mu` stands in — treated as a possibly-wrong sensor input, never an oracle. Replaceable with no interface change. |
| forward `lead_gap_m` (bumper-to-bumper), `lead_speed` | `TrafficModel.sense_lead_vehicle()` — the forward collision corridor, range ≤ 100 m, optional Gaussian noise + dropout | it is the ego's own forward sensor; already a perception/control boundary |
| `left/right_lane_clear` booleans | `TrafficModel.sense_lane_clear()` | a single boolean per adjacent lane, not agent state |
| per-agent `AgentSlot` (relative `ds/dd`, closing speed, size, track confidence) | `SurroundPerceptionEngine.step()` output (`SurroundTrack` list) — frustum-gated, EKF-filtered, optionally noised / dropped / occluded / delayed via `PerceptionNoiseConfig` | this IS the ego's multi-sensor rig + tracker; nothing here is ground truth |
| per-agent `role` (ego/left/right lane × lead/follow, else "other") | geometric classification of the track's own `ds/dd` against the highway lane prior | a fixed-width, fixed-order typed slot layout; still just perception geometry |
| per-agent `track_age_ticks`, `pos_sigma_m` (I2) | `Track.age_ticks`; `√(½·tr P₂ₓ₂)` from the EKF covariance | the tracker's own confidence in its own estimate — never ground truth |
| per-agent `d_rate_mps`, `d_accel_mps2` (I2) | `PredictionResult.lateral_rate_by_track` / `long_accel_by_track` = the EMA-smoothed, Frenet-geometry-corrected `drift_ema` / `accel_ema` the intent stage already computes from the sensor tracks | a de-biased derivative of the tracked offset; the noise-robust replacement for leaning on the heuristic `p_cut_in` scalar |
| road context ahead: `curvature_profile_1pm` (at 0/15/30/50/80 m), `dist_to_next_maneuver_m` (I2) | `signed_curvature_at()` sampled along the route `FrenetFrame` | map geometry, sampled ahead — same class as the single `road_curvature_1pm`, just a profile |
| per-agent intent (`p_cut_in`, `p_decel`, `p_lane_keep`) | `PredictionEngine` intent distributions, computed **only** from the sensor tracks | derived from perception, not from oracle intent; the curved-road phantom is quantitatively bounded (`f302ea2`). **Retained as ONE input, no longer the sole temporal signal** — I2 Experiment A showed it triples under measurement noise; a learned head is expected to prefer `d_rate_mps` + the K-frame history. |
| `binding_constraint` | `speed_limit_reason` | which deterministic cap is active — a fact about the ego's own controller, not the world |

## Forbidden inputs

- `TrafficModel`, `NpcVehicle`, `traffic.npcs`, or any sequence of them.
- Exact NPC positions / velocities (the sim's own state, before the sensor
  model). Only `SurroundTrack` values (post-frustum, post-EKF, post-noise)
  are allowed.
- Exact future NPC trajectories / the sim's own rollout. (These may be used
  as **training labels** or for **evaluation scoring** — never as a runtime
  **input**.)
- Oracle intent (the scripted `wander`/lane-change a scenario applied). Only
  the *estimated* intent distribution is allowed.
- `PhysicsEngine` itself, or any handle that could reach the world model.

## Enforcement

1. **Static** — `test_learning_package_does_not_import_privileged_modules`
   parses every file in `app/services/learning/` and fails on an import of
   `traffic` / `physics_engine` / `scenario_engine`.
2. **API** — `build_observation` raises `TypeError` if any argument is a
   `TrafficModel` / `NpcVehicle` / `PhysicsEngine` (or a sequence of them).
3. **Runtime semantics** —
   `test_observation_depends_only_on_perception_not_ground_truth` runs the
   engine, builds an observation, then **teleports the ground-truth NPCs**
   without re-running perception, rebuilds, and asserts the two observations
   are **identical**. The observation is a pure function of the sensor
   tracks + prediction + ego state.

## I2 status

The **representation** is now decided by evidence (see
`I2_EXPERIMENT_A_FINDINGS.md` Part B): **Rep A** — a flat, fixed-role agent
substrate + K-frame history stack. Rep B (a permutation-invariant set encoder)
was built and measured on the same held-out data with measurement noise on and
did not beat Rep A on either probe task; it stays a candidate to re-evaluate at
I3 *trained*. The boundary itself is unchanged: sensor-only, one choke point,
enforced by test. History length (K = 10 @ 10 Hz) and normalisation live in the
consumer / probe harness, not in `build_observation` (which stays single-frame
so the runtime-invariance test still holds).
