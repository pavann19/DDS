# I2 Experiment A — Findings

**Harness:** `experiments/i2_experiment_a.py` (tracked; raw per-tick logs go to
`experiments/_out/`, gitignored). **No neural network trained.**
**Data:** the 8 built-in scenarios × 3 seeds + 12 randomised traffic/route runs,
260 ticks each, ≈ 8 400 logged `Observation` frames per configuration, run twice
— once with `PerceptionNoiseConfig` **off** (default), once **on**
(pos σ 0.25 m + 0.6 m/100 m, vel σ 0.5 m/s, 6 % dropout, occlusion, 1-tick
latency).

Purpose: decide whether the current sensor-only `Observation` representation
carries enough **temporal**, **multi-agent**, **uncertainty** and
**road-context** information for a learned component, and quantify
privileged-information leakage, coverage, distribution, missingness and
repeatability — then choose the I2 representation **experimentally**.

---

## 1. Quantified results

### Repeatability — PASS
`(scenario, seed)` produces a **bit-identical** `Observation` stream: **8 / 8**
scenario pairs identical, both noise-off and noise-on (the perception-noise RNG
is seeded from the engine seed). Reproducibility invariant (ADR-003 V) holds at
the observation level.

### Privileged-information leakage — NO TRUE LEAK
Cheap ridge linear probe, `Observation` numeric vector → ground-truth quantity.
High R² on a quantity the ego's sensors could **not** produce = a leak.

| ground-truth target | R² (noise off) | R² (noise on) | reading |
|---|---:|---:|---|
| `ego_true_d` | 1.000 | 1.000 | **not a leak** — `lane_offset_m` is a permitted localisation input |
| `path_curvature_true` | 1.000 | 1.000 | **not a leak** — `road_curvature_1pm` is a permitted map input |
| `n_npcs` (total NPC count) | 0.91 | 0.86 | count of **tracked** agents ≈ count of in-range NPCs. A real radar also reports its return count; it reveals nothing about NPCs outside sensor range. Acceptable, but see §2.4. |
| `nearest_ahead_gap_m` | 0.50 | 0.44 | blurred lead range — the forward sensor + agent slots carry a noisy version, as intended |
| `nearest_ahead_speed_mps` | 0.64 | 0.62 | blurred lead speed |
| `nearest_ahead_lane_offset` | 0.48 | 0.34 | blurred; noise reduces it further |
| `any_npc_lat_speed` (exact NPC wander) | 0.45 | 0.37 | mostly hidden; noise reduces it |

**Conclusion:** the only R² = 1.0 targets are the two inputs the boundary
explicitly permits (ego localisation + map curvature). Everything a sensor
should blur is blurred (R² 0.34–0.64) and enabling measurement noise reduces
recoverability across the board. `test_observation_boundary` already proves the
observation is invariant to teleporting the ground-truth NPCs. **No privileged
runtime input.**

### Coverage — adequate, two thin regions

| situation | frac (noise off) | frac (noise on) |
|---|---:|---:|
| curve present (κ > 0.002) | 0.25 | 0.25 |
| has a forward lead | 0.35 | 0.34 |
| lane change in progress (gt) | 0.14 | 0.14 |
| near-stop / queue (v < 3 m/s) | 0.13 | 0.13 |
| ≥ 1 tracked agent | 0.96 | 0.95 |
| ≥ 4 tracked agents | 0.50 | 0.34 |
| all 8 agent slots full (saturated) | **0.21** | 0.08 |
| a cut-in signalled (any slot p_cut > 0.5) | **0.17** | **0.54** |
| low friction (< 0.9) | **0.08** | 0.08 |
| Safety Shield active (gt) | **0.06** | 0.05 |

- **Low-friction (8 %) and shield-active (6 %) are under-represented** — I2/I3
  training and eval need targeted scenarios / seeds for these.
- **The cut-in signal triples (17 % → 54 %) under measurement noise.** Position
  jitter feeds spurious lateral rate into the *heuristic* `estimate_intent`.
  The current observation carries the heuristic's pre-integrated `p_cut_in`,
  which is not noise-robust — a strong argument for a **learned** predictor
  (I3) that sees raw agent history rather than the observation shipping a
  fragile heuristic scalar.

### Missingness — three real gaps

| field | None / dead fraction |
|---|---:|
| `lead_gap_m` (and `lead_closing_mps`) | **0.65** — None most of the time |
| `AgentSlot.lateral_rate_mps` | **1.00 — always 0, never populated** |
| agent slots all-full (agents being dropped) | 0.21 (noise off) |

### Distribution — one genuinely degenerate field
Of 30 flattened features, exactly **one is constant**: `n_same_direction_lanes`
= 3 (a hardcoded highway prior; real value needs the Phase-10 semantic map —
accept, do not fake). `ego_lane_index` **was** constant-0 and is fixed in this
change (now derived from `lane_offset_m` vs the lane centres). Everything else
varies with non-zero spread. `binding_constraint` takes 8 distinct values in
the data (`cruise`, `car_following`, `tracking_correction`,
`lateral_accel_limit`, `low_friction_curvature`, `predictive_cut_in`,
`spatiotemporal_plan`, `safety_shield_override`) — informative.

---

## 2. Sufficiency verdict per dimension

### 2.1 Temporal — **INSUFFICIENT**
The `Observation` is a single Markov frame. It carries two ego first
derivatives (`accel_mps2`, `lateral_rate_mps`) and one per-agent rate
(`closing_mps`); `AgentSlot.lateral_rate_mps` is dead. No history buffer, no
agent acceleration, no time-since-detection. Cut-ins / merges / braking are
temporal events; the only temporal signal is the heuristic `p_cut_in`, which
§1 shows is noise-fragile. A learned component that must form its own
predictions needs raw recent history.

### 2.2 Multi-agent — **PARTIALLY SUFFICIENT, STRUCTURALLY FLAWED**
`MAX_AGENTS = 8` masked slots exist and are exercised (≥ 4 agents 50 % of
ticks). But slots are filled **nearest-first by raw `range_m`**, mixing
oncoming, rear and ahead traffic with no role typing, and **saturate 21 % of
ticks** — relevant agents are dropped for nearer irrelevant ones. Per-agent
lateral motion is absent.

### 2.3 Uncertainty — **WEAK**
Only `track_confidence ∈ {0.4, 1.0}` (COASTED vs CONFIRMED). No covariance, no
per-field confidence, no track age. With measurement noise enabled there is
real uncertainty to represent and the observation cannot.

### 2.4 Road-context — **INSUFFICIENT**
One scalar `road_curvature_1pm` at the current station only — no curvature
*profile* ahead (a driver needs to see the bend coming). `n_same_direction_lanes`
hardcoded (Phase-10 gap). No distance-to-next-required-maneuver, no lane
legality / markings. `binding_constraint` is the one strong road-context
feature. `n_npcs` recoverability (§1) is partly because the frustum sensor with
default settings sees essentially all in-range NPCs — a tighter occlusion /
range model would make "scene density" less trivially exposed.

---

## 3. I2 representation choice — driven by the evidence, not by ease

Adding more scalar fields to the flat single frame (the easiest change) does
**not** address §2.1 (no history), §2.2 (no typing / relevance), or §2.3 (no
per-agent uncertainty). The evidence forces a structured representation with a
short history and a typed agent set. The I2 experiment will build and **measure**
two candidates on held-out probe tasks — do **not** pick the first one because
it is a plain array:

**Common substrate (both candidates):**
- **K-frame history stack**, ~1 s (K ≈ 10 at 10 Hz), for the ego block and the
  agent terms — gives temporal + lets a learned head do its own prediction
  instead of consuming the fragile heuristic `p_cut_in`.
- **Role-typed agent set** instead of nearest-8-by-range: ego-lane
  lead/follower, left-lane lead/follower, right-lane lead/follower, + 2 "other
  nearest" — removes saturation and role mixing. Per agent:
  `(ds, dd, closing, d_rate, d_accel, length, track_age, track_conf)`.
  (`lead_gap` is absorbed here — kills the 65 %-None standalone field.)
- **Curvature + lane profile:** `road_curvature_1pm` sampled at 5 lookahead
  distances; `dist_to_next_maneuver`; `ego_lane_index` (fixed); lane-legality
  left as a Phase-10 prior.
- Per-track **uncertainty**: age + a coarse position-covariance proxy from the
  EKF, so the model can down-weight stale / noisy tracks.

**Rep A — flat hand-features over the substrate.** Concatenate the history
stack + typed agent slots + road profile into one vector. Cheapest inference;
no permutation handling needed (roles are fixed).

**Rep B — small permutation-invariant set encoder (DeepSets)** over the typed,
history-stacked agent set, then concat ego + road context. Handles a variable
"other" count cleanly; can learn interaction structure the flat concat cannot.

**Decision procedure (I2 proper):** train each on two probe tasks — (1) predict
the deterministic planner's next lane choice, (2) predict true next-3 s NPC
positions (ADE/FDE) — on scenario-seed-held-out data, **with measurement noise
on**, and record CPU inference latency at 10 Hz. Keep the smaller/faster one
unless the set encoder wins the probe accuracy by a margin that matters for I3.
No architecture is selected before those numbers exist.

---

## 4. Actions before I3

1. **Populate `AgentSlot.lateral_rate_mps`** — keep per-track Frenet-`d`
   history in `build_observation` / a small stateful wrapper (the de-biased
   rate from `prediction_engine` is the right source).
2. **Absorb `lead_gap` into the role-typed agent set** — drop the standalone
   nullable field.
3. **Add per-track `age` + covariance proxy** to `AgentSlot`.
4. **Curvature profile** (5 samples) + `dist_to_next_maneuver` in `Observation`.
5. **Targeted low-friction + shield-active scenarios/seeds** for coverage.
6. Re-run Experiment A on the revised representation to confirm the gaps close
   and leakage stays clean, then start I2 proper (Rep A vs Rep B probe).

---

# Part B — revised substrate + Rep A vs Rep B (executed)

**Code:** observation-substrate changes landed in
`app/services/learning/observation.py`,
`app/services/perception/perception_engine.py` (`SurroundTrack.age_ticks`,
`SurroundTrack.pos_sigma_m`),
`app/services/prediction/prediction_engine.py`
(`PredictionResult.lateral_rate_by_track` / `long_accel_by_track`),
`app/services/physics_engine.py` (`build_learning_observation` now emits the
curvature profile + `dist_to_next_maneuver_m` + passes the per-track rates).
**Probe harness:** `experiments/i2_experiment_ab.py` (tracked; raw output to
`experiments/_out/`, gitignored). **No policy and no neural network trained** —
the only thing fitted is a closed-form ridge analysis probe; Rep B's set
encoder is a *fixed random* ReLU projection.

## 5. What changed in the representation (I1 §4 actions)

| Action | Done | How |
|---|---|---|
| 1. populate `AgentSlot.lateral_rate_mps` | ✅ | = `d_rate_mps`, from `PredictionResult.lateral_rate_by_track` (the EMA-smoothed, Frenet-geometry-corrected `drift_ema` — noise-robust, not the raw jitter) |
| 2. absorb `lead_gap` into the typed set | ✅ (partial) | the `ego_lead` role slot carries the tracked lead; the standalone `lead_gap_m` is **retained** as a distinct longer-range forward 1-D corridor sensor (needed by the boundary contract/tests, genuinely a separate sensor) |
| 3. per-track `age` + covariance proxy | ✅ | `AgentSlot.track_age_ticks` (from `Track.age_ticks`); `AgentSlot.pos_sigma_m` = √(½·tr(P₂ₓ₂)) from the EKF covariance |
| 4. curvature profile + `dist_to_next_maneuver` | ✅ | `Observation.curvature_profile_1pm` at look-aheads (0,15,30,50,80) m; `dist_to_next_maneuver_m` = distance to the next `|κ|>1/50 m` bend, capped 200 m |
| 5. role-typed agent set | ✅ | `ROLE_SLOTS` = ego/left/right × lead/follow (6) + 2 nearest "other"; nearest-per-role by `|ds|`. Replaces nearest-8-by-raw-range. |
| 6. fragile heuristic cut-in fields | ⚠️ replaced-as-primary | `p_cut_in`/`p_decel`/`p_lane_keep` retained as ONE input; the primary temporal substrate is now `d_rate_mps` + `d_accel_mps2` + the K-frame history stack, so an I3 learned head does not depend on the noise-inflated heuristic scalar |
| 7. targeted low-friction / shield scenarios | ❌ deferred | needs new scenario definitions; still 8 % / 5 % coverage. Tracked for the pre-I3 data pass. |

## 6. Re-run of Experiment A on the revised substrate (noise ON, 9 360 ticks)

| dimension | I1 (noise on) | revised | reading |
|---|---|---|---|
| `AgentSlot.lateral_rate_mps` dead fraction | **1.00** | **0.078** | populated; 0 only for brand-new / unconfirmed tracks |
| agent-slot saturation (`8/8 full`) | 0.08 (0.21 noise-off) | **0.00** | role typing removed saturation and role-mixing |
| repeatability (`(scenario,seed)` identical stream) | 8/8 | **8/8** | still bit-reproducible (ADR-003 V) |
| constant features | 1 (`n_same_direction_lanes`) | **1** (same; Phase-10 map gap, not faked) | — |
| leakage: R²=1.0 targets | `ego_true_d`, `path_curvature_true` only | **same** | both are permitted localisation/map inputs; `n_npcs` R² 0.86→0.77; everything a sensor should blur stays 0.32–0.61 → **no privileged runtime input** |
| `low_friction` / `shield_active` coverage | 0.08 / 0.05 | 0.083 / 0.054 | **still thin** — action 7, deferred |

## 7. I2 proper — Rep A vs Rep B (held-out (scenario,seed), noise ON)

`experiments/_out/experiment_ab_summary.json`. Train 5 784 frames
(scenarios×seeds {1,2} + 8 randomised), test 2 892 (scenarios×seed {3} + 4
randomised). K-frame history = 10 (1 s @ 10 Hz).

| | **Rep A** (flat, fixed-role slots) | **Rep B** (fixed-random ReLU φ → sum+mean pool) | constant-velocity baseline |
|---|---:|---:|---:|
| lane-choice probe — accuracy | 0.972 | 0.968 | (base rate 0.958) |
| lane-choice probe — **macro-F1** | **0.542** | 0.493 | — |
| NPC next-3 s — **ADE (m)** | **7.38** | 8.07 | 14.63 |
| NPC next-3 s — FDE (m) | 12.13 | 13.26 | 24.64 |
| feature-build latency — median (µs) | **19.4** | 51.3 | — |
| feature-build latency — p95 (µs) | 30.2 | 85.9 | — |
| representation dim | 149 | 102 | — |

**Reading of the evidence**

- **Temporal dimension materially improved.** Both representations roughly
  **halve** the constant-velocity NPC-prediction error (ADE 14.6 → 7.4 / 8.1 m).
  The K-frame history + `d_rate` / `closing` carry real predictive signal the
  I1 single Markov frame did not — I1 §2.1 ("temporal INSUFFICIENT") is
  addressed at the substrate level.
- **Rep B (untrained) does not beat Rep A on either probe** and costs 2.6×
  the feature-build latency. Permutation-invariant pooling with a *random*
  φ discards the fixed-role directional structure the lane probe needs
  (macro-F1 0.49 vs 0.54) and adds nothing to NPC prediction here.
- **Latency** is a non-issue for both at 10 Hz (< 55 µs vs a 100 ms budget) —
  it is not the deciding factor; the probe numbers are.

**Decision (evidence-based, not ease-based):** adopt **Rep A** — the flat,
fixed-role substrate — as the I2 representation. Rep B was built and measured
on the same data and lost on both probe tasks; it is not selected. This does
**not** foreclose a *trained* DeepSets: at I3, when the actual learned head
exists and can fit φ/ρ by SGD, Rep B must be re-evaluated (a trained set
encoder can learn interaction structure a random projection cannot). The
decision rule and both runs are reproducible via
`python -m experiments.i2_experiment_ab --noise`.

## 8. Exact next step

1. **Coverage (action 7):** add targeted low-friction and Safety-Shield-active
   scenario/seed definitions so I3 training/eval is not <10 % on those regimes.
2. **I3 (still gated):** build the learned head on Rep A + the K-frame stack;
   re-run the Rep A vs *trained* Rep B comparison. Do **not** start I3 model
   training until the CARLA-backed observe→decide→control→feedback loop is
   functioning and validated (per the standing instruction).
