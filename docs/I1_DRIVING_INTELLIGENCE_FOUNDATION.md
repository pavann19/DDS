# I1 — Driving Intelligence Foundation

**Companion to:** `ADR-003-LEARNED-DRIVING-INTELLIGENCE.md` (invariants).
**Purpose:** answer the 12 investigation questions from the *actual* DDS
codebase, then propose the experimental path for I2 (learned world
understanding) → I3 (learned prediction) → I4 (learned policy). This document
does **not** pick a neural architecture — it scopes what must be learned and
how we will know it worked.
**Status:** draft for owner review. No model/training code until the I2/I3/I4
plans below are accepted (instruction §9, §17 step 9).

Code references are to the tree at the time of writing (branch
`phase-7.5-ui-design`, after the Step-1/Step-2 control fixes).

---

## Q1. What information does the vehicle currently receive?

Per tick, `PhysicsEngine.update()` assembles its control inputs from:

| Input | Source | Nature |
|---|---|---|
| Ego proprioception: `speed_kmh`, `acceleration_mps2`, `heading`, `steering_angle_rad`, `current_station_m`, `current_lateral_offset_m` (Frenet `d`), `path_curvature`, `_road_kappa_signed`, `yaw_rate_degps`, `kinematic_sideslip_rad`, `friction_mu` | integrated internally | exact (it *is* the sim state) |
| Route geometry | `smooth_route()` → `FrenetFrame` (`build_frenet_frame`) | exact, static per route; Catmull-Rom-smoothed OSRM/synthetic polyline resampled ~5 m |
| Forward lead: bumper-to-bumper `gap_m`, `lead_speed_kmh` | `TrafficModel.sense_lead_vehicle()` (collision corridor) | sensor-gated; optional Gaussian gap/speed noise + dropout (`sensor_noise_enabled`, `sensor_dropout_rate`, opt-in) |
| Adjacent-lane clear | `TrafficModel.sense_lane_clear()` → **bool** | sensor-gated, reduced to a single boolean |
| Surround tracks: per-track `x, z, vx, vz, range_m, azimuth_deg, entity_class, status, size` | `SurroundPerceptionEngine.step()` (frustum rig + EKF + occupancy grid) | frustum-gated; **detections are noise-free NPC positions**, EKF adds process noise + confirm/coast latency |
| Prediction: per-agent forecasts, intent distribution, `cut_in` summary, `proactive_decel_mps2`, a spatiotemporal `risk_field` | `PredictionEngine.step()` off the surround tracks | derived; CTRA forecast + heuristic intent (`estimate_intent`) |
| Powertrain telemetry: RPM, CO2, coolant, fuel rate + deltas | `get_ml_features()` | exact; **analytics only** (ADR-001) |

## Q2. Which sensor observations are genuinely available (vs privileged)?

**Genuinely sensor-derived (the ego is *not* given oracle NPC state):**
- `sense_lead_vehicle` — one forward collision corridor, range ≤ 100 m,
  reports only `{gap, lead_speed}` of the nearest in-corridor vehicle.
- `sense_lane_clear` — a boolean per adjacent lane.
- `SurroundPerceptionEngine` — 5 frustums (forward long/wide, L/R blind spot,
  rear), range 60–150 m, FOV 30–120°. An NPC outside every frustum this tick
  is simply not detected; brief dropouts are coasted by the EKF.

**Privileged / not realistic yet:**
- **Perception detections carry no positional noise.** `DetectedEntity.x/z` is
  the exact projected NPC position; only the EKF *process* model adds
  uncertainty. Real perception has measurement noise, false positives,
  misclassification, and range-dependent error — none of that exists on the
  ego's detection path by default.
- **`DetectedEntity.vx/vz = speed_mps · route_tangent`** — the NPC's exact
  speed along the *route* tangent, not an estimate from position history. (This
  exact-velocity-on-tangent is what produced the curved-road phantom-cut-in
  `v_d` bias.)
- **NPC-to-NPC IDM uses oracle state** (`traffic.py` module comment) — fine for
  the *world sim*, but it means NPC behaviour is itself scripted/deterministic,
  not learned or reactive to the ego in a rich way.
- The ego's own Frenet `s/d` and `path_curvature` are computed from the exact
  route geometry — a real vehicle estimates these from localisation + a map.

**Implication for I2/VII:** if a learned component is to satisfy invariant VII
(no privileged-state shortcut), the sensor layer needs a configurable
*measurement* noise/false-positive/occlusion model on the **detection** path
(not just the EKF process model), so the learned representation is built from
something a real sensor could produce.

## Q3. Which information is privileged simulator state? — (answered in Q2)

Privileged: exact NPC pose/velocity in detections, NPC-NPC oracle IDM, exact
ego Frenet state, exact route curvature, `friction_mu` as a known scalar.
Not privileged: which NPCs are in frustum/corridor, the forward gap, the
lane-clear booleans, the EKF-tracked surround picture.

## Q4. Which current decisions are handcrafted?

Every behavioural decision is deterministic/handcrafted today:

| Decision | Producer | Method |
|---|---|---|
| Target speed | `physics_engine` | `min(cruise, curve-cap √(a_lat·μ/κ), tracking-error cap, IDM, proactive-decel, spatiotemporal-plan cap)` |
| Longitudinal accel | `physics_engine` | bounded P-controller + jerk limiter, composed with IDM via `min()` |
| Front-wheel steering | `planner.ff_stanley_steer` (Step 1) | feed-forward `atan(L·κ)` + Stanley heading + cross-track feedback |
| Lateral offset target | `driver.plan_lateral_offset` / `planner.spatiotemporal` | quintic-cost lattice; joint `(s,d,t)` optimiser during a committed lane change |
| Lane-change lifecycle | `planner.state_machine` | finite state machine (`LANE_KEEP`/`PREPARE`/`EXECUTE`/`ABORT`), threshold-gated on lead gap + adjacent-clear |
| Proactive cut-in ease-off | `prediction_engine` | heuristic `estimate_intent` (sigmoid on lane-relative drift) → fixed `PROACTIVE_DECEL_MPS2`, bounded ease-off |
| Emergency brake / veto | `safety_shield` | TTC thresholds, RSS-style margins, hard proximity floor, boundary/lateral-accel limits |
| Route | `routing.py` | OSRM contraction-hierarchies Dijkstra + spline smoothing, or synthetic highway |

The lattice/quintic **optimiser** is genuine decision-making, but the *cost
weights*, the *lane-change triggers*, the *intent sigmoid shape*, and the
*speed-cap composition* are all hand-tuned constants.

## Q5. Which current ML components are irrelevant to driving?

- **XGBoost eco-driving classifier** (`inference.py`) — powertrain telemetry
  only; `tests/test_ai_attribution.py` proves it cannot change the trajectory.
  Analytics / driver-score only.
- **SHAP TreeExplainer** — explains the classifier; HMI only.
- **Isolation Forest anomaly detector** — 8-D powertrain vector; monitoring only.

None of these observe the road, traffic, or geometry. They are irrelevant to
"driving on AI" and stay as analytics.

The prediction engine's **intent estimation is a hand-written heuristic**, not
a learned model — it's the most obvious first target for I3.

## Q6. What information does a learned driver actually need?

To choose behaviour (lane, gap acceptance, when to commit/abort a change, how
much to ease off), it needs, ego-relative and road-frame-normalised:

- ego kinematics: speed, longitudinal accel, lateral offset + rate, heading
  error to the road tangent, upcoming curvature profile (not just a scalar),
  distance-to-goal / route context, `friction_mu`;
- for each nearby agent in the ego lane and each adjacent lane (a small set,
  masked): relative longitudinal gap + rate, relative lateral offset + rate,
  a behaviour/intent signal, size class, and *how confident perception is*;
- lane structure: how many same-direction lanes, which are legal to occupy
  (needs the Phase-10 semantic map to be meaningful; a highway prior until
  then), current lane index, distance to the next required maneuver;
- which deterministic constraint is currently binding (so the policy can
  learn "the curve cap is limiting me, a lane change won't help").

It does **not** need raw pixel/point-cloud tensors in this sim — the world is
low-dimensional and the sensor model is geometric. A hand-derived *or* a
small learned encoding of the above is sufficient; which one is an I2
experiment, not a decision here.

## Q7. What should the AI *learn* rather than receive as a hand-engineered feature?

Strong candidates to learn instead of hand-coding:

1. **Agent behaviour prediction** — replace `estimate_intent`'s sigmoid +
   CTRA forecast with a learned multi-modal predictor (I3). The heuristic
   already has a known failure (curved-road false-positive cut-in); a learned
   predictor trained on sim + NGSIM should generalise better.
2. **Behaviour selection / value** — replace the hand-tuned lane-change
   triggers and the fixed proactive-decel with a learned estimate of "which
   behaviour leads to the best outcome here" (I4).
3. **A compact world/interaction representation** — whether an explicit
   hand-derived vector is enough, or a small learned latent over the agent set
   + history captures interaction structure the hand features miss, is the
   central I2 experiment.

Keep hand-coded (do NOT learn): vehicle physics, Frenet geometry, feasibility
filtering, the comfort/grip envelope, the Safety Shield, the actuator model.

## Q8. What representation preserves temporal and multi-agent context?

Requirements: variable agent count (mask/pad), permutation-invariance over
agents, a few hundred ms of history (to read acceleration/intent), and the
road frame (so a curve doesn't look like a maneuver). Options to evaluate in
I2 (not decide now): stacked hand-feature frames; a per-agent MLP + symmetric
pool (DeepSets); a small graph/attention block over the agent set; a short
recurrent state. Selection criterion: smallest representation that lets I3's
predictor and I4's policy beat the deterministic baseline on the closed-loop
metrics, within the inference budget.

## Q9. What training signals are available?

- **Imitation labels** — log `(observation, deterministic-planner decision)`
  over the scenario suite + randomised traffic/routes. Warm start only
  (invariant VIII).
- **Counterfactual outcome scores** — for a logged state, roll each candidate
  behaviour forward under the deterministic planner + world sim, score it
  (progress, comfort/jerk, min-TTC, shield interventions, off-lane). Cheap,
  unlimited, grounds learning in *outcomes*.
- **Human trajectories** — FHWA NGSIM US-101 / I-80 (Phase 13): reconstruct the
  observation per real vehicle-frame, label the behaviour the human actually
  took. The "expert" signal that lets the policy exceed the deterministic
  teacher.
- **Closed-loop reward** — run the policy in the sim and score whole episodes
  (for any RL/offline-RL arm).
- **Prediction targets** — the sim's own future NPC trajectories are exact
  labels for I3 (used as *labels*, never as runtime *inputs* — invariant VII).

## Q10. What experiments distinguish genuine learning from memorisation / imitation?

- **Held-out generalisation** — train on scenario seeds `S_train`, evaluate on
  disjoint `S_test` + unseen traffic densities + sensor degradation
  (`sensor_noise_enabled`, `sensor_dropout_rate`) + (Phase 13) held-out NGSIM
  time-segments. Memorisation collapses here.
- **Counterfactual perturbation** — nudge one agent's gap/speed by a small
  amount and confirm the policy's output changes *continuously and sensibly*
  (a memoriser is brittle / discontinuous).
- **Beats-the-teacher check** — on metrics where the deterministic planner is
  known to be suboptimal (late lane-change commitment, conservative gap
  acceptance, the curved-road cut-in false-positive), the learned component
  should measurably improve, not just match.
- **Ablate the history / interaction** — if removing temporal or multi-agent
  context doesn't hurt, the model wasn't using it (it memorised static cues).

## Q11. What architecture is appropriate for the available compute/data?

Constraints: CPU real-time at the 10 Hz control tick (single-digit ms
budget), reproducible inference, small labelled-data volume initially (sim +
a few NGSIM hours). This *rules out* large transformers and heavy RL from the
start and *favours* the smallest model that clears the gates — a GBM ranker,
a small MLP, or a small set/graph encoder. **Which one is decided by measured
accuracy/latency/closed-loop performance in I2–I4, not asserted here.** Scale
up only when a smaller model demonstrably plateaus and more data exists.

## Q12. What is evidence that the learned component truly controls behaviour?

All of, per invariant I + the ADR evaluation section:

1. **AI ON/OFF A/B** on identical `(scenario, seed, route, traffic, physics,
   safety)` shows a *measurable behavioural difference* (lane choice, commit
   time, gap at acceptance, speed profile) on a meaningful fraction of
   situations — **with no regression** in collisions, min-TTC distribution,
   comfort cost, or progress.
2. **Causal attribution** — the provenance record shows the learned output was
   the thing that changed the selected behaviour (the deterministic
   alternative was different), not a coincident change.
3. **Closed-loop, not snapshot** — the difference holds over full episodes in
   the observe→infer→act→new-observation loop, not just on isolated examples.
4. **Generalises** — the difference and the safety record hold on the held-out
   set from Q10.
5. **Fails safe** — with the model force-disabled mid-episode, the system
   degrades to the deterministic baseline, never worse.

A model whose output changes while the vehicle stays behaviourally identical,
or that only moves dashboard numbers, is explicitly **not** evidence.

---

## Proposed experimental path (for owner approval)

Each stage: define what must be learned → build the smallest thing → measure
against the criteria → keep only if it beats the baseline → document evidence.
Do not proceed to the next stage without evidence the current one succeeded
(instruction §10, §12).

### I2 — Learned world / interaction understanding

- **DONE (boundary prerequisites, no model yet):**
  - `PerceptionNoiseConfig` -- configurable measurement position/velocity
    noise, dropout, occlusion, latency, ghosts on the detection path; default
    no-op is byte-identical to today (`f87d35b`).
  - Curved-road phantom cut-in quantitatively bounded: <=12% of ticks at
    R=60 m, 0% at R>=120 m, <0.6 km/h speed cost (`f302ea2`).
  - Sensor-only `Observation` builder + `build_observation()` choke point +
    static/runtime enforcement that it never sees `traffic.npcs` or
    ground-truth NPC state (`748bb65`, `docs/OBSERVATION_BOUNDARY.md`).
- **Remaining:** log the `Observation` over the scenario suite / randomised
  traffic / routes; run Experiment A.
- **Experiment A:** hand-feature vector vs a small learned encoder (DeepSets /
  small attention) as the *input to a fixed probe task* (predict the
  deterministic planner's next lane choice + the true next-3 s NPC positions).
  Metric: probe accuracy + latency. Keep the smaller one unless the learned
  encoder clearly wins.
- **Deliverable:** `docs/I2_*` with the chosen representation + evidence.

### I3 — Learned prediction

- Replace `estimate_intent` + the CTRA forecast with a learned multi-modal
  agent predictor (history + interaction context from I2). Targets: the sim's
  exact future NPC trajectories (labels) + NGSIM when available.
- **Must handle:** multiple futures, uncertainty, lane change / brake / merge /
  yield, and the curved-road case the heuristic gets wrong.
- **Gate:** better trajectory ADE/FDE than the heuristic on held-out seeds +
  NGSIM segments; the curved-road false-positive cut-in rate drops; feeding it
  into the existing `risk_field` + proactive-ease-off path does not regress the
  scenario suite.
- **Deliverable:** `docs/I3_*` + provenance for prediction outputs.

### I4 — Learned driving policy

- With I2's representation and I3's prediction, build the first learned
  behaviour selector. **Hand-off mechanism chosen by experiment** among:
  re-rank the Phase-8 lattice; emit a behaviour token to the FSM/planner;
  reshape the planner cost field; propose a reference line. Whatever it is,
  it passes through the unchanged feasibility filter + Safety Shield (II).
- Behind a flag, default OFF; invariant-III CI test (OFF == deterministic
  byte-for-byte).
- Training: imitation warm start → counterfactual outcome scoring →
  (optionally) closed-loop RL in sim, plus NGSIM.
- **Gates:** the Q12 evidence set — ON/OFF A/B behavioural delta with no
  safety regression, causal attribution, closed-loop, generalisation,
  fail-safe.
- **Deliverable:** `docs/I4_*` + full `AIPolicyDecision` provenance + the A/B
  report + the robustness eval.

### I5+ (closed-loop learning, trajectory intelligence, social driving,
uncertainty-aware behaviour, memory, failure-driven improvement) — scoped only
after I4 has evidence of success; each defined the same way (what to learn →
smallest build → measure → keep only if it improves → document). Not
pre-planned here.

---

## Immediate next actions

1. Owner reviews ADR-003 (invariants) + this I1.
2. On acceptance: implement the I2 sensor *measurement*-noise model + the
   `Observation` builder + logging harness (no model yet), with the
   OFF==today regression test.
3. Run I2 Experiment A; write `docs/I2_*` with the evidence and the chosen
   representation.
