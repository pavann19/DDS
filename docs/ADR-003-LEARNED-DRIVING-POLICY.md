# ADR-003 (SUPERSEDED) — see ADR-003-LEARNED-DRIVING-INTELLIGENCE.md

> **This draft pre-committed to a specific architecture (64 hand-derived
> features -> gradient-boosted candidate re-ranker). Per the next-phase
> instructions (do not pre-decide the model/architecture), it has been
> replaced by `ADR-003-LEARNED-DRIVING-INTELLIGENCE.md`, which fixes the
> INVARIANTS (causal authority, deterministic safety supremacy, OFF==
> deterministic, fail-safe, reproducibility, provenance, honest
> observation, generalisation) and leaves the architecture to be
> determined by the I1 investigation + experiments. The re-ranker below
> is retained only as ONE hypothesis to evaluate.**

---

# ADR-003: A Learned Behaviour-Ranking Driving Policy (constrained by the deterministic planner)

**Status:** Proposed — awaiting project-owner review. **No implementation until accepted.**
**Date:** 2026-09-10
**Deciders:** Project owner (sole maintainer)
**Related:** ADR-001 (World/Driver split; ML kept out of the control path), Phase 8
(joint `(s, d, t)` lattice planner), Phase 13 (NGSIM replay), the P0 backlog item
*"Learned Driving Policy Architecture"* and its five siblings (Policy–Planner
Integration, AI Policy Provenance, End-to-End AI Closed Loop, AI ON/OFF A/B,
Unseen-Scenario Robustness).

---

## Context

### What the owner asked for

> "The system must drive on AI, not by ML models."
> "Right now there is no Artificial Intelligence making decisions driving — we need
> to implement this from now."
> Target: a Tesla/Waymo-level driving system, the only difference being that theirs
> run on real hardware and roads and this one runs in software.

Today DDS has a genuine decision-making planner, but it is **entirely
deterministic**: Frenet lattice generation, quintic/quartic trajectory
primitives, an IDM longitudinal model, a joint `(s, d, t)` cost optimiser
(Phase 8), a lane-change finite-state machine, and an independent Safety
Shield. `tests/test_ai_attribution.py` proves the only ML in the repo (an
XGBoost eco-driving classifier over powertrain telemetry) has **no path into
control** — three different classifier outputs produce a bit-identical driven
trajectory. ADR-001 put it there deliberately.

So "drive on AI" is not yet true in the sense the owner means: no *learned*
component influences *what the car decides to do*. This ADR defines the
component that changes that, and the contract that keeps it safe and honest.

### Why now, and why this shape

Two failure modes bound the design:

1. **End-to-end learned control** (behaviour cloning or RL that outputs
   steering/accel directly, or outputs a raw trajectory) cannot be formally
   verified against collision boundaries (ISO 26262 / SOTIF), is not
   inspectable, and throws away a working, tested planner. Rejected — it also
   directly contradicts the ADR-001 rationale the owner accepted.

2. **Pure imitation of the current deterministic planner** produces a model
   that, at best, equals its teacher and usually underperforms it, while
   adding opacity. It would technically be "a learned model in the loop" but
   would demonstrate nothing.

The industry answer (Waymo, Mobileye, NVIDIA) is the middle path: learned
models do the parts that require interpreting messy, multi-agent, partially
observed situations — **behaviour prediction and behaviour selection** — and a
deterministic optimiser + formal safety monitor turn the selected behaviour
into a feasible, safe trajectory and retain veto authority.

DDS already has the deterministic optimiser (Phase 8) and the safety monitor
(Safety Shield). The missing piece is the learned *behaviour selector*.

---

## Decision

Add a **learned behaviour-ranking policy** that scores the driving options the
deterministic planner is already considering, and feeds that score in as an
additional **prior / soft cost term** — never as a hard command.

Concretely, the policy answers: *given the current scene, which of the
candidate behaviours (lane choice × target-speed profile × manoeuvre timing)
would an expert driver prefer?* Its output biases the Phase 8 lattice search.
The deterministic feasibility filter, the physical comfort/grip envelope, and
the Safety Shield are **unchanged and remain authoritative** — the policy can
only re-rank options that are *already* feasible and *already* safe.

### The contract (this is the load-bearing part)

| Property | Guarantee |
|---|---|
| **Authority** | The policy produces a bounded score per candidate. It cannot add a candidate, remove the feasibility filter, widen any physical limit, or touch the Safety Shield. |
| **Bounded influence** | The policy's contribution to a candidate's total cost is clamped to `±W_POLICY` where `W_POLICY` is chosen so it can flip the winner *only among candidates whose deterministic costs are within a small band* — it breaks ties and expresses preference, it does not override a clearly-better deterministic choice. |
| **Fail-safe** | If the policy errors, times out (> `POLICY_BUDGET_MS`), or returns non-finite / out-of-range values, its contribution is zero for that tick and `speed_limit_reason` / provenance records `policy_unavailable`. Driving continues on the pure deterministic planner. |
| **Shield untouched** | The Safety Shield runs after the planner exactly as today. A policy-preferred trajectory that the shield vetoes is vetoed. `tests/test_ai_attribution.py::test_emergency_brake_is_attributable_to_the_safety_shield` must still pass verbatim. |
| **Determinism when OFF** | With the policy disabled (`DDS_POLICY_ENABLED=0`, the default until accepted) the planner is byte-for-byte identical to today. A regression test asserts this. |
| **Reproducible when ON** | Inference is deterministic for a fixed `(model version, observation)`; no sampling at drive time (or a fixed seed if sampling is ever used). |

### Observation space (`PolicyObservation`)

Ego-centric, rotation-normalised to the road tangent at the ego's station, all
values already computed elsewhere in the tick:

- **Ego**: speed, longitudinal accel, lateral offset `d`, lateral velocity `ḋ`,
  heading error to road tangent, `path_curvature` ahead (windowed, from the
  Phase-8-adjacent estimator), `friction_mu`, distance-to-goal bucket.
- **Lead / follower in ego lane and each adjacent lane** (up to 6 agents):
  relative `Δs`, relative speed, relative `d`, lane-relative lateral drift
  `v_d` (the same quantity intent estimation uses), and the agent's
  predicted intent distribution from `app/services/prediction/` (LANE_KEEP /
  MERGE_L / MERGE_R / DECEL / STOP).
- **Lane structure**: number of same-direction lanes, which are legal to
  occupy (from Phase 10 when it lands; a 3-lane highway prior until then),
  ego's current lane index.
- **Static**: speed limit, whether a curvature/tracking speed cap is currently
  binding.

Fixed-width, zero-padded, masked. ~64 floats. Deliberately *hand-derived
features*, not raw sensor tensors — it keeps the model small, the training
data requirement modest, and every input inspectable and loggable.

### Action / output space

Two viable encodings; **start with (A)**, keep (B) as the growth path:

- **(A) Candidate re-ranking (chosen).** The planner emits its lattice of
  feasible candidates (lane centre × terminal speed fraction × horizon). The
  policy scores each; scores are min-max normalised and added to the Phase-8
  cost as `-W_POLICY · score_norm` (lower cost = preferred). This needs no
  change to the action interface as behaviours grow, and the feasibility
  filter has already run.
- **(B) Behaviour token.** The policy emits a discrete intent
  `{HOLD, PREP_LEFT, PREP_RIGHT, FOLLOW_CLOSER, EASE_OFF}` + a target-speed
  delta; the FSM and planner consume it as the *desired* behaviour, still
  filtered by feasibility + shield. More like a classical BP head; revisit
  once (A) is working and NGSIM data volume supports it.

### Model class

Staged, smallest-thing-that-works first:

1. **v0 — gradient-boosted ranker** (LightGBM/XGBoost `rank:pairwise`) over the
   ~64-float observation, one row per candidate. CPU inference in tens of µs,
   trivially within `POLICY_BUDGET_MS`, deterministic, and SHAP-explainable
   (reuses existing tooling). This is enough to demonstrate a *learned* driving
   decision and to pass the A/B and robustness gates.
2. **v1 — small MLP** (2–3 layers, ~50k params) if v0's ranking accuracy
   plateaus. Still CPU-real-time, still deterministic.
3. **v2 — set/graph encoder** (tiny DeepSets or GNN over the agent set) only if
   v1 is agent-count-limited. Not before there is NGSIM-scale data.

No transformer, no RL, no end-to-end anything in this ADR's scope.

### Training data

Three sources, combined:

1. **Imitation warm-start** — log `(observation, deterministic-planner's chosen
   candidate)` from the current planner over the scenario suite + randomised
   traffic + randomised routes. Gives a policy that starts no worse than the
   deterministic planner. This is the floor, not the goal.
2. **NGSIM (Phase 13)** — FHWA US-101 / I-80 trajectories. For each real
   vehicle-frame, reconstruct the observation and label the candidate closest
   to what the human actually did over the next horizon. This is the "expert
   driver" signal that lets the policy *exceed* the deterministic teacher
   (smoother gap acceptance, earlier lane-change commitment, human-like
   cooperation).
3. **Counterfactual sim rollouts** — for each logged state, roll every
   candidate forward under the deterministic planner + traffic model, score the
   outcome (progress, comfort, min-TTC, shield interventions), and use it as a
   ranking target. Cheap, unlimited, and grounds the policy in *outcomes* not
   just imitation.

Held-out by scenario seed and by NGSIM time-segment for the robustness gate.

### Integration point

- `PlannerContext` gains an optional `policy_prior: Optional[Callable[[Candidate], float]]`
  (or a precomputed `dict[candidate_key, float]`), default `None`.
- `spatiotemporal.plan()` adds one term to the cost sum:
  `+ (-cfg.w_policy * _clamp(prior_score_norm, -1, 1))` guarded by `if ctx.policy_prior is not None`.
  With `w_policy == 0` (default) or `prior is None`, `plan()` is unchanged —
  the existing `test_*` suite is the regression guard.
- `physics_engine` builds the `PolicyObservation`, calls the policy behind a
  try/except + time budget, and passes the result into `PlannerContext`. On any
  failure it passes `None`.
- The lane-change FSM is **not** changed in encoding (A). It may consume the
  behaviour token in a later (B) iteration.

### Provenance (`AIPolicyDecision`, one per tick the policy runs)

Logged to the same event/telemetry path the Safety Shield uses:

```
model_id, model_version, model_sha256
observation            (the ~64 floats, named)
raw_scores             (per candidate)
selected_candidate     (lane, v1, horizon)  + its deterministic cost + policy delta
would_have_selected    (the pure-deterministic winner, if different)
shield_intervened      (bool)  + override_action if so
latency_ms
status                 (ok | policy_unavailable | clamped)
```

This makes every AI-influenced decision auditable after the fact and drives
the ON/OFF A/B comparison and the "which module decided this" HUD.

---

## Consequences

### Positive

- The system genuinely *drives on a learned policy* for behaviour selection,
  while every hard safety and feasibility guarantee from ADR-001 / Phase 8 /
  the Safety Shield is retained and test-locked.
- The learned component is small, CPU-real-time, deterministic, explainable
  (SHAP), and fully logged — it survives the same honesty bar as the rest of
  the project.
- Clear, measurable success criteria (below) rather than "we added a model".
- Reversible: delete the `policy_prior` wiring and set `w_policy = 0` and the
  system is exactly today's.

### Negative / costs

- New training pipeline, data-logging harness, model-versioning discipline, and
  a second thing that can regress. ~30–50 h to a defensible v0 with all six P0
  siblings satisfied (roughly: 8 h observation + logging harness, 6 h imitation
  dataset + v0 train/eval, 6 h planner integration + provenance, 6 h ON/OFF A/B
  harness, 8 h NGSIM labelling (depends on Phase 13), 6 h robustness eval +
  docs, buffer).
- NGSIM ingestion (Phase 13) is a soft dependency for the "exceeds the
  deterministic teacher" claim; v0 can ship and pass ON/OFF + robustness on
  imitation + counterfactual data alone, with NGSIM as a follow-up.
- Curved-road behaviour prediction currently has a known false-positive
  (see `OVERNIGHT_CHECKPOINT_2026-09-10.md` — phantom cut-in on curves); the
  observation includes intent-distribution features, so that bug should be
  fixed first or the policy will learn around a broken input.

### Risks

| Risk | Mitigation |
|---|---|
| Policy learns a shortcut that games the ranking target | Counterfactual outcome scoring (not pure imitation); robustness eval on held-out seeds; provenance review. |
| Policy degrades on out-of-distribution scenes | `W_POLICY` clamp bounds worst-case influence; fail-safe to deterministic; explicit unseen-scenario gate below. |
| "Determinism when OFF" silently breaks | Regression test: planner output byte-identical with policy disabled, run in CI. |
| Model version drift between train and serve | `model_sha256` in every provenance record; serve refuses to load a model whose hash isn't registered. |
| Scope creep toward end-to-end control | This ADR's contract table is the boundary; changing it needs a new ADR. |

---

## Success criteria (map to the P0 siblings)

- **Policy–Planner Integration** — with the policy ON, `plan()` still only ever
  returns a feasible candidate; Safety-Shield veto rate does **not** increase
  vs OFF on the scenario suite.
- **AI Policy Provenance** — every tick the policy runs emits a complete
  `AIPolicyDecision`; a test asserts all fields are populated and the
  `model_sha256` matches the loaded model.
- **End-to-End AI Closed Loop** — a test drives Perception → Prediction →
  Policy → Planner → Safety → Controller → Physics for N ticks with the policy
  ON and asserts the loop is causal (policy output demonstrably changes the
  chosen candidate on ≥ some fraction of ticks) and safe (zero at-fault
  collisions, no envelope breach).
- **AI ON/OFF Behavioural A/B** — same seeds, policy ON vs OFF: measurable
  behavioural difference (e.g. mean lane-change commit time, mean gap at
  acceptance, comfort cost) **with no regression** in collisions, min-TTC
  distribution, or progress.
- **Unseen-Scenario Robustness** — on held-out scenario seeds + traffic
  densities + a sensor-degradation setting (`sensor_noise_enabled`,
  `sensor_dropout_rate` from the scenario params) and, when available, held-out
  NGSIM segments: no collisions, bounded comfort regression, fail-safe never
  worse than deterministic.
- **Learned Driving Policy Architecture** — this document, accepted.

---

## Rollout (only after this ADR is Accepted)

| Step | Deliverable | Gate |
|---|---|---|
| B1 | This ADR — accepted | owner sign-off |
| B2 | `PolicyObservation` builder + logging harness; imitation dataset from the deterministic planner | dataset stats + observation unit tests |
| B3 | v0 ranker trained; `policy_prior` wiring in `plan()` behind `w_policy` / `DDS_POLICY_ENABLED` (default OFF); determinism-when-OFF regression test | suite green, OFF == today byte-for-byte |
| B4 | `AIPolicyDecision` provenance on the telemetry path | provenance completeness test |
| B5 | ON/OFF A/B harness + report; end-to-end closed-loop test | measurable diff, zero safety regression |
| B6 | Robustness eval on held-out seeds / densities / sensor degradation (+ NGSIM if Phase 13 done) | criteria above |

Until B1 is accepted, `DDS_POLICY_ENABLED` does not exist and no policy code is
written.

---

## Alternatives considered

- **End-to-end BC / RL control** — rejected: unverifiable against collision
  bounds, opaque, discards a working planner, contradicts ADR-001.
- **Pure imitation of the deterministic planner** — rejected as an end state:
  cannot exceed its teacher, adds opacity for no capability gain. Retained only
  as the *warm-start* for the ranker.
- **Learned cost-weight tuner** (a model that outputs the Phase-8 `w_*`
  weights) — simpler, but it can only scale existing terms uniformly across all
  candidates; it cannot express "prefer the left lane *here* because that
  specific truck is about to brake". Insufficient for the owner's intent.
- **Do nothing / call the deterministic planner "the AI"** — rejected: the
  owner has explicitly and repeatedly said this does not satisfy the goal.
