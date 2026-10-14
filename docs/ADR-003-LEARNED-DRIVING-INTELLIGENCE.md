# ADR-003: Learned Driving Intelligence — Invariants, not Architecture

**Status:** Proposed — awaiting project-owner review. **No model or training code
until accepted, and the architecture is deliberately NOT decided here.**
**Date:** 2026-09-10 (revised from an earlier prescriptive draft)
**Deciders:** Project owner (sole maintainer)
**Supersedes:** the earlier `ADR-003-LEARNED-DRIVING-POLICY.md` draft, which
pre-committed to a 64-feature gradient-boosted candidate re-ranker. Per the
next-phase instructions (§5, §7) that choice is now **one hypothesis to test,
not the design**.
**Related:** ADR-001 (World/Driver split; ML kept out of the control path),
Phase 8 (joint `(s, d, t)` lattice planner), Phase 13 (NGSIM replay), the P0
"Learned Driving Policy Architecture" item and its siblings (Policy–Planner
Integration, AI Policy Provenance, End-to-End AI Closed Loop, AI ON/OFF A/B,
Unseen-Scenario Robustness).

---

## Context

### What the owner wants

> "The system must drive on AI, not by ML models."
> "Right now there is no Artificial Intelligence making decisions driving — we
> need to implement this from now."
> A Tesla/Waymo-*inspired* original research implementation. Do **not** claim
> equivalence to proprietary production systems.

Today `tests/test_ai_attribution.py` proves the only ML in the repo (an XGBoost
eco-driving classifier over powertrain telemetry) has **no path into control** —
three different classifier outputs produce a bit-identical driven trajectory
(ADR-001 put it there on purpose). So "drives on AI" is not yet true in the
sense the owner means: no *learned* component influences *what the vehicle
decides to do*.

### Why this ADR does not name an architecture

The next-phase instructions are explicit: **do not pre-decide** model family,
network architecture (transformer / MLP / GNN), imitation vs reinforcement
learning, modular vs hierarchical vs end-to-end, raw-sensor vs learned
intermediate representation, or candidate-ranking vs trajectory-prediction vs
direct-policy output. Those are to be settled by the I1 investigation and by
experiments with measured results, not by this document.

What this ADR *does* is fix the **invariants** any learned driving intelligence
in DDS must satisfy, and define what counts as **evidence** that it is genuinely
driving. Everything inside those invariants is an open design space.

---

## Decision — the invariants (binding)

### I. Causal authority

The learned component **must materially change what the vehicle chooses to do.**
A model that is loaded but leaves behaviour unchanged, or that only feeds
analytics / SHAP / driver-score / dashboard metrics / offline classification,
**does not count** and does not close the P0.

*Test:* AI ON vs AI OFF on identical `(scenario, seed, route, traffic, physics,
safety config)` must produce a **measurable behavioural difference** (e.g. lane
choice, gap at acceptance, lane-change commit time, speed profile, cooperation)
on a meaningful fraction of situations. A model whose *output* changes while the
vehicle stays behaviourally identical fails this ADR.

### II. Deterministic physics & safety remain authoritative

Deterministic mathematics stays in charge of, and the learned component may
**never** weaken:

- vehicle physics / integration / actuator execution;
- geometric feasibility and the physical comfort/grip envelope;
- collision constraints and hard road/lane bounds;
- the Safety Shield (independent post-hoc veto — TTC, RSS-style margins, the
  hard proximity floor, boundary and lateral-accel limits);
- the deterministic fallback that runs when the learned component is
  unavailable.

The learned component's influence enters **above** feasibility and **below** the
safety monitor:

```
Learned intelligence
        v
behaviour / trajectory intent
        v
deterministic feasibility & physical envelope   (unchanged; filters/limits)
        v
Safety authority (Safety Shield / RSS / MRM)     (unchanged; can always veto)
        v
vehicle execution (physics)                      (unchanged)
```

The exact mechanism by which "behaviour intent" is expressed and consumed —
re-ranking the Phase-8 lattice, emitting a behaviour token the FSM/planner
consume, predicting agent trajectories that reshape the cost field, proposing a
reference line, or something else — is an **open choice for I4**, to be decided
by experiment. Whatever it is, it passes through the two boxes above unchanged.

### III. Determinism when OFF

With the learned component disabled (a flag; default OFF until this ADR is
accepted **and** the chosen architecture has passed its gates), the driving
stack is **byte-for-byte identical** to the pre-learned build. A CI regression
test asserts this.

### IV. Fail-safe

Any learned-component error, timeout beyond its inference budget, or
non-finite / out-of-contract output ⇒ its contribution is zero for that tick,
provenance records the failure, and driving continues on the deterministic
stack. The system is never *worse* than deterministic because the model failed.

### V. Reproducibility

Inference is deterministic for a fixed `(model version, observation)` — no
sampling at drive time, or a fixed seed if sampling is ever used. `(scenario,
seed, model sha256)` reproduces a run bit-for-bit (subject to the existing
`PhysicsEngine(seed=...)` contract).

### VI. Provenance — every AI-influenced decision is auditable

One record per tick the learned component runs, on the same telemetry/event
path the Safety Shield uses:

```
model_id, model_version, model_sha256
observation              (named, the exact input the model saw)
raw_inference_output      (scores / logits / predicted trajectories / token — whatever it is)
selected_behaviour        (what the vehicle actually did as a result)
deterministic_alternative (what OFF would have done, when different)
downstream_modification   (feasibility filter / cost change / clamp applied)
safety_intervention       (did the Shield veto? which override?)
latency_ms
status                    (ok | unavailable | clamped)
```

The serve path refuses to load a model whose sha256 is not registered.

### VII. Honest observation — no privileged-state shortcut

The learned component is trained and evaluated on what a **sensor-derived world
representation** actually exposes, not on privileged simulator ground truth.
The pipeline explicitly separates:

```
ground truth  (simulator internal)
sensor observation  (what the sensor model produces, with noise/dropout/occlusion/latency)
learned representation  (what the model builds from the observation)
```

Feeding ground-truth NPC state, exact future trajectories, or oracle intent
straight into the learned component is prohibited. (Using ground truth as a
*training label* or for *evaluation scoring* is fine; using it as a runtime
*input* is not.)

### VIII. Generalisation is the goal, not imitation

The objective is **not** "learn to imitate the current deterministic planner."
The deterministic planner may be a warm-start teacher or a baseline where
scientifically useful, but it is not the definition of good driving. The
learned intelligence must be evaluated on scenarios/seeds/traffic **not seen in
training**, including sensor degradation, and must show behaviour that
generalises beyond the fixed handcrafted rules.

---

## What is deliberately left open (decide by evidence, in I1–I4)

| Open question | Candidate answers to *evaluate*, not assume |
|---|---|
| What must be learned first? | world understanding / representation · agent-behaviour prediction · behaviour selection · trajectory preference · full policy — I1 decides the ordering |
| Observation encoding | hand-derived feature vector · rasterised BEV · agent-set (DeepSets/GNN) · sequence of past frames · learned latent from raw sensor streams |
| Temporal context | none (Markov) · fixed history window · recurrent state · attention over history |
| Model family | GBM ranker · MLP · CNN/BEV · set/graph encoder · small transformer — smallest that meets the gates, chosen by measured accuracy/latency, not fashion |
| Learning paradigm | imitation (BC) · inverse RL · offline RL · model-based RL · closed-loop RL in sim · hybrid — chosen by which actually improves the closed-loop metrics |
| Output / hand-off | re-rank Phase-8 lattice candidates · behaviour token to the FSM/planner · predicted agent trajectories reshaping the cost field · proposed reference line · direct (constrained) trajectory |
| Structure | modular (perception→prediction→policy) · hierarchical (behaviour then trajectory) · partially end-to-end — never fully end-to-end control (violates II) |
| Training data | sim self-play · counterfactual sim rollouts · imitation of the deterministic planner (warm start only) · real-world trajectory datasets (NGSIM, Phase 13) · combinations |

I1 produces an updated design section answering these **from the codebase and
from small experiments**, and only then does model/training implementation
begin (instruction §9, §17 step 9).

---

## Evaluation requirements (binding for every learned-driving milestone)

1. **AI provenance** — the record in invariant VI, complete, every tick.
2. **AI ON/OFF A/B** — same scenario, same seed, ON vs OFF; report the
   behavioural delta and confirm **no regression** in collisions, min-TTC
   distribution, comfort cost, or progress.
3. **Causal attribution** — demonstrate the learned output *is what changed*
   the selected behaviour (not a coincident deterministic change).
4. **Baseline comparison** — against the current deterministic stack, on the
   same metrics.
5. **Unseen scenarios** — held-out scenarios/seeds/traffic densities +
   sensor degradation (`sensor_noise_enabled`, `sensor_dropout_rate`), and
   held-out NGSIM segments when Phase 13 lands.
6. **Failure analysis** — when the model fails, localise it along
   perception → representation → prediction → policy → trajectory → safety →
   control → physics.

A milestone that does not clear its evaluation gates is rejected and the
approach revised — phases are not implemented just to tick a table (§12).

---

## Consequences

- **Positive:** the system can genuinely drive on learned intelligence with
  every hard physical/safety guarantee retained and test-locked; the design is
  driven by evidence rather than by a premature architecture bet; reversible
  (flag OFF ⇒ today's behaviour byte-for-byte).
- **Negative / cost:** a training/data/versioning pipeline, a second regressible
  subsystem, and real experimental work in I1–I4 before any model ships. Rough
  order-of-magnitude unchanged from the prior draft (~30–50 h to a defensible
  first learned component with all six evaluation gates), but the *shape* of
  that work is set by I1, not assumed now.
- **Risk:** scope creep toward end-to-end control (bounded by invariant II — a
  new ADR is required to change it); model gaming a proxy reward (bounded by
  closed-loop + unseen-scenario evaluation); silent OFF-path regression
  (bounded by the invariant-III CI test).

---

## Alternatives considered

- **Keep a prescribed architecture in the ADR** (the earlier draft) — rejected
  per §5: it pre-decides the very thing the investigation is meant to determine.
- **End-to-end learned control** — rejected: violates invariant II
  (unverifiable against collision bounds, discards a working planner,
  contradicts ADR-001).
- **Pure imitation of the deterministic planner as the end state** — rejected
  per invariant VIII: cannot exceed its teacher; allowed only as a warm start.
- **Do nothing / relabel the deterministic planner as "the AI"** — rejected:
  the owner has explicitly and repeatedly said this does not satisfy the goal
  (invariant I).

---

## Next

1. Owner reviews and accepts/revises these invariants.
2. **I1 — Driving Intelligence Foundation** (`docs/I1_DRIVING_INTELLIGENCE_FOUNDATION.md`,
   drafted alongside this): answers the 12 investigation questions from the
   actual codebase, then proposes the experimental path for I2 (learned world
   understanding) → I3 (learned prediction) → I4 (learned policy), each with
   its own success criteria.
3. Model/training implementation begins only after the architecture for a given
   milestone is justified by I1 findings + small experiments (§9).
