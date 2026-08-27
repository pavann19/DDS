# ADR-001: Restructure DDS into a World/Driver Autonomy Architecture

**Status:** Accepted — partially implemented (Phase 6.5 in progress; see *Implementation status* below)
**Date:** 2026-08-27
**Deciders:** Project owner (sole maintainer)
**Supersedes:** the implicit architecture that grew organically through Phases 1–6

---

## Context

DDS has real autonomy capability today: an exact Frenet frame, quintic lateral
candidate generation, IDM car-following, an independent Safety Shield, a
deterministic Scenario Engine, and (Phase 6) a 360° sensor rig with
multi-target Kalman tracking and a log-odds occupancy grid. 189 tests pass.

But that capability accreted phase by phase inside one class. Before Phases
7–13 (prediction, spatiotemporal planning, dynamic tire physics, semantic
maps, RSS safety, teleop, NGSIM replay) are built **on top of** the current
structure, four structural problems have to be fixed — because every one of
those phases makes them worse, and each is expensive to unwind later.

### Problem 1 — The learned model is in the wrong place, and it's in the control path

`PhysicsEngine.get_ml_features()` feeds the XGBoost classifier exactly this:

```
Altitude, CO2, Coolant, Litre per 100km(Instant), RPM,
RPM_Delta, CO2_Delta, Fuel_Rate_Delta
```

Every one of these is ego powertrain/emissions telemetry. **None of them
describe the world outside the vehicle** — no traffic, no lane geometry, no
obstacle, no road curvature. The model is structurally incapable of being a
driving policy, because it cannot observe the thing being driven through.

Yet it sits inside the control path (`physics_engine.py:622-627`):

```python
elif ai_decision == 'Accelerate':
    target_speed = min(base_target_speed + 15.0, 120.0)
elif ai_decision == 'Decelerate':
    target_speed = max(base_target_speed - 20.0, 0.0)
```

An eco-driving classifier reading engine diagnostics injects ±15–20 km/h of
target-speed perturbation into a safety-critical loop, and is then clamped
back down by curvature caps, the tracking-error cap, IDM, and the Safety
Shield. It is simultaneously **unable to help** (it can't see hazards) and
**able to hurt** (it can command acceleration for powertrain reasons while
the scene says otherwise).

This inverts the real-world arrangement. In production AV stacks, learned
models do **perception and prediction** — the parts that require interpreting
messy sensor data — while trajectory planning is a deterministic, inspectable
optimizer with hard feasibility constraints. DDS currently has it backwards:
classical perception/planning, with ML bolted onto the throttle.

### Problem 2 — The simulator and the autonomy stack are the same object

`PhysicsEngine` is both the **world** (integrates vehicle dynamics, owns
`TrafficModel`, holds ground-truth NPC state) and the **driver** (projects
Frenet, scores candidates, runs IDM, invokes the Safety Shield, decides
steering and acceleration).

Consequences that already bite:
- The perception boundary is a *convention*, not a structure. Nothing
  prevents the planner from reading ground-truth NPC state directly; only
  discipline and code comments do. Phase 6 had to re-assert this boundary by
  hand.
- The autonomy stack cannot be run against recorded data, a different world,
  or a different vehicle model — which Phase 13 (NGSIM replay) requires.
- The "driver" cannot be swapped or A/B'd, which Phase 11 (injected planner
  failure → Minimum Risk Maneuver) requires.
- There is no way to evaluate the driver open-loop.

### Problem 3 — Single-rate wall-clock execution

Everything runs in one 10 Hz `while` loop, stepped by wall-clock delta:

```python
dt = min(now - self.last_update_time, 0.5)
```

Two failures follow. First, **nondeterminism**: identical inputs produce
different trajectories depending on machine load, which contradicts the
Scenario Engine's own reproducibility guarantee. That is not theoretical —
`websockets.py` calls `scenario_engine.update(physics, 0.1)` with a hardcoded
0.1 s while `physics.update()` uses real elapsed time, so scripted milestones
keyed on tick count already drift against the physics they're describing.

Second, **10 Hz is too slow for control**. Real stacks run perception at
10–30 Hz and control at 50–100 Hz, because a controller correcting cross-track
error at 10 Hz is a controller with 100 ms of dead time. Phase 9 (Stanley
controller, ABS/ESP slip modulation) and Phase 12 (<100 ms manual takeover)
are not achievable at a single 10 Hz rate.

### Problem 4 — One god-payload to the frontend

Every tick serializes one JSON blob containing ego, all traffic, perception,
planner candidates, SHAP, anomaly, driver score, safety shield, scenario, and
now surround perception. Phase 6's occupancy grid is 160,000 cells; Phase 7
adds 30 agents × 30 predicted states each. This channel does not survive
Phases 7–13 unchanged.

---

## Decision

Restructure DDS into an explicit **World / Driver** architecture with typed
interfaces, a multi-rate deterministic executor, an independent safety
monitor, and a layered streaming protocol — and **relocate the learned model
out of the control path into perception/prediction**, where learned components
belong.

```
┌─ WORLD (ground truth) ─────────────────────────────────────┐
│  SimClock          authoritative fixed-step time           │
│  VehicleDynamics   bicycle now → Pacejka tire model (P9)   │
│  TrafficWorld      NPCs, pedestrians (P10)                 │
│  Environment       map, signals, surface friction (P9/P10) │
└──────────────┬─────────────────────────────────────────────┘
               │  SensorInterface — the ONLY legal boundary
               │  (returns observations, never ground truth)
               ↓
┌─ DRIVER (autonomy stack) ──────────────────────────────────┐
│  Perception   20 Hz   sensor rig → tracker → occupancy     │
│  Prediction   10 Hz   forecaster → intent → risk field     │
│  Behavior     10 Hz   FSM: keep / prepare / change / abort │
│  Planner      10 Hz   spatiotemporal (s,d,t) optimizer     │
│  Controller   50 Hz   Stanley/MPC tracks planned trajectory│
│                                                            │
│  ┌ SafetyMonitor  50 Hz ── INDEPENDENT ─────────────────┐  │
│  │ own sensor feed · RSS envelope · MRM · veto authority │  │
│  └──────────────────────────────────────────────────────┘  │
└──────────────┬─────────────────────────────────────────────┘
               │  ActuatorCommand(steer_rad, accel_mps2)
               ↓  back into World
```

Two rules make this real rather than cosmetic:

1. **The Driver may only read what `SensorInterface` returns.** It never
   receives a `TrafficModel` or an `NpcVehicle`. The boundary becomes a type
   signature, not a comment.
2. **The Controller tracks a trajectory, it does not decide one.** The
   Planner emits a timed trajectory at 10 Hz; the Controller consumes it at
   50 Hz. This is what decouples the rates.

---

## Options Considered

### Option A — Keep the monolith, keep adding phases

| Dimension | Assessment |
|---|---|
| Complexity | Low now, compounding later |
| Cost | Zero today; Phases 9/11/13 become very expensive or impossible |
| Scalability | Poor — every phase adds state to one class |
| Risk | High — perception boundary stays a convention |

**Pros:** No refactor. Phase 7 starts immediately.
**Cons:** Phase 9 needs a control rate the loop can't provide; Phase 11 needs
a swappable/failable driver; Phase 13 needs the stack to run against recorded
data. All three are blocked. The ML stays miswired in the safety path.

### Option B — Full message-bus node graph (ROS-style, out-of-process)

| Dimension | Assessment |
|---|---|
| Complexity | High |
| Cost | Weeks of infrastructure before one driving improvement |
| Scalability | Excellent |
| Risk | High — most of the work is plumbing, not autonomy |

**Pros:** Genuine industry-standard structure; true process isolation.
**Cons:** Enormous cost for a single-process simulator with one consumer. IPC
serialization at 50 Hz for occupancy grids introduces the very latency budget
Phase 6 fought for. Optimizes for a multi-team, multi-machine constraint this
project does not have.

### Option C — In-process typed pipeline with explicit interfaces *(chosen)*

| Dimension | Assessment |
|---|---|
| Complexity | Medium |
| Cost | ~20–30 h, mostly mechanical relocation of working code |
| Scalability | Good — matches every remaining phase's needs |
| Risk | Low — behavior-preserving, guarded by 189 existing tests |

**Pros:** Gets every structural property that matters (boundary enforcement,
multi-rate, determinism, recordability, swappable driver) without IPC cost.
Modules stay plain Python objects with typed inputs/outputs, so the existing
test suite keeps working and each stage becomes independently testable.
**Cons:** Not literally ROS — no process isolation, no out-of-the-box
introspection tooling. Discipline still required at module boundaries, though
now backed by type signatures instead of comments.

---

## Trade-off Analysis

**B vs C** is the real decision. Both give correct structure; B costs ~5× more
and buys process isolation this project cannot use. C is chosen because the
architectural properties come from the **interface contracts and the executor**,
not from the transport. If DDS ever needs true node isolation, C's typed
message classes are exactly what a bus would carry — C is a strict subset of B,
not a divergent path.

**The ML relocation is the contentious call.** Removing XGBoost from the
control path could read as deleting the project's original contribution. It
is the opposite: the model, its SHAP explainability, its calibration analysis,
and its OOD robustness evaluation all remain — and become *more* defensible
once the model is scored on a task it can actually perform (driver-behavior
and eco-efficiency analytics from powertrain telemetry) rather than one it
cannot (deciding vehicle speed while blind to the road). A reviewer who asks
"how does a model reading coolant temperature decide when to brake?" currently
has no good answer. After this change, the answer is that it doesn't, and the
things that do are the perception, prediction, and planning stack.

**Multi-rate cost.** Running control at 50 Hz means 5× the controller
invocations per second. The Phase 6 budget work is what makes this affordable:
perception stays at 20 Hz (~1.8 ms/tick measured), and the 50 Hz controller is
a few hundred microseconds of arithmetic. Total remains well inside a 20 ms
real-time budget.

---

## Consequences

**What becomes easier**
- Phase 9's Stanley controller and ABS/ESP get the control rate they need.
- Phase 11's injected-failure → MRM test becomes trivial: swap the Driver.
- Phase 13's NGSIM replay becomes a different `World` behind the same
  `SensorInterface` — no autonomy changes at all.
- Deterministic replay: fixed-step sim time + recorded message streams means a
  bug reproduces exactly. This is a genuine capability *beyond* what an
  outside observer gets from a commercial stack.
- Every stage becomes independently unit-testable against synthetic inputs.

**What becomes harder**
- One refactor before Phase 7 delivers no new driving behavior. It must be
  strictly behavior-preserving, proven by the existing 189 tests.
- More files and explicit types; a small change now touches an interface.

**What we'll need to revisit**
- Whether `SafetyMonitor` eventually needs true process isolation (Option B)
  to claim genuine independence. In-process today, it is independent *by
  construction* (own sensor feed, own logic, veto-only authority) but shares a
  failure domain. That limitation gets stated honestly rather than papered over.
- The frontend protocol version bumps to v3 (layered channels).

---

## Honest positioning on "better than Waymo/Tesla"

This architecture is **structurally faithful** to how production AV stacks are
organized: hard sensor boundary, multi-rate execution, learned perception with
deterministic planning, independent safety supervision. That is a real and
defensible claim.

It is **not** and will not be better *at driving*. Those systems are validated
against billions of real-world miles on physical sensor hardware; DDS drives in
simulation against synthetic and recorded traffic. Claiming otherwise fails
this project's own audit discipline.

There is one axis where DDS can legitimately exceed them, and this architecture
is what unlocks it: **total auditability**. Every decision traceable to its
inputs, every run bit-exactly reproducible, every safety intervention explained,
the whole stack open and inspectable. Commercial stacks are black boxes to
everyone outside them. That is the claim worth making, and it is provable.

---

## Action Items

Sequenced so each step is independently verifiable and the suite stays green.

1. [x] **Define the contracts** — `app/services/interfaces.py`: `SensorObservation`,
   `PerceptionOutput`, `PredictionOutput`, `PlannedTrajectory`, `ActuatorCommand`,
   `SimClock`. Pure dataclasses, no logic.
2. [~] **Extract the World** — dynamics integration (`world/vehicle_dynamics.py`:
   `step_powertrain`, `advance_position`) moved out verbatim; `PhysicsEngine`
   delegates. `TrafficModel` ownership left in place (hybrid decision — see below).
3. [~] **Extract the Driver** — lateral planning + pure-pursuit tracking moved to
   `driver/lateral_planner.py`, reading only the scalar subset of a
   `SensorObservation` (no `TrafficModel`/NPC list). IDM composition and the
   no-route fallback still inline pending the full `SensorObservation` wiring.
4. [x] **Multi-rate executor** — `app/services/executor.py` (`MultiRateExecutor`,
   100 Hz base, deterministic rate dispatch, tested). `SimClock` wired into
   `PhysicsEngine` (fixed 20 ms substep grid) and into `websockets.py` as the
   single dt source for scenario + physics, removing the hardcoded-`0.1` desync.
   Wall-clock `dt` fallback **retained** (hybrid decision — existing `_tick()`
   tests drive it); determinism runs through the explicit-dt + seeded path.
5. [ ] **Relocate the ML** — *deferred to Phase 7.* Not behavior-preserving
   (removing the `ai_decision` speed modulation changes trajectories and breaks
   existing assertions), so it moves with the deep driver/scenario decouple.
6. [x] **Split the Safety Shield out** as a parallel `SafetyMonitor` node
   (`driver/safety_monitor.py`) with veto-only authority. Own sensor feed is
   structural prep only today (shared `sensed_lead`); real separation + RSS/MRM
   is Phase 11.
7. [ ] **Protocol v3** — *deferred to Phase 7.* A layered/delta-encoded channel
   split is not behavior-preserving (breaks the v2 payload shape the frontend
   and `test_websocket_smoke` depend on) and the heavy data it restructures for
   (per-agent predictions) does not exist until Phase 7.
8. [x] **Determinism test** — `tests/test_determinism.py`: same seed + same
   scenario ⇒ bit-identical ego trajectory (and `SafetyMonitor` verdict
   sequence) across two runs, on the explicit-dt path.

**Acceptance:** all 189 existing tests pass unchanged (behavior-preserving),
plus the new determinism test, plus `tsc --noEmit` clean. No new driving
capability is claimed from this ADR — it is the foundation Phases 7–13 are
built on.

## Implementation status (2026-08-27)

Branch `phase-6.5-world-driver`. Delivered: items 1, 4, 6, 8 in full; items 2
and 3 as a **hybrid extraction** — the math is moved into `world/` and
`driver/` as pure, independently-tested modules that `PhysicsEngine` calls,
but `PhysicsEngine` stays the public object and `scenario_engine` still
assigns/mutates `physics.traffic` directly. Rationale: the existing 189 tests
and `scenario_engine` reach deep into `PhysicsEngine` internals, so a full
type-signature boundary would require rewriting them, violating the
"189 unchanged" gate. The full decouple (and items 5 and 7, which are also not
behavior-preserving) is sequenced into Phase 7, which needs a swappable driver
regardless.

Gate status: **6.5.1** met (189 unchanged, +36 new unit tests, 225 total).
**6.5.2** met on the explicit-dt path (`test_determinism.py`). **6.5.3** —
`MultiRateExecutor` exists and is tested but does not yet drive `PhysicsEngine`
at granular rates (deferred with the deep split). **6.5.4** — enforced for the
extracted `driver/` modules (handed no `TrafficModel`/NPC list), not yet for
`PhysicsEngine` as a whole. **6.5.5** — `tsc --noEmit` still clean (no frontend
change); the protocol v3 migration itself is deferred.
