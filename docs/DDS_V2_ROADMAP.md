# DDS V2 Roadmap — Real Self-Driving, Waymo-Grade Presentation

Source: not an external proposal this time — this is the user's own stated
goal, given directly in-session (2026-08-25): *"I just want to see the real
self-driving by the car itself with smooth UI, path planning and other
things that I've seen in Waymo."* Scoped against this project's actual
codebase state by the implementing agent, not designed speculatively ahead
of what already exists. Supersedes `ROADMAP.md`'s Phase 7 (P6 control-stack
completion) as the more precise statement of what "done" means for that
work — the phases below fold it in rather than duplicate it.

**What "Waymo-grade" is being scoped to mean, concretely** (so this doesn't
become an unbounded aesthetic target): visible path-planning reasoning (the
rider-app signature — dimmed candidate paths + a highlighted chosen one,
not just a car that moves), a demo that runs smoothly with no visible jank
or fabricated-looking elements, and traffic that reacts like traffic. It
does NOT mean camera perception, LiDAR point clouds, or a learned driving
policy — those are real, separate, already-deferred scope (see the bottom
of this document), not something a UI/planning pass can produce.

**Overlap note:** as of this document, the following are already real and
working, verified live this session — NOT re-counted as open work below
even though they're exactly the kind of thing a "make it look like Waymo"
ask would otherwise re-request:
- Kinematic bicycle model + jerk-limited longitudinal control (`app/services/physics_engine.py`)
- Frenet local planner with a REAL lane-change candidate, gated by a real
  adjacent-lane-clear sensor check (`app/services/planner.py`, `traffic.py::sense_lane_clear`)
- IDM car-following for the ego (`app/services/car_following.py`) AND
  NPC-to-NPC (`traffic.py::TrafficModel.update`) — traffic queues realistically now
- An independent Safety Shield (TTC, road-boundary, hard-lateral-accel
  checks) that can override the planner (`app/services/safety_shield.py`)
- Real-world-space road-edge rendering — the 3D scene now shows the route's
  actual curves instead of a generic straight corridor
  (`frontend/src/lib/routeGeometry.ts`, `SimulationScene.tsx`)
- SHAP explainability, anomaly detection, and driver score streamed live to
  the UI (previously computed and silently discarded server-side)
- A Tesla/Waymo-styled mode-based UI (Drive / Developer / Research) with a
  command palette, destination picker, and live safety/explainability panels

---

## Phase 1 — Driving intelligence foundation (done, this session, 2026-08-25)
Not a roadmap item in the usual sense — recorded here because everything
in Phase 3+ below assumes it exists and works. Full detail already in the
session transcript; formal `_evidence/` writeups are Phase 2's job below,
not skipped, just not done yet.

- [x] Forward-sensor lane bug fixed — `sense_lead_vehicle` was checking a
      **hardcoded** lane position instead of the ego's real lateral
      offset, so once the car legitimately drifted lanes (cornering, a
      real lane change) it stopped detecting traffic actually in front of
      it. One-line fix, `app/services/physics_engine.py`.
- [x] Startup tracking-error speed cap — the car's fixed starting heading
      is often badly mismatched to the real route's initial direction;
      flooring the throttle before steering authority caught up sent it
      up to 26m off a 7m-wide road on the real SF OSRM route. Capped
      speed by realised steering demand (`TRACKING_ERROR_STEER_FRACTION_THRESHOLD`),
      cut the worst case to under 10m. Regression test:
      `tests/test_physics_engine.py::test_tracking_error_speed_cap_bounds_startup_excursion_off_a_mismatched_route`.
- [x] Safety Shield built — three independent checks (TTC, road boundary,
      hard lateral-accel limit), evaluated AFTER the planner/IDM decide,
      can override toward safety via `min()` composition. Caught its own
      bug live during testing: an early version braked to a full stop on
      a road-boundary violation, which is a livelock (a stopped car
      cannot generate yaw rate to steer back onto the road). Fixed with a
      `RECOVER_LOW_SPEED` override distinct from `EMERGENCY_BRAKE`.
      `app/services/safety_shield.py`, regression test:
      `test_safety_shield_road_boundary_override_does_not_livelock_the_car`.
- [x] Real road-edge world-space rendering — `frontend/src/lib/routeGeometry.ts`
      (new) ports the miter-limited road-ribbon technique from the
      pre-rewrite frontend, and `getWorldPosAtFrenet(s, d)` converts the
      backend's real Frenet coordinates into a genuine point on the real
      curved road, replacing the previous Frenet-space-as-world-space
      passthrough that rendered every route as a straight corridor.
      Live-verified: screenshots show the road visibly curving through
      the real route, "CORNERING" status firing exactly on a real turn.
- [x] SHAP/anomaly/driver-score/planner-candidates restored to the live
      WS payload — these were computed every tick and only ever logged to
      SQLite, never sent to any connected client. `app/api/websockets.py`.

158 backend tests passing at the close of this phase (`tests/`), `tsc --noEmit` clean.

---

## Phase 2 — Land this session's work properly
Original estimate: ~2-4h

Everything in Phase 1 is real, tested, and live-verified — but sitting
uncommitted, with no `_evidence/` folder, which is this project's own
established bar for "done" (see `_evidence/README.md`: *"a task without a
folder here has not cleared Gate 4, regardless of what its diff looks
like"*). Doing this before more feature work stacks on top of it, not after.

- [ ] Commit the backend fixes (sensor lane bug, tracking-error cap,
      IDM ego + NPC-to-NPC, real lane-change + `sense_lane_clear`, Safety
      Shield + livelock fix) as their own logical commit(s).
- [ ] Commit the frontend rewire (protocol/store/worker fixes, SHAP/
      anomaly/driver-score/shield panels, destination input, real road
      geometry) separately from the backend commit.
- [ ] Write `_evidence/P6-3/`, `_evidence/P6-4/` (IDM ego + NPC, matching
      the task board's existing P6-3/P6-4 IDs), and a new evidence folder
      for the Safety Shield + road-geometry work (no existing task ID
      covers either — name them accordingly, e.g. `_evidence/SAFETY-SHIELD/`,
      `_evidence/ROAD-GEOMETRY/`).
- [ ] Update `PHASE_6_TASK_BOARD.md` (mark P6-3/P6-4 done) and `STATE.md`
      (append a dated session entry, matching every prior session's
      pattern) so a future session doesn't have to reconstruct this from
      chat history.
- [ ] Mark this document's Phase 1 checkboxes `[x]` → cite the real
      evidence paths once they exist (they're prose-only right now,
      exactly the gap `ROADMAP.md`'s own Phase 3 flagged as a pattern to
      avoid repeating).

---

## Phase 3 — The Waymo-signature visual: make planning visible
Original estimate: ~10-15h — the highest-leverage item for "looks like
Waymo," because the data already exists and is already streamed; this is
almost entirely a rendering task, not a new capability.

The planner already scores multiple candidate paths every tick
(`app/services/planner.py::generate_candidates`, streamed as
`data.planner.candidates` — confirmed live: real candidate sets with
`is_chosen`/`is_lane_change` flags are already reaching the browser). None
of it is drawn in the 3D scene. This is the single biggest gap between
what DDS already computes and what it currently *shows*.

- [x] Render every candidate lateral offset as a real, dimmed path segment
      in `SimulationScene.tsx` (short corridor at each `d_target`, using
      the same `getWorldPosAtFrenet` lookup the ego/traffic already use),
      not just the text list currently in `DeveloperMode`.
- [x] Render the CHOSEN candidate distinctly (brighter, thicker, glowing)
      — visually distinguish it from the dimmed alternatives, the exact
      rider-app pattern this phase is named for.
- [x] Shield override made visible in the 3D scene itself: a pulsing ring
      indicator on the ego vehicle when `RECOVER_LOW_SPEED`/
      `EMERGENCY_BRAKE` is active, distinct from a normal decision.
      (Tying the "Decision Intent" panel's copy to the specific
      lane-change reason string is deferred — the underlying real fields
      are already in the payload and consumed by `ShieldPanel`/
      `DriveMode`'s `deriveStatus`, but a dedicated reasoning sentence in
      the intent panel wasn't built as part of this pass.)
- [x] Predicted NPC paths — a constant-velocity projection drawn ahead of
      each visible NPC (same station/lane-offset math, extrapolated
      forward via `sampleFrenetCorridor`), rendered dashed and dimmed to
      read as a projection, not a claim of real trajectory prediction.

**Acceptance.** Live-verified in-browser: dimmed alternative candidate
paths render alongside a highlighted chosen path, predicted NPC paths
render dashed ahead of traffic, `tsc --noEmit` is clean, and the full
backend suite (159/159) still passes. Screenshots taken during this
session's live verification pass (not committed as repo assets).

---

## Phase 4 — Smoothness pass (kill remaining jank + fabricated elements)
Original estimate: ~6-10h

"Smooth" is a real, checkable property, not a vibe — each item below is a
specific rendering behaviour to fix, verified by watching it, not assumed.

- [x] Camera smoothing tuning through turns — live-verified: watched a real
      cornering sequence (17-21km/h, "Speed capped by lateral-acceleration
      limit") frame-by-frame; the chase camera's existing exponential
      lerp (already tracking the road's real forward tangent, not a fixed
      world-Z offset) showed no jitter or overshoot. No code change was
      needed here — the smoothing added when the scene moved to
      world-space rendering (Phase 1/3) was already sufficient.
- [x] NPC recycling pop-in — `traffic.py`'s `VISIBILITY_WINDOW_M` recycling
      relocates an NPC to a new station in a single backend tick; naively
      lerping toward that made the car slide across the map at high speed
      rather than pop, which read as a worse glitch. Fixed in
      `NpcVehicle`: a single-frame position jump past a real-car-motion
      threshold snaps instantly instead of sliding, then fades the
      vehicle's materials back in via a real opacity ramp
      (`NPC_FADE_IN_DURATION_S`).
- [x] `LidarRadarSweep`'s pulsing ring was pure decoration with no
      connection to `SENSOR_MAX_RANGE_M` or any real sensor event. Fixed:
      the ring's radius now scales to the real 100m forward-sensor range,
      and it colour-codes and flashes (same severity thresholds as the
      NPC bounding boxes) specifically when its expanding radius reaches
      the real `sensed_lead_vehicle` detection distance, instead of
      pulsing regardless of whether anything was ever detected.
- [x] Startup correction visual smoothness — live-verified: watched from
      a fresh drive start, lateral offset stayed within ~1.7-2.2m
      throughout (the Phase 1 tracking-error cap holding well under its
      ~10-20m worst-case bound in this run), speed recovered smoothly to
      cruise with no visible fighting-the-road artifact. No code change
      needed.

**Acceptance.** Live-verified in-browser across a fresh drive start and a
real cornering sequence: camera tracking was smooth with no jitter,
lateral offset stayed bounded through the startup correction, the radar
sweep now reflects the real sensor range and real detections, and
`tsc --noEmit` is clean with the full backend suite (159/159) still
passing (frontend-only change).

---

## Phase 5 — Scenario Engine
Original estimate: ~10-15h — carried forward from `ROADMAP.md`'s Phase 5,
unchanged; still not started. Turns the demo from "watch it drive" into
"pick a scenario and watch it handle it" — the other half of a real
Waymo-style demo (their own demo reels are curated scenarios, not raw
unscripted drives).

- [x] Deterministic scenario definitions (normal, traffic, maneuver,
      safety-critical), fixed seeds (`app/services/scenario_engine.py`:
      `normal_cruising` [seed 42], `traffic_overtake` [seed 101],
      `emergency_cut_in` [seed 202], `queue_stop_and_go` [seed 303]).
- [x] Scenario control surface: select scenario / traffic density / initial
      speed; start / pause / resume / reset / step (`ScenarioEngine` + WebSocket
      command protocol + REST `GET /api/scenarios`).
- [x] Frontend control room UI for the above (`ScenarioControlRoom.tsx`,
      integrated into `DriveMode.tsx`, `DeveloperMode.tsx`, `CommandPalette.tsx`).
- [x] At least 3 scenarios demoable end-to-end with the Safety Shield
      (Phase 1) visibly engaging in at least one of them (`emergency_cut_in`
      triggering TTC override `OVERRIDE_EMERGENCY_BRAKE`), and Phase 3's
      candidate-path visualization visibly showing a lane-change decision
      in another (`traffic_overtake` triggering `is_lane_change=True`).
      All 9 automated scenario unit/integration tests and 168/168 suite tests passing.

---

## Phase 6 — Real traffic data (NGSIM)
Original estimate: ~10-20h — carried forward from `ROADMAP.md`'s Phase 6.
Feasibility already confirmed: NGSIM is public domain, direct FHWA
download, no request required, real 10Hz lane-level highway trajectories.

- [ ] Replay pipeline: parse NGSIM trajectories into the same NPC state
      shape `traffic.py`'s `TrafficModel` already produces.
- [ ] Evidence: a committed side-by-side comparison (synthetic
      seeded-random NPCs vs. NGSIM-replayed NPCs).

---

## Phase 7 — Evaluation rigor for the new control stack (P6-6)
Original estimate: ~8-12h — carried forward, still owed. `ROADMAP.md` has
flagged this repeatedly: P6-1's 3.18/5.80m, P6-1d's 3.30m, and P6-2's
0.63/1.42m are four differently-defined, mutually incomparable cross-track
numbers. `app/services/frenet.py::project_to_frenet` is the tool to finally
do this once, consistently, on a fixed real route set, before any
before/after claim about this phase's work goes anywhere public-facing
(a portfolio README, an application, etc).

- [ ] One consistent perpendicular-projection cross-track metric, measured
      for legacy vs. the current bicycle+Frenet+IDM+shield stack, on the
      same fixed real route(s).
- [ ] Report collision rate, near-miss rate (Safety Shield engagement
      count), and lane-change success rate over N scripted runs — the
      closed-loop metrics that actually matter for "does this look like
      it's driving well," not just cross-track error in isolation.

---

## Explicitly deferred

Unchanged from `ROADMAP.md`, reaffirmed here so this document doesn't
silently imply any of these are now in scope just because "Waymo" was
said out loud:

- **Vision/CV (P6-7)** — rendering the Three.js camera view and running a
  real detector on it. Zero CV dependencies exist in this repo today
  (confirmed: no opencv/torch/detector in `requirements.txt`). Real,
  substantial, separate work — not a byproduct of a UI polish pass.
- **RL / learned driving policy (P5-3)** — MetaDrive environment
  feasibility is confirmed (`_evidence/P5-3a/`) but nothing from that work
  is wired into this repo. If "real self-driving" is ever meant as *"a
  neural network that learned to drive,"* as opposed to classical
  planning/control (Frenet + pure pursuit + IDM, all of which this
  project already has and are real, citable, standard AV-literature
  methods) — that specific claim requires this phase, not Phases 3-4
  above.
- **Original-DDS-baseline reproduction** — dropped, not deferred-in-name-
  only (see `ROADMAP.md`'s original reasoning: the measured majority-class
  baseline already beats all three historical numbers).
- Hardware sensor adapters, full sensor abstraction layer, dataset scaling
  to 100k+ rows, full developer-facing digital-twin UI (trace viewer,
  benchmark dashboard) — all as previously scoped in `ROADMAP.md`.
