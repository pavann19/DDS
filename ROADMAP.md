# DDS Rebuild Roadmap

Source: an external ("Gemini") DDS V2 architecture proposal, reviewed
2026-08-11 and reordered by leverage-per-hour against this project's actual
purpose — a solo-built portfolio piece for German MSc applications, no hard
deadline, "Chrome v1 → v150" pacing, not a thesis defended against an
examiner. Kept because the layered architecture (perception → decision →
safety → planning → control → render) is the real canonical AV decomposition,
not because every phase or hour estimate in the original proposal survived
review — three specific things were cut outright (see "Explicitly deferred"
and the note under Phase 2), not silently dropped.

**Overlap note:** kinematic bicycle-model control, jerk-limited longitudinal
control, server-authoritative NPC traffic + forward range sensor, route
spline-smoothing, and a Frenet local planner + pure-pursuit lateral control
already exist and are NOT re-counted here even though the original proposal's
D11–D14 implied building them from scratch.

**Reframing note (2026-08-11):** the original proposal's flagship research
question — "does internal vehicle state improve driving decisions vs.
external/environmental state alone" (its ablation study, D18) — was checked
against the actual dataset and against every public trajectory/AV dataset
investigated (NGSIM, highD, comma2k19, nuScenes, Waymo) and found
**infeasible**: this project's 8 features are 100% combustion/emissions
telemetry (RPM, coolant, CO2, fuel rate — zero external/traffic features
exist in the source data), and no public dataset combines that class of
internal telemetry with rich external traffic context (nuScenes' CAN
expansion comes closest but is from a Renault Zoe, an EV, with no
RPM/coolant/fuel signals). The ablation is dropped as a goal. NGSIM survives
as a **traffic-realism** source instead — real recorded human highway
driving, public domain, no request required — which is a stronger portfolio
line than a synthetic-scenario library regardless.

---

## Phase 1 — Motion, sensing & planning foundation (6/6 done)
Original estimate: n/a — completed prior to this roadmap existing

- [x] Kinematic bicycle model + jerk-limited longitudinal control,
      replacing point-mass speed lerp — done 2026-07-20, see
      `_evidence/P6-1/SUMMARY.md`. Peak jerk 195 → 2.5 m/s³, peak
      acceleration 22.2 → 3.0 m/s². Legacy controller retained behind
      `PhysicsEngine(controller="legacy")` as the fixed A/B control —
      not deleted.
- [x] Server-side NPC traffic + forward range sensor — done 2026-07-20,
      see `_evidence/P6-1b/SUMMARY.md`. `traffic.py`'s sensor exposes gap
      + relative speed only, never NPC identity/position — a deliberate
      perception/control boundary, test-enforced.
- [x] Route spline-smoothing (centripetal Catmull-Rom + uniform
      arc-length resample) — done 2026-08-07, see
      `_evidence/P6-1d/SUMMARY.md`. Max deviation from the real OSRM
      polyline 1.12 m, length preserved to 0.01%.
- [x] Frontend: real `THREE.Raycaster` sensor visualisation against
      backend-authoritative NPCs and road-edge geometry (replaced a
      cosmetic distance formula and a client-invented NPC pool) — done
      2026-08-10, see `_evidence/P6-1c/SUMMARY.md`.
- [x] Frenet local planner + pure-pursuit lateral control (exact
      station/lateral projection, candidate lateral-offset scoring,
      Coulter-1992 steering geometry) — done 2026-08-11, see
      `_evidence/P6-2/SUMMARY.md`. Car now targets a lane centre, not the
      raw route centreline; frontend's hard-coded `LANE_OFFSET_M` render
      hack removed in favour of the real backend value.
- [x] Backend test suite as a going concern (132 tests as of Phase 1's
      close, all passing) — accumulated across the above, not a
      standalone task.

## Phase 2 — Evaluation rigor: correct the ML headline before building on it (1/3 done)
Original estimate: n/a

- [x] Baselines: majority-class + logistic-regression, evaluated beside
      XGBoost on one identical split with 95% Wilson intervals — done
      2026-08-11, see `_evidence/P5-2/SUMMARY.md`. **Changed what this
      project may honestly claim.** A plain linear model is MORE accurate
      than the deployed XGBoost (0.8667 vs 0.8500); XGBoost's accuracy
      gain over the majority-class floor is NOT statistically significant
      at n=180 (CI [0.791, 0.895] contains the floor's 0.7944). Its real,
      significant contribution is minority-class recall — non-overlapping
      95% CIs vs. the floor on both Accelerate and Decelerate, which
      accuracy alone hides. Reproduce: `python baselines.py`.
- [ ] Control-authority experiment: the classifier's entire influence on
      the car is currently a ±15/−20 km/h nudge on a hardcoded 50 km/h
      cruise (`physics_engine.py`'s `ai_decision` branch). Measure what
      changes, if anything, when the classifier is given real command
      authority — hypothesis going in: it likely can't drive meaningfully
      on its own, because its 8 features carry zero information about
      road, traffic, or destination (see the reframing note above). A
      clean negative result here is a legitimate, useful finding, not a
      failure — and it is the direct, honest motivation for Phase 8/9's
      vision and RL work rather than an assumed one.
- [ ] P5-9 (larger/second dataset): contingent on the control-authority
      result above — if the classifier's ceiling is capped by having no
      external features at all, a bigger combustion-telemetry-only
      dataset does not fix that, and the honest move is to say so rather
      than scale rows for their own sake.

## Phase 3 — Repository & claims integrity (0/6 done)
Original estimate: ~4–8h

An evidence-first repo audit was run this session (checkpoint-graded per
capability: MEASURED / IMPLEMENTED / SCAFFOLDED / PLANNED). Its findings
aren't yet committed anywhere — closing that gap is the first item below,
consistent with this project's own stated discipline (`AUDIT_PROTOCOL.md`:
no claim without a reproducible, committed artifact). The two most
consequential findings:

- [ ] **Commit the audit write-up itself** as `_evidence/AUDIT-2026-08-11/`
      (or similar) — the findings below currently exist only as a chat
      transcript, which is exactly the "not a description of what the
      implementing agent claims it did" gap `_evidence/README.md` warns
      against for every other task in this repo.
- [ ] **Fix the `genetic_optimizer.py` naming mismatch.** The file
      implements exhaustive search over all 2^N feature subsets, NOT a
      genetic algorithm — its own docstring says so and explains why GA
      was rejected for this problem size (2^N ≤ 256 subsets is small
      enough that brute force is provably optimal and deterministic; a GA
      is not guaranteed to find the global optimum here). The filename
      and any "DDS/GA" framing in surrounding docs still implies
      otherwise. Rename the file (e.g. `feature_selection.py`), update
      every reference, and audit `THESIS_PLAN.md`/`STATE.md` for
      "genetic algorithm" language that needs the same correction. This
      is the single most likely false claim a reader would form from this
      repo's naming, and it is directly falsifiable by opening one file.
- [ ] **State authorship plainly, in the README, not just in evidence-doc
      fine print.** `git log` shows a single-author, 9-commit history —
      but two of those commits were authored by the AI agent itself, in
      one session, as large squashed checkpoints, which is exactly the
      "rebuilt repo, single-author history is not evidence of sole
      authorship" pattern worth naming explicitly rather than leaving
      implicit. The honest framing: architecture direction, review, and
      verification are the user's; a large share of the code text was
      written by an AI coding agent under that direction across many
      sessions. A CV bullet that says "built"/"implemented" without that
      qualification claims something this repo's own documentation
      already contradicts.
- [ ] **Add at least one automated frontend test.** Zero exist today —
      no `.test.tsx`/`.spec.tsx` files, no test script in
      `frontend/package.json`. `tsc --noEmit` passing is a type check, not
      a behaviour test. One Playwright/Vitest smoke test (boot the dev
      server, mount `DriveScene`, assert the canvas renders with zero
      console errors) would move the entire frontend from SCAFFOLDED to
      IMPLEMENTED grade honestly.
- [ ] **Re-commit a live-verification screenshot as a real artifact.**
      The screenshots that were the only evidence for P6-1c/P6-2's
      "live-verified in the browser" claims were deleted in a later
      repo-hygiene pass. Nothing currently in the repo substitutes for
      them — the claim is currently prose describing a past session, not
      something the repo itself proves.
- [ ] **Refresh the stale 80% coverage figure** (`_evidence/P1-6/`,
      measured at 37 tests) or stop citing it. The suite is now 132 tests
      across 4 modules (`frenet.py`, `planner.py`, `traffic.py`,
      `path_smoothing.py`) that didn't exist at measurement time.

## Phase 4 — Safety Shield (0/4 done)
Original estimate: ~8–12h — highest credibility-per-hour item in the whole
external proposal; no new data or infrastructure required, sits directly on
Phase 1's planner output

- [ ] Independent TTC / collision-risk computation, run AFTER the planner
      picks a candidate, not as part of its cost function — the entire
      point is a second, independently-reasoned check, not the same
      logic asked twice.
- [ ] Road-boundary and vehicle-dynamics feasibility check on the chosen
      trajectory (does it exceed `A_LAT_MAX_MPS2`, does it exit the
      modelled road half-width).
- [ ] Override mechanism: when the shield rejects the planner's choice,
      substitute a safe fallback (e.g. hold lane, brake) and surface
      **both** decisions distinctly in `get_navigation_state()` — "AI
      decision → safety validation → final action" as three visibly
      different fields, not one collapsed outcome.
- [ ] HMI: a visible shield panel showing approved vs. overridden, with
      the reason (e.g. "TTC 1.2s, unsafe → overridden to brake"). This is
      the single most demoable addition available — a screen recording of
      a real override is a stronger portfolio artifact than any
      accuracy table.

## Phase 5 — Scenario Engine (0/4 done)
Original estimate: ~10–15h — unlocks interactive demoing and every
downstream evaluation

- [ ] Deterministic scenario definitions: normal (straight/curve/light
      traffic), traffic (slow lead, dense traffic, cut-in), maneuver
      (turn, merge), safety-critical (sudden braking, blocked lane) —
      fixed seeds, reproducible.
- [ ] Scenario control surface: select scenario / traffic density /
      initial speed; start / pause / reset.
- [ ] Frontend control room UI for the above (this is what turns the demo
      from "watch it drive" into "pick a scenario and watch it handle
      it").
- [ ] At least 3 scenarios demoable end-to-end with the Safety Shield
      (Phase 4) visibly engaging in at least one of them.

## Phase 6 — Real traffic data (NGSIM) (1/3 done)
Original estimate: ~10–20h — depends on final feature/format needs once
started

- [x] Feasibility/licensing check — done 2026-08-11 (in-session, not yet
      a committed artifact — folding the write-up into Phase 3's audit
      commit is the cleanest fix). NGSIM: public domain, direct FHWA
      download, no request required, 10Hz lane-level trajectories,
      US-101/I-80/Lankershim/Peachtree. highD is richer (110,500
      vehicles, <10cm accuracy, German highways) but needs a non-commercial
      access request — worth starting in parallel given the German-MSc
      framing, but NGSIM is the unblocked starting point.
- [ ] Replay pipeline: parse NGSIM trajectories into the same NPC state
      shape `traffic.py`'s `TrafficModel` already produces, so the
      existing sensor/rendering pipeline consumes real recorded driving
      without caring where it came from.
- [ ] Evidence: a committed side-by-side comparison (synthetic
      seeded-random NPCs vs. NGSIM-replayed NPCs) — "traffic isn't
      scripted, it's real recorded highway driving from the US Federal
      Highway Administration" is the target claim, and it needs a
      reproducible artifact behind it like everything else in this repo.

## Phase 7 — Complete the P6 control stack (0/4 done)
Original estimate: n/a (already scoped in `PHASE_6_TASK_BOARD.md`,
carried forward here for one combined view)

- [ ] P6-3 — IDM car-following for the ego, driven by P6-1b's sensed
      gap/speed (never raw NPC state).
- [ ] P6-4 — NPC-to-NPC IDM car-following, so traffic queues realistically
      instead of driving independently; MOBIL lane-changing only if
      stable, flagged as stretch within this item.
- [ ] P6-5 — HMI: render candidate trajectories (dimmed) + the selected
      one (highlighted), tracked NPCs, and the currently-binding
      constraint (e.g. "lateral-accel limited", "following lead
      vehicle") — natural pairing with Phase 4's shield panel.
- [ ] P6-6 — A/B evaluation of the new stack vs. the legacy controller,
      on ONE consistent perpendicular-projection cross-track metric.
      **Flagged repeatedly and still owed:** P6-1's 3.18/5.80m,
      P6-1d's 3.30m, and P6-2's 0.63/1.42m are FOUR differently-defined,
      mutually incomparable numbers. `frenet.py`'s `project_to_frenet`
      is now the consistent tool to finally do this once, on a fixed
      real route set, before any A/B claim is made anywhere else.

## Phase 8 — Production hardening (continuous, not a final phase)
Original estimate: ongoing

Same discipline used throughout this project so far — real benchmarks
before/after, a committed evidence folder per task, honest "what this does
not close" notes — applied continuously rather than as a step done once at
the end.

- [ ] Extend backend CI to the frontend (once Phase 3's frontend test
      exists, run it in `.github/workflows/`).
- [ ] Re-run and re-date the coverage report after each phase closes,
      not just once (Phase 3 flags the current figure as stale; this
      item is what stops it going stale again).
- [ ] Keep `STATE.md`/this roadmap's checkboxes in sync with
      `_evidence/` — a checked box without an evidence folder is not
      done, per this project's own long-standing rule.

---

## Explicitly deferred

Not started, not scheduled — revisit only once Phases 3–7 are solid.
Ordered roughly by how directly each one answers the "no real ML/CV/AI
work" critique that opened this reprioritisation, since that's the
variable most likely to change when to pull one forward:

- **P6-7 — Vision/CV**: render the existing Three.js camera view, run a
  real pretrained detector on it, feed real detections into the planner
  alongside (not instead of) the range sensor. The most direct answer to
  "there's no real CV in this project" — currently the repo has zero CV
  dependencies (confirmed: no opencv/torch/detector in `requirements.txt`).
- **P5-3 — RL policy vs. classifier (MetaDrive)**: environment feasibility
  is already confirmed (`_evidence/P5-3a/`), but nothing from that work is
  committed to this repo (no script, no artifact — `venv_metadrive/` is
  gitignored). Wiring an actual state/action/reward mapping and running a
  real A/B is unstarted. Contingent on Phase 2's control-authority result.
- **Original-DDS-baseline reproduction** (the historical 73%/67%/48%
  DDS-GA/RF/MLP numbers): dropped, not deferred-in-name-only. The measured
  majority-class baseline on this dataset is 79.4% — all three historical
  numbers are below a model that ignores its inputs entirely, so
  reproducing them would not be an informative comparison even if the
  original preprocessing were fully recoverable.
- **Hardware sensor adapters / three operating modes** (simulation /
  dataset replay / real hardware) from the original proposal's D23. No
  hardware exists to adapt to; premature without it.
- **Full sensor abstraction layer** (radar/LiDAR/IMU/GPS simulators)
  beyond what Phases 4–7 actually need. Build the specific sensor a
  specific phase requires, not a general abstraction speculatively.
- **Dataset scaling to 100k+ rows.** Sample count was never the
  constraint per the reframing note above — the dataset has zero external
  features regardless of row count. More rows of the same 8 combustion
  signals do not add information the classifier is missing.
- **Full developer-facing digital-twin UI** (request inspector, trace
  viewer, benchmark dashboard). Valuable eventually; the general-user demo
  surface (Phases 4–6) matters more for a portfolio audience first.
- **Multi-objective GA-style fitness weighting** for driving quality
  (comfort/efficiency/safety/stability combined). Revisit only if Phase 7
  produces enough real controller variants to make weighting them a real
  question rather than a guess.
