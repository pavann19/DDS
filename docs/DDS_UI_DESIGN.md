# ADR-002: Restructure the DDS HMI into one honest AV-operator console

**Status:** Proposed
**Date:** 2026-08-28
**Deciders:** Project owner (sole maintainer)
**Supersedes:** the three-mode HMI that grew organically through Phases 2–7
**Related:** ADR-001 (`docs/DDS_ARCHITECTURE.md`) did this for the backend

---

## Context

The backend is now a structurally faithful AV stack: hard sensor boundary,
multi-rate executor, learned perception/analytics with deterministic
planning, independent safety supervision, multi-agent prediction. The
frontend has not kept pace. Like the backend before ADR-001, the HMI
accreted screen by screen, and four problems now compound with every phase.

### Problem 1 — Two visual languages, two HUDs, two component trees

The same information is rendered twice, differently:

| Concern | "Drive" implementation | "Developer" implementation |
|---|---|---|
| HUD chrome | `bg-black/40 backdrop-blur-xl border-white/10 rounded-2xl` (ad-hoc glass, hard-coded) | `bg-[var(--bg-panel)] border-[var(--border-default)] rounded-lg` (design tokens) |
| Speed / decision | `components/modes/DriveMode.tsx` **and** `app/components/DriveHUD.tsx` — two overlapping overlays | `components/modes/DeveloperMode.tsx` inline grid |
| Safety Shield | `panels/ShieldPanel.tsx` compact variant | `panels/ShieldPanel.tsx` full variant |
| Anomaly / score | `panels/SafetyPanel.tsx` compact variant | `panels/SafetyPanel.tsx` full variant |

Components live in **two directories** (`src/app/components/` and
`src/components/`) with no rule for which. `DriveHUD.tsx` (322 lines) and
`DriveMode.tsx` (207 lines) are near-duplicate overlays that both mount in
Drive mode.

### Problem 2 — Labels that the backend no longer backs

The UI still speaks the pre-ADR-001 language:

- `DriveMode.deriveStatus()` switches on `speed_limit_reason === 'ai_decelerate'`
  — that value was **deleted** in ADR-001 item 5. It is dead UI.
- `ShieldPanel` frames the pipeline as **"AI Decision → Safety Validation →
  Final Action"** and prints `ego.decision` as the thing being validated.
  Since item 5 the learned model does **not** decide anything; the shield
  validates the *deterministic planner's* output.
- `DeveloperMode` labels the stream **"30Hz Stream"** / a panel **"10Hz"**;
  the protocol is a single 10 Hz tick (`STREAM_HZ = 10.0`).
- The earlier "Waymo Vision + LiDAR" subtitle was already caught and
  corrected once (see `DriveMode.tsx`'s comment) — the class of error
  (claiming capability the code does not have) keeps recurring because
  nothing structurally ties a label to its source field.

### Problem 3 — Phase 7 has almost no surface

Phase 7 shipped `data.channels.heavy.prediction` (per-agent 3 s forecasts,
5-way intent distributions, `p_cut_in`, `time_to_cross_s`, a proactive
`speed_limit_reason = "predictive_cut_in"`). The frontend renders **only**
the 3D ribbons and a single HUD chip. There is no panel for the intent
distribution, no per-agent forecast readout, no risk-field visualisation,
no "why is the ego easing off" explanation surface. The single most
capable new behaviour in the stack is invisible as *data*.

### Problem 4 — Modes are mutually exclusive full-screen takeovers

`activeMode` (`drive` | `developer` | `research`), toggled by a Ctrl+K
command palette, swaps the **entire** screen. `research` even unmounts
`<SimulationScene />`. There is no way to watch the drive *and* read the
safety panel *and* see a chart. "Research Lab" is a stub: uncontrolled
sliders, a "Deploy Experiment" button with no handler. The protocol v3
channel split (`pose` / `semantic` / `heavy`) — designed precisely so the
UI can show layered detail — is flattened straight back into one store and
never used to drive what's shown at what density.

---

## Decision

Rebuild the HMI as **one operator console** — a single coherent screen,
not three modes — organised around the protocol v3 channels, with every
readout traceable to a backend field.

```
┌────────────────────────────────────────────────────────────────────────┐
│  TOP BAR   connection · sim clock · scenario · density toggle           │
├──────────────────────────────────────────────┬─────────────────────────┤
│                                              │  RIGHT RAIL             │
│                                              │  (collapsible truth-    │
│         STAGE                                │   panels, one per       │
│         <SimulationScene /> — the drive      │   channel/subsystem)    │
│         + Phase 7 ribbons + risk tint        │                         │
│                                              │  ▸ Ego & Control        │
│    ┌─ HUD (overlay, minimal) ─────────┐      │  ▸ Perception           │
│    │ speed · target · binding         │      │  ▸ Prediction & Intent  │
│    │ constraint · steering            │      │  ▸ Safety Shield        │
│    │ predictive-slowdown chip         │      │  ▸ Planner              │
│    └──────────────────────────────────┘      │  ▸ Driver Analytics     │
│                                              │    (ML: decision/SHAP/  │
│                                              │     anomaly/score)      │
├──────────────────────────────────────────────┴─────────────────────────┤
│  BOTTOM STRIP   scenario timeline · events · playback · destination     │
└────────────────────────────────────────────────────────────────────────┘
```

Four rules make this real rather than cosmetic:

1. **One panel per subsystem, and it maps 1:1 to a backend field group.**
   `Prediction & Intent` reads `channels.heavy.prediction`. `Safety Shield`
   reads `channels.semantic.safety_shield`. `Driver Analytics` reads
   `channels.semantic.driver_analytics`. A panel that has no backing
   channel does not exist. This is Problem 2's structural fix: the label
   text and the field it reads sit in the same small component.

2. **Density, not modes.** A single `density` control (`focus` / `standard`
   / `inspect`) changes how much each panel shows — `focus` = HUD only,
   rail collapsed; `inspect` = every panel expanded with raw numbers and
   the `heavy` channel fully rendered. The 3D stage is *always* mounted.
   "Developer" and "Research" stop being places you go and become how much
   the one console tells you.

3. **The ML is an analytics panel, not the headline.** Per ADR-001 item 5,
   `decision` / `confidence` / `shap` / `anomaly` / `driver_score` live in
   one `Driver Analytics` panel, clearly labelled *"powertrain-telemetry
   analytics — does not drive the vehicle."* The HUD's binding-constraint
   readout (`speed_limit_reason`) is what explains the ego's behaviour.

4. **One token set, one component library.** `globals.css` tokens are the
   only colour/spacing source; the ad-hoc `bg-black/40` glass is deleted.
   All components move under `src/components/` with a flat taxonomy
   (`console/`, `panels/`, `hud/`, `3d/`, `charts/`, `primitives/`).
   `src/app/components/` is emptied.

---

## Visual language — Waymo stage, Tesla overlay

Two references, applied to two distinct layers. Both are used as *style*
references for what to build, not as a claim of parity or of matching
sensor hardware (the roadmap's standing framing).

### The 3D stage reads like the Waymo Driver visualization

A calm, semi-abstract world where the road is muted and the *semantics*
are bright and labelled. Everything on the stage is a real backend field
rendered in 3D — nothing decorative.

| Element | Treatment | Source field |
|---|---|---|
| **Road** | Extruded ribbon from the real smoothed route: dark matte asphalt (`--surface` darkened), crisp lane lines, subtle bloom on the centre line. Ground plane fades to `--bg-app` at the horizon — no skybox. | `route` message → `routeGeometry.ts` |
| **Ego** | A clean low-poly vehicle (not a photoreal model): dark body, cyan underglow, steer-linked front wheels, brake-linked taillight. | `pose.ego` (`yaw`, `steering_angle`, `acceleration`) |
| **Other cars** | Low-poly NPC bodies, class-coloured (`SEDAN`/`SUV`/`TRUCK`/…), each wrapped in a **thin wireframe bounding box** with a floating **label card** — id, class, speed, range — that always faces the camera. Confirmed = solid box; coasted = dashed, dimmed. | `heavy.surround_perception` (`class`, `x/z`, `vx/vz`, `range_m`, `dims`) |
| **Forward lead** | The one same-lane vehicle IDM is tracking gets a highlighted box + a gap line drawn on the road surface with the metre value. | `semantic.perception` (`distance`, `rel_velocity`) |
| **Path planner** | A flowing translucent **corridor ribbon** along the chosen trajectory (Tesla-blue), width ≈ lane; the dimmed alternative candidates as thin lines; a small lookahead marker. Animates as the plan updates. | `semantic.planner` (`trajectory`, `candidates`, `lane_center`) |
| **Predicted paths** | Per agent, a **translucent tapered tube** along its 3 s forecast, coloured by dominant intent (green keep / amber merge / red stop), fading toward the horizon end. The cut-in agent's tube is opaque and thicker. | `heavy.prediction` (`agents[].trail`, `agents[].intent`) |
| **Risk / occupancy** | A soft ground-projected heat wash near the ego from the risk field; optional log-odds occupancy grid at `inspect` density. | `heavy.prediction` risk, `heavy.surround_perception` occupancy |
| **Camera** | Smooth chase rig — spring-damped follow, slight look-ahead into turns, never snaps. Free-orbit available. | — |

Palette on the stage: road and world stay low-contrast and desaturated so
the semantic overlays (boxes, tubes, corridor, labels) carry all the
colour. This is the Waymo move — the interesting thing is what the car
*understands*, not the scenery.

### The overlay reads like Tesla FSD — smooth, minimal, animated

The HUD and rail sit *on top* of the stage and behave like Tesla's
visualization chrome: low chrome, glassy, and everything **eases** rather
than cuts.

- **Motion tokens** (new in `globals.css`): `--ease-out: cubic-bezier(.16,1,.3,1)`,
  `--dur-fast: 140ms`, `--dur: 240ms`, `--dur-slow: 420ms`, `--spring`
  for physical elements. Numbers **tween** to new values (speed, gap,
  probability) — no digit-snapping. Panels **slide + fade** on
  expand/collapse. The predictive-slowdown chip **grows in** from the HUD,
  it doesn't pop.
- **HUD**: a single frosted block, generous corner radius, hairline
  border, a soft inner top highlight. Speed in a light weight, large;
  everything else small and mono. The steering indicator rotates
  continuously with `--spring`.
- **Rail panels**: frosted like the HUD, not opaque cards. Expand/collapse
  is a height + opacity transition on `--ease-out`. A state change
  (shield override, cut-in engaged) triggers a one-shot accent pulse on
  that panel's border, then settles.
- **State in form, not just colour**: a severity stripe on the left edge
  of a panel, a filling meter for probabilities, a chip for binding
  constraint — so "something needs attention" reads pre-attentively, the
  way Tesla's red-car / blue-path contrast does.
- `prefers-reduced-motion`: all tweens/transitions drop to `0.01ms`;
  values update instantly; the pulse becomes a static border colour.

### Fidelity is not photorealism

Low-poly, flat-shaded, few lights. The target is *legible and smooth at
60 FPS with 30 agents*, not a render. A photoreal asphalt shader and
headlight cones are explicitly Phase 12 (Advanced Visualizer), not 7.5.

---

## Options Considered

### Option A — Incremental polish (fix labels, unify tokens, leave structure)

| Dimension | Assessment |
|---|---|
| Effort | Low (~6–10 h) |
| Risk | Low |
| Outcome | The two HUDs, two dirs, and mode-takeover model all remain; Phase 7 still has no data surface; Phase 8/9's new panels have nowhere consistent to land |

**Pros:** cheap, no regression risk. **Cons:** does not fix Problems 1, 3,
or 4 — the structural ones. Every future phase keeps paying the "where
does this panel go / which HUD do I edit" tax.

### Option B — Keep three modes, redesign each to a shared grid

| Dimension | Assessment |
|---|---|
| Effort | Medium (~15–20 h) |
| Risk | Medium |
| Outcome | Consistent within each mode; the mode-switch model and its full-screen takeover survive; still cannot watch the drive and read a chart together |

**Pros:** familiar; smaller conceptual change. **Cons:** the mode model is
itself a Problem (4). Three grids to maintain instead of one. The v3
channel layering still isn't used.

### Option C — One console, density-driven, channel-aligned *(chosen)*

| Dimension | Assessment |
|---|---|
| Effort | Medium–high (~20–30 h) |
| Risk | Medium — a real frontend restructure, guarded by `tsc --noEmit` + a component-contract test |
| Outcome | Every subsystem has exactly one panel bound to exactly one channel; Phase 7 fully surfaced; Phases 8–13 each add *one* panel to a known place; label↔field coupling is structural |

**Pros:** fixes all four problems; the protocol v3 split finally earns its
keep; the "add a panel" cost for future phases drops to near zero.
**Cons:** touches most frontend files; the cinematic Drive-mode feel has to
be preserved deliberately (it is the portfolio hook) rather than inherited.

---

## Trade-off Analysis

**B vs C** is the real decision. Both give visual consistency. B costs less
now and preserves a mental model the maintainer already has; C costs more
now and deletes that model in favour of one screen. C is chosen because
the mode model is *itself* one of the four problems, and because the
channel-aligned panel taxonomy is what makes Phases 8–13 cheap to surface
— exactly the "every remaining phase makes this worse" logic that
justified ADR-001. C is also a strict superset of B's visual work: the
token unification and grid discipline are done either way.

**Preserving the cinematic feel.** The Drive-mode glass HUD *looks* good
and is the reason a portfolio reviewer stays on the page. C keeps it — but
as a deliberately-built `hud/` component on the shared token set, with the
`focus` density making the rail and bottom strip disappear so the screen
is *exactly* today's Drive view. Nothing visual is lost; the duplicate
implementation is.

**Label honesty as a testable property.** Today "don't claim capability
the code lacks" is a convention enforced by review (and it has failed
twice). C makes it a test: a `panel-contract` spec asserts every panel
component imports its channel type from `protocol.ts` and that no panel
string literal matches a small denylist (`LiDAR`, `Vision`, `camera`,
`neural`, `deciding`, `AI decides`, …) unless it is inside a documented
disclaimer.

---

## Consequences

**What becomes easier**
- Phase 8 (spatiotemporal planner) adds one `Planner` panel section; Phase
  9 (tire physics) adds one `Vehicle Dynamics` panel; Phase 11 (RSS/MRM)
  extends the `Safety` panel. Each has a known home and a known channel.
- The `density` control gives the thesis/portfolio "evidence view"
  (`inspect`) and the demo "cinematic view" (`focus`) from one codebase.
- A new contributor (or a future AI session) can find "the safety UI" in
  exactly one file.

**What becomes harder**
- One restructure before Phase 8 delivers no new *driving* behaviour. It
  must be justified the same way ADR-001 was: as the foundation the rest
  of the roadmap's UI is built on.
- More discipline at the panel boundary: a panel now owns its channel
  import and its own empty/loading/error states.

**What we'll revisit**
- Whether `research`-style aggregate analysis (multi-run charts, cert
  metrics) belongs in the console at `inspect` density or stays a separate
  route. Deferred until Phase 13 (certification suite) produces real
  aggregate data to show.
- Mobile/tablet layout. The console is desktop-first; a stacked
  single-column fallback is in scope, a genuine mobile HMI is not.

---

## Honest positioning

This is a **UI restructure**, not a new capability. It does not make the
car drive better and will not be claimed to. What it does claim:

- **Every number on screen is traceable to a backend field** — provable by
  the panel-contract test, and the honest counter-position to a demo UI
  that shows impressive-looking metrics with nothing behind them.
- **The interface reflects the actual architecture** — sensor boundary,
  deterministic planner, independent safety, learned *analytics* (not
  control), multi-rate prediction — rather than a "Full Self-Driving"
  framing the stack does not support.

---

## Action Items

Sequenced so each step is independently shippable and `tsc --noEmit` stays
clean.

1. [ ] **Tokens & primitives** — audit `globals.css`, add the missing
   scale (spacing, radius, elevation, motion). Build `primitives/`:
   `Panel`, `PanelSection`, `Stat`, `Readout`, `Chip`, `Meter`,
   `Disclosure`. Delete the ad-hoc glass classes; the HUD's glass becomes a
   `hud/` primitive on tokens.
2. [ ] **Console shell** — `console/ConsoleLayout.tsx`: top bar / stage /
   right rail / bottom strip grid, with the `density` control in a new
   `useConsole` store (replaces `useUISettings`'s mode enum). Single-column
   fallback under a breakpoint.
3. [ ] **HUD** — one `hud/DriveHUD.tsx` on primitives: speed, target,
   `speed_limit_reason` binding-constraint readout, steering, predictive
   -slowdown chip. Delete `app/components/DriveHUD.tsx` and `DriveMode`'s
   inline overlay.
4. [ ] **Waymo-style stage** — rework `3d/SimulationScene.tsx` into the
   visual language above: extruded route road with lane lines and horizon
   fade; low-poly ego + class-coloured NPC bodies; per-track wireframe
   bounding box + camera-facing label card (id/class/speed/range) from
   `heavy.surround_perception`; the planned-path **corridor ribbon** +
   dimmed candidates from `semantic.planner`; per-agent **tapered forecast
   tubes** intent-coloured from `heavy.prediction`; ground risk wash;
   spring-damped chase camera. Desaturated world, bright semantics.
5. [ ] **Channel-aligned panels** — one component each, each importing its
   `protocol.ts` type: `EgoControlPanel` (`pose.ego`), `PerceptionPanel`
   (`semantic.perception` + `heavy.surround_perception`), `PredictionPanel`
   (`heavy.prediction` — intent bars, cut-in P + TTC, per-agent forecast
   list, risk summary), `SafetyPanel` (`semantic.safety_shield`),
   `PlannerPanel` (`semantic.planner`), `DriverAnalyticsPanel`
   (`semantic.driver_analytics`, with the disclaimer).
6. [ ] **Tesla-style overlay motion** — add the motion tokens to
   `globals.css`; tween all HUD/panel numeric values (no digit-snap);
   slide+fade panel expand/collapse on `--ease-out`; one-shot accent pulse
   on a panel's border for a state change; `--spring` steering indicator;
   full `prefers-reduced-motion` path.
7. [ ] **Bottom strip** — `console/ScenarioStrip.tsx`: scenario state +
   event timeline (from the `"event"` messages) + pause/step/reset +
   destination input. Folds in `ScenarioControlRoom`.
8. [ ] **Density wiring** — `focus` (HUD only) / `standard` (HUD + rail) /
   `inspect` (all panels expanded, raw numbers, full `heavy` render incl.
   occupancy grid). Each panel reads `density` and renders accordingly.
9. [ ] **Kill the stale labels** — remove `ai_decelerate` handling, rewrite
   `ShieldPanel`'s "AI Decision" framing, fix the Hz labels, move
   `decision`/`confidence` off the HUD into `DriverAnalyticsPanel`.
10. [ ] **Panel-contract test** — `tsc`-checked assertion that every
   `panels/*Panel.tsx` imports a channel type from `protocol.ts`, plus the
   capability-claim denylist scan.
11. [ ] **Retire `research` route or fold to `inspect`** — decide per the
   "what we'll revisit" note; remove the dead "Deploy Experiment" stub.

**Acceptance gates**

- **UI.1** `tsc --noEmit` exits 0.
- **UI.2** Exactly one HUD component and one component directory
  (`src/app/components/` empty); no `bg-black/4` / `border-white/1` literal
  in `src/` (grep clean).
- **UI.3** Every `src/components/panels/*Panel.tsx` imports at least one
  type from `../../types/protocol`; panel-contract test passes.
- **UI.4** Capability-claim denylist scan over `src/` is clean (no
  `LiDAR`/`Vision`/`camera`/`neural`/`AI decides` outside a marked
  disclaimer).
- **UI.5** `PredictionPanel` renders, at `inspect` density, for every field
  in `PredictionState` (intent distribution, `p_cut_in`, `time_to_cross_s`,
  ≥1 per-agent forecast, `proactive_decel_mps2`) — asserted by a component
  test against a fixture payload.
- **UI.6** `focus` density is pixel-equivalent in layout to today's Drive
  mode (stage + HUD, nothing else) — visual check + a snapshot of the
  rendered DOM having only the HUD + canvas.
- **UI.7** 60 FPS at 1080p with 30 vehicles + forecast tubes + bounding
  boxes at `standard` density (reuses Phase 12's Gate 12.1 harness once it
  exists; until then, a manual `performance.now()` frame-time log
  < 16.7 ms p95).
- **UI.8** Keyboard: the console is operable without a mouse (tab order
  through rail panels, density toggle, scenario controls); no `tabindex`
  traps.
- **UI.9** Stage-object traceability: every rendered stage object (NPC box,
  label card, corridor, forecast tube, gap line) is driven by a store
  field — asserted by a test that clears the store and confirms the scene
  renders only the road + ego.
- **UI.10** `prefers-reduced-motion: reduce` — no transition longer than
  1 ms; numeric values update without tween; the state pulse is a static
  colour. Checked in a jsdom test.

No new driving capability is claimed from this ADR — it is the interface
foundation Phases 8–13 render into.
