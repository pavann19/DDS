# Antigravity Assignment — Visual / Browser Verification

**From:** Claude (AI / autonomy / backend owner)
**Scope:** ONLY work that genuinely requires the live browser, the running 3D scene,
and frame-by-frame visual inspection. Do **not** modify backend driving logic
(`app/services/**`), the planner, the controller, physics, perception, prediction,
the Safety Shield, or any protocol shape. Do **not** implement any learned-policy
work. If a visual defect traces to the backend, **report it** — do not patch the
backend to make a screenshot look right.

**Backend state you are verifying against (commits on `phase-7.5-ui-design`):**
- `41cfb96` — feed-forward + Stanley path tracker (cornering). Steady-state and
  committed-maneuver steering now come from `planner.ff_stanley_steer`, not pure
  pursuit. On constant-radius arcs the ego holds its lane centre to < 0.5 m
  (measured 0.03–0.28 m for R = 30–250 m), both turn directions, no exit overshoot.
- `e1967ae` — collision-awareness. `traffic.sense_lead_vehicle` now returns a
  **bumper-to-bumper** gap (both half-lengths subtracted; can be negative) and
  works as a **collision corridor** that also covers the lane the ego is
  committing into during a lane change. `safety_shield` has a hard proximity
  floor (`HARD_MIN_LEAD_GAP_M = 1.2 m`).
- Authoritative pose: the 3D ego already consumes `ego.world_pos` /
  `ego.world_heading_deg` from the backend (commit `d2ce28e`), with a 5 m
  guard falling back to the frenet reconstruction.

Run the app the normal way (`.claude/launch.json` `dds-backend` on :8000 +
`dds-frontend` on :3000). Report per section F at the end.

---

## A. Frame-by-frame cornering verification

Drive (or scenario-drive) the ego through several curves — the default synthetic
highway has only gentle bends, so **also** load a route/scenario with a real turn
(e.g. set a destination that routes through a corner, or the traffic-overtake
scenario which forces a lane change). For **entry → apex → exit** of each curve,
capture enough frames to judge:

1. **Ego position vs the road ribbon** — does the car body stay within the
   painted lane through the whole curve, or does it visibly cross a lane line
   (wide on entry / tight on exit)? Measure against the rendered lane markings.
2. **Lane-centre tracking** — the ego should track the lane centre, not the road
   polyline centreline and not an outer/inner edge.
3. **Steering wheels** — the front wheels should visibly steer into the curve
   (left wheels turn left for a left turn), proportional to curve tightness, and
   return to centre on exit. No jitter, no lock-to-stop unless the corner is a
   genuine hairpin.
4. **Vehicle heading / body orientation** — the body should point along its
   direction of travel (tangent to its actual path), not along the road tangent
   and not lagging/leading by a visible angle.
5. **Body roll / pitch** — roll into the curve (outside suspension compresses),
   pitch under braking. Magnitude should look plausible (a few degrees), not
   cartoonish, and roll direction must match the turn.
6. **Planned-path corridor** — the rendered corridor/trajectory should follow the
   lane through the curve and stay ahead of the car, not snap or lag.

**Distinguish the failure class** (this is the key deliverable of section A):
- **backend trajectory problem** — `ego.frenet.d` / `ego.world_pos` itself is off
  the lane in the telemetry (check the HUD / a channel readout), i.e. the car is
  really mistracking. → report to Claude with the frame + the telemetry values.
- **backend pose problem** — `world_pos` and `world_heading_deg` disagree with
  `frenet.s/d` + the route, or the guard fallback is triggering (ego jumps).
- **rendering / transform problem** — telemetry says the car is on the lane but
  it renders off it (a `toLocalXZ` / smoothing / origin mismatch between the
  ribbon and `world_pos`).
- **camera / visualization problem** — the car is fine, the camera framing makes
  it *look* off (e.g. chase cam lag reads as the car drifting).

Capture: 3–6 annotated frames per curve (entry/apex/exit), the corresponding HUD
readout (speed, steering, `frenet.d`, the "WHY" strip), and a one-line verdict of
which failure class (or "clean").

---

## B. Frame-by-frame obstacle-response verification

Use a reproducible scenario with a lead vehicle. Two cases, both required:

**B1 — stopped/slow lead, adjacent lane CLEAR** (ego should brake then go around):
- when does the ego first visibly react (brake lights / deceleration)?
- the rendered gap at the moment braking starts, and at closest approach;
- the ego must **never visually overlap** the lead's body — capture the closest
  frame and confirm clear air (or a small clear gap) between the two bodies;
- the lane-change trajectory around the lead, and the ego pose during it
  (no crab-walk — see section C);
- the HUD attribution: does the maneuver card / "WHY" strip name the right
  producer (car-following / planner lane change), not a learned model?

**B2 — stopped lead, adjacent lane BLOCKED alongside** (ego must stop behind it):
- the ego must come to a full stop with clear air behind the lead;
- capture the stopped frame with the rendered bumper-to-bumper gap;
- confirm the ego does **not** creep forward after stopping.

Backend expectation (for your comparison): B1 min body clearance ~1.5 m, B2 stop
~1.8 m short. If the render shows overlap or a much smaller gap while telemetry
shows a safe gap → rendering problem; if telemetry itself shows overlap → report
to Claude (backend regression).

---

## C. NPC / surrounding-vehicle visual consistency

For the ambient traffic and any scripted NPCs:
1. **Pose & heading** — each NPC body points along its own direction of travel.
   Oncoming NPCs face the ego; same-direction NPCs face away. No sideways
   translation ("crab-walking" / sliding without turning).
2. **Lane placement** — NPCs sit on lane centres (or, for the lane-straddler
   scenario, wander across a line by the scripted amount), not between lanes or
   off the road.
3. **Movement direction** — NPCs move along the road, following its curve, not in
   a straight line through a bend.
4. **Curve behaviour** — an NPC rounding the same curve as the ego stays in its
   lane and its heading sweeps with the road.
5. **Relative position & interaction** — the lead the ego is following renders
   directly ahead in the ego's lane at the gap the HUD reports; a cut-in NPC
   visibly crosses into the ego's lane before the ego reacts.

Note any NPC that translates without rotating, sits off-lane, or ignores road
curvature — with a frame and which NPC (id if visible).

---

## D. Scenario visual verification

Run each built-in scenario that has a visible outcome and confirm the render
matches the backend behaviour:
- `normal_cruising`, `traffic_overtake` (lane change), `erratic_cut_in`
  (hard brake), `highway_merge`, `lane_straddler`, `low_friction_curve`.
For each: does what you SEE (brake, swerve, stop, ease off, hold) match the
scenario's stated intent and the event strip / HUD? Flag any scenario where the
visual outcome contradicts the backend (e.g. the event log says "emergency
brake" but the car doesn't visibly brake).

---

## E. Data-honesty visual verification

Confirm the UI shows **only** what the backend actually reports:
- every number on the HUD / rail / maneuver card traces to a real channel value
  (cross-check a few against the raw telemetry);
- no invented AI decision, confidence value, prediction, trajectory, or sensor
  reading that the backend did not send;
- the autonomy-state pill / "WHY" strip reflects `speed_limit_reason` +
  `safety_shield` + `cut_in`, not a fabricated state;
- the maneuver card has **no** learned-model "confidence gauge" (the planner is
  deterministic — ADR-001);
- the Autonomy Evaluation modal's numbers come from the backend, not the frontend.
If the frontend synthesises any of the above, that is a data-honesty defect —
report it; do not "fix" it by making the backend emit the missing field.

---

## F. After completing the visual work

1. **Commit** your changes (frontend only).
2. List **exactly which files / areas changed** and why.
3. Provide the **verification evidence**: the annotated frames, the HUD/telemetry
   cross-checks, and per-section verdicts (clean / defect + class).
4. List any **remaining visual defects** you could not resolve, with the failure
   class (backend trajectory / backend pose / rendering / camera).
5. Do **not** silently modify backend intelligence, the planner, the controller,
   physics, perception, prediction, the Safety Shield, the protocol shape, or any
   `app/services/**` file. If a defect is backend, hand it back to Claude.

Claude will then review your commits and walkthrough per section 15 of the
next-phase instructions (inspect the diff, confirm no backend logic was
duplicated in the frontend, confirm authoritative backend state stays
authoritative, run the automated tests, and check that visual claims are
supported by the implementation).
