# Overnight Autonomous Session — Checkpoint (2026-09-10)

Started 00:17 IST. Branch `phase-7.5-ui-design`. Base HEAD at start: `85e97a2`.

## Objective
Work the planned P0 execution plan (Tranche A hardening; Tranche B learned-policy
DESIGN only) autonomously overnight. No live-browser UI verification. Non-browser
validation only (pytest, tsc, eslint, verify:ui). Checkpoint continuously.

## Execution plan (priority order)
- A2  physics semantics: fix path_curvature spike (P-CURV), known-geometry heading
      tests, cornering overshoot (P-CORNER)
- A3  authoritative pose: TTC doc fix, FSM diagram fix, frontend consume world_pos,
      scratch/ gitignore
- A4  attribution trace: promote scratch audit -> tracked asserting test
- B1  learned driving policy ADR (design doc only)

---

## COMPLETED
- (pre-session) A1 friction -> planner coupling: PlannerContext.friction_mu scales
  joint-planner lateral accel/jerk caps (*mu, *sqrt(mu)); 5 deterministic tests.
- (pre-session) F2 friction de-rate FACT recording in physics_engine
  (friction_speed_derate_active / friction_derate_kmh / friction_curve_cap_kmh),
  exposed in get_navigation_state(); scenario + tests key off it.
- (pre-session) F3 tracking-error speed cap now scales steer limit by friction_mu
  (was dry-only) — P0.4 consistency.
- (pre-session) F1 scenario realism: strengthened all 4 new-scenario tests to assert
  real dynamics (hard braking / measured merge envelope / ego yields to straddler);
  MERGE_COMPLETED reports measured gap+shield instead of claiming "no violation".
  Suite 316 passed / 2 skipped.

## IN PROGRESS
- A2 DONE. A3 DONE (TTC/FSM doc fixes f1ea14d; frontend world_pos consumption
  + frame-contract tests). scratch/ gitignore: NOT NEEDED (only 6 .py harnesses
  tracked, no pycache/pyc; kept as basis for A4). Next: A4 (promote attribution
  trace to a tracked asserting test), then B1 (learned-policy ADR).

## SESSION LOG
- 00:40  0f6eb96  A2.1 robust path_curvature estimator (sliding window + artefact
         rejection + physical R>=8 m cap). Killed the phantom-hairpin brake. On clean
         arcs path_curvature now tracks 1/R within ~5%; ego holds lane (max ~2.5 m off
         a 7 m road, 0 shield overrides) where it swung 11-21 m off-road before.
- 01:40  bdbfeea  bounded proactive cut-in response (ease-off cap). Cornering works:
         R=80/150/300 arcs -> lane held within 2-3 m, no stall.
- 02:05  50f4eef  A2.2 P0.1 semantics: 4 known-geometry tests -- road_heading vs
         known route bearing (0/45/90/135 deg, <3 deg err), yaw_rate ~= v/R and
         lateral_accel ~= v^2/R on a steady arc (<15% mean err), sideslip ==
         atan(0.5 tan delta) exactly every tick, road_heading sweeps monotonically
         through a left arc with no discontinuity.

## PENDING / POSTPONED
- Live-browser UI verification of all Antigravity frontend commits (414d098, 0756eab,
  72a290b, 637364e, 72dc049) — POSTPONED to a deliberate session per instruction.
- P0.2-a frontend consume world_pos — code change planned tonight; visual verify postponed.

## DECISIONS
- Committing my own work under my own messages (feature branch, validated, user wants
  checkpoints). Antigravity was auto-committing concurrently earlier; if it resumes,
  my commits interleave — acceptable.
- low_friction_curve scenario left on straight route (honest note in code); curved
  version blocked on P-CURV + cornering stability.

## ISSUES / KNOWN BUGS
- P-CURV: path_curvature spikes to ~0.6 (1/m) on smoothed authored arcs -> car floored
  to MIN_CORNER_SPEED_KMH (~8 km/h) on any curve. ROOT of "can't drive curves".
- [FIXED 0f6eb96 + bounded-proactive] P-CURV + P-CORNER: curvature spike killed;
  cornering now holds lane within ~2-3 m on R=80-300 m arcs.
- PHANTOM-CUT-IN: two distinct sources found.
  (a) FIXED: oncoming traffic in the opposing lane triggered a proactive cut-in
      from small lateral components of its fast oncoming velocity vector.
      prediction_engine now excludes tracks with v_s_ema < OPPOSING_TRAFFIC_VS_MPS
      (-2.0 m/s, i.e. moving against the route) from the cut-in gate. Regression:
      test_oncoming_traffic_does_not_trigger_a_proactive_cut_in.
  (b) RESIDUAL, bounded: same-direction NPCs on a curve still get a small spurious
      Frenet v_d (~0.1-0.25 m/s on R=80-200 m) because project_agent_frenet()
      evaluates the road tangent at the agent's CURRENT station while the EKF
      velocity reflects motion over the preceding interval, during which the
      tangent rotated by ~kappa*ds. Fix (next session): in project_agent_frenet,
      evaluate the tangent at s_mid = s - 0.5*v_s*dt_window instead of s. The
      unit test test_intent_gate_7_3 uses an idealised construction that does not
      exhibit this. PARTIALLY MITIGATED: project_agent_frenet now resolves the
      velocity onto the route tangent AVERAGED over the last ~3 m (constant
      _TANGENT_AVG_LOOKBACK_M) instead of the tangent exactly at the current
      station -- cut-in ticks on R=120 dropped ~48->32, R=300 ~59->37. Residual
      remains (EKF velocity lag likely > 3 m); fully bounded by the ease-off cap.
      Cornering works (R=80-300 m arcs hold lane within 2-3 m, no stall).
- P-DET: default unseeded PhysicsEngine cornering still non-deterministic (task_882f57f5
  running separately; 85e97a2 fixes only the seeded path).
- AI_VS_DETERMINISTIC_ATTRIBUTION.md: TTC "1.8 s" should be 2.0; FSM diagram labels wrong.
- scratch/ committed but should be local-only per conventions.

## FINAL STATE (session end ~04:55 IST, well before the 06:00 deadline)
- Working tree CLEAN. 10 commits on top of Antigravity's 10 (19b7357..b9ca55b).
- Full backend suite: 332 passed / 2 skipped, exit 0, stable across 3 runs.
- Frontend: tsc 0, eslint 0, verify:ui clean (23 files).
- Nothing pushed to origin (still 18 ahead of origin/phase-7.5-ui-design). Branch
  NOT merged to master. No push performed (not requested).

## NEXT SESSION START POINT (priority order)
1. LIVE-BROWSER VISUAL VERIFICATION (postponed all session):
   - Confirm the 3D ego sits on the rendered road ribbon through corners now that
     EgoVehicle consumes ego.world_pos (d2ce28e). The 5 m guard should hide any
     backend-frame-vs-ribbon mismatch; confirm visually.
   - Sanity-check Antigravity's UI commits (414d098/0756eab/72a290b/637364e/72dc049)
     -- camera modes, environment, HUD dock, autonomy-eval modal.
2. FINISH the phantom-cut-in-on-curves fix (HARD-2 only partially mitigated):
   - project_agent_frenet averaged-tangent lookback is a fixed 3 m guess; the EKF
     velocity lag is likely larger. Options: derive the lookback from the track's
     own speed * tick dt; or compensate v_d analytically by subtracting
     v_s * (tangent rotation over the measurement window). Target: cutin_ticks ~0
     on R>=120 m arcs with same-direction traffic.
   - Verify with the probe in this file's session log (arc R=80..300, mid_min_speed
     should approach cruise, cutin_ticks -> 0).
3. B2+ (learned policy) ONLY IF the owner has accepted ADR-003. Do not start
   otherwise.
4. Optional: promote the remaining scratch/*.py live-WS harnesses into a marked
   integration-test module or delete them (they are redundant with
   tests/test_ai_attribution.py + the live checks are non-CI).
5. Consider merging PR #3 / this branch to master + branch cleanup (owner decision).

## VERIFICATION STILL OWED (non-blocking, tracked)
- Live-browser visual pass on d2ce28e (world_pos) and all Antigravity UI commits.
- Re-measure the telemetry stream cadence with a strict frame filter (the earlier
  "23.56 Hz" figure was not the payload rate) -- see the P1.3 finding.

- 02:35  f1ea14d  A3.1 attribution doc: TTC 1.8 -> 2.0 s, FSM state names corrected.
- 02:55  (pending) A3.2 EgoVehicle consumes backend ego.world_pos as the AUTHORITATIVE
         render position, GUARDED: falls back to the frenet reconstruction if world_pos
         is non-finite or > 5 m from it (origin/axis desync safety net). Frame contract
         locked by test_world_pos_is_consistent_with_emitted_latlng (< 1.5 m) and
         test_world_pos_and_frenet_pose_share_a_frame. tsc 0 / eslint 0 / verify:ui clean.
         VISUAL VERIFICATION PENDING (no-browser session).

- 03:15  (pending) A4 tests/test_ai_attribution.py: 5 checked-in tests replacing the
         scratch audit harness -- (1) three different ML decision strings produce a
         BIT-IDENTICAL driven trajectory (classifier is analytics-only), (2) steering
         is a reproducible pure function of (route,dt,seed), (3) an emergency brake is
         attributable to the Safety Shield override not the planner/ML, (4) a lane
         change is driven by the LaneChangeStateMachine, (5) the ML feature vector is
         powertrain/emissions telemetry only -- no driving-state leak (ADR-001).
         scratch/*.py live-WS harnesses now redundant (kept, low priority to remove).

- 03:40  (pending) B1 ADR-003: learned BEHAVIOUR-RANKING policy that scores the
         already-feasible Phase-8 lattice candidates and feeds a bounded soft cost
         term into plan(); deterministic feasibility + Safety Shield stay
         authoritative; OFF by default; full provenance; 6 success criteria mapped to
         the P0 siblings; staged model (GBM ranker -> MLP -> set encoder); training =
         imitation warm-start + NGSIM + counterfactual rollouts. Roadmap Phase 8.5
         (PROPOSED) added. NO implementation -- awaiting owner review.

## TRANCHE A + B1 COMPLETE (this session)
- A1  friction -> planner coupling + de-rate fact + scenario realism        19b7357
- A2.1 robust path_curvature estimator (killed the phantom-hairpin brake)   0f6eb96
- A2.3 bounded proactive cut-in response (cornering now works)              bdbfeea
- A2.2 P0.1 known-geometry semantics tests                                  50f4eef
- A3.1 attribution doc: TTC 1.8->2.0, FSM state names                       f1ea14d
- A3.2 3D ego renders at backend-authoritative world_pos (+frame tests)    d2ce28e
- A4  checked-in runtime AI-attribution audit (5 tests)                     997ad19
- B1  ADR-003 learned driving policy (design only)                         99e3747
- HARD-1 prediction_engine excludes opposing traffic from the cut-in gate  683eae4
- HARD-2 project_agent_frenet averaged-tangent velocity resolution         b9ca55b
Suite: 332 passed / 2 skipped. tsc 0, eslint 0, verify:ui clean.
