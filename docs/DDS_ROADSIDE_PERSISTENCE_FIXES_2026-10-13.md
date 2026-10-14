# Roadside, surrounding streets and persistence fixes — 2026-10-13

Baseline preserved: beea197ef357e2cefe358bd385440993e62c6e05. Changes developed and checked in DDS_Next_Sandbox; publication remains unauthorized.

## Changes

Signs use the same mitered asphalt vertices as the rendered route, interpolated at the modeled stop station, plus1.2m outward clearance. This replaces the incorrect5.5m offset inside an8.8m half-width road. Board radius0.59m remains outside the modeled ribbon. Placement is modeled, not surveyed. Straight and curved placement checks retain finite coordinates and clearance.

Nearby recorded OSM centerlines are available through a bounded read-only API:220m display window,100ways maximum,128segments/way. Crossing segments are retained even when endpoints are outside the window. Frontend refreshes independently of control, clears unavailable context, rejects aborted old requests and associates data with its route geometry. The 3D view draws contextual street lines; it does not claim lane-accurate paved intersections, verified widths or a routable complete neighborhood. Unknown coverage stays absent. Query data are dated recorded centerlines, not live road conditions.

Map edges fade through a pointer-transparent inset overlay on the actual viewport, avoiding the earlier zero-size Leaflet-pane mask defect. Street tiles remain real, neutral off-white/gray; the selected route stays dark blue. Attribution and accessible controls remain.

Persistence skips absent/malformed driver scores instead of inventing100/A+. Genuine zero scores persist. Session averages/counts derive from committed DB rows. Batch write errors propagate; worker latches persistence_failed and affected sessions do not silently claim completed. Finalization failure is also observable in memory. REST stats preserves unknown/null and genuine zero. No migration required: score aggregate Float was already nullable. Historical fabricated values cannot be identified reliably and remain untouched. Failed batches are logged and marked failed, not automatically retried or recovered.

## Evidence

44 focused persistence/runtime/REST tests passed, including actual SQLite failures, committed score aggregates, missing scores and zero scores. Two recorded-map context tests passed. TypeScript, UI contract and straight/curved roadside checks passed. Final build/lint and combined regression results are recorded below once finished.

Real headed browser: recorded_stop,9s running then paused, active stop29.81m ahead, sign outside asphalt;62 actual recorded street segments rendered,6 tile images loaded. Zero application console errors; existing Three.Clock warning remains. Screenshots output/playwright/roadside-fixed.png and nearby-street-layout.png; raw output/roadside-browser-audit.txt. These observations are finite visual validation, not collision avoidance or physical sign placement proof.

## Remaining improvements

- Lane-accurate intersection surfaces and surveyed road widths; contextual centerlines are not those surfaces.
- Render persistent mapped controls independent of currently binding junction state, and validate red/yellow/green transitions and phase freshness.
- Friction-aware stopping prediction, alongside existing aggregate modeled grip.
- Initial anomaly NORMAL before ML readiness remains unverified analytics metadata.
- Worst-case runtime jitter, detailed-mode rendering and continuous actual swept-body validation remain open.

## Exact local commits

- `b00f669c9fadb8a64f5515bfe947a7b091c6a415` — fix(persistence): preserve missing scores and report failed writes honestly
- `d2008708d2c8d37ff55b83aed50128c3ecff470d` — fix(hmi): place signs beyond asphalt and display nearby recorded streets
- `a0d5906de8495c417b259c3780df1ce63607d0c0` — fix(hmi): keep nearby street layout visible above the ground grid
- `75eed07be08a7a9dca7ff75744e98a8923562672` — fix(hmi): preserve street context when superseded requests abort
- `93c6f458d2ad7da45bd6c4c0936f6cc896b9c470` — fix(hmi): associate street context with its route geometry

## Final gates

Full combined backend suite: **916 passed,3 skipped**,61 warnings,326.55s (output/roadside-persistence-regression.txt). Collection at d200870; subsequent commits only refine frontend rendering; app/scripts/tests content unchanged. Final TypeScript and production build passed; ESLint zero errors/five existing warnings. Final frontend code revision93c6f45; details available in Git ledger; raw output/roadside-final-build.txt. Browser repeated against final code before main integration; offline/reconnect check recorded separately. No push, PR or deployment.
