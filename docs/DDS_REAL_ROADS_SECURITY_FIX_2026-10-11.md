# Real-road defaults and dependency security fixes — 2026-10-11

## Changes

- Default driving and the deterministic route provider use a recorded 3.9 km
  southbound Bayshore Freeway / US 101 polyline instead of a generated highway.
- All eight built-in scenarios install recorded road geometry. The wet-curve
  scenario uses a real curved US 101 approach near SFO instead of a generated
  arc. Resets restore the same original geographic location and geometry.
- Source URL, retrieval date, OSM attribution, license link and SHA-256 checks
  are stored with the coordinate data. Startup is network-independent;
  missing/corrupt data raises an error without a synthetic fallback.
- Live destination requests now use HTTPS OSRM. Route switching remains atomic
  and rejects geometry outside the current corridor.
- Next.js and eslint-config-next upgraded from 16.2.10 to 16.4.0; affected
  transitive dependencies refreshed. The unpatched braces chain in Next's
  ESLint plugin was replaced by a scoped tinyglobby adapter preserving its
  root-directory glob behavior. Lint rules were not disabled and audit
  findings were not suppressed.
- Map caption identifies real roads and the simulated vehicle. Vehicle motion,
  surrounding traffic, friction and sensors remain software simulations.

## Validation observed

- Clean `npm ci`: successful, zero known vulnerabilities reported.
- Full `npm audit` and `npm audit --omit=dev`: zero findings.
- `npm run build`, TypeScript, focused ESLint: passed.
- `node scripts/verify-lint-dependencies.mjs`: brace-expanded directory roots
  match expected paths, and an intentional img violation is still detected
  by the Next.js lint rule.
- Focused backend run: 77 passed, 1 skipped. Final real-road regression file:
  12 passed, including deterministic resets across eight scenarios, corrupt
  data rejection, and road smoothing fidelity.
- Maximum smoothed-route distance from the recorded source polyline measured
  0.270 m for highway and 0.371 m for wet curve; regression bound is 1 m.
- WebSocket captures covered all eight scenarios with acknowledged resets.
  Emergency cut-in capture reached -4.5 m/s². Short ordinary cruising capture
  measured at most 0.003 m displacement from the initial lane center; overtaking
  captures intentionally changed lanes and are not lane-keeping measurements.
- Real browser: highway displayed its road name; wet-curve selection and
  Restart current drive produced Changes applied, the appropriate road name,
  a visible route/vehicle overlay and six loaded OSM tiles, without horizontal
  overflow. Screenshots and captures are under `output/`.

## Timing limitation discovered and reported

The full suite under simultaneous live backend/browser/build load finished
with **424 passed, 2 failed, 3 skipped** in 316.74 seconds. Failures were existing
timing gates: perception best batch 2.572 ms vs a 2 ms limit, and planner best
tick 4.635 ms vs a 4 ms limit. Both passed on a separate reduced-load run after
the live backend was briefly stopped: **2 passed** in 9.76 seconds. Assertions
and timing thresholds were not changed. This is load-sensitive timing evidence,
not an all-green full-suite claim or a hard real-time guarantee.

After these checks, backend and frontend were restored and the default drive
was restarted on the recorded highway.

Evidence:

- `output/frontend-audit-fixed.json`
- `output/backend-real-roads-tests.txt`
- `output/timing-recheck-real-roads.txt`
- `output/real-roads-live.json`
- `output/playwright/cockpit-redesign/real-road-highway-final.png`
- `output/playwright/cockpit-redesign/real-road-wet-reset-final.png`

No push, PR or deployment was performed. Changes remain local.
