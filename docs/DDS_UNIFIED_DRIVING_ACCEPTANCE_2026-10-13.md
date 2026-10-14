# Unified driving environment — 2026-10-13

Preserved baseline: `155b2d89bf257c693ac749b82088af2f46892774`, tag `dds-unified-driving-baseline-20261013`. Changes developed in `DDS_Next_Sandbox`; all commits remain local. No push, PR or deployment.

## One driving experience

The application starts paused in `driving_environment`, using the full recorded approximately 4.75 km highway-to-parking mission. Traffic density and road grip configure this same environment. Individual fixtures and alternative sensing modes remain under collapsed **Advanced testing**. Cruise preference lives in Settings; the main driving view no longer contains the SET SPEED card. Environment changes and restart pause the drive. Choosing a new destination retains finite traffic and camera requirements, clears old camera evidence, and does not invent a verified parking goal.

The combined scene has no scenario tick callback, forced ego decision, timed emergency trigger or milestone that asserts successful driving after a fixed tick. Roads/parking geometry are recorded map input. Vehicle motion, traffic, sensors, light cycles and modeled lane placement are explicitly simulated. A fixed seed permits replay; it is not a recorded ego trajectory or a classifier driving the car.

Ordinary generated traffic now uses a finite road domain in this mode: actors leave naturally rather than wrapping or teleporting around ego. Default density is medium; supported densities generate 4/8/14 actors. Traffic occupies one declared same-direction modeled lane and stays out of the dedicated parking access area. Background actors use simplified world-owned red/amber, STOP dwell and mapped-speed responses. World actors may read simulated world controls; the ego driver receives sensor observations instead. These are kinematic background vehicles, not complete autonomous agents with surveyed lane topology, tire dynamics or intersection conflict negotiation.

## Camera and sensor combination

Scene-camera RGB observations handle supported roadside controls; forward/surround simulated range observations handle traffic and safety. Both are already supplied to the deterministic planner/control/safety stack. A new required-camera producer-clock contract makes the combined drive hold when either range sensing or its required camera is missing, stale, invalid or regressed. Camera recovery requires three distinct accepted capture timestamps; replays or consumer reads cannot restore health. Camera age limit is 0.35 s; range-feed limits retain the previous contract. Route changes and scenario resets clear old confirmation state.

This is combined availability gating and complementary observations, **not camera/radar object-level fusion**. A fresh blank camera frame establishes capture availability, not recognition of all hazards. Map priors still associate supported controls; arbitrary unmapped controls and camera-based vehicle depth/tracking remain unfinished. ML remains internal vehicle analytics. The driver computes steering and braking from observations, route constraints and safety checks.

## Defects found and corrected

The legacy traffic model recycled actors near ego and wrapped them at road ends; those behaviors are disabled only for the combined finite environment. Its four fixed lane offsets could not justify actors on narrow access roads, so combined traffic uses a declared single modeled lane and excludes the parking access area. Destination changes initially recreated legacy traffic; the combined configuration is now reapplied with camera evidence cleared. NPC world signal time stays aligned with session time after rerouting.

An intermediate sandbox edit corrupted three existing Unicode fixture names through a Windows text-encoding conversion. Commit `3627d08` restores the original names and adds checks; final reads/writes use explicit UTF-8. A traffic speed callback initially captured a mutable physics object; it now retains its route value and is covered by copied-world tests. These intermediate defects were reported and corrected before integration.

## Validation

Backend application revision `3387d45e2a5b43a91535b3833e97b38ad0a85442`: **1,020 passed, 3 skipped**, 65 warnings, 343.15 s. `output/unified-driving-full-regression.txt`. Backend source stayed fixed during this run. Includes 27 combined-world cases, 13 new combined-sensor cases and 3 session lifecycle cases. Existing fixtures remain in the suite. No recognition or collision success rate is inferred from these counts.

Actual headed browser plus live WebSocket checks:

- Startup is the combined drive, paused. No main SET SPEED card; cruise preference is visible in Settings. Advanced fixtures are initially hidden and expand on request. Changing traffic to high and grip to 0.5 produces acknowledged paused configuration. `output/unified-driving-ui-raw.txt`, `output/playwright/unified-driving-{main,settings}.png`.
- Deliberate camera HTTP loss while range sensing stays current: initial **13.788 m/s**, reason `car_following`; after a 15 s interruption the car reaches **0 m/s**, `sensor_camera_stale`, capture age 15.1 s. After restoring transport, three distinct fresh captures restore readiness and motion resumes, final **13.109 m/s**, reason `cruise`. `output/unified-camera-loss-raw.txt`, `output/playwright/unified-camera-recovery.png`. This is one controlled interruption observation, not a general stopping-distance guarantee.
- Actual telemetry-worker WebSocket closure uses the existing retry path, rather than fabricated state messages. The UI shows **Waiting for connection**, then **Drive paused** after reconnecting to the real backend. The temporary constructor capture wrapper is restored after the test. `output/unified-ws-reconnection-raw.txt`, `output/playwright/unified-ws-{disconnected,reconnected}.png`.

The initial camera-loss trial was shorter and did not establish complete standstill/recovery; it is retained separately. A selector spelling error and an invalid client WebSocket close code occurred in the acceptance harness, not application code; corrected reruns are separate artifacts. Deliberate camera HTTP aborts generate expected browser network errors. No zero-console-error claim is made for those negative trials.

TypeScript, UI contract and lint checks passed; lint retains five existing warnings. Final production frontend build **passed** at revision `930a488b3403a1b86b5303c532e9301c767b7a4d`; raw `output/unified-driving-final-build.txt`. Camera readback is still browser-produced and subject to foreground scheduling. No new frame-rate or production operation guarantee is claimed.

## Local implementation ledger

- `82fdcc280c65ca3a3e76fc70ea584a1a99e0b211` — require fresh scene camera alongside range sensors.
- `c8ec3aac70cc3334a38fe6b68cad2c0b9acc764c` — combined recorded-road mission and finite background traffic.
- `3627d08812d243517b5b983fb3585b645bdc54eb` — preserve text encoding, route capture and medium-density default.
- `88c2e9bd81d8d06c47726a48a2b7de0786928794` — reuse finite traffic configuration after destination changes.
- `3387d45e2a5b43a91535b3833e97b38ad0a85442` — paused product startup, reset and route/sensing continuity.
- `930a488b3403a1b86b5303c532e9301c767b7a4d` — unified Settings, collapsed tests and cruise relocation.

This completes the combined-environment/settings and required-camera integration gate. It does not mark the entire autonomous-driving roadmap complete. Independent unmapped-control interpretation, calibrated depth and camera/object fusion, richer road/lane coverage, intersection negotiation, physical sensors and real-vehicle autonomy remain outside this gate.
