# DDS Drive cockpit redesign

Implemented locally on 2026-10-10. Existing engineering changes were preserved. No commit, push or deployment was performed.

## Result

The primary interface now uses a warm white information panel, quiet green accents, an unobstructed road view and a simple Drive / Vehicle / Navigate navigation bar. The visual direction draws on calm passenger-facing navigation and restrained automotive controls, without copying either brand's identity.

Speed, actual controller target, driving status, lead distance, safety state, current route and pause/resume remain visible. Destination selection and driving conditions use focused drawers. Vehicle information is grouped into Vehicle, Surroundings, Safety and Readings with plain-language labels. Internal telemetry analytics remain separate from driving controls. No invented ETA, battery level or model-driven control claim was added.

The road and vehicle palette is lighter. Detailed tracks and prediction paths are optional; decorative radar sweeps, night buildings, default bounding boxes and candidate-path overlays no longer crowd the default view. Rendering pixel ratio is capped at 1.5. The 3D canvas stays mounted while drawers open, preserving camera continuity. Reduced-motion preferences remain supported.

## Defects discovered and corrected

- Road triangles faced downward, causing the asphalt surface to be culled from the driving view. Triangle winding now faces upward. A geometry regression checks five cardinal/curved routes and 566 triangles.
- Drawer closing did not reliably restore keyboard focus. Opening controls now regain focus after dismissal.
- Decorative route-map CSS also enlarged the caption icon. Styling is scoped to the main SVG.
- Old vehicle panels reduced quiet readings to low opacity on a light surface. The new vehicle drawer uses clear, consistently readable rows instead.
- The development indicator overlapped mobile navigation. It is disabled through the documented Next.js setting; errors still surface normally.

## Validation

- Production Next.js build passed (`output/playwright/cockpit-redesign/build-final.log`).
- TypeScript, UI contract and focused ESLint checks passed. The UI contract still verifies the existing channel-bound panels and capability wording.
- Geometry check passed: five routes, 566 upward-facing triangles. Command: `node --experimental-strip-types scripts/verify-route-geometry.mjs` from `frontend` (executed with local Node 25.9.0).
- Real browser checks at 1440×900 and 390×844 found no horizontal overflow. Pause, camera selection, return-to-chase, destinations, environment selection, drawer dismissal, focus restoration and mobile controls were exercised.
- All four new vehicle information sections rendered real telemetry. An emergency scenario displayed the emergency braking banner, shield state and controller target zero.
- A five-second requestAnimationFrame capture during overtaking recorded 288 intervals, median 16.7 ms, p95 17.0 ms, p99 33.4 ms, and no intervals over 50 ms. This measures browser animation callback cadence in one local session, not a guarantee of GPU presentation timing, continuous 60 FPS or performance on other hardware.
- Final browser observations showed no runtime errors; the existing THREE.Clock deprecation warning remains.

Screenshots, browser scripts and output are kept locally under `output/playwright/cockpit-redesign/`, including `preview-final.png`, `vehicle-final.png`, `mobile-final.png`, `mobile-settings.png`, `emergency-final.png`, `validation-final.log` and `vehicle-validation.log`. The earlier full backend suite was not rerun for this frontend redesign; no backend behavior was changed in this task.

The local application remains available at http://localhost:3000 with the open-road drive paused for manual review. This is a product-style presentation of the existing local simulator, not a claim of physical-vehicle or production-operation validation.
