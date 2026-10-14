# Real road rules and runtime follow-up — 11 October 2026

Local continuation of `DDS_FIXES_1_6_2026-10-11.md`. The target remains a
real-road simulator with explicitly simulated vehicle and sensors. ML has no
new control authority. No commits, pushes, PRs or deployments were made.

## Recorded map data and driving behavior

Downloaded four small bounding-box extracts through the documented
[OpenStreetMap map API](https://wiki.openstreetmap.org/wiki/API_v0.6#Retrieving_map_data_by_bounding_box:_GET_/api/0.6/map).
The filtered dataset is `app/data/road_semantics.json`; its importer is
`scripts/import_road_map.py`. It preserves original coordinates, connectivity,
OSM way IDs, versions, timestamps, relevant tags and source-response hashes.
It includes roads, parking lots and aisles. Contributor account details and
unrelated POIs are omitted. Attribution: OpenStreetMap contributors, ODbL 1.0.

Direction-aware local matching applies numeric recorded speed limits to the
driver. US 101's recorded `65 mph` tag becomes 104.60736 km/h. Missing,
symbolic or conflicting speed tags remain unknown. The UI shows a mapped
limit and explicitly identifies unknown/ambiguous coverage. Lane counts are
reported as map information; they do not make the modeled lane widths or
four-lane rendering verified. Signals and junction rules remain incomplete.

The first speed-limit live test exposed an ambiguity bug: the cruise target
rose to 120 km/h at shared ramp endpoints, and jerk-limited braking allowed
actual speed to reach 106.31106 km/h before settling to the mapped cap.
Evidence is retained in `output/mapped-speed-before-gap-fix.json`.

The corrected driver holds the previous mapped cap for at most 50 m and
2 seconds of missing/ambiguous coverage. Continued uncertainty uses a 50 km/h
simulator cap; that value is a conservative modeled policy, not an asserted
legal limit. The map status stays unknown. Route/scenario resets clear held
rules. Both lane-change and longitudinal planning use the same cruise cap;
the previously hardcoded 50 km/h lane-change target is removed.

The repeated live test captured **851 frames**, with **824 matched-map frames**
and maximum actual speed **104.60735829345941 km/h**, while preference remained
120 km/h. `output/mapped-speed-live.json` retains the trace. The browser showed
Set speed 120, actual speed about 105, mapped limit 105 and target 105.
This one recorded-road test does not establish anticipation of every lower
speed-limit transition or compliance on arbitrary city routes.

## Runtime and sensor corrections

- The Kalman position-update algebra uses the position selector's structure
  instead of multiplying sparse selector and identity matrices. A randomized
  covariance regression compares it to the full scalar Kalman equations.
- Occupancy box and display-track assembly share one pass. Fixed ray fractions
  are cached, and small transient perception records use dataclass slots.
- Measurement-noise reconstruction now preserves orientation, including
  stopped vehicles; velocity noise updates heading from the noisy velocity.
- The map index is built at initialization, before timed control steps. One
  cold profile measured 108.28 ms initialization and approximately 0.15 ms
  per warm match. These are local measurements, not latency guarantees.
- Hidden tabs stop their 3D render loop while backend driving and telemetry
  continue. The browser fault-injection check counted 689 WebGL draw calls
  when visible, 0 while hidden, and 1,097 after resuming over separate 700 ms
  sampling windows; simulation time advanced. This is not an FPS benchmark.
  Evidence: `output/visibility-qa-result.txt`. Rendering control follows the
  [React Three Fiber performance guidance](https://r3f.docs.pmnd.rs/advanced/scaling-performance).
- Loaded XGBoost analytics models use `n_jobs=1`, set during initialization
  before workers run. The classifier and SHAP model get the same budget.
  [XGBoost documents this thread parameter and contention risk](https://xgboost.readthedocs.io/en/release_3.0.0/python/python_api.html).
  A regression compared 32 inputs before/after configuration: probabilities
  were exactly equal and tree dumps unchanged. Eight analytics/inference
  tests passed. This does not establish that analytics caused the timing miss.

## Validation and remaining work

The intermediate full suite reported **468 passed, 1 failed, 3 skipped** in
286.76 s. Its failure was the unchanged perception timing budget. It preceded
the final gap-hold and noisy-heading changes; it is not presented as the final
code's complete validation. Follow-up functional checks passed; the latest
focused run reported **51 passed, 1 failed**, with perception's best batch
2.120 ms against the unchanged 2.000 ms threshold. One earlier reduced-load
run passed all 31 perception/noise checks, so the timing issue remains sensitive
to active workload. No performance improvement percentage is inferred.

The production frontend build passed. The updated browser screenshot
`output/playwright/cockpit-redesign/road-rules-live.png` was visually inspected.
All eight scenario commands applied in another live WebSocket check; malformed
scenario rejection preserved physical state and epoch. The scenario check
then restored normal cruising with an 80 km/h preference.

A visible headed browser on the actual DDS page, using the AMD RX 6500M via
ANGLE/D3D11, sampled 600 animation callbacks over 10,000.2 ms: **59.90 callbacks/s**,
median interval **16.7 ms**, p95 **16.9 ms**. Evidence:
`output/headed-ui-cadence.txt`; visually inspected screenshot:
`output/playwright/cockpit-redesign/road-rules-headed.png`. This measures browser
animation cadence in one ten-second window, not guaranteed presented FPS or
all-scenario UI performance. An earlier blank-tab sample was discarded.

A subsequent full run reported 470 passed, 2 failed, 3 skipped. Besides the
timing gate, it collected the old cruise test expecting immediate acceleration
above 50 km/h at the ambiguous start endpoint. That expectation was updated
to verify conservative initial speed, preserved preference, then higher speed
once matching resolves. All 40 road/scenario/world-boundary checks passed.
The final release check after analytics thread budgeting is recorded below
when complete; no intermediate failing suite is being represented as green.

**Final full-suite result:** `output/road-runtime-release-check.txt` completed
with **473 passed, 3 skipped** in **268.94 s**, including the unchanged timing
gates. This is one complete passing run after thread budgeting and the final
functional corrections; prior load-sensitive failures remain recorded above.
It does not prove a causal performance percentage or hard real-time execution.

The three-column reading row initially wrapped `Not detected` onto a second
line, overlapping the safety line in a short headed window. The row now has
adequate width and nonwrapping values; short-window spacing adjusts and the
detail row hides at the shortest supported desktop heights. Browser bounds at
the observed window showed readings ending at y=287.99, safety beginning at
y=297.99 and ending at y=315.98, with the navigation card starting at y=340.76.
The controls' element bounds fit the viewport; apparent right-side clipping
in the zoomed headed screenshot was a capture-scaling artifact, not an
off-screen control. Headless screenshots avoid that zoom artifact.

**Parking remains unfinished.** The successful map downloads replace the
earlier network-only blocker, but none of these extracts contains an individual
mapped parking bay. Lot polygons, aisles and access restrictions are retained;
they are not mislabeled as available bays. The current arrival remains an
in-lane stop. A validated maneuver still needs an observable permitted parking
space, its drivable approach, footprint/collision checks and full browser/live
parking validation. No generated shoulder or bay is presented as mapped data.

**Runtime headroom and complete semantic-road driving also remain open.**
The simulator is not being labeled complete autonomy on arbitrary real roads.
