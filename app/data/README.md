# Recorded real-road routes

`real_road_routes.json` contains recorded OSRM geometry derived from
[OpenStreetMap contributors](https://www.openstreetmap.org/copyright), under
the Open Database License (ODbL). This is a small extracted road dataset,
not a generated approximation of a highway.

The source request, retrieval date, coordinate arrays and geometry SHA-256
digests are stored in the file. Coordinates use `[latitude, longitude]`.

- `highway`: the Bayshore Freeway step (step 6 of the source response),
  approximately 3.9 km of US 101 southbound.
- `wet_curve`: the final portion of steps 3 through 5, starting at the last
  source vertex before 1100 m of cumulative distance along those steps. This
  retains the actual curved southbound approach near SFO, with a lead-in.

Both start on their recorded road and remain at their original location.
Scenario restarts install the same geometry without consulting a network
service. Missing or corrupt route data is an error; there is no synthetic
fallback. Runtime smoothing resamples the recorded polyline at about 5 m;
regressions constrain its deviation from the source polyline to under 1 m.

Live destination requests still use HTTPS OSRM routing. Road geometry does
not supply live traffic, road closures, speed limits, signals or intersection
priority; traffic actors and vehicle readings remain simulated.

The pinned profiles also include approximately 300 m of road continuation
from the same source response after their navigation endpoint, with separate
integrity digests. It is display context only: it does not extend the driving
goal or claim a parking space. Dynamic routes without recorded continuation
remain explicitly without this context; no straight road is fabricated.

## Recorded road semantics (11 October 2026)

`road_semantics.json` is a filtered OpenStreetMap extract downloaded through
the documented `GET /api/0.6/map?bbox=...` API. `sources` records bounding boxes
and SHA-256 hashes of the original XML responses; each way retains its OSM ID,
version, timestamp, node connectivity, tags and original coordinates. Import
with `scripts/import_road_map.py`. Contributor account details and unrelated
POI information are excluded. Attribution: OpenStreetMap contributors, ODbL 1.0.

The snapshot covers the recorded US 101 route and nearby parking lots/aisles.
It is not a live road feed. Numeric `maxspeed` values and `mph` values are
parsed; symbolic or missing values stay unknown. Direction-aware matching
rejects incompatible overlapping speed tags. Lane counts are informational;
lane widths and the lane geometry of the simulator remain modeled. No mapped
individual parking bays were found in these extracts. Lot polygons are not
treated as invented parking spaces, and access restrictions are retained.

When matching becomes ambiguous, control holds the previous mapped cap for at
most 50 m and 2 seconds. Longer/missing coverage uses a conservative 50 km/h
simulator cap, not an asserted legal limit. The UI explicitly shows unknown
map coverage. Signals, stop rules and complete junction behavior remain open.
