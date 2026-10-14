# Recorded junction controls: follow-up

Implemented locally on 2026-10-11. This is a map-ingestion correction, not
completion of autonomous parking or junction behavior.

The previous importer discarded control-node tags while preserving road ways.
Schema 2 preserves stop, give-way and traffic-signal nodes, direction metadata,
node version/timestamp, original coordinates and way membership. The existing
four recorded OSM extracts produce 213 controls: 159 stops, 50 traffic signals,
4 give-way nodes. All have recorded way membership. Counts cover the snapshot,
not the active route. No live signal phase is supplied or inferred.

Telemetry reports `recorded_control_count` separately from
`traffic_control_behavior_verified: false`. Static map coverage cannot authorize
crossing a junction. Driving authority remains with the deterministic driver and
safety supervisor; no ML control integration was added.

Validation:

- `output/control-map-regressions.txt`: 36 passed, 8 warnings, 10.39 seconds.
  Includes importer provenance/deduplication/direction checks, existing map rules
  and simulation-session regressions. The full suite was not rerun in this pass.
- `output/control-map-live.json`: 20 actual WebSocket state packets after backend
  restart; snapshot control count 213 and behavior verification false.
- Existing Playwright sessions reconnected to port 8000. Both browser snapshots
  subsequently displayed 50 km/h, Cruising, mapped limit 105 km/h and the
  simulated-vehicle/sensors label. No new browser console errors were reported;
  existing warnings remained. A transient 0 km/h reading during reconnect did
  not persist and is not classified as a confirmed stationary-status defect.
- Before restart the browser showed Destination reached / Stopped, parking
  location unverified. This reconfirms endpoint stopping, not pull-aside parking.

Still unfinished: association of controls with the directed active route and
stop lines, modeled signal observations with explicit provenance, junction
conflict handling, and an accessible mapped parking approach with a collision
checked low-speed maneuver. Parking-area geometry alone is not an executable
parking plan. No claim of successful junction crossing or parking validation.

Backend and frontend remain running locally on ports 8000 and 3000. No commit,
push, pull request or deployment was performed.
