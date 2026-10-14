# Recorded road and parking evidence

Baseline: `42aff5ef6d4d59aca635b4ed1cc7ed54da952fcf`. Isolated branch:
`feat/map-evidence-20261013` in `DDS_Map_Sandbox`.

Local snapshot audit: 3,127 features; 471 `lanes` tags; one explicit `width`
tag (way 256787724, 21 m); zero individual `parking_space` features. All
feature node and geometry lengths agree. Original XML source files are not
present locally: discarded tags cannot be recovered from this snapshot.
No downloaded dataset, recorded coordinates or modeled driving widths changed.

The selected lot (1356718570) has `parking=surface`, `surface=asphalt`, closed
polygon geometry, and no capacity/width/access tag. Its aisle (499187289) has
`service=parking_aisle`, no width/access/oneway tag. The mission follows that
aisle opposite stored node order; absent oneway remains eligible, without
asserting live permission. The goal is planner-selected, not a recorded bay.

Additions expose dated way/version/timestamp and explicit width tags without
inferring per-lane widths. Parking telemetry exposes lot/aisle provenance,
raw access tags and unknown capacity/bays. Occupancy remains simulated sensors.

Fixes: retain vehicle/motorcar restrictions and conditional tags on future
imports; parking assembly rejects unresolved or restrictive car-access tags;
reject contrary or unresolved aisle oneway directions. This is conservative
mission eligibility, not a complete OSM mode hierarchy/conditional resolver.
The importer also retains `oneway:conditional`; parking assembly rejects
unresolved conditional aisle directions even when the base tag is two-way.
`mission_eligible` describes simulator policy only, not actual access permission.
A general restriction with a specific allowance remains blocked pending
review. Missing access tags remain `not_tagged`, never live permission.

Definitions checked against primary OSM documentation:
[access](https://wiki.openstreetmap.org/wiki/Key:access),
[width](https://wiki.openstreetmap.org/wiki/Key:width).

Validation includes malformed/ambiguous width tags, imported access retention,
actual parking assembly rejection fixtures, both aisle directions, original
road coverage tests and complete actual simulated parking mission with held
stop. Browser validation belongs to combined integration. This evidence does
not certify surveyed lanes, live access, parking vacancy or physical autonomy.
