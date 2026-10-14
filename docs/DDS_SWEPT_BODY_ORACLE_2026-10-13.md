# Offline swept-body clearance oracle

`scripts/swept_body_oracle.py` evaluates **an explicit interpolant of observed poses**, not the unobserved actual plant trajectory. It never feeds privileged ground truth into driving or changes control. Samples compared in one call must describe the same time endpoints.

Center motion is linear; yaw follows the shortest linear angular displacement. Each supplied body part is a fixed, nondegenerate convex polygon in local x/z coordinates. The union can describe a chassis and fixed attachments. Independently steering wheels require additional articulated interpolation and are **not certified** by this rigid-part API. Supply chassis-only geometry and label that scope, or a separately justified fixed conservative body envelope. Geometry dimensions are caller inputs, not inferred real-world measurements.

For each simultaneous time subinterval, an endpoint vertex convex hull is inflated by `2 R sin²(|Δyaw|/4)`, equivalent to the circular-arc sagitta `R(1−cos(|Δyaw|/2))`. Linear center interpolation introduces no additional deviation. The greatest vertex radius R bounds every rigid body point. Shortest yaw guarantees |Δyaw|≤π before subdivision. Normalized separating axes test both inflated hulls using the sum of their inflation bounds. A 64-ULP coordinate guard plus outward `nextafter` handles numerical ambiguity conservatively; this is engineering floating-point protection, not a formally verified interval-arithmetic library.

Results distinguish `clear`, `overlap_observed` at an evaluated interpolant time, and `unresolved` at the subdivision limit. Spatially intersecting sweeps can still clear when their simultaneous subintervals separate. Touching, tiny clearance, and exhausted subdivision remain unresolved. An observed overlap is an interpolant overlap, not proof of a physical collision between recorded samples.

Acceptance fixtures cover translation tunneling, rotating-corner collision, time-separated spatial crossings, static separation/overlap/touching, yaw wrapping, fixed attachment unions, large-coordinate ambiguity, and invalid geometry. Live WebSocket sample application is separate acceptance evidence.
