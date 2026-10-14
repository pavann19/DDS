import assert from "node:assert/strict";
import {
  buildRouteGeometry,
  buildRoadRibbon,
  toLocalXZ,
  getPathPosAtStation,
} from "../src/lib/routeGeometry.ts";

// A downward-facing asphalt mesh disappears under back-face culling.
// Check every triangle on cardinal and curved roads, not just one fixture.
const routes = [
  [
    [37, -122],
    [37.001, -122],
  ],
  [
    [37.001, -122],
    [37, -122],
  ],
  [
    [37, -122],
    [37, -121.999],
  ],
  [
    [37, -121.999],
    [37, -122],
  ],
  [
    [37, -122],
    [37.0004, -122],
    [37.0008, -121.9996],
    [37.001, -121.999],
  ],
];
let triangles = 0;
for (const route of routes) {
  const geometry = buildRouteGeometry(route);
  assert.ok(geometry);
  assert.equal(geometry.points.length,route.length);
  for(let point=0;point<route.length;point++) {
    const expected=toLocalXZ(...route[point],...route[0]);
    assert.ok(geometry.points[point].distanceTo(expected)<1e-6,'Renderer must preserve authoritative point');
    assert.ok(getPathPosAtStation(geometry,geometry.distances[point]).pos.distanceTo(expected)<1e-6,'Station must retain authoritative position');
  }
  const road = buildRoadRibbon(geometry).roadGeometry;
  const positions = road.getAttribute("position");
  const indices = road.getIndex();
  for (let i = 0; i < indices.count; i += 3) {
    const [a, b, c] = [
      indices.getX(i),
      indices.getX(i + 1),
      indices.getX(i + 2),
    ];
    const abX = positions.getX(b) - positions.getX(a),
      abZ = positions.getZ(b) - positions.getZ(a);
    const acX = positions.getX(c) - positions.getX(a),
      acZ = positions.getZ(c) - positions.getZ(a);
    assert.ok(
      abZ * acX - abX * acZ > 0,
      `Route triangle ${i / 3} faces away from the driving view`,
    );
    triangles++;
  }
  road.dispose();
}
console.log(
  `Road geometry OK: ${routes.length} routes, ${triangles} upward-facing triangles.`,
);
