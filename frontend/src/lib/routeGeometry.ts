import * as THREE from 'three';

// Same equirectangular projection app/services/frenet.py's latlng_to_local()
// uses server-side (x=East, z=South/negated-latitude) -- required so the
// frontend's route geometry and the backend's real Frenet frame agree on
// where every point actually is. A sign mismatch here would silently
// desync the rendered road from the road the car is actually driving on,
// exactly the class of bug this project has hit and fixed before.
const EARTH_RADIUS_M = 6371000;

export function toLocalXZ(lat: number, lng: number, originLat: number, originLng: number): THREE.Vector3 {
  const latRad = (originLat * Math.PI) / 180;
  const x = (lng - originLng) * Math.cos(latRad) * (Math.PI / 180) * EARTH_RADIUS_M;
  const z = -(lat - originLat) * (Math.PI / 180) * EARTH_RADIUS_M;
  return new THREE.Vector3(x, 0, z);
}

export interface RouteGeometry {
  points: THREE.Vector3[];
  distances: number[]; // cumulative arc length (metres), same length as points
  totalLength: number;
  originLat: number;
  originLng: number;
}

/** Builds a local-frame route geometry from real [lat, lng] waypoints
 * (the backend's spline-smoothed route, streamed once per destination --
 * app/api/websockets.py's "route" message). Returns null if there's
 * nothing usable yet (route not fetched, or degenerate). */
export function buildRouteGeometry(waypoints: [number, number][]): RouteGeometry | null {
  if (!waypoints || waypoints.length < 2) return null;
  const [originLat, originLng] = waypoints[0];

  const rawPoints = waypoints
    .filter((p) => p && typeof p[0] === 'number' && typeof p[1] === 'number')
    .map(([lat, lng]) => toLocalXZ(lat, lng, originLat, originLng))
    .filter((p) => !isNaN(p.x) && !isNaN(p.z));
  if (rawPoints.length < 2) return null;

  // De-dupe near-identical points -- a near-zero-length segment produces a
  // degenerate/NaN tangent direction in getPathPosAtStation below.
  const points: THREE.Vector3[] = [rawPoints[0]];
  for (let i = 1; i < rawPoints.length; i++) {
    if (rawPoints[i].distanceTo(points[points.length - 1]) > 0.1) points.push(rawPoints[i]);
  }
  if (points.length < 2) return null;

  const distances = [0];
  let totalLength = 0;
  for (let i = 1; i < points.length; i++) {
    totalLength += points[i].distanceTo(points[i - 1]);
    distances.push(totalLength);
  }

  return { points, distances, totalLength, originLat, originLng };
}

/** Position + forward tangent direction at a given station (arc-length
 * metres along the route). Mirrors app/services/frenet.py's
 * frenet_to_local_xz() -- same station-latitude parameterisation, so a
 * given `s` means the same physical point on both sides. */
export function getPathPosAtStation(
  geom: RouteGeometry,
  station: number,
): { pos: THREE.Vector3; dir: THREE.Vector3 } {
  const { points, distances, totalLength } = geom;
  if (totalLength === 0) return { pos: points[0].clone(), dir: new THREE.Vector3(0, 0, -1) };

  const s = Math.max(0, Math.min(station, totalLength));
  let idx = 0;
  while (idx < distances.length - 2 && distances[idx + 1] < s) idx++;

  const start = points[idx];
  const end = points[idx + 1] ?? points[idx];
  const startDist = distances[idx];
  const endDist = distances[idx + 1] ?? startDist;

  const segLen = endDist - startDist;
  const t = segLen > 1e-9 ? (s - startDist) / segLen : 0;

  const pos = new THREE.Vector3().lerpVectors(start, end, t);
  const dir = new THREE.Vector3().subVectors(end, start).normalize();
  return { pos, dir };
}

export interface RoadRibbon {
  roadGeometry: THREE.BufferGeometry;
  centerLine: THREE.Vector3[];
  leftBound: THREE.Vector3[];
  rightBound: THREE.Vector3[];
  leftLane: THREE.Vector3[];
  rightLane: THREE.Vector3[];
  halfWidth: number;
}

/** Builds a real road-surface ribbon mesh + lane-marking polylines from the
 * route geometry, so the rendered road actually shows the real route's
 * turns instead of a generic straight strip. Ported from the pre-P6-2
 * frontend's RoadMesh (frontend/src/app/components/DriveScene.tsx), which
 * solved the same "road folds/self-intersects at sharp turns" problem this
 * would otherwise hit: a naive symmetric perpendicular offset stretches by
 * 1/cos(turnAngle/2) and blows up approaching a hairpin. Clamping the miter
 * scale (the same fix SVG/Cairo stroke rendering uses via
 * stroke-miterlimit) keeps every offset vertex bounded through turns of
 * any sharpness. */
export function buildRoadRibbon(geom: RouteGeometry, halfWidth = 7): RoadRibbon {
  const MITER_LIMIT = 2.0;
  const { points } = geom;

  const vertices: number[] = [];
  const indices: number[] = [];
  const centerLine: THREE.Vector3[] = [];
  const leftBound: THREE.Vector3[] = [];
  const rightBound: THREE.Vector3[] = [];
  const leftLane: THREE.Vector3[] = [];
  const rightLane: THREE.Vector3[] = [];

  for (let i = 0; i < points.length; i++) {
    let dirIn: THREE.Vector3;
    let dirOut: THREE.Vector3;

    if (i === 0) {
      dirOut = new THREE.Vector3().subVectors(points[1], points[0]).normalize();
      dirIn = dirOut.clone();
    } else if (i === points.length - 1) {
      dirIn = new THREE.Vector3().subVectors(points[i], points[i - 1]).normalize();
      dirOut = dirIn.clone();
    } else {
      dirIn = new THREE.Vector3().subVectors(points[i], points[i - 1]).normalize();
      dirOut = new THREE.Vector3().subVectors(points[i + 1], points[i]).normalize();
    }
    if (dirIn.lengthSq() < 0.0001) dirIn.set(0, 0, 1);
    if (dirOut.lengthSq() < 0.0001) dirOut.set(0, 0, 1);

    const rightIn = new THREE.Vector3(dirIn.z, 0, -dirIn.x).normalize();
    const rightOut = new THREE.Vector3(dirOut.z, 0, -dirOut.x).normalize();

    const right = new THREE.Vector3().addVectors(rightIn, rightOut);
    if (right.lengthSq() < 0.0001) right.copy(rightIn);
    else right.normalize();

    const cosHalfAngle = Math.max(right.dot(rightIn), 0.01);
    const miterScale = Math.min(1 / cosHalfAngle, MITER_LIMIT);
    const offsetLen = halfWidth * miterScale;

    const leftVert = points[i].clone().sub(right.clone().multiplyScalar(offsetLen));
    const rightVert = points[i].clone().add(right.clone().multiplyScalar(offsetLen));
    vertices.push(leftVert.x, leftVert.y, leftVert.z);
    vertices.push(rightVert.x, rightVert.y, rightVert.z);

    const yOffset = 0.02; // slightly above asphalt to prevent Z-fighting
    centerLine.push(points[i].clone().setY(yOffset));
    leftBound.push(points[i].clone().sub(right.clone().multiplyScalar(6.5 * miterScale)).setY(yOffset));
    rightBound.push(points[i].clone().add(right.clone().multiplyScalar(6.5 * miterScale)).setY(yOffset));
    leftLane.push(points[i].clone().sub(right.clone().multiplyScalar(3.5 * miterScale)).setY(yOffset));
    rightLane.push(points[i].clone().add(right.clone().multiplyScalar(3.5 * miterScale)).setY(yOffset));

    if (i < points.length - 1) {
      const base = i * 2;
      indices.push(base, base + 1, base + 2);
      indices.push(base + 1, base + 3, base + 2);
    }
  }

  const roadGeometry = new THREE.BufferGeometry();
  roadGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  roadGeometry.setIndex(indices);
  roadGeometry.computeVertexNormals();

  return { roadGeometry, centerLine, leftBound, rightBound, leftLane, rightLane, halfWidth };
}

/** World position + heading (degrees, compass convention: 0=N, 90=E) for a
 * Frenet (s, d) pair -- s = station along the route, d = signed lateral
 * offset (positive = right of travel direction, matching traffic.py's
 * LANE_OFFSETS / planner.py's convention exactly). This is what turns the
 * backend's Frenet-space ego/NPC state into a real point on the actual
 * curved road, instead of rendering (d, -s) directly as if it were already
 * a world position (which is what made every route render as a straight
 * corridor regardless of the real road's turns). */
export function getWorldPosAtFrenet(
  geom: RouteGeometry,
  s: number,
  d: number,
): { pos: THREE.Vector3; headingDeg: number } {
  const { pos, dir } = getPathPosAtStation(geom, s);
  // Same "right" convention as app/services/frenet.py's frenet_to_local_xz
  // (right_x, right_z = dz/seg_len, -dx/seg_len) and the old RoadMesh's
  // miter offset (dir.z, -dir.x) -- all three must agree, or the ego,
  // NPCs, and the road edges would each use a different idea of "which
  // side is which".
  const right = new THREE.Vector3(dir.z, 0, -dir.x).normalize();
  const worldPos = pos.clone().add(right.multiplyScalar(d));
  // Inverse of headingToForward(h) = (sin(h), -cos(h)): solve h from dir.
  const headingRad = Math.atan2(dir.x, -dir.z);
  const headingDeg = (headingRad * 180) / Math.PI;
  return { pos: worldPos, headingDeg };
}

/** Samples a short world-space path from (startS, startD) to (startS +
 * horizonM, endD), for rendering a planner candidate or an NPC's
 * projected path. The lateral interpolation uses smoothstep, not a
 * straight lerp -- a straight lerp from the current offset to a lane-
 * change target reads as an instant diagonal cut on screen; smoothstep
 * front-loads and back-loads the transition the way an actual quintic
 * lateral maneuver (app/services/planner.py's
 * quintic_lateral_maneuver_cost) is shaped: gentle at both ends, not
 * linear throughout. This is a visual approximation of that real cost
 * model, not a re-derivation of it -- the backend already decided the
 * candidate; this only has to look like a plausible path to it. */
export function sampleFrenetCorridor(
  geom: RouteGeometry,
  startS: number,
  startD: number,
  endD: number,
  horizonM: number,
  steps = 12,
): THREE.Vector3[] {
  const points: THREE.Vector3[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const smooth = t * t * (3 - 2 * t); // smoothstep
    const s = startS + horizonM * t;
    const d = startD + (endD - startD) * smooth;
    const { pos } = getWorldPosAtFrenet(geom, s, d);
    points.push(pos.setY(0.15));
  }
  return points;
}
