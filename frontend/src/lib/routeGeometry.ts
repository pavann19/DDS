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
  /** Uniformly (~RESAMPLE_M) spaced samples along the SMOOTHED centreline. */
  points: THREE.Vector3[];
  /** Unit forward tangent at each sample (continuous through corners). */
  tangents: THREE.Vector3[];
  /** Cumulative arc length (metres), same length as `points`. */
  distances: number[];
  totalLength: number;
  originLat: number;
  originLng: number;
}

// The backend streams a coarse polyline (real OSRM route, resampled every
// few metres). Rendering it directly gives ~90 deg cusps at every junction
// with no turn radius -- the road looks unbuildable and vehicles snap
// their heading through the corner. We fit a centripetal Catmull-Rom
// through those waypoints (same technique the reference proving-ground
// road uses) and resample it uniformly, so every corner becomes a real
// swept arc and every tangent is continuous. Station `s` from the backend
// still maps by arc length; the smoothing shortcuts corners by at most a
// few metres over a multi-km route, which is invisible next to the win.
const RESAMPLE_M = 2.0;
const CATMULL_TENSION = 0.5;

/** Builds a smoothed local-frame route geometry from real [lat, lng]
 * waypoints (app/api/websockets.py's "route" message). Returns null if
 * there's nothing usable yet. */
export function buildRouteGeometry(waypoints: [number, number][]): RouteGeometry | null {
  if (!waypoints || waypoints.length < 2) return null;
  const [originLat, originLng] = waypoints[0];

  const rawPoints = waypoints
    .filter((p) => p && typeof p[0] === 'number' && typeof p[1] === 'number')
    .map(([lat, lng]) => toLocalXZ(lat, lng, originLat, originLng))
    .filter((p) => !isNaN(p.x) && !isNaN(p.z));
  if (rawPoints.length < 2) return null;

  // De-dupe near-identical points -- a near-zero-length segment gives the
  // spline a degenerate control point and a NaN tangent.
  const ctrl: THREE.Vector3[] = [rawPoints[0]];
  for (let i = 1; i < rawPoints.length; i++) {
    if (rawPoints[i].distanceTo(ctrl[ctrl.length - 1]) > 0.5) ctrl.push(rawPoints[i]);
  }
  if (ctrl.length < 2) return null;

  const curve = new THREE.CatmullRomCurve3(ctrl, false, 'centripetal', CATMULL_TENSION);

  // Uniform arc-length resample of the SMOOTH curve; tangents cached so
  // per-frame station lookups never re-run the arc-length remap.
  const rawLen = curve.getLength();
  const divisions = Math.max(2, Math.min(4000, Math.ceil(rawLen / RESAMPLE_M)));
  const points = curve.getSpacedPoints(divisions).map((p) => p.setY(0));
  const tangents = points.map((_, i) => {
    const u = divisions > 0 ? i / divisions : 0;
    return curve.getTangentAt(u).setY(0).normalize();
  });

  const distances = [0];
  let totalLength = 0;
  for (let i = 1; i < points.length; i++) {
    totalLength += points[i].distanceTo(points[i - 1]);
    distances.push(totalLength);
  }

  return { points, tangents, distances, totalLength, originLat, originLng };
}

/** Position + forward tangent at a given station (arc-length metres). The
 * tangent comes from the smoothed curve, so it rotates continuously
 * through every corner instead of snapping between polyline segments. */
export function getPathPosAtStation(
  geom: RouteGeometry,
  station: number,
): { pos: THREE.Vector3; dir: THREE.Vector3 } {
  const { points, tangents, distances, totalLength } = geom;
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
  const dir = new THREE.Vector3()
    .lerpVectors(tangents[idx], tangents[idx + 1] ?? tangents[idx], t)
    .setY(0);
  if (dir.lengthSq() < 1e-9) dir.subVectors(end, start);
  dir.normalize();
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

/** Builds the road-surface ribbon + lane-marking polylines. With the
 * centreline now densely sampled and smooth, consecutive segments turn by
 * a fraction of a degree, so the perpendicular offset never blows up --
 * the miter clamp is kept only as a belt-and-braces guard for any
 * residual kink. Default width is a real 4-lane cross-section (~17.6 m). */
export function buildRoadRibbon(geom: RouteGeometry, halfWidth = 8.8): RoadRibbon {
  const MITER_LIMIT = 3.0;
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

    const yOffset = 0.02; // above asphalt, avoids Z-fighting
    const laneEdge = halfWidth * 0.86;
    const laneDiv = halfWidth * 0.42;
    centerLine.push(points[i].clone().setY(yOffset));
    leftBound.push(points[i].clone().sub(right.clone().multiplyScalar(laneEdge * miterScale)).setY(yOffset));
    rightBound.push(points[i].clone().add(right.clone().multiplyScalar(laneEdge * miterScale)).setY(yOffset));
    leftLane.push(points[i].clone().sub(right.clone().multiplyScalar(laneDiv * miterScale)).setY(yOffset));
    rightLane.push(points[i].clone().add(right.clone().multiplyScalar(laneDiv * miterScale)).setY(yOffset));

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

/** World position + heading (degrees, compass: 0=N, 90=E) for a Frenet
 * (s, d) pair -- s = station along the route, d = signed lateral offset
 * (positive = right of travel, matching traffic.py's LANE_OFFSETS). */
export function getWorldPosAtFrenet(
  geom: RouteGeometry,
  s: number,
  d: number,
): { pos: THREE.Vector3; headingDeg: number } {
  const { pos, dir } = getPathPosAtStation(geom, s);
  // Same "right" convention as app/services/frenet.py's frenet_to_local_xz
  // (right_x, right_z = dz/seg_len, -dx/seg_len).
  const right = new THREE.Vector3(dir.z, 0, -dir.x).normalize();
  const worldPos = pos.clone().add(right.multiplyScalar(d));
  const headingRad = Math.atan2(dir.x, -dir.z);
  const headingDeg = (headingRad * 180) / Math.PI;
  return { pos: worldPos, headingDeg };
}

/** Samples a short world-space path from (startS, startD) to (startS +
 * horizonM, endD) -- for a planner candidate or an NPC's projected path.
 * Lateral interpolation is smoothstep (a real quintic lateral maneuver is
 * gentle at both ends, not linear). */
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
