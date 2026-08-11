"use client";

import React, { useMemo, useRef, useEffect } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Float } from '@react-three/drei';
import * as THREE from 'three';
import SimulatedTraffic, { NpcState } from './SimulatedTraffic';

interface NavState {
  lat: number;
  lng: number;
  target_lat: number;
  target_lng: number;
  heading: number;
  speed: number;
  steering: number;
  route_index?: number;
  has_route?: boolean;
  lateral_offset_m?: number;
}

const EARTH_RADIUS_M = 6371000;

function toLocalXZ(lat: number, lng: number, originLat: number, originLng: number) {
  const latRad = (originLat * Math.PI) / 180;
  const x = (lng - originLng) * Math.cos(latRad) * (Math.PI / 180) * EARTH_RADIUS_M;
  const z = -(lat - originLat) * (Math.PI / 180) * EARTH_RADIUS_M;
  return new THREE.Vector3(x, 0, z);
}

// This local coordinate system has +X = East, +Z = SOUTH (note the negated
// lat term above) -- i.e. -Z = North. A compass heading (0=N, 90=E,
// clockwise) therefore maps to a world-space forward direction of
// (sin(heading), -cos(heading)), NOT the naive (sin(-heading), cos(-heading))
// that both EgoCar's body rotation and the chase camera used to compute.
// That sign mismatch pointed the car's mesh (and the camera's forward/behind
// placement) a full 180 degrees off from the car's actual direction of
// travel. Found and fixed 2026-07-20.
function headingToForward(headingDeg: number): THREE.Vector3 {
  const headingRad = (headingDeg * Math.PI) / 180;
  return new THREE.Vector3(Math.sin(headingRad), 0, -Math.cos(headingRad));
}

// True driver's-right vector (90 degrees clockwise from forward, viewed from
// above) -- used to offset the ego car off the route centerline and into a
// real lane (right-hand traffic), and to offset the chase camera to match.
// Previously nothing offset the car at all, so it rendered driving straight
// down the double-yellow centerline of the whole road, and looked like it
// was "drifting" whenever the lerp/heading lagged slightly through a turn
// since there was no lane discipline to visually anchor it to.
function headingToDriverRight(headingDeg: number): THREE.Vector3 {
  const forward = headingToForward(headingDeg);
  return new THREE.Vector3(-forward.z, 0, forward.x);
}

// P6-2: the ego's lateral lane offset is now REAL backend planner state
// (PhysicsEngine.current_lateral_offset_m, streamed as navState.lateral_offset_m)
// -- a genuine Frenet local planner choosing/tracking a lane-centre target,
// not a hard-coded render constant. FALLBACK_LANE_OFFSET_M is only used
// before any navState has arrived (or if a legacy-controller drive somehow
// never got a real value), so the car isn't drawn on the bare centreline
// for one frame while data is still loading.
const FALLBACK_LANE_OFFSET_M = 3.5;

function RoadMesh({ route, boundaryRefs }: { route: [number, number][], boundaryRefs?: React.MutableRefObject<{ left: THREE.Line | null; right: THREE.Line | null }> }) {
  const data = useMemo(() => {
    if (route.length < 2) return null;

    const originLat = route[0][0];
    const originLng = route[0][1];

    let rawPoints = route
      .filter(p => p && typeof p[0] === 'number' && typeof p[1] === 'number')
      .map(p => toLocalXZ(p[0], p[1], originLat, originLng))
      .filter(p => !isNaN(p.x) && !isNaN(p.z));

    if (rawPoints.length < 2) return null;

    // Filter out duplicate or extremely close points that cause NaN normals (zero-length tangents)
    let points = [rawPoints[0]];
    for (let i = 1; i < rawPoints.length; i++) {
      if (rawPoints[i].distanceTo(points[points.length - 1]) > 0.1) {
        points.push(rawPoints[i]);
      }
    }

    if (points.length < 2) return null;

    const halfWidth = 7; // 14m total width, assuming 2 lanes each way
    // At sharp turns (real intersections), a symmetric perpendicular offset
    // computed from the averaged in/out direction stretches by a factor of
    // 1/cos(turnAngle/2), which blows up toward infinity as the turn
    // approaches a hairpin -- this is what folded the road mesh into a
    // broken/self-intersecting shape right at turns. Clamping the miter
    // scale (same fix SVG/Cairo stroke rendering uses via stroke-miterlimit)
    // keeps every offset vertex bounded and the ribbon well-formed through
    // turns of any sharpness.
    const MITER_LIMIT = 2.0;

    const vertices: number[] = [];
    const indices: number[] = [];

    const curbVertices: number[] = [];
    const curbIndices: number[] = [];

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

      // Miter bisector -- the direction the offset vertex moves along.
      const right = new THREE.Vector3().addVectors(rightIn, rightOut);
      if (right.lengthSq() < 0.0001) {
        right.copy(rightIn); // near-180deg reversal -- fall back to the incoming segment's own perpendicular
      } else {
        right.normalize();
      }

      // Miter length = halfWidth / cos(turnAngle/2), clamped to MITER_LIMIT x halfWidth.
      const cosHalfAngle = Math.max(right.dot(rightIn), 0.01);
      const miterScale = Math.min(1 / cosHalfAngle, MITER_LIMIT);
      const offsetLen = halfWidth * miterScale;

      // Road surface (asphalt)
      const leftVert = points[i].clone().sub(right.clone().multiplyScalar(offsetLen));
      const rightVert = points[i].clone().add(right.clone().multiplyScalar(offsetLen));

      vertices.push(leftVert.x, leftVert.y, leftVert.z);
      vertices.push(rightVert.x, rightVert.y, rightVert.z);

      // Curbs (raised 0.15m, 1m wide -- also miter-scaled to stay flush with the road edge through turns)
      const curbWidth = 1 * miterScale;
      const curbLeftOuter = leftVert.clone().sub(right.clone().multiplyScalar(curbWidth)).setY(0.15);
      const curbLeftInner = leftVert.clone().setY(0.15);

      const curbRightInner = rightVert.clone().setY(0.15);
      const curbRightOuter = rightVert.clone().add(right.clone().multiplyScalar(curbWidth)).setY(0.15);

      curbVertices.push(curbLeftOuter.x, curbLeftOuter.y, curbLeftOuter.z);
      curbVertices.push(curbLeftInner.x, curbLeftInner.y, curbLeftInner.z);
      curbVertices.push(curbRightInner.x, curbRightInner.y, curbRightInner.z);
      curbVertices.push(curbRightOuter.x, curbRightOuter.y, curbRightOuter.z);

      // Lines (offsets scaled the same way so lane markings stay inside the miter-clamped road edge)
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

        const curbBase = i * 4;
        // Left curb quad
        curbIndices.push(curbBase, curbBase + 1, curbBase + 4);
        curbIndices.push(curbBase + 1, curbBase + 5, curbBase + 4);
        // Right curb quad
        curbIndices.push(curbBase + 2, curbBase + 3, curbBase + 6);
        curbIndices.push(curbBase + 3, curbBase + 7, curbBase + 6);
      }
    }

    const roadGeo = new THREE.BufferGeometry();
    roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    roadGeo.setIndex(indices);
    roadGeo.computeVertexNormals();

    const curbGeo = new THREE.BufferGeometry();
    curbGeo.setAttribute('position', new THREE.Float32BufferAttribute(curbVertices, 3));
    curbGeo.setIndex(curbIndices);
    curbGeo.computeVertexNormals();

    return { roadGeo, curbGeo, leftBound, rightBound, leftLane, rightLane };
  }, [route]);

  if (!data) return null;

  return (
    <group>
      {/* Asphalt */}
      <mesh geometry={data.roadGeo}>
        <meshStandardMaterial color="#dfdbdf" roughness={0.8} />
      </mesh>

      {/* Curbs / Sidewalk edge */}
      <mesh geometry={data.curbGeo}>
        <meshStandardMaterial color="#b0acb0" roughness={0.9} />
      </mesh>

      {/* 3D Guardrail / Concrete Barrier Walls along Left and Right Road Borders */}
      <mesh geometry={data.roadGeo} position={[0, 0.4, 0]}>
        <meshStandardMaterial color="#475569" metalness={0.8} roughness={0.2} />
      </mesh>

      {/* Outer 3D Barrier Posts/Rails on Left & Right bounds */}
      <line>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.leftBound.map(p => p.clone().setY(0.75))) as any)} />
        <lineBasicMaterial attach="material" color="#94a3b8" linewidth={4} />
      </line>
      <line>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.rightBound.map(p => p.clone().setY(0.75))) as any)} />
        <lineBasicMaterial attach="material" color="#94a3b8" linewidth={4} />
      </line>

      {/* Solid White Boundary Lines -- also the real road-edge geometry SensorRays raycasts against */}
      <line ref={(el: any) => { if (boundaryRefs) boundaryRefs.current.left = el; }}>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.leftBound) as any)} />
        <lineBasicMaterial attach="material" color="#ffffff" linewidth={2} />
      </line>
      <line ref={(el: any) => { if (boundaryRefs) boundaryRefs.current.right = el; }}>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.rightBound) as any)} />
        <lineBasicMaterial attach="material" color="#ffffff" linewidth={2} />
      </line>

      {/* Dashed White Lane Dividers */}
      <line>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.leftLane) as any)} />
        <lineBasicMaterial attach="material" color="#e0e0e0" linewidth={1} />
      </line>
      <line>
        <bufferGeometry attach="geometry" {...(new THREE.BufferGeometry().setFromPoints(data.rightLane) as any)} />
        <lineBasicMaterial attach="material" color="#e0e0e0" linewidth={1} />
      </line>
    </group>
  );
}

// The AI's predicted-path visualization (the "blue confidence line" seen on
// real Tesla FSD screens). Rebuilt from a SHORT local window of upcoming
// route waypoints (not the entire route) -- the previous implementation
// built one Catmull-Rom tube over the WHOLE route (hundreds of points) and
// used a draw-range trick to only display 15m of it. Catmull-Rom splines
// overshoot at sharp direction changes, and with real intersection turns in
// the route data, that overshoot produced the broken leaf/blob shape seen in
// testing -- especially since the curve was fit to points from the FULL
// route, including ones far from the visible window. Building the curve
// from ONLY the next ~12 waypoints (a few hundred meters, comparable to what
// real FSD shows) avoids that: the spline only ever has to bend through the
// real geometry actually in front of the car, not the whole trip.
// Radius/opacity are driven by the AI's actual prediction confidence
// (0..1), matching the "how confident is the car in this path" read from a
// real Tesla screen -- previously fixed regardless of confidence.
const CONFIDENCE_PATH_LOOKAHEAD_POINTS = 12;

function ConfidencePath({ route, routeIndex, confidence }: { route: [number, number][], routeIndex: number, confidence: number }) {
  const originLat = route.length > 0 ? route[0][0] : 0;
  const originLng = route.length > 0 ? route[0][1] : 0;

  const geometry = useMemo(() => {
    if (route.length < 2) return null;
    const windowPoints = route
      .slice(routeIndex, routeIndex + CONFIDENCE_PATH_LOOKAHEAD_POINTS)
      .map(([lat, lng]) => toLocalXZ(lat, lng, originLat, originLng).setY(0.4));

    // De-dupe near-identical points (same reasoning as RoadMesh) -- a
    // Catmull-Rom curve through duplicate/near-duplicate points produces
    // degenerate/NaN tangents.
    const points: THREE.Vector3[] = [];
    for (const p of windowPoints) {
      if (points.length === 0 || points[points.length - 1].distanceTo(p) > 0.1) points.push(p);
    }
    if (points.length < 2) return null;

    const curve = new THREE.CatmullRomCurve3(points);
    const tubularSegments = Math.max(points.length * 4, 8);
    return new THREE.TubeGeometry(curve, tubularSegments, 0.4 + confidence * 0.8, 12, false);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [route, routeIndex, originLat, originLng]);

  if (!geometry) return null;

  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial
        color="#1c75db"
        emissive="#3b82f6"
        emissiveIntensity={0.4 + confidence * 0.5}
        transparent
        opacity={0.3 + confidence * 0.55}
        roughness={0.2}
        metalness={0.1}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function PredictiveTrail({ navState, originLat, originLng }: { navState: NavState | null, originLat: number, originLng: number }) {
  const geometry = useMemo(() => {
    if (!navState || navState.speed < 1) return null;

    const centerline = toLocalXZ(navState.lat, navState.lng, originLat, originLng);
    if (isNaN(centerline.x) || isNaN(centerline.z)) return null;

    const heading = navState.heading ?? 0;
    const driverRight = headingToDriverRight(heading);

    // P6-2: real backend lateral offset (the Frenet planner's tracked
    // target), not a hard-coded constant plus an ad-hoc client-side evasion
    // nudge -- the backend planner already accounts for a sensed lead
    // vehicle when it scores lateral candidates (planner.py), so a second,
    // separate frontend evasion heuristic would double-count that response.
    const activeOffset = navState.lateral_offset_m ?? FALLBACK_LANE_OFFSET_M;
    const clampedOffset = Math.max(1.2, Math.min(5.6, activeOffset)); // Strictly bounded inside 3D guardrails (7m outer edge)
    const startPos = centerline.clone().add(driverRight.multiplyScalar(clampedOffset));

    // Extrapolate physics: speed is in km/h, convert to m/s
    const speedMs = navState.speed / 3.6;
    const steering = navState.steering ?? 0;

    const points: THREE.Vector3[] = [startPos.clone().setY(0.2)];
    let currentPos = startPos.clone();
    let currentHeading = heading;

    const dt = 0.1; // 100ms
    const steps = 30; // 3 seconds

    for (let i = 0; i < steps; i++) {
      // Kinematic bicycle model approximation of the real backend controller.
      const turnRate = speedMs * Math.tan(steering * Math.PI/180) / 2.8;
      currentHeading = (currentHeading + turnRate * dt * (180/Math.PI)) % 360;

      const forward = headingToForward(currentHeading);
      currentPos.add(forward.multiplyScalar(speedMs * dt));
      points.push(currentPos.clone().setY(0.2));
    }

    const curve = new THREE.CatmullRomCurve3(points);
    return new THREE.TubeGeometry(curve, 30, 0.15, 8, false);
  }, [navState, originLat, originLng]);

  if (!geometry) return null;

  return (
    <mesh geometry={geometry}>
      <meshBasicMaterial
        color="#00ffff"
        transparent
        opacity={0.7}
        depthWrite={false}
      />
    </mesh>
  );
}

// Real THREE.Raycaster casts against actual scene geometry (P6-1c) --
// replaces the earlier cosmetic angle-based distance formula. Rays are cast
// in world space from the ego car's live position against the NPC meshes
// (npcMeshRefs, populated by SimulatedTraffic's NpcCar every frame) and the
// real road-edge boundary lines (roadBoundaryRefs, populated by RoadMesh).
// Ray length visually reflects a genuine raycaster.intersectObjects() hit
// distance, not a formula.
const SENSOR_NUM_RAYS = 12;
const SENSOR_MAX_RANGE_M = 30;
const SENSOR_NO_HIT_LEN_M = SENSOR_MAX_RANGE_M * 0.6;
const UP_AXIS = new THREE.Vector3(0, 1, 0);

function SensorRays({ carPositionRef, navState, npcMeshRefs, roadBoundaryRefs }: { carPositionRef: React.MutableRefObject<{ x: number, z: number }>, navState: NavState | null, npcMeshRefs: React.MutableRefObject<Record<string, THREE.Object3D>>, roadBoundaryRefs: React.MutableRefObject<{ left: THREE.Line | null; right: THREE.Line | null }> }) {
  const groupRef = useRef<THREE.Group>(null);
  const raycasterRef = useRef<THREE.Raycaster>(new THREE.Raycaster());

  useEffect(() => {
    raycasterRef.current.params.Line = { threshold: 1.0 };
    raycasterRef.current.far = SENSOR_MAX_RANGE_M;
  }, []);

  const lines = useMemo(() => {
    return Array.from({ length: SENSOR_NUM_RAYS }).map((_, i) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
      return (
        <line key={i}>
          <primitive object={geo} attach="geometry" />
          <lineBasicMaterial color="#33ff33" transparent opacity={0.5} linewidth={2} />
        </line>
      );
    });
  }, []);

  useFrame(({ clock }) => {
    if (!groupRef.current || !navState) return;
    const { x, z } = carPositionRef.current;
    const originY = 0.5;
    groupRef.current.position.set(x, originY, z);
    const groupYaw = Math.PI - ((navState.heading ?? 0) * Math.PI / 180);
    groupRef.current.rotation.y = groupYaw;

    const time = clock.getElapsedTime();
    const raycaster = raycasterRef.current;
    const origin = new THREE.Vector3(x, originY, z);

    const targets: THREE.Object3D[] = [];
    for (const mesh of Object.values(npcMeshRefs.current)) if (mesh) targets.push(mesh);
    if (roadBoundaryRefs.current.left) targets.push(roadBoundaryRefs.current.left);
    if (roadBoundaryRefs.current.right) targets.push(roadBoundaryRefs.current.right);

    groupRef.current.children.forEach((ray: any, i) => {
      const angle = (i / SENSOR_NUM_RAYS) * Math.PI * 2;
      // Ray direction in the sensor group's local frame (matches how the
      // line geometry itself is drawn, in group-local space below).
      const localDir = new THREE.Vector3(Math.sin(angle), 0, Math.cos(angle));
      const worldDir = localDir.clone().applyAxisAngle(UP_AXIS, groupYaw).normalize();

      raycaster.set(origin, worldDir);
      const hits = targets.length > 0 ? raycaster.intersectObjects(targets, true) : [];
      const hit = hits.length > 0 ? hits[0] : null;

      const detectedDist = hit ? hit.distance : SENSOR_NO_HIT_LEN_M;
      const hasObstacle = !!hit;

      const pulse = (Math.sin(time * 8 + i) + 1) / 2;
      const currentLen = Math.max(2.0, detectedDist * (0.85 + pulse * 0.15));

      const positions = ray.geometry.attributes.position.array as Float32Array;
      positions[3] = localDir.x * currentLen;
      positions[5] = localDir.z * currentLen;
      ray.geometry.attributes.position.needsUpdate = true;

      const material = ray.material as THREE.LineBasicMaterial;
      if (hasObstacle && currentLen < 8) {
          material.color.setHex(0xff3333); // Red collision warning
      } else if (hasObstacle && currentLen < 15) {
          material.color.setHex(0xffff33); // Yellow caution
      } else {
          material.color.setHex(0x33ff33); // Green clear path
      }
    });
  });

  return <group ref={groupRef}>{lines}</group>;
}

function CameraController({ navState, route, originLat, originLng }: { navState: NavState | null, route: [number, number][], originLat: number, originLng: number }) {
  const { camera } = useThree();
  const initialized = useRef(false);
  const currentCamPos = useRef(new THREE.Vector3(0, 30, 30));
  const currentLookAt = useRef(new THREE.Vector3(0, 0, 0));

  useFrame((_, delta) => {
    const amt = Math.min(1, delta * 5);

    if (navState && route.length > 0) {
      const carCenterline = toLocalXZ(navState.lat, navState.lng, originLat, originLng);
      if (isNaN(carCenterline.x) || isNaN(carCenterline.z)) return; // Prevent NaN crash

      const forward = headingToForward(navState.heading ?? 0);
      const driverRight = headingToDriverRight(navState.heading ?? 0);
      // Track the SAME lane-offset position EgoCar renders at, not the raw
      // route centerline -- otherwise the camera frames empty centerline
      // while the car sits off to one side of the shot.
      const carLocal = carCenterline.clone().add(driverRight.clone().multiplyScalar(navState.lateral_offset_m ?? FALLBACK_LANE_OFFSET_M));

      const camDist = 15;
      const camHeight = 8;

      const targetCamPos = new THREE.Vector3(
        carLocal.x - forward.x * camDist,
        camHeight,
        carLocal.z - forward.z * camDist
      );

      const lookAheadDist = 12;
      const targetLookAt = new THREE.Vector3(
        carLocal.x + forward.x * lookAheadDist,
        1,
        carLocal.z + forward.z * lookAheadDist
      );

      if (isNaN(targetCamPos.x) || isNaN(targetLookAt.x)) return;

      if (!initialized.current) {
        currentCamPos.current.copy(targetCamPos);
        currentLookAt.current.copy(targetLookAt);
        initialized.current = true;
      } else {
        currentCamPos.current.lerp(targetCamPos, amt);
        currentLookAt.current.lerp(targetLookAt, amt);
      }
    }

    camera.position.copy(currentCamPos.current);
    camera.lookAt(currentLookAt.current);
  });

  return null;
}

function EgoCar({ navState, action, originLat, originLng, carPositionRef }: { navState: NavState | null, action: string, originLat: number, originLng: number, carPositionRef: React.MutableRefObject<{ x: number, z: number }> }) {
  const groupRef = useRef<THREE.Group>(null);

  const currentPos = useRef<{ x: number, z: number, heading: number } | null>(null);
  const targetPos = useRef<{ x: number, z: number, heading: number } | null>(null);

  const isBraking = action === 'Decelerate';
  const taillightColor = isBraking ? '#ff0000' : '#880000';
  const taillightIntensity = isBraking ? 4 : 1.5;

  useEffect(() => {
    if (navState) {
      const centerline = toLocalXZ(navState.lat, navState.lng, originLat, originLng);
      if (isNaN(centerline.x) || isNaN(centerline.z)) return;

      const heading = navState.heading ?? 0;

      // P6-2: render at the REAL backend lateral offset (the Frenet local
      // planner's tracked target -- app/services/planner.py already scores
      // candidates against the sensed lead vehicle and road edge, so there
      // is no separate frontend "evasion" decision to make here anymore;
      // doing so would be a second, uncoordinated lateral controller
      // fighting the real one). Clamp is a pure rendering safety bound
      // (never draw outside the 3D guardrails), not a planning decision.
      const baseOffset = navState.lateral_offset_m ?? FALLBACK_LANE_OFFSET_M;
      const clampedOffset = Math.max(1.2, Math.min(5.6, baseOffset));

      const driverRight = headingToDriverRight(heading);
      const laned = centerline.clone().add(driverRight.multiplyScalar(clampedOffset));

      targetPos.current = { x: laned.x, z: laned.z, heading };
      if (!currentPos.current) {
        currentPos.current = { ...targetPos.current };
      }
    }
  }, [navState, originLat, originLng]);

  useFrame((_, delta) => {
    if (!currentPos.current || !targetPos.current || !groupRef.current) return;

    const lerp = (start: number, end: number, amt: number) => (1 - amt) * start + amt * end;
    const lerpAngle = (start: number, end: number, amt: number) => {
      let diff = end - start;
      while (diff < -180) diff += 360;
      while (diff > 180) diff -= 360;
      return start + diff * amt;
    };

    const amt = Math.min(1, delta * 10);
    currentPos.current.x = lerp(currentPos.current.x, targetPos.current.x, amt);
    currentPos.current.z = lerp(currentPos.current.z, targetPos.current.z, amt);
    currentPos.current.heading = lerpAngle(currentPos.current.heading, targetPos.current.heading, amt);

    groupRef.current.position.set(currentPos.current.x, 0, currentPos.current.z);
    groupRef.current.rotation.y = Math.PI - (currentPos.current.heading * Math.PI / 180);
    carPositionRef.current.x = currentPos.current.x;
    carPositionRef.current.z = currentPos.current.z;

    const targetPitch = isBraking ? 0.05 : action === 'Accelerate' ? -0.02 : 0;
    groupRef.current.rotation.x += (targetPitch - groupRef.current.rotation.x) * 0.1;
    groupRef.current.rotation.z = -((navState?.steering || 0) * 0.1);
  });

  if (!navState) return null;

  return (
    <group ref={groupRef}>
      <Float speed={1.5} rotationIntensity={0.1} floatIntensity={0.3}>
        <group position={[0, 0.35, 0]}>
          {/* Main Body */}
          <mesh position={[0, 0.15, 0]}>
            <boxGeometry args={[1.8, 0.45, 4.2]} />
            <meshStandardMaterial color="#222" metalness={0.6} roughness={0.3} />
          </mesh>
          {/* Cabin (Glass area) */}
          <mesh position={[0, 0.6, -0.2]}>
            <boxGeometry args={[1.4, 0.4, 2.2]} />
            <meshStandardMaterial color="#111" metalness={0.9} roughness={0.1} />
          </mesh>

          {/* Headlights */}
          <mesh position={[-0.65, 0.3, 2.05]}>
            <boxGeometry args={[0.4, 0.1, 0.1]} />
            <meshStandardMaterial color="white" emissive="white" emissiveIntensity={1.5} />
          </mesh>
          <mesh position={[0.65, 0.3, 2.05]}>
            <boxGeometry args={[0.4, 0.1, 0.1]} />
            <meshStandardMaterial color="white" emissive="white" emissiveIntensity={1.5} />
          </mesh>

          {/* Taillights */}
          <mesh position={[0, 0.35, -2.08]}>
            <boxGeometry args={[1.6, 0.1, 0.05]} />
            <meshStandardMaterial color={taillightColor} emissive={taillightColor} emissiveIntensity={taillightIntensity} />
          </mesh>

          {/* Wheels */}
          {[[-0.85, -0.05, 1.3], [0.85, -0.05, 1.3], [-0.85, -0.05, -1.3], [0.85, -0.05, -1.3]].map((pos, i) => (
            <group key={i} position={pos as [number, number, number]}>
              <mesh rotation={[0, 0, Math.PI / 2]}>
                <cylinderGeometry args={[0.3, 0.3, 0.2, 24]} />
                <meshStandardMaterial color="#111" roughness={0.9} />
              </mesh>
            </group>
          ))}
        </group>
      </Float>
    </group>
  );
}

export default function DriveScene({ route, navState, action, confidence = 1, npcs = [] }: { route: [number, number][], navState: NavState | null, action: string, confidence?: number, npcs?: NpcState[] }) {
  const originLat = route.length > 0 ? route[0][0] : (navState?.lat ?? 37.7749);
  const originLng = route.length > 0 ? route[0][1] : (navState?.lng ?? -122.4194);
  const routeIndex = navState?.route_index ?? 0;

  const carPositionRef = useRef({ x: 0, z: 0 });
  const npcMeshRefs = useRef<Record<string, THREE.Object3D>>({});
  const roadBoundaryRefs = useRef<{ left: THREE.Line | null; right: THREE.Line | null }>({ left: null, right: null });

  return (
    <div className="absolute inset-0 z-0 bg-[#c6b5c7]">
      <Canvas camera={{ position: [0, 30, 30], fov: 60, near: 0.1, far: 2000 }}>
        <ambientLight intensity={1.5} />
        <directionalLight position={[10, 50, -20]} intensity={0.5} />
        <fog attach="fog" args={['#c6b5c7', 30, 400]} />

        {/* Ground plane - always visible */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.2, 0]}>
          <planeGeometry args={[2000, 2000]} />
          <meshStandardMaterial color="#c6b5c7" roughness={1} />
        </mesh>

        {/* Road + traffic + car (only when route data is available) */}
        {route.length > 0 && (
          <>
            <CameraController navState={navState} route={route} originLat={originLat} originLng={originLng} />
            <RoadMesh route={route} boundaryRefs={roadBoundaryRefs} />
            <ConfidencePath route={route} routeIndex={routeIndex} confidence={Math.max(0, Math.min(1, confidence))} />
            <PredictiveTrail navState={navState} originLat={originLat} originLng={originLng} />
            <SensorRays carPositionRef={carPositionRef} navState={navState} npcMeshRefs={npcMeshRefs} roadBoundaryRefs={roadBoundaryRefs} />
            <SimulatedTraffic route={route} npcs={npcs} npcMeshRefs={npcMeshRefs} />
            {navState && <EgoCar navState={navState} action={action} originLat={originLat} originLng={originLng} carPositionRef={carPositionRef} />}
          </>
        )}
      </Canvas>

      {/* Loading indicator while waiting for route data */}
      {route.length === 0 && (
        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <div className="bg-black/60 backdrop-blur-xl rounded-2xl px-8 py-5 text-center">
            <div className="w-8 h-8 border-2 border-blue-400 border-t-transparent rounded-full animate-spin mx-auto mb-3" />
            <div className="text-white/80 text-sm font-medium">Loading route...</div>
            <div className="text-white/40 text-xs mt-1">Fetching road geometry from OSRM</div>
          </div>
        </div>
      )}
    </div>
  );
}
