"use client";

import React, { useMemo, useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

const EARTH_RADIUS_M = 6371000;

function toLocalXZ(lat: number, lng: number, originLat: number, originLng: number) {
  const latRad = (originLat * Math.PI) / 180;
  const x = (lng - originLng) * Math.cos(latRad) * (Math.PI / 180) * EARTH_RADIUS_M;
  const z = -(lat - originLat) * (Math.PI / 180) * EARTH_RADIUS_M;
  return new THREE.Vector3(x, 0, z);
}

// Compute cumulative distances along the route points
function buildRouteData(points: THREE.Vector3[]) {
  const distances = [0];
  let totalLength = 0;
  for (let i = 1; i < points.length; i++) {
    totalLength += points[i].distanceTo(points[i - 1]);
    distances.push(totalLength);
  }
  return { points, distances, totalLength };
}

// Get position and heading along the path at a specific distance
function getPathPosAtDistance(dist: number, routeData: { points: THREE.Vector3[], distances: number[], totalLength: number }) {
  const { points, distances, totalLength } = routeData;
  if (totalLength === 0) return { pos: points[0].clone(), dir: new THREE.Vector3(0,0,-1) };

  const clampedDist = Math.max(0, Math.min(dist, totalLength));
  
  // Find segment
  let idx = 0;
  while (idx < distances.length - 1 && distances[idx + 1] < clampedDist) {
    idx++;
  }
  
  const start = points[idx];
  const end = points[idx + 1] || points[idx];
  const startDist = distances[idx];
  const endDist = distances[idx + 1] || startDist;
  
  const segmentLength = endDist - startDist;
  const t = segmentLength > 0 ? (clampedDist - startDist) / segmentLength : 0;
  
  const pos = new THREE.Vector3().lerpVectors(start, end, t);
  const dir = new THREE.Vector3().subVectors(end, start).normalize();
  
  return { pos, dir };
}

// NPC state (id, lane_offset, speed_kmh, station_m) is authoritative on the
// backend (app/services/traffic.py, P6-1b) and streamed over the WS "npcs"
// field every tick (P6-1c) -- this component now RENDERS that state, it
// does not invent it. Positions are derived from station_m via the same
// route-distance parameterization the backend uses (station-latitude), so
// frontend and backend agree on where each NPC actually is. Recycling
// (keeping NPCs near the ego on long routes) and lane assignment are also
// backend-owned (traffic.py's VISIBILITY_WINDOW_M); nothing here re-derives
// or overrides them.
export interface NpcState {
  id: string;
  lane_offset: number;
  speed_kmh: number;
  station_m: number;
}

// Between WS ticks (10Hz) we dead-reckon each NPC forward using its real
// backend speed so motion reads as continuous at 60fps, then continuously
// pull the dead-reckoned position back toward the last authoritative
// station_m so error never accumulates past one tick interval. A station
// jump larger than one visibility-window recycle is a genuine backend
// teleport (NPC recycled to a new spot) and is snapped instantly rather
// than animated across the map.
const RECYCLE_JUMP_THRESHOLD_M = 30;
const CORRECTION_RATE = 2.0;

function NpcCar({ id, routeData, npc, npcMeshRefs }: { id: string, routeData: any, npc: NpcState, npcMeshRefs?: React.MutableRefObject<Record<string, THREE.Object3D>> }) {
  const groupRef = useRef<THREE.Group>(null);
  const displayStation = useRef(npc.station_m);
  const targetStation = useRef(npc.station_m);

  useEffect(() => {
    const jump = Math.abs(npc.station_m - targetStation.current);
    targetStation.current = npc.station_m;
    if (jump > RECYCLE_JUMP_THRESHOLD_M) displayStation.current = npc.station_m;
  }, [npc.station_m]);

  useFrame((_, delta) => {
    if (!groupRef.current || !routeData) return;

    const speedMs = npc.speed_kmh / 3.6;
    displayStation.current += speedMs * delta;
    displayStation.current += (targetStation.current - displayStation.current) * Math.min(1, delta * CORRECTION_RATE);

    const { pos, dir } = getPathPosAtDistance(displayStation.current, routeData);

    // Offset by lane
    // dir is forward. Right is (dir.z, 0, -dir.x)
    const right = new THREE.Vector3(dir.z, 0, -dir.x).normalize();
    pos.add(right.multiplyScalar(npc.lane_offset));

    // Expose the real Object3D for SensorRays' raycasting against actual
    // geometry (P6-1c/P6-2) -- the only thing anything still reads.
    if (npcMeshRefs && npcMeshRefs.current && groupRef.current) {
      npcMeshRefs.current[id] = groupRef.current;
    }

    groupRef.current.position.copy(pos);

    // Heading: Math.atan2(x, z) gives rotation around Y.
    const angle = Math.atan2(dir.x, dir.z);

    // The car faces -Z, so if dir is +Z (0,0,1), atan2(0,1)=0, but we need it to rotate 180 degrees.
    groupRef.current.rotation.y = angle + Math.PI;
  });

  // Cheaper placeholder car model for NPC, styled like FSD simplified blobs
  // -- headlights/taillights added so a stationary-looking screenshot still
  // reads as "a car" (front/back distinguishable) rather than a plain gray
  // box, which was part of why traffic looked "fake."
  return (
    <group ref={groupRef}>
      <mesh position={[0, 0.35, 0]}>
        <boxGeometry args={[1.7, 0.7, 4.0]} />
        <meshStandardMaterial color="#8a8a92" metalness={0.2} roughness={0.8} />
      </mesh>
      <mesh position={[-0.55, 0.35, 2.01]}>
        <boxGeometry args={[0.3, 0.12, 0.05]} />
        <meshStandardMaterial color="white" emissive="white" emissiveIntensity={1.2} />
      </mesh>
      <mesh position={[0.55, 0.35, 2.01]}>
        <boxGeometry args={[0.3, 0.12, 0.05]} />
        <meshStandardMaterial color="white" emissive="white" emissiveIntensity={1.2} />
      </mesh>
      <mesh position={[0, 0.35, -2.01]}>
        <boxGeometry args={[1.5, 0.1, 0.05]} />
        <meshStandardMaterial color="#880000" emissive="#880000" emissiveIntensity={0.8} />
      </mesh>
    </group>
  );
}

export default function SimulatedTraffic({ route, npcs, npcMeshRefs }: { route: [number, number][], npcs: NpcState[], npcMeshRefs?: React.MutableRefObject<Record<string, THREE.Object3D>> }) {
  const routeData = useMemo(() => {
    if (!route || route.length < 2) return null;
    const originLat = route[0][0];
    const originLng = route[0][1];
    let rawPoints = route
      .filter(p => p && typeof p[0] === 'number' && typeof p[1] === 'number')
      .map(p => toLocalXZ(p[0], p[1], originLat, originLng))
      .filter(p => !isNaN(p.x) && !isNaN(p.z));

    if (rawPoints.length < 2) return null;

    // Filter out duplicate or extremely close points that cause NaN normals
    let points = [rawPoints[0]];
    for (let i = 1; i < rawPoints.length; i++) {
      if (rawPoints[i].distanceTo(points[points.length - 1]) > 0.1) {
        points.push(rawPoints[i]);
      }
    }

    if (points.length < 2) return null;
    return buildRouteData(points);
  }, [route]);

  if (!routeData) return null;

  return (
    <group>
      {npcs.map(npc => (
        <NpcCar
          key={npc.id}
          id={npc.id}
          routeData={routeData}
          npc={npc}
          npcMeshRefs={npcMeshRefs}
        />
      ))}
    </group>
  );
}
