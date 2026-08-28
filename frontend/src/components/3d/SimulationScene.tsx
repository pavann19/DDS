'use client';

import React, { useRef, useMemo, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { PerspectiveCamera, Grid, Html, Line } from '@react-three/drei';
import { useSimulationStore } from '../../store/useSimulationStore';
import * as THREE from 'three';
import { CarFront } from 'lucide-react';
import {
  RouteGeometry,
  buildRouteGeometry,
  buildRoadRibbon,
  getWorldPosAtFrenet,
  sampleFrenetCorridor,
} from '../../lib/routeGeometry';
import { annotateTracks, ROLE_LABEL, type AnnotatedTrack, type TrackRole } from '../../lib/roles';

// Shared GPU resources — created once, never per-frame or per-track.
const UNIT_EDGES = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
const EMPTY_GEOM = new THREE.BufferGeometry();

// Real pixel-width lines: native THREE.Line/lineBasicMaterial's `linewidth`
// is capped at 1px on most GPUs/browsers (a long-standing WebGL
// limitation), which makes "dimmed alternative vs. highlighted chosen
// path" impossible to actually distinguish on screen. drei's `Line`
// (Line2/LineMaterial under the hood) gives real width control -- already
// a project dependency (@react-three/drei), no new install.
const CANDIDATE_PATH_HORIZON_M = 40;

// Waymo / Tesla Autonomous Palette
const THEME = {
  brandCyan: '#00E5FF',
  brandCyanGlow: '#00B4D8',
  teslaBlueRibbon: '#00B0FF',
  egoChassis: '#0E1726',
  egoRoof: '#1B2A47',
  headlightCyan: '#A5F3FC',
  taillightRed: '#FF2A4D',
  npcChassis: '#334155',
  npcRoof: '#1E293B',
  npcTaillight: '#F87171',
  detectedSafe: '#10B981',
  detectedWarning: '#F59E0B',
  detectedCritical: '#EF4444',
  roadSurface: '#070A10',
  laneWhite: '#E2E8F0',
  laneYellow: '#FBBF24',
  laneCyan: '#00E5FF',
  gridSection: '#1E293B',
  gridCell: '#0D131F',
};

// Real-world-space ego state (x, z, heading), consumed by the camera and
// the decorative ribbon/sweep effects. headingRad is the TRUE compass
// heading (0=N, 90=E, matching app/services/frenet.py's convention) --
// this used to be a decorative wobble derived from steering angle, which
// only looked plausible because everything rendered in Frenet space
// (constant -Z "forward" by construction, so the road's real turns were
// invisible). Now that the ego renders at a real point on the real curved
// road, its heading has to be the road's real tangent direction, or the
// car and the road it's drawn on visibly disagree.
const sharedVehicleState = {
  x: 0,
  z: 0,
  headingRad: 0,
  initialized: false,
};

function headingToForward(headingRad: number): THREE.Vector3 {
  return new THREE.Vector3(Math.sin(headingRad), 0, -Math.cos(headingRad));
}

function useRouteGeometry(): RouteGeometry | null {
  const routeWaypoints = useSimulationStore((state) => state.routeWaypoints);
  return useMemo(() => buildRouteGeometry(routeWaypoints), [routeWaypoints]);
}

// --- 1. Planned-path corridor (§6): the chosen lateral plan sampled
// through the REAL route geometry so it follows the road's curves, built
// as a tapered ribbon rather than a line. Colour tracks the binding
// constraint (speed_limit_reason). When the plan changes materially the
// previous ribbon fades out while the new one fades in -- no visual jump.
const CORRIDOR_HORIZON_M = 46;
const CORRIDOR_STEPS = 20;
const CORRIDOR_HALF_W = 1.25;

const CORRIDOR_COLOR: Record<string, string> = {
  cruise: THEME.teslaBlueRibbon,
  lateral_accel_limit: THEME.detectedWarning,
  tracking_correction: THEME.teslaBlueRibbon,
  car_following: THEME.detectedWarning,
  predictive_cut_in: THEME.detectedWarning,
  safety_shield_override: THEME.detectedCritical,
};

/** Tapered triangle-strip ribbon from a centreline. */
function buildRibbonGeometry(points: THREE.Vector3[]): THREE.BufferGeometry {
  const n = points.length;
  const pos = new Float32Array(n * 2 * 3);
  const tan = new THREE.Vector3();
  for (let i = 0; i < n; i++) {
    const p = points[i];
    const prev = points[Math.max(0, i - 1)];
    const next = points[Math.min(n - 1, i + 1)];
    tan.subVectors(next, prev);
    const nx = tan.z, nz = -tan.x;
    const len = Math.hypot(nx, nz) || 1;
    const w = CORRIDOR_HALF_W * (1 - 0.45 * (i / (n - 1)));
    const ox = (nx / len) * w, oz = (nz / len) * w;
    pos[i * 6 + 0] = p.x + ox; pos[i * 6 + 1] = p.y; pos[i * 6 + 2] = p.z + oz;
    pos[i * 6 + 3] = p.x - ox; pos[i * 6 + 4] = p.y; pos[i * 6 + 5] = p.z - oz;
  }
  const idx: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    const a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
    idx.push(a, b, c, b, d, c);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(idx);
  return g;
}

function PlannedCorridor() {
  const ego = useSimulationStore((s) => s.ego);
  const planner = useSimulationStore((s) => s.planner);
  const routeGeom = useRouteGeometry();

  const chosenD =
    planner?.candidates.find((c) => c.is_chosen)?.d_target ??
    planner?.lane_center ??
    ego?.frenet?.d ??
    0;
  const reason = ego?.speed_limit_reason ?? 'cruise';
  const color = CORRIDOR_COLOR[reason] ?? THEME.teslaBlueRibbon;

  // Rebuild only when the plan actually moves (quantised signature).
  const sig = ego
    ? `${Math.round(ego.frenet.s)}:${(ego.frenet.d).toFixed(1)}:${chosenD.toFixed(1)}:${Boolean(routeGeom)}`
    : 'none';

  const geometry = useMemo(() => {
    if (!ego) return null;
    let pts: THREE.Vector3[];
    if (routeGeom) {
      pts = sampleFrenetCorridor(routeGeom, ego.frenet.s, ego.frenet.d, chosenD, CORRIDOR_HORIZON_M, CORRIDOR_STEPS);
    } else {
      const f = headingToForward(sharedVehicleState.headingRad);
      pts = [];
      for (let i = 0; i <= CORRIDOR_STEPS; i++) {
        const t = (i / CORRIDOR_STEPS) * CORRIDOR_HORIZON_M;
        pts.push(new THREE.Vector3(sharedVehicleState.x + f.x * t, 0.15, sharedVehicleState.z + f.z * t));
      }
    }
    return buildRibbonGeometry(pts);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sig]);

  // Crossfade handled entirely in the three layer: two persistent meshes,
  // geometry swapped on the inactive one in an effect, opacity driven in
  // useFrame. No React state / no ref reads during render.
  const meshA = useRef<THREE.Mesh>(null);
  const meshB = useRef<THREE.Mesh>(null);
  const active = useRef(0); // which mesh currently shows the live plan
  const fade = useRef(1);

  useEffect(() => {
    if (!geometry) return;
    const incoming = active.current === 0 ? meshB.current : meshA.current;
    const outgoing = active.current === 0 ? meshA.current : meshB.current;
    if (!incoming || !outgoing) return;
    const stale = incoming.geometry;
    incoming.geometry = geometry;
    if (stale && stale !== geometry && stale !== EMPTY_GEOM) stale.dispose();
    active.current = active.current === 0 ? 1 : 0;
    fade.current = 0;
  }, [geometry]);

  useFrame((_, delta) => {
    fade.current = Math.min(1, fade.current + delta * 3.5);
    const inM = (active.current === 0 ? meshA.current : meshB.current);
    const outM = (active.current === 0 ? meshB.current : meshA.current);
    if (inM) (inM.material as THREE.MeshBasicMaterial).opacity = 0.34 * fade.current + 0.12;
    if (outM) {
      (outM.material as THREE.MeshBasicMaterial).opacity = 0.34 * (1 - fade.current);
      outM.visible = fade.current < 1;
    }
  });

  if (!geometry) return null;
  return (
    <group>
      <mesh ref={meshA} geometry={EMPTY_GEOM}>
        <meshBasicMaterial color={color} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      <mesh ref={meshB} geometry={EMPTY_GEOM}>
        <meshBasicMaterial color={color} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
    </group>
  );
}

// --- 2. Forward Range-Sensor Sweep: tied to the REAL sensor, not a
// decorative loop. Radius is scaled to app/services/traffic.py's actual
// SENSOR_MAX_RANGE_M (100m forward radar/LiDAR range, not omnidirectional
// -- this ring is a simplified 360-degree visualization of that forward
// cone, same honesty tradeoff as every other simplified-for-legibility
// element in this scene). When the sweep's expanding radius reaches the
// real sensed lead vehicle's real distance, it flashes and colour-codes
// by severity (same thresholds NpcVehicle's bounding box uses) instead of
// pulsing decoratively regardless of whether anything was ever detected.
const RADAR_MAX_RANGE_M = 100;
const RADAR_SWEEP_PERIOD_S = 3.0;
const RADAR_BASE_RING_RADIUS = 4;

function LidarRadarSweep() {
  const ringRef = useRef<THREE.Mesh>(null);
  const perception = useSimulationStore((state) => state.perception);
  const leadDetection = perception.find((p) => p.id === 'sensed_lead_vehicle');

  useFrame((state) => {
    if (!ringRef.current) return;
    ringRef.current.position.x = sharedVehicleState.x;
    ringRef.current.position.z = sharedVehicleState.z;

    const progress = (state.clock.elapsedTime % RADAR_SWEEP_PERIOD_S) / RADAR_SWEEP_PERIOD_S;
    const radiusM = progress * RADAR_MAX_RANGE_M;
    const scale = radiusM / RADAR_BASE_RING_RADIUS;
    ringRef.current.scale.set(scale, scale, 1);

    const material = ringRef.current.material as THREE.MeshBasicMaterial;
    if (leadDetection && leadDetection.distance <= radiusM) {
      const isCritical = leadDetection.distance < 15;
      const isClose = leadDetection.distance < 25;
      material.color.set(isCritical ? THEME.detectedCritical : isClose ? THEME.detectedWarning : THEME.detectedSafe);
      material.opacity = 0.55;
    } else {
      material.color.set(THEME.brandCyan);
      material.opacity = Math.max(0, 0.35 - progress * 0.3);
    }
  });

  return (
    <mesh ref={ringRef} position={[0, 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
      <ringGeometry args={[RADAR_BASE_RING_RADIUS, RADAR_BASE_RING_RADIUS * 1.075, 32]} />
      <meshBasicMaterial color={THEME.brandCyan} transparent opacity={0.35} side={THREE.DoubleSide} />
    </mesh>
  );
}

// --- 3. Ego Vehicle (Tesla/Waymo Sleek Model) ---
function EgoVehicle() {
  const ego = useSimulationStore((state) => state.ego);
  const routeGeom = useRouteGeometry();
  const groupRef = useRef<THREE.Group>(null);

  useFrame((_, delta) => {
    if (!groupRef.current || !ego) return;

    let targetX: number, targetZ: number, targetHeadingRad: number;
    if (routeGeom) {
      // Real world position: walk the actual route to the ego's real
      // station (frenet.s) and offset by its real lateral position
      // (frenet.d) -- this is what makes the car follow the road's real
      // curves instead of rendering (d, -s) directly as a flat corridor.
      const { pos, headingDeg } = getWorldPosAtFrenet(routeGeom, ego.frenet.s, ego.frenet.d);
      targetX = pos.x;
      targetZ = pos.z;
      targetHeadingRad = THREE.MathUtils.degToRad(headingDeg);
    } else {
      // No route yet (or none at all) -- Frenet passthrough fallback,
      // same as before route geometry existed.
      targetX = ego.pose?.x ?? 0;
      targetZ = ego.pose?.z ?? 0;
      targetHeadingRad = THREE.MathUtils.degToRad(ego.yaw ?? 0);
    }

    if (!sharedVehicleState.initialized) {
      groupRef.current.position.set(targetX, 0.4, targetZ);
      groupRef.current.rotation.y = -targetHeadingRad;
      sharedVehicleState.x = targetX;
      sharedVehicleState.z = targetZ;
      sharedVehicleState.headingRad = targetHeadingRad;
      sharedVehicleState.initialized = true;
      return;
    }

    const alpha = Math.min(1, delta * 14);
    groupRef.current.position.x = THREE.MathUtils.lerp(groupRef.current.position.x, targetX, alpha);
    groupRef.current.position.z = THREE.MathUtils.lerp(groupRef.current.position.z, targetZ, alpha);
    groupRef.current.position.y = 0.4;

    // Shortest-path heading lerp (through +/-180deg wrap), not a raw
    // rotation.y lerp -- a naive lerp spins the long way round whenever
    // the heading crosses the +/-180deg seam, which real routes with a
    // U-ish turn or a heading near due-south hit constantly.
    const currentHeading = -groupRef.current.rotation.y;
    let diff = targetHeadingRad - currentHeading;
    diff = ((diff + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
    const newHeading = currentHeading + diff * alpha;
    groupRef.current.rotation.y = -newHeading;

    sharedVehicleState.x = groupRef.current.position.x;
    sharedVehicleState.z = groupRef.current.position.z;
    sharedVehicleState.headingRad = newHeading;
  });

  return (
    <group ref={groupRef} position={[0, 0.4, 0]}>
      {/* Lower Body / Chassis */}
      <mesh position={[0, 0.3, 0]}>
        <boxGeometry args={[1.95, 0.5, 4.3]} />
        <meshStandardMaterial color={THEME.egoChassis} metalness={0.9} roughness={0.15} />
      </mesh>

      {/* Cyber Neon Trim */}
      <mesh position={[0, 0.08, 0]}>
        <boxGeometry args={[1.98, 0.06, 4.32]} />
        <meshBasicMaterial color={THEME.brandCyan} />
      </mesh>

      {/* Cabin Roof / Tinted Glass */}
      <mesh position={[0, 0.75, -0.2]}>
        <boxGeometry args={[1.5, 0.45, 2.3]} />
        <meshStandardMaterial color={THEME.egoRoof} roughness={0.1} metalness={0.9} transparent opacity={0.85} />
      </mesh>

      {/* Front Headlights (Facing -Z) */}
      <mesh position={[-0.72, 0.3, -2.16]}>
        <boxGeometry args={[0.32, 0.12, 0.04]} />
        <meshBasicMaterial color={THEME.headlightCyan} />
      </mesh>
      <mesh position={[0.72, 0.3, -2.16]}>
        <boxGeometry args={[0.32, 0.12, 0.04]} />
        <meshBasicMaterial color={THEME.headlightCyan} />
      </mesh>

      {/* Rear Taillights (Facing +Z) */}
      <mesh position={[-0.72, 0.35, 2.16]}>
        <boxGeometry args={[0.35, 0.1, 0.04]} />
        <meshBasicMaterial color={THEME.taillightRed} />
      </mesh>
      <mesh position={[0.72, 0.35, 2.16]}>
        <boxGeometry args={[0.35, 0.1, 0.04]} />
        <meshBasicMaterial color={THEME.taillightRed} />
      </mesh>

      {/* Waymo Sensor Dome */}
      <mesh position={[0, 1.05, -0.2]}>
        <cylinderGeometry args={[0.16, 0.16, 0.12, 16]} />
        <meshStandardMaterial color={THEME.brandCyan} emissive={THEME.brandCyanGlow} emissiveIntensity={0.8} />
      </mesh>

      {/* Dynamic Ego Perception Box */}
      <lineSegments geometry={UNIT_EDGES} scale={[2.15, 1.35, 4.5]} position={[0, 0.55, 0]}>
        <lineBasicMaterial color={THEME.brandCyan} />
      </lineSegments>
    </group>
  );
}

// traffic.py's VISIBILITY_WINDOW_M recycling relocates an NPC to a fresh
// station instantly (a real, correct backend behaviour -- it's how a long
// route stays populated). Naively lerping toward the new position instead
// slides the car across the map at high speed, which reads as a worse
// glitch than a clean teleport. Any single-frame jump bigger than a real
// car could ever cover is treated as a recycle: snap instantly, then fade
// the vehicle back in via opacity so it visibly "appears" rather than
// sliding or popping at full opacity.
const NPC_RECYCLE_JUMP_THRESHOLD_M = 60;
const NPC_FADE_IN_DURATION_S = 0.5;

// --- 4. Detected NPC Vehicles with Tesla Distance Badges & Color Coding ---
function NpcVehicle({ worldPos, worldHeadingRad, isDetected, perceptionInfo }: {
  worldPos: { x: number; z: number };
  worldHeadingRad: number;
  isDetected: boolean;
  perceptionInfo?: import('../../types/protocol').PerceptionObject;
}) {
  const groupRef = useRef<THREE.Group>(null);
  const lastWorldPos = useRef<{ x: number; z: number } | null>(null);
  const fadeElapsed = useRef(Infinity); // Infinity = fully faded in, not mid-fade
  const materials = useRef<Set<THREE.Material>>(new Set());

  const registerMaterial = (mat: THREE.Material | null) => {
    if (mat) materials.current.add(mat);
  };

  useFrame((_, delta) => {
    if (!groupRef.current) return;

    if (lastWorldPos.current) {
      const jump = Math.hypot(worldPos.x - lastWorldPos.current.x, worldPos.z - lastWorldPos.current.z);
      if (jump > NPC_RECYCLE_JUMP_THRESHOLD_M) {
        groupRef.current.position.set(worldPos.x, 0.4, worldPos.z);
        groupRef.current.rotation.y = -worldHeadingRad;
        fadeElapsed.current = 0;
      }
    }
    lastWorldPos.current = worldPos;

    const alpha = Math.min(1, delta * 12);
    groupRef.current.position.x = THREE.MathUtils.lerp(groupRef.current.position.x, worldPos.x, alpha);
    groupRef.current.position.z = THREE.MathUtils.lerp(groupRef.current.position.z, worldPos.z, alpha);
    groupRef.current.position.y = 0.4;
    groupRef.current.rotation.y = THREE.MathUtils.lerp(groupRef.current.rotation.y, -worldHeadingRad, alpha);

    if (fadeElapsed.current < NPC_FADE_IN_DURATION_S) {
      fadeElapsed.current += delta;
      const opacity = THREE.MathUtils.clamp(fadeElapsed.current / NPC_FADE_IN_DURATION_S, 0, 1);
      materials.current.forEach((mat) => {
        (mat as THREE.MeshStandardMaterial | THREE.MeshBasicMaterial).opacity = opacity;
      });
    }
  });

  const distance = perceptionInfo?.distance;
  const isClose = distance && distance < 25;
  const isCritical = distance && distance < 15;

  const boxColor = isCritical
    ? THEME.detectedCritical
    : isClose
    ? THEME.detectedWarning
    : isDetected
    ? THEME.detectedSafe
    : '#64748B';

  return (
    <group ref={groupRef} position={[worldPos.x, 0.4, worldPos.z]}>
      {/* Chassis */}
      <mesh position={[0, 0.3, 0]}>
        <boxGeometry args={[1.85, 0.5, 4.1]} />
        <meshStandardMaterial ref={registerMaterial} transparent color={THEME.npcChassis} metalness={0.5} roughness={0.4} />
      </mesh>

      {/* Cabin */}
      <mesh position={[0, 0.7, -0.1]}>
        <boxGeometry args={[1.4, 0.4, 2.1]} />
        <meshStandardMaterial ref={registerMaterial} transparent color={THEME.npcRoof} roughness={0.2} metalness={0.8} />
      </mesh>

      {/* Front Headlights */}
      <mesh position={[-0.65, 0.3, -2.06]}>
        <boxGeometry args={[0.28, 0.08, 0.04]} />
        <meshBasicMaterial ref={registerMaterial} transparent color="#FEF08A" />
      </mesh>
      <mesh position={[0.65, 0.3, -2.06]}>
        <boxGeometry args={[0.28, 0.08, 0.04]} />
        <meshBasicMaterial ref={registerMaterial} transparent color="#FEF08A" />
      </mesh>

      {/* Taillights */}
      <mesh position={[-0.65, 0.35, 2.06]}>
        <boxGeometry args={[0.3, 0.08, 0.04]} />
        <meshBasicMaterial ref={registerMaterial} transparent color={THEME.npcTaillight} />
      </mesh>
      <mesh position={[0.65, 0.35, 2.06]}>
        <boxGeometry args={[0.3, 0.08, 0.04]} />
        <meshBasicMaterial ref={registerMaterial} transparent color={THEME.npcTaillight} />
      </mesh>

      {/* Detected 3D Bounding Box */}
      <lineSegments geometry={UNIT_EDGES} scale={[2.05, 1.3, 4.3]} position={[0, 0.55, 0]}>
        <lineBasicMaterial ref={registerMaterial} transparent color={boxColor} />
      </lineSegments>

      {/* Tesla / Waymo Floating Distance Tag */}
      {isDetected && distance && (
        <Html position={[0, 1.8, 0]} center distanceFactor={18} zIndexRange={[100, 0]}>
          <div className="pointer-events-none select-none flex flex-col items-center">
            <div className={`px-2.5 py-1 rounded-md text-[11px] font-mono font-bold tracking-wider text-black shadow-lg flex items-center gap-1.5 whitespace-nowrap ${
              isCritical ? 'bg-[var(--critical)] text-white animate-pulse' : isClose ? 'bg-[var(--warning)]' : 'bg-[var(--brand)]'
            }`}>
              <CarFront className="h-3 w-3" aria-hidden="true" />
              <span>{distance.toFixed(1)}m</span>
              {perceptionInfo?.rel_velocity !== undefined && (
                <span className="text-[9px] opacity-80">
                  ({perceptionInfo.rel_velocity > 0 ? '+' : ''}{perceptionInfo.rel_velocity.toFixed(1)}m/s)
                </span>
              )}
            </div>
            <div className={`w-0 h-0 border-l-[4px] border-l-transparent border-r-[4px] border-r-transparent border-t-[4px] ${
              isCritical ? 'border-t-[var(--critical)]' : isClose ? 'border-t-[var(--warning)]' : 'border-t-[var(--brand)]'
            }`} />
          </div>
        </Html>
      )}
    </group>
  );
}

function TrafficManager() {
  const traffic = useSimulationStore((state) => state.traffic);
  const perception = useSimulationStore((state) => state.perception);
  const ego = useSimulationStore((state) => state.ego);
  const routeGeom = useRouteGeometry();
  const egoS = ego?.frenet?.s ?? 0;

  // Filter traffic within 200m of the ego ALONG THE ROUTE (Frenet station
  // distance) -- not raw world-Z distance, which stops being a meaningful
  // "distance ahead/behind" proxy once the road actually curves.
  const visibleTraffic = traffic.filter((npc) => Math.abs((npc.frenet?.s ?? 0) - egoS) < 220);

  const leadVehiclePerception = perception.find((p) => p.id === 'sensed_lead_vehicle');

  if (!routeGeom) return null;

  return (
    <group>
      {visibleTraffic.map((npc) => {
        const { pos, headingDeg } = getWorldPosAtFrenet(routeGeom, npc.frenet.s, npc.frenet.d);
        const isLead = leadVehiclePerception && Math.abs((npc.frenet?.s ?? 0) - (leadVehiclePerception.frenet?.s ?? 0)) < 10;
        return (
          <NpcVehicle
            key={npc.id}
            worldPos={{ x: pos.x, z: pos.z }}
            worldHeadingRad={THREE.MathUtils.degToRad(headingDeg)}
            isDetected={Boolean(isLead)}
            perceptionInfo={isLead ? leadVehiclePerception : undefined}
          />
        );
      })}
    </group>
  );
}

// --- 4b. Planner Candidate Paths (the Waymo rider-app signature): every
// lateral candidate app/services/planner.py::generate_candidates scored
// this tick, dimmed, with the REAL chosen one highlighted -- this data
// was already streamed (data.planner.candidates) but never drawn; only
// listed as text in Developer Mode. ---
function PlannerCandidatePaths() {
  const ego = useSimulationStore((state) => state.ego);
  const planner = useSimulationStore((state) => state.planner);
  const routeGeom = useRouteGeometry();

  if (!routeGeom || !ego || !planner || planner.candidates.length === 0) return null;

  const startS = ego.frenet.s;
  const startD = ego.frenet.d;

  return (
    <group>
      {planner.candidates.map((c, i) => {
        const points = sampleFrenetCorridor(routeGeom, startS, startD, c.d_target, CANDIDATE_PATH_HORIZON_M);
        if (c.is_chosen) {
          // The chosen path: bright, thick, and using the lane-change
          // colour when this tick's decision IS a real lane change
          // (app/services/planner.py's BLOCKED_LANE_PENALTY made an
          // in-lane option cost ~13x more before this candidate ever
          // wins -- it is a genuine decision, not a cosmetic default).
          const color = c.is_lane_change ? THEME.detectedWarning : THEME.teslaBlueRibbon;
          return (
            <Line
              key={`candidate-${i}`}
              points={points}
              color={color}
              lineWidth={5}
              transparent
              opacity={0.95}
            />
          );
        }
        return (
          <Line
            key={`candidate-${i}`}
            points={points}
            color={THEME.laneWhite}
            lineWidth={1.5}
            transparent
            opacity={0.22}
          />
        );
      })}
    </group>
  );
}

// --- 4c. Predictive Ribbons (Phase 7): the real per-agent trajectory
// forecast (app/services/prediction/), coloured by dominant intent. The
// forecast trail points are already in the same local (x, z) metric frame
// this scene renders in (frenet_to_local_xz on the backend === toLocalXZ
// here, same route origin), so they plot directly -- no ego-relative
// transform. ---
const INTENT_COLOR: Record<string, string> = {
  LANE_KEEP: THEME.detectedSafe,
  MERGE_LEFT: THEME.detectedWarning,
  MERGE_RIGHT: THEME.detectedWarning,
  DECELERATING: THEME.laneYellow,
  STOPPING: THEME.detectedCritical,
};

function PredictedAgentRibbons() {
  const prediction = useSimulationStore((state) => state.prediction);
  if (!prediction || prediction.agents.length === 0) return null;

  return (
    <group>
      {prediction.agents.map((agent) => {
        if (!agent.trail || agent.trail.length < 2) return null;
        const points = agent.trail.map((p) => new THREE.Vector3(p.x, 0.16, p.z));
        const dominant = agent.intent[0]?.label ?? 'LANE_KEEP';
        const color = INTENT_COLOR[dominant] ?? THEME.npcTaillight;
        const isCutInAgent = prediction.cut_in.active && prediction.cut_in.track_id === agent.track_id;
        return (
          <Line
            key={`forecast-${agent.track_id}`}
            points={points}
            color={color}
            lineWidth={isCutInAgent ? 4.5 : 2.5}
            transparent
            opacity={isCutInAgent ? 0.95 : 0.5}
            dashed={!isCutInAgent}
            dashSize={0.9}
            gapSize={0.5}
          />
        );
      })}
    </group>
  );
}

// --- 4c-ii. Predictive risk tint: a soft amber ground halo under the ego
// while the prediction stage is running its comfort-bounded proactive
// slowdown for a developing cut-in (speed_limit_reason "predictive_cut_in").
// Distinct from ShieldOverrideIndicator's red emergency halo -- this fires
// EARLIER and is not an override. ---
function PredictiveRiskTint() {
  const prediction = useSimulationStore((state) => state.prediction);
  const ringRef = useRef<THREE.Mesh>(null);
  const active = !!prediction && prediction.cut_in.active;

  useFrame((state) => {
    if (!ringRef.current) return;
    ringRef.current.visible = active;
    if (!active) return;
    ringRef.current.position.x = sharedVehicleState.x;
    ringRef.current.position.z = sharedVehicleState.z;
    const pulse = (Math.sin(state.clock.elapsedTime * 5) + 1) / 2;
    const p = prediction?.cut_in.probability ?? 0;
    (ringRef.current.material as THREE.MeshBasicMaterial).opacity = 0.14 + pulse * 0.12 + p * 0.14;
  });

  return (
    <mesh ref={ringRef} position={[0, 0.025, 0]} rotation={[-Math.PI / 2, 0, 0]} visible={false}>
      <ringGeometry args={[3.4, 9.0, 48]} />
      <meshBasicMaterial color={THEME.detectedWarning} transparent opacity={0.2} side={THREE.DoubleSide} />
    </mesh>
  );
}

// --- 4d. Safety Shield Override Indicator: a pulsing warning halo under
// the ego, visible directly in the 3D scene (not just the HUD panel),
// when app/services/safety_shield.py's independent check has overridden
// the planner/IDM decision. ---
function ShieldOverrideIndicator() {
  const safetyShield = useSimulationStore((state) => state.safetyShield);
  const ringRef = useRef<THREE.Mesh>(null);
  const overridden = !!safetyShield && !safetyShield.approved;

  useFrame((state) => {
    if (!ringRef.current) return;
    ringRef.current.visible = overridden;
    if (!overridden) return;
    ringRef.current.position.x = sharedVehicleState.x;
    ringRef.current.position.z = sharedVehicleState.z;
    const pulse = (Math.sin(state.clock.elapsedTime * 10) + 1) / 2;
    const scale = 1.3 + pulse * 0.3;
    ringRef.current.scale.set(scale, scale, 1);
    (ringRef.current.material as THREE.MeshBasicMaterial).opacity = 0.35 + pulse * 0.35;
  });

  return (
    <mesh ref={ringRef} position={[0, 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]} visible={false}>
      <ringGeometry args={[2.6, 3.1, 32]} />
      <meshBasicMaterial color={THEME.detectedCritical} transparent opacity={0.5} side={THREE.DoubleSide} />
    </mesh>
  );
}

// --- 5. Highway Road: real route geometry when available, generic
// infinite strip as a fallback before any route has been fetched. ---
function HighwayRoad() {
  const routeGeom = useRouteGeometry();
  const ribbon = useMemo(() => (routeGeom ? buildRoadRibbon(routeGeom) : null), [routeGeom]);

  // Both hooks called unconditionally every render (Rules of Hooks) --
  // the fallback branch below is selected in the JSX, not via an early
  // return before these run.
  const roadMesh = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!ribbon && roadMesh.current) roadMesh.current.position.z = sharedVehicleState.z;
  });

  if (ribbon) {
    return (
      <group>
        <mesh geometry={ribbon.roadGeometry}>
          <meshStandardMaterial color={THEME.roadSurface} roughness={0.95} />
        </mesh>
        <Line points={ribbon.centerLine} color={THEME.laneYellow} lineWidth={2} dashed dashSize={2} gapSize={2} />
        <Line points={ribbon.leftBound} color={THEME.laneCyan} lineWidth={2} />
        <Line points={ribbon.rightBound} color={THEME.laneCyan} lineWidth={2} />
        <Line points={ribbon.leftLane} color={THEME.laneWhite} lineWidth={1} transparent opacity={0.6} dashed dashSize={1.5} gapSize={1.5} />
        <Line points={ribbon.rightLane} color={THEME.laneWhite} lineWidth={1} transparent opacity={0.6} dashed dashSize={1.5} gapSize={1.5} />
      </group>
    );
  }

  // Fallback: no route fetched yet -- a generic straight strip under the
  // ego so the scene isn't bare while the async route fetch is in flight.
  return (
    <group ref={roadMesh}>
      <mesh position={[0, 0, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[16, 800]} />
        <meshStandardMaterial color={THEME.roadSurface} roughness={0.95} />
      </mesh>
    </group>
  );
}

// --- 5b. Surround perception tracks (heavy.surround_perception): the
// ego's own 360-degree sensor-resolved picture -- confirmed tracks only.
// Each track carries a *semantic role* (lib/roles.ts) derived from real
// state (the sensor's own lead call, the Phase 7 cut_in.track_id, the
// track's own class/kinematics) and a *relevance tier* that drives a
// three-level visual hierarchy: primary tracks get a solid box + full
// role card + ground ring; ambient tracks are a faint wire outline only.
// Same honesty boundary as the rest of the scene -- no map, no crosswalks,
// no signals; a track that matches no rule is just "tracked". ---
const ROLE_COLOR: Record<TrackRole, string> = {
  'cut-in': THEME.detectedCritical,
  lead: THEME.brandCyan,
  oncoming: '#A78BFA',
  adjacent: THEME.detectedWarning,
  pedestrian: THEME.detectedWarning,
  cyclist: THEME.detectedWarning,
  static: '#64748B',
  tracked: '#64748B',
};

function SurroundTrackBox({ a }: { a: AnnotatedTrack }) {
  const groupRef = useRef<THREE.Group>(null);
  const { track, role, relevance, closingMps, speedMps } = a;
  const [len, wid, hei] = track.dims;
  const color = ROLE_COLOR[role];
  const heading = Math.atan2(track.vx, -track.vz);

  const boxOpacity = relevance === 'primary' ? 0.92 : relevance === 'secondary' ? 0.5 : 0.2;

  useFrame((_, delta) => {
    if (!groupRef.current) return;
    const k = Math.min(1, delta * 12);
    groupRef.current.position.x = THREE.MathUtils.lerp(groupRef.current.position.x, track.x, k);
    groupRef.current.position.z = THREE.MathUtils.lerp(groupRef.current.position.z, track.z, k);
    groupRef.current.rotation.y = THREE.MathUtils.lerp(groupRef.current.rotation.y, -heading, k);
  });

  return (
    <group ref={groupRef} position={[track.x, 0.02, track.z]}>
      <lineSegments geometry={UNIT_EDGES} scale={[wid, hei, len]} position={[0, hei / 2, 0]}>
        <lineBasicMaterial color={color} transparent opacity={boxOpacity} />
      </lineSegments>

      {relevance === 'primary' && (
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
          <ringGeometry args={[Math.max(wid, len) * 0.62, Math.max(wid, len) * 0.7, 24]} />
          <meshBasicMaterial color={color} transparent opacity={0.4} side={THREE.DoubleSide} />
        </mesh>
      )}

      {relevance !== 'ambient' && (
        <Html position={[0, hei + 0.5, 0]} center distanceFactor={relevance === 'primary' ? 22 : 16} zIndexRange={[90, 0]}>
          <div className="pointer-events-none select-none flex flex-col items-center">
            <div
              className="px-2 py-0.5 rounded text-[10px] font-mono tracking-wide whitespace-nowrap border"
              style={{
                color: 'var(--text-bright)',
                background: 'var(--bg-frost)',
                borderColor: color,
                backdropFilter: 'blur(4px)',
              }}
            >
              <span style={{ color, fontWeight: 600 }}>{ROLE_LABEL[role]}</span>
              <span style={{ color: 'var(--text-faint)' }}> · {track.range_m.toFixed(0)}m</span>
              {relevance === 'primary' && (
                <>
                  {Number.isFinite(closingMps) && (
                    <span style={{ opacity: 0.75 }}>
                      {' · '}
                      {closingMps > 0 ? '▼' : '▲'}
                      {Math.abs(closingMps).toFixed(1)}
                    </span>
                  )}
                  <span style={{ opacity: 0.5 }}> · {(speedMps * 3.6).toFixed(0)}km/h</span>
                </>
              )}
            </div>
          </div>
        </Html>
      )}
    </group>
  );
}

function SurroundPerceptionLayer() {
  const tracks = useSimulationStore((state) => state.surroundPerception);
  const prediction = useSimulationStore((state) => state.prediction);
  const perception = useSimulationStore((state) => state.perception);
  const planner = useSimulationStore((state) => state.planner);
  const ego = useSimulationStore((state) => state.ego);

  const annotated = useMemo(() => {
    if (!tracks || tracks.length === 0) return [];
    return annotateTracks(tracks, {
      egoHeadingRad: sharedVehicleState.headingRad,
      prediction,
      perception,
      egoLateralM: ego?.frenet?.d ?? 0,
      laneChanging: Boolean(planner?.is_changing_lane),
    });
  }, [tracks, prediction, perception, planner, ego]);

  if (annotated.length === 0) return null;
  return (
    <group>
      {annotated.map((a) => (
        <SurroundTrackBox key={a.track.id} a={a} />
      ))}
    </group>
  );
}

// --- 6. Smooth Tesla-Style Chase Camera ---
function SynchronizedChaseCamera() {
  const cameraInitialized = useRef(false);

  useFrame((state, delta) => {
    if (!sharedVehicleState.initialized) return;

    const forward = headingToForward(sharedVehicleState.headingRad);
    const vX = sharedVehicleState.x;
    const vZ = sharedVehicleState.z;

    // Chase from behind along the vehicle's REAL forward direction (not a
    // fixed world-Z offset, which only looked right when every route
    // rendered as a straight corridor along Z).
    const camDist = 13.5;
    const camHeight = 5.6;
    const lookAheadDist = 18.0;

    const targetCamPos = new THREE.Vector3(
      vX - forward.x * camDist,
      camHeight,
      vZ - forward.z * camDist,
    );
    const targetLookAt = new THREE.Vector3(
      vX + forward.x * lookAheadDist,
      0.9,
      vZ + forward.z * lookAheadDist,
    );

    if (!cameraInitialized.current) {
      state.camera.position.copy(targetCamPos);
      state.camera.lookAt(targetLookAt);
      cameraInitialized.current = true;
      return;
    }

    const alpha = Math.min(1, delta * 6);
    state.camera.position.lerp(targetCamPos, alpha);
    state.camera.lookAt(targetLookAt);
  });

  return null;
}

export function SimulationScene() {
  return (
    <div className="absolute inset-0 w-full h-full z-0 overflow-hidden">
      <Canvas
        gl={{ antialias: true, alpha: false }}
        onCreated={({ gl }) => {
          gl.setClearColor(THEME.roadSurface);
        }}
      >
        <PerspectiveCamera makeDefault position={[0, 5.6, 13.5]} fov={45} near={0.1} far={800} />

        {/* Ambient & Studio Lights */}
        <ambientLight intensity={0.65} />
        <directionalLight position={[20, 35, 20]} intensity={1.1} color="#E0F2FE" />
        <pointLight position={[0, 8, 0]} intensity={0.4} color={THEME.brandCyan} />

        {/* Dark Grid Background */}
        <Grid
          position={[0, -0.02, 0]}
          infiniteGrid
          fadeDistance={250}
          fadeStrength={1.5}
          sectionColor={THEME.gridSection}
          cellColor={THEME.gridCell}
          cellSize={2}
          sectionSize={20}
        />

        {/* Real Route Road (or a generic fallback until one is fetched) */}
        <HighwayRoad />

        {/* Planned-path corridor — chosen lateral plan through the real
            road geometry, constraint-coloured, crossfaded on change. */}
        <PlannedCorridor />

        {/* Waymo Radar Sweep */}
        <LidarRadarSweep />

        {/* Real path-planning visualisation: every scored candidate,
            dimmed, with the actual chosen path highlighted -- the
            rider-app signature this phase is named for. */}
        <PlannerCandidatePaths />
        <PredictedAgentRibbons />
        <SurroundPerceptionLayer />
        <PredictiveRiskTint />
        <ShieldOverrideIndicator />

        {/* Dynamic Vehicles */}
        <EgoVehicle />
        <TrafficManager />
        <SynchronizedChaseCamera />
      </Canvas>
    </div>
  );
}
