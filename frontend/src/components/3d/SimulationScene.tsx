'use client';

import React, { useRef, useMemo, useEffect, useState } from 'react';
import { routeRenderWindow } from '../../lib/routeRenderWindow';
import { runtimeCapture, epochNow } from '../../lib/runtimeMetrics';
import { Canvas, useFrame } from '@react-three/fiber';
import { PerspectiveCamera, Grid, Html, Line, RoundedBox } from '@react-three/drei';
import { useSimulationStore } from '../../store/useSimulationStore';
import * as THREE from 'three';
import { CarFront } from 'lucide-react';
import {
  RouteGeometry,
  buildRouteGeometry,
  buildRoadRibbon,
  roadsidePosition,
  getWorldPosAtFrenet,
  sampleFrenetCorridor,
  toLocalXZ,
} from '../../lib/routeGeometry';
import { annotateTracks, type AnnotatedTrack, type TrackRole } from '../../lib/roles';
import { CameraRig } from './CameraRig';
import { SurroundTrackLabel } from './SurroundTrackLabel';
import { SceneControlCamera } from './SceneControlCamera';
import type { SceneControl } from '../../types/protocol';
import { surroundLabelText } from '../../lib/surroundLabel';
import {MappedEnvironment,SimulatedRain,useMappedRoadContext} from './MappedEnvironment';

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
  brandCyan: '#4D8C7B',
  brandCyanGlow: '#559582',
  teslaBlueRibbon: '#6C9E94',
  egoChassis: '#F4F6F3',
  egoRoof: '#607875',
  headlightCyan: '#A5F3FC',
  taillightRed: '#FF2A4D',
  npcChassis: '#B7C2BB',
  npcRoof: '#7F918B',
  npcTaillight: '#F87171',
  detectedSafe: '#10B981',
  detectedWarning: '#F59E0B',
  detectedCritical: '#EF4444',
  bgApp: '#E9EDE8',
  roadSurface: '#D1D9D3',
  laneWhite: '#F7FAF5',
  laneYellow: '#A7B5A3',
  laneCyan: '#B3C1B4',
  gridSection: '#DFE5DD',
  gridCell: '#E4E9E1',
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

/** Allocation-free variant for hot loops. */
function headingToForwardInto(out: THREE.Vector3, headingRad: number): THREE.Vector3 {
  return out.set(Math.sin(headingRad), 0, -Math.cos(headingRad));
}

// Real vehicles don't pivot on the spot. Ease `current` heading (rad)
// toward `target` along the SHORTEST arc, but never faster than a real
// car's yaw rate -- this is what stops NPCs "spinning like toys" when a
// station step lands them a big heading delta (route recycles, residual
// corner kinks, the +/-180 deg seam).
const MAX_YAW_RATE = 1.6; // rad/s (~92 deg/s) — covers hard cornering, kills spins
function approachHeading(current: number, target: number, alpha: number, dt: number): number {
  let diff = target - current;
  diff = ((diff + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  const eased = diff * alpha;
  const cap = MAX_YAW_RATE * Math.max(dt, 1 / 120);
  return current + Math.max(-cap, Math.min(cap, eased));
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

  const reason = ego?.speed_limit_reason ?? 'cruise';
  const color = CORRIDOR_COLOR[reason] ?? THEME.teslaBlueRibbon;

  // Rebuild only when the plan actually moves (quantised signature).
  const sig = JSON.stringify(planner?.trajectory ?? []);

  const corridorData = useMemo(() => {
    if (!ego || !routeGeom || !planner || planner.trajectory.length < 2) return null;
    const pts = planner.trajectory.map(p => {
      const world = getWorldPosAtFrenet(routeGeom, -p.z, p.x);
      return new THREE.Vector3(world.pos.x, .15, world.pos.z);
    });
    return { ribbon: buildRibbonGeometry(pts), points: pts };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sig, routeGeom]);

  // Crossfade handled entirely in the three layer: two persistent meshes,
  // geometry swapped on the inactive one in an effect, opacity driven in
  // useFrame. No React state / no ref reads during render.
  const meshA = useRef<THREE.Mesh>(null);
  const meshB = useRef<THREE.Mesh>(null);
  const active = useRef(0); // which mesh currently shows the live plan
  const fade = useRef(1);

  useEffect(() => {
    if (!corridorData) return;
    const incoming = active.current === 0 ? meshB.current : meshA.current;
    const outgoing = active.current === 0 ? meshA.current : meshB.current;
    if (!incoming || !outgoing) return;
    const stale = incoming.geometry;
    incoming.geometry = corridorData.ribbon;
    if (stale && stale !== corridorData.ribbon && stale !== EMPTY_GEOM) stale.dispose();
    active.current = active.current === 0 ? 1 : 0;
    fade.current = 0;
  }, [corridorData]);

  useFrame((_, delta) => {
    fade.current = Math.min(1, fade.current + delta * 3.5);
    const inM = (active.current === 0 ? meshA.current : meshB.current);
    const outM = (active.current === 0 ? meshB.current : meshA.current);
    if (inM) (inM.material as THREE.MeshBasicMaterial).opacity = 0.42 * fade.current + 0.18;
    if (outM) {
      (outM.material as THREE.MeshBasicMaterial).opacity = 0.42 * (1 - fade.current);
      outM.visible = fade.current < 1;
    }
  });

  if (!corridorData) return null;
  return (
    <group>
      <mesh ref={meshA} geometry={EMPTY_GEOM}>
        <meshBasicMaterial color={color} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      <mesh ref={meshB} geometry={EMPTY_GEOM}>
        <meshBasicMaterial color={color} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      {/* High-visibility center trajectory spine line */}
      {corridorData.points.length >= 2 && (
        <Line
          points={corridorData.points}
          color={color}
          lineWidth={3.5}
          transparent
          opacity={0.92}
        />
      )}
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
    // §18 — no decorative constant animation: the sweep only runs while
    // the forward sensor actually has a detection to resolve.
    ringRef.current.visible = !!leadDetection;
    if (!leadDetection) return;
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
    <mesh ref={ringRef} position={[0, 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]} visible={false}>
      <ringGeometry args={[RADAR_BASE_RING_RADIUS, RADAR_BASE_RING_RADIUS * 1.075, 32]} />
      <meshBasicMaterial color={THEME.brandCyan} transparent opacity={0.35} side={THREE.DoubleSide} />
    </mesh>
  );
}

// Wheel pivots and rolling axles are separate: steering never rolls the tire.
function VehicleWheel({ pivotRef, rollingRef, x, z }: {
  pivotRef?: React.RefObject<THREE.Group | null>;
  rollingRef: React.RefObject<THREE.Group | null>; x: number; z: number;
}) {
  const wheelName = `${z < 0 ? 'front' : 'rear'}-${x < 0 ? 'left' : 'right'}`;
  return <group name={`ego-wheel-${wheelName}`} ref={pivotRef} position={[x, -0.04, z]}>
    <group name={`ego-roll-${wheelName}`} ref={rollingRef}>
      <mesh rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.34, 0.34, 0.22, 20]} />
        <meshStandardMaterial color="#26322F" roughness={0.85} />
      </mesh>
      <mesh rotation={[0, 0, Math.PI / 2]} position={[Math.sign(x) * 0.12, 0, 0]}>
        <cylinderGeometry args={[0.21, 0.21, 0.02, 16]} />
        <meshStandardMaterial color="#58645F" metalness={0.65} roughness={0.35} />
      </mesh>
      {[0, 1, 2, 3, 4].map(i => <mesh key={i}
        position={[Math.sign(x) * 0.135, 0, 0]} rotation={[i * Math.PI * 2 / 5, 0, 0]}>
        <boxGeometry args={[0.025, 0.035, 0.36]} />
        <meshStandardMaterial color="#C4CEC8" metalness={0.65} roughness={0.3} />
      </mesh>)}
    </group>
  </group>;
}

function IntentLights(){
 const left=useRef<THREE.Group>(null),right=useRef<THREE.Group>(null),brake=useRef<THREE.Group>(null);
 useFrame(()=>{const s=useSimulationStore.getState();const signals=(s.ego as typeof s.ego & {signals?:{left:boolean;right:boolean;hazards:boolean;brake:boolean}})?.signals;const on=Math.floor(s.simulationTime*3)%2===0;if(left.current)left.current.visible=Boolean(signals&&(signals.left||signals.hazards)&&on);if(right.current)right.current.visible=Boolean(signals&&(signals.right||signals.hazards)&&on);if(brake.current)brake.current.visible=signals?.brake??false;});
 return <group name="actual-planner-intent-lamps"><group ref={left} visible={false}>{[-2.18,2.18].map(z=><mesh key={z} position={[-.87,.35,z]}><boxGeometry args={[.17,.14,.06]}/><meshBasicMaterial color="#ffad24" toneMapped={false}/></mesh>)}</group><group ref={right} visible={false}>{[-2.18,2.18].map(z=><mesh key={z} position={[.87,.35,z]}><boxGeometry args={[.17,.14,.06]}/><meshBasicMaterial color="#ffad24" toneMapped={false}/></mesh>)}</group><group ref={brake} visible={false}>{[-.6,.6].map(x=><mesh key={x} position={[x,.35,2.19]}><boxGeometry args={[.4,.12,.05]}/><meshBasicMaterial color="#ff243b" toneMapped={false}/></mesh>)}</group></group>;
}

// --- 3. Ego Vehicle (Tesla/Waymo Sleek Model with Steerable Wheels & Dynamic Roll) ---
function EgoVehicle() {
  const detailed = true;
  const ego = useSimulationStore((state) => state.ego);
  const routeGeom = useRouteGeometry();
  const groupRef = useRef<THREE.Group>(null);
  const chassisRef = useRef<THREE.Group>(null);
  const frontLeftWheelRef = useRef<THREE.Group>(null);
  const frontRightWheelRef = useRef<THREE.Group>(null);
  const frontLeftRollRef = useRef<THREE.Group>(null);
  const frontRightRollRef = useRef<THREE.Group>(null);
  const rearLeftRollRef = useRef<THREE.Group>(null);
  const rearRightRollRef = useRef<THREE.Group>(null);
  const poseEpoch = useRef<string | null>(null);

  useFrame((_, delta) => {
    if (!groupRef.current || !ego) return;

    let targetX: number, targetZ: number, targetHeadingRad: number;
    if (routeGeom) {
      const { pos, headingDeg } = getWorldPosAtFrenet(routeGeom, ego.frenet.s, ego.frenet.d);
      // Show actual simulated motion even when it departs from the route.
      const wp = ego.world_pos;
      const wpUsable = wp != null && Number.isFinite(wp.x) && Number.isFinite(wp.z);
      targetX = wpUsable ? wp.x : pos.x;
      targetZ = wpUsable ? wp.z : pos.z;
      // Authoritative heading: backend world heading, else road tangent + relative.
      const relativeYaw = ego.relative_heading_deg ?? ego.yaw ?? 0;
      const totalHeadingDeg =
        ego.world_heading_deg !== undefined ? ego.world_heading_deg : headingDeg + relativeYaw;
      targetHeadingRad = THREE.MathUtils.degToRad(totalHeadingDeg);
    } else {
      // No route yet: fallback to pose / yaw
      targetX = ego.pose?.x ?? 0;
      targetZ = ego.pose?.z ?? 0;
      targetHeadingRad = THREE.MathUtils.degToRad(ego.world_heading_deg ?? ego.yaw ?? 0);
    }

    const store = useSimulationStore.getState();
    const epochKey = `${store.simulationId}:${store.epoch}`;
    const resetPose = poseEpoch.current !== epochKey || !sharedVehicleState.initialized;
    poseEpoch.current = epochKey;
    if (resetPose) {
      groupRef.current.position.set(targetX, 0.4, targetZ);
      groupRef.current.rotation.y = -targetHeadingRad;
      sharedVehicleState.x = targetX;
      sharedVehicleState.z = targetZ;
      sharedVehicleState.headingRad = targetHeadingRad;
      sharedVehicleState.initialized = true;
    }

    const alpha = resetPose ? 1 : 1 - Math.exp(-delta * 14);
    groupRef.current.position.x = THREE.MathUtils.lerp(groupRef.current.position.x, targetX, alpha);
    groupRef.current.position.z = THREE.MathUtils.lerp(groupRef.current.position.z, targetZ, alpha);
    groupRef.current.position.y = 0.4;

    // Shortest-arc, yaw-rate-capped heading (no long-way-round spins)
    const newHeading = approachHeading(-groupRef.current.rotation.y, targetHeadingRad, alpha, delta);
    groupRef.current.rotation.y = -newHeading;

    // Telemetry owns wheel geometry and distance-integrated roll. Compass
    // clockwise-positive yaw is negative Y rotation in this Three.js scene.
    const angles = ego.wheels?.steering_rad ?? [ego.steering_angle_rad ?? 0, ego.steering_angle_rad ?? 0, 0, 0];
    if (frontLeftWheelRef.current) frontLeftWheelRef.current.rotation.y = THREE.MathUtils.lerp(frontLeftWheelRef.current.rotation.y, -angles[0], alpha);
    if (frontRightWheelRef.current) frontRightWheelRef.current.rotation.y = THREE.MathUtils.lerp(frontRightWheelRef.current.rotation.y, -angles[1], alpha);
    const rolls = [frontLeftRollRef, frontRightRollRef, rearLeftRollRef, rearRightRollRef];
    for (let i = 0; i < rolls.length; i++) {
      const wheel = rolls[i].current;
      // Cumulative phase cannot choose the wrong arc at high road speed.
      if (wheel) wheel.rotation.x = THREE.MathUtils.lerp(wheel.rotation.x, -(ego.wheels?.rotation_rad[i] ?? 0), alpha);
    }

    // Dynamic body roll (Z-rotation) and pitch (X-rotation)
    if (chassisRef.current) {
      const signedLatAccel = ego.signed_lateral_accel_mps2 ?? 0;
      // Right-turn acceleration leans the body outward (toward local -X).
      const targetRoll = THREE.MathUtils.clamp(signedLatAccel * 0.015, -0.08, 0.08);
      const accel = ego.acceleration ?? 0;
      const targetPitch = THREE.MathUtils.clamp(-accel * 0.012, -0.06, 0.06);

      chassisRef.current.rotation.z = THREE.MathUtils.lerp(chassisRef.current.rotation.z, targetRoll, alpha);
      chassisRef.current.rotation.x = THREE.MathUtils.lerp(chassisRef.current.rotation.x, targetPitch, alpha);
    }

    sharedVehicleState.x = groupRef.current.position.x;
    sharedVehicleState.z = groupRef.current.position.z;
    sharedVehicleState.headingRad = newHeading;
  });

  return (
    <group name="ego-vehicle" ref={groupRef} position={[0, 0.4, 0]}>
      {/* Ground Contact Shadow (anchors vehicle to the road surface) */}
      <mesh position={[0, -0.38, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[2.3, 4.8]} />
        <meshBasicMaterial color="#405544" transparent opacity={0.12} depthWrite={false} />
      </mesh>

      {/* Subtle Projected Forward Headlight Beams */}
      <mesh position={[0, -0.37, -5.5]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[3.2, 8.5]} />
        <meshBasicMaterial color={THEME.headlightCyan} transparent opacity={0.09} depthWrite={false} />
      </mesh>

      <IntentLights />
      {/* Dynamic Chassis Group (Subject to Roll & Pitch) */}
      <group ref={chassisRef}>
        {/* Lower Body / Chassis */}
        <RoundedBox position={[0, 0.3, 0]} args={[1.95, 0.5, 4.3]} radius={0.18} smoothness={3}>
          <meshStandardMaterial color={THEME.egoChassis} metalness={0.12} roughness={0.45} />
        </RoundedBox>

        {/* Lower body accent */}
        <mesh position={[0, 0.08, 0]}>
          <boxGeometry args={[1.98, 0.06, 4.32]} />
          <meshBasicMaterial color={THEME.brandCyan} />
        </mesh>

        {/* Cabin Roof / Tinted Glass */}
        <RoundedBox position={[0, 0.75, -0.2]} args={[1.5, 0.45, 2.3]} radius={0.15} smoothness={3}>
          <meshStandardMaterial color={THEME.egoRoof} roughness={0.3} metalness={0.12} transparent opacity={0.9} />
        </RoundedBox>

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
        {detailed && <lineSegments geometry={UNIT_EDGES} scale={[2.15, 1.35, 4.5]} position={[0, 0.55, 0]}>
          <lineBasicMaterial color={THEME.brandCyan} />
        </lineSegments>}
      </group>

      <VehicleWheel pivotRef={frontLeftWheelRef} rollingRef={frontLeftRollRef} x={-0.98} z={-1.4} />
      <VehicleWheel pivotRef={frontRightWheelRef} rollingRef={frontRightRollRef} x={0.98} z={-1.4} />
      <VehicleWheel rollingRef={rearLeftRollRef} x={-0.98} z={1.4} />
      <VehicleWheel rollingRef={rearRightRollRef} x={0.98} z={1.4} />
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
  const detailed = true;
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
    groupRef.current.rotation.y = -approachHeading(-groupRef.current.rotation.y, worldHeadingRad, alpha, delta);

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
    <group ref={groupRef} position={[worldPos.x, 0.4, worldPos.z]} userData={{cameraPose:{x:worldPos.x,z:worldPos.z,heading:worldHeadingRad}}}>
      {/* NPC Ground Contact Shadow */}
      <mesh position={[0, -0.38, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[2.2, 4.4]} />
        <meshBasicMaterial ref={registerMaterial} color="#405544" transparent opacity={0.10} depthWrite={false} />
      </mesh>

      {/* Chassis */}
      <mesh position={[0, 0.3, 0]}>
        <boxGeometry args={[1.85, 0.5, 4.1]} />
        <meshStandardMaterial ref={registerMaterial} transparent color={THEME.npcChassis} metalness={0.12} roughness={0.5} />
      </mesh>

      {/* Cabin */}
      <mesh position={[0, 0.7, -0.1]}>
        <boxGeometry args={[1.4, 0.4, 2.1]} />
        <meshStandardMaterial ref={registerMaterial} transparent color={THEME.npcRoof} roughness={0.4} metalness={0.12} />
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
      {detailed && <lineSegments geometry={UNIT_EDGES} scale={[2.05, 1.3, 4.3]} position={[0, 0.55, 0]}>
        <lineBasicMaterial ref={registerMaterial} transparent color={boxColor} />
      </lineSegments>}

      {/* Nearby distance labels are available in the surroundings detail view. */}
      {detailed && isDetected && distance && (
        <Html eps={0.5} position={[0, 1.8, 0]} center distanceFactor={18} zIndexRange={[100, 0]}>
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
  const cameraTick = useSimulationStore(s=>s.tick);
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
    <group name="camera-traffic" userData={{cameraTick}}>
      {visibleTraffic.map((npc) => {
        const { pos, headingDeg } = getWorldPosAtFrenet(routeGeom, npc.frenet.s, npc.frenet.d);
        const isLead = leadVehiclePerception && Math.abs((npc.frenet?.s ?? 0) - (leadVehiclePerception.frenet?.s ?? 0)) < 10;
        const relativeYaw = npc.relative_heading_deg ?? npc.yaw ?? 0;
        const totalHeadingDeg = headingDeg + relativeYaw;
        return (
          <NpcVehicle
            key={npc.id}
            worldPos={{ x: pos.x, z: pos.z }}
            worldHeadingRad={THREE.MathUtils.degToRad(totalHeadingDeg)}
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

  // Build the Vector3 trails once per prediction update, not every render.
  const lines = useMemo(() => {
    if (!prediction || prediction.agents.length === 0) return [];
    return prediction.agents
      .filter((agent) => agent.trail && agent.trail.length >= 2)
      .map((agent) => ({
        key: agent.track_id,
        points: agent.trail.map((p) => new THREE.Vector3(p.x, 0.16, p.z)),
        color: INTENT_COLOR[agent.intent[0]?.label ?? 'LANE_KEEP'] ?? THEME.npcTaillight,
        isCutIn: prediction.cut_in.active && prediction.cut_in.track_id === agent.track_id,
      }));
  }, [prediction]);

  if (lines.length === 0) return null;

  return (
    <group>
      {lines.map(({ key, points, color, isCutIn }) => (
        <Line
          key={`forecast-${key}`}
          points={points}
          color={color}
          lineWidth={isCutIn ? 4.5 : 2.5}
          transparent
          opacity={isCutIn ? 0.95 : 0.5}
          dashed={!isCutIn}
          dashSize={0.9}
          gapSize={0.5}
        />
      ))}
    </group>
  );
}

// --- 4c-ii. Spatial risk region (§10): instead of washing the whole
// ground, place a localized patch *between the ego and the actual threat*
// and stretch it along that line. The threat is the real cut-in agent
// (Phase 7 cut_in.track_id) or, failing that, the nearest closing track
// while the safety shield reports MEDIUM+ risk. Intensity scales with
// cut_in.probability / risk_level. Silent when risk is NONE. ---
const RISK_RANK: Record<string, number> = { NONE: 0, LOW: 1, MEDIUM: 2, HIGH: 3, CRITICAL: 4 };

function SpatialRiskRegion() {
  const prediction = useSimulationStore((s) => s.prediction);
  const shield = useSimulationStore((s) => s.safetyShield);
  const tracks = useSimulationStore((s) => s.surroundPerception);

  const patchRef = useRef<THREE.Mesh>(null);

  // Resolve the threat position (local x/z) + a 0..1 severity.
  const threat = useMemo(() => {
    const cutInId = prediction?.cut_in.active ? prediction.cut_in.track_id : null;
    const risk = shield?.risk_level ?? 'NONE';
    const riskRank = RISK_RANK[risk] ?? 0;

    let pos: { x: number; z: number } | null = null;
    let severity = 0;

    if (cutInId != null) {
      const t = tracks.find((tr) => tr.id.match(/(\d+)/)?.[1] === String(cutInId));
      if (t) {
        pos = { x: t.x, z: t.z };
        severity = Math.max(0.4, prediction?.cut_in.probability ?? 0.5);
      }
    }
    if (!pos && riskRank >= 2 && tracks.length) {
      // nearest track ahead-ish as the shield's implied threat
      let best = Infinity;
      for (const t of tracks) {
        if (Math.abs(((t.azimuth_deg + 180) % 360) - 180) > 60) continue;
        if (t.range_m < best) {
          best = t.range_m;
          pos = { x: t.x, z: t.z };
        }
      }
      severity = riskRank / 4;
    }
    return pos ? { pos, severity } : null;
  }, [prediction, shield, tracks]);

  useFrame((state) => {
    const m = patchRef.current;
    if (!m) return;
    if (!threat) {
      m.visible = false;
      return;
    }
    m.visible = true;
    const ex = sharedVehicleState.x;
    const ez = sharedVehicleState.z;
    const dx = threat.pos.x - ex;
    const dz = threat.pos.z - ez;
    const dist = Math.hypot(dx, dz) || 1;
    // patch centred ~60% of the way to the threat, aligned along the line
    m.position.set(ex + dx * 0.6, 0.03, ez + dz * 0.6);
    m.rotation.set(-Math.PI / 2, 0, -Math.atan2(dx, dz));
    const pulse = (Math.sin(state.clock.elapsedTime * 4) + 1) / 2;
    const s = threat.severity;
    m.scale.set(3 + s * 3, Math.min(dist * 0.7, 6 + s * 10), 1);
    const mat = m.material as THREE.MeshBasicMaterial;
    mat.color.set(s > 0.75 ? THEME.detectedCritical : THEME.detectedWarning);
    mat.opacity = 0.1 + s * 0.16 + pulse * 0.06 * s;
  });

  return (
    <mesh ref={patchRef} rotation={[-Math.PI / 2, 0, 0]} visible={false}>
      <circleGeometry args={[1, 32]} />
      <meshBasicMaterial color={THEME.detectedWarning} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
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
  const route = useSimulationStore((state) => state.routeWaypoints);
  const context = useSimulationStore((state) => state.roadContext);
  const routeGeom = useMemo(() => buildRouteGeometry(context.length > 1 ? context : route), [context, route]);
  const mapped = useMappedRoadContext(state => state.items);
  // Rebuild at bounded 100m station cells, not every control/telemetry tick.
  const renderCell = useSimulationStore(state => Math.floor((state.ego?.frenet.s ?? 0)/100));
  const renderWindow = useMemo(()=>routeGeom ? routeRenderWindow(routeGeom,renderCell*100,300,700) : null,[routeGeom,renderCell]);
  const ribbons = useMemo(() => {
    if(!routeGeom)return [];
    // The selected route is a navigation curve, not a second 17.6m-wide road.
    // Keep modeled fallback only where actual mapped road surfaces are absent.
    const segments = mapped.filter(f=>f.kind==="road"&&!f.tunnel).flatMap(f=>{
      const points=f.geometry.map(([lat,lng])=>toLocalXZ(lat,lng,routeGeom.originLat,routeGeom.originLng));
      return points.slice(1).map((b,i)=>({a:points[i],b,width:f.width_m}));
    });
    // Query only nearby road segments. Scanning the entire mapped network for
    // every point of a kilometres-long route can block the rendering thread.
    const cellSize=32;
    const cells=new Map<string,typeof segments>();
    for(const segment of segments){
      const {a,b,width}=segment,pad=width/2;
      for(let x=Math.floor((Math.min(a.x,b.x)-pad)/cellSize);x<=Math.floor((Math.max(a.x,b.x)+pad)/cellSize);x++)
        for(let z=Math.floor((Math.min(a.z,b.z)-pad)/cellSize);z<=Math.floor((Math.max(a.z,b.z)+pad)/cellSize);z++){
          const key=`${x}:${z}`,bucket=cells.get(key)??[];bucket.push(segment);cells.set(key,bucket);
        }
    }
    const covered=(p:THREE.Vector3)=>(cells.get(`${Math.floor(p.x/cellSize)}:${Math.floor(p.z/cellSize)}`)??[]).some(({a,b,width})=>{
      const dx=b.x-a.x,dz=b.z-a.z,l=dx*dx+dz*dz;
      const t=l>0?Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.z-a.z)*dz)/l)):0;
      return Math.hypot(p.x-a.x-t*dx,p.z-a.z-t*dz)<=Math.max(.1,width/2-.05);
    });
    const result: ReturnType<typeof buildRoadRibbon>[]=[];
    let run:THREE.Vector3[]=[];
    const flush=()=>{
      if(run.length>1){const distances=[0];for(let i=1;i<run.length;i++)distances.push(distances[i-1]+run[i].distanceTo(run[i-1]));
        result.push(buildRoadRibbon({...routeGeom,points:run,distances,totalLength:distances[distances.length-1]},7));}
      run=[];
    };
    const visiblePoints=renderWindow?.points ?? [];
    for(let i=1;i<visiblePoints.length;i++){
      const start=visiblePoints[i-1],end=visiblePoints[i];
      // Clip long navigation segments in world metres rather than dropping
      // a whole segment because only its midpoint touches a mapped surface.
      const count=Math.max(1,Math.ceil(start.distanceTo(end)/2));
      for(let j=0;j<count;j++){
        const a=start.clone().lerp(end,j/count),b=start.clone().lerp(end,(j+1)/count);
        if(covered(a.clone().lerp(b,.5))){flush();continue;}
        if(!run.length)run.push(a);run.push(b);
      }
    }flush();return result;
  },[routeGeom,mapped,renderWindow]);
  useEffect(()=>()=>ribbons.forEach(r=>r.roadGeometry.dispose()),[ribbons]);
  if(ribbons.length)return <group name="modeled-route-surface-only-outside-mapped-coverage">
    {ribbons.map((r,i)=><mesh key={i} geometry={r.roadGeometry}><meshStandardMaterial color="#a1aca1" roughness={1}/></mesh>)}
  </group>;

  // No road geometry means no rendered road; never invent a straight strip.
  return null;
}

function MappedParkingArea() {
  const polygon = useSimulationStore((state) => state.ego?.parking_area);
  const route = useSimulationStore((state) => state.routeWaypoints);
  const geometry = useMemo(() => {
    if (!polygon || polygon.length < 4 || route.length < 2) return null;
    const points = polygon.map(([lat, lng]) => toLocalXZ(lat, lng, route[0][0], route[0][1]));
    const shape = new THREE.Shape(points.map(p => new THREE.Vector2(p.x, -p.z)));
    const mesh = new THREE.ShapeGeometry(shape);
    mesh.rotateX(-Math.PI / 2);
    return mesh;
  }, [polygon, route]);
  useEffect(() => () => geometry?.dispose(), [geometry]);
  if (!geometry) return null;
  return <mesh geometry={geometry} position={[0, -.01, 0]}>
    <meshBasicMaterial color="#a9b4ad" transparent opacity={.7} side={THREE.DoubleSide} />
  </mesh>;
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
    groupRef.current.rotation.y = -approachHeading(-groupRef.current.rotation.y, heading, k, delta);
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
        <Html eps={0.5} position={[0, hei + 0.5, 0]} center distanceFactor={relevance === 'primary' ? 22 : 16} zIndexRange={[90, 0]}>
          <SurroundTrackLabel
            role={role}
            relevance={relevance}
            color={color}
            {...surroundLabelText(relevance === 'primary', track.range_m, speedMps, closingMps)}
          />
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
    const all = annotateTracks(tracks, {
      egoHeadingRad: sharedVehicleState.headingRad,
      prediction,
      perception,
      egoLateralM: ego?.frenet?.d ?? 0,
      laneChanging: Boolean(planner?.is_changing_lane),
    });
    return all;
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

// Only the currently observed control is rendered. Position is the same
// modeled stop-line offset used by the driver, not surveyed sign placement.
type ContextRoad = {way_id: number; segments: [number, number][][]};
function NearbyMappedRoads() {
  const geom = useRouteGeometry();
  const connected = useSimulationStore(s => s.isConnected);
  const [context, setContext] = useState<{geom: RouteGeometry; roads: ContextRoad[]} | null>(null);
  useEffect(() => {
    if (!geom || !connected) return;
    let disposed = false;
    let controller: AbortController | undefined;
    let previous: {lat:number; lng:number} | undefined;
    const update = async () => {
      const position = useSimulationStore.getState().ego?.geographic_position;
      if (!position || (previous && Math.hypot(position.lat-previous.lat, position.lng-previous.lng)<.0005)) return;
      controller?.abort(); controller = new AbortController();
      try {
        const response = await fetch(`${process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'}/api/map/context?lat=${position.lat}&lng=${position.lng}`, {signal:controller.signal});
        if (!response.ok) throw new Error('Map context unavailable');
        const data = await response.json();
        if (!disposed) { setContext({geom, roads:data.roads}); previous = position; }
      } catch (error) { if (!disposed && !(error instanceof DOMException && error.name === 'AbortError')) setContext(null); }
    };
    void update(); const timer = setInterval(update, 1500);
    return () => {disposed = true; clearInterval(timer); controller?.abort();};
  }, [geom, connected]);
  if (!geom || !connected || context?.geom !== geom) return null;
  const roads = context.roads;
  return <group name="recorded-nearby-roads" position={[0, .045, 0]}>
    {roads.flatMap(road => road.segments.map((segment, i) => <Line key={`${road.way_id}:${i}`} points={segment.map(([lat,lng])=>toLocalXZ(lat,lng,geom.originLat,geom.originLng))} color="#aebdb2" lineWidth={7} transparent opacity={.75} />))}
  </group>;
}

function WorldRoadControl({control}: {control:SceneControl}) {
  const ego = useSimulationStore(s => s.ego);
  const geom = useRouteGeometry();
  const ribbon = useMemo(() => geom ? buildRoadRibbon(geom) : null, [geom]);
  useEffect(() => () => ribbon?.roadGeometry.dispose(), [ribbon]);
  const junction = control;
  if (!geom || !ribbon || !ego || !junction || !Number.isFinite(control.station_m)) return null;
  const station = control.station_m;
  const roadside = roadsidePosition(geom, ribbon, station);
  const line = getWorldPosAtFrenet(geom, control.stop_station_m, ego.frenet.d);
  const signal = junction.kind === 'traffic_signals';
  const stop = junction.kind === 'stop';
  if (!signal && !stop && junction.kind !== 'give_way') return null;
  const phase = junction.phase?.toUpperCase();
  return <group name={`world-road-control-${control.node_id}`}>
    <group position={[roadside.pos.x, 0, roadside.pos.z]} rotation={[0, -roadside.headingDeg * Math.PI / 180, 0]}>
      <mesh position={[0, 1.55, 0]}><cylinderGeometry args={[.045, .045, 3.1, 8]} /><meshStandardMaterial color="#78847c" /></mesh>
      {signal ? <group position={[0, 3.1, 0]}>
        <RoundedBox args={[.55, 1.5, .3]} radius={.08}><meshBasicMaterial color="#121212" toneMapped={false} /></RoundedBox>
        {(['RED', 'YELLOW', 'GREEN'] as const).map((color, i) => {
          const active = phase === color;
          const tint = color === 'RED' ? '#f51919' : color === 'YELLOW' ? '#f5b90f' : '#14eb32';
          return <mesh key={color} name={`signal-${color.toLowerCase()}`} position={[0, .45 - i*.45, .17]} rotation={[Math.PI/2, 0, 0]}>
            <cylinderGeometry args={[.16, .16, .035, 20]} /><meshBasicMaterial color={active ? tint : '#2d2d2d'} toneMapped={false} />
          </mesh>;
        })}
      </group> : <group position={[0, 2.8, 0]}>
        <mesh rotation={[0, 0, stop ? Math.PI/8 : -Math.PI/2]}><circleGeometry args={[.59, stop ? 8 : 3]} /><meshStandardMaterial color="#ffffff" side={THREE.DoubleSide} /></mesh>
        <mesh position={[0, 0, .012]} rotation={[0, 0, stop ? Math.PI/8 : -Math.PI/2]}><circleGeometry args={[.53, stop ? 8 : 3]} /><meshBasicMaterial color="#f01919" side={THREE.DoubleSide} toneMapped={false} /></mesh>
        <Html center transform position={[0, 0, .025]} distanceFactor={1.2}><span style={{color:'white',font:'700 32px system-ui',whiteSpace:'nowrap'}}>{stop ? 'STOP' : 'YIELD'}</span></Html>
      </group>}
    </group>
    <mesh name="modeled-stop-line" position={[line.pos.x, .035, line.pos.z]} rotation={[-Math.PI/2, 0, -line.headingDeg*Math.PI/180]}>
      <planeGeometry args={[3.3, .3]} /><meshBasicMaterial color="#f8faf6" side={THREE.DoubleSide} />
    </mesh>
  </group>;
}

function WorldRoadControls() {
  const cameraTick = useSimulationStore(s=>s.tick);
  const controls=useSimulationStore(s=>s.sceneControls);
  return <group name="world-road-controls" userData={{cameraTick}}>{controls.map(control=><WorldRoadControl key={control.node_id} control={control} />)}</group>;
}

function RuntimeFrameProbe() {
  useEffect(() => {
    if (!runtimeCapture.enabled) return;
    const changed = () => { if (document.visibilityState !== 'visible') runtimeCapture.frame(epochNow(), false); };
    changed();
    document.addEventListener('visibilitychange', changed);
    return () => document.removeEventListener('visibilitychange', changed);
  }, []);
  useFrame(() => {
    if (runtimeCapture.enabled) runtimeCapture.frame(epochNow(), document.visibilityState === 'visible');
  });
  return null;
}

export function SimulationScene() {
  const detailed=true;
  const rain=useSimulationStore(s=>s.scenario?.params?.rain_enabled===true);
  const [visible, setVisible] = useState(true);
  useEffect(() => {
    const update = () => setVisible(document.visibilityState === 'visible');
    update();
    document.addEventListener('visibilitychange', update);
    return () => document.removeEventListener('visibilitychange', update);
  }, []);
  return (
    <div className="absolute inset-0 w-full h-full z-0 overflow-hidden">
      <Canvas
        frameloop={visible ? 'always' : 'never'}
        dpr={[1, 1.5]}
        gl={{ antialias: true, alpha: false }}
        onCreated={({ gl, scene }) => {
          gl.setClearColor(THEME.bgApp);
          gl.toneMapping = THREE.ACESFilmicToneMapping;
          gl.toneMappingExposure = 1.15;
          scene.fog = new THREE.FogExp2(THEME.bgApp, 0.0075);
          // Development inspection reads the actual rendered transforms.
          if (process.env.NODE_ENV === 'development') {
            Object.defineProperty(gl.domElement, '__ddsScene', { value: scene, configurable: true });
          }
        }}
      >
        <PerspectiveCamera makeDefault position={[0, 5.6, 13.5]} fov={48} near={0.1} far={800} />

        {/* Studio key + cyan rim, matched to the cockpit palette */}
        <ambientLight intensity={0.6} />
        <directionalLight position={[40, 70, 50]} intensity={1.2} color="#E0F2FE" />
        <directionalLight position={[-50, 22, -50]} intensity={0.4} color={THEME.brandCyan} />
        <pointLight position={[0, 8, 0]} intensity={0.35} color={THEME.brandCyan} />

        {/* Dark Grid Background */}
        <group name="display-only-grid">
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
        </group>


        {/* Recorded road geometry; absent geometry stays absent. */}
        <MappedEnvironment />
        <SimulatedRain enabled={rain} />
        <HighwayRoad />
        <MappedParkingArea />
        <WorldRoadControls />

        {/* Planned-path corridor — chosen lateral plan through the real
            road geometry, constraint-coloured, crossfaded on change. */}
        <group name="display-only-driving-overlays">
        <PlannedCorridor />


        {/* Real path-planning visualisation: every scored candidate,
            dimmed, with the actual chosen path highlighted -- the
            rider-app signature this phase is named for. */}
        {detailed && <PlannerCandidatePaths />}
        {detailed && <PredictedAgentRibbons />}
        {detailed && <SurroundPerceptionLayer />}
        {detailed && <SpatialRiskRegion />}
        <ShieldOverrideIndicator />
        </group>

        {/* Dynamic Vehicles */}
        <EgoVehicle />
        <TrafficManager />

        {/* 5-Mode Camera Rig (Chase / Cockpit / Top-down / Orbit / Cinematic) */}
        <CameraRig sharedVehicleState={sharedVehicleState} headingToForwardInto={headingToForwardInto} />
        <SceneControlCamera />
        <RuntimeFrameProbe />
      </Canvas>
    </div>
  );
}
