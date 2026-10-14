'use client';

import React, { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import {
  RouteGeometry,
  buildRouteGeometry,
  getWorldPosAtFrenet,
} from '../../lib/routeGeometry';

// Module-scope scratch objects to prevent per-frame garbage collection
const SCRATCH_MAT = new THREE.Matrix4();
const SCRATCH_POS = new THREE.Vector3();
const SCRATCH_QUAT = new THREE.Quaternion();
const SCRATCH_SCALE = new THREE.Vector3();
const ZERO_SCALE = new THREE.Vector3(0, 0, 0);

// Deterministic PRNG — mulberry32 keyed by integer station
function mulberry32(seed: number) {
  let s = Math.floor(seed) >>> 0;
  return function () {
    s = (s + 0x6d2d79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Sky Dome Shader with seamless fog integration
const SKY_VERTEX_SHADER = `
  varying vec3 vWorldPosition;
  void main() {
    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
    vWorldPosition = worldPosition.xyz;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const SKY_FRAGMENT_SHADER = `
  uniform vec3 topColor;
  uniform vec3 bottomColor;
  uniform float offset;
  uniform float exponent;
  varying vec3 vWorldPosition;
  void main() {
    float h = normalize(vWorldPosition + offset).y;
    gl_FragColor = vec4(mix(bottomColor, topColor, max(pow(max(h, 0.0), exponent), 0.0)), 1.0);
  }
`;

interface BuildingData {
  s: number;
  x: number;
  y: number;
  z: number;
  rotY: number;
  sx: number;
  sy: number;
  sz: number;
  isGlass: boolean;
}

interface WindowData {
  s: number;
  x: number;
  y: number;
  z: number;
  rotY: number;
  sx: number;
  sy: number;
  sz: number;
  isCyan: boolean;
}

interface LightData {
  s: number;
  poleX: number;
  poleZ: number;
  lampX: number;
  lampZ: number;
  rotY: number;
}

/** Builds raised sidewalk curb mesh geometry alongside road ribbon */
function buildCurbGeometry(geom: RouteGeometry, halfRoadWidth = 8.8, curbWidth = 1.6, curbHeight = 0.14): THREE.BufferGeometry {
  const { points } = geom;
  const vertices: number[] = [];
  const indices: number[] = [];

  for (let i = 0; i < points.length; i++) {
    let dir: THREE.Vector3;
    if (i === 0) {
      dir = new THREE.Vector3().subVectors(points[1], points[0]).normalize();
    } else if (i === points.length - 1) {
      dir = new THREE.Vector3().subVectors(points[i], points[i - 1]).normalize();
    } else {
      dir = new THREE.Vector3().subVectors(points[i + 1], points[i - 1]).normalize();
    }
    const right = new THREE.Vector3(dir.z, 0, -dir.x).normalize();

    // Left curb: outer edge to inner edge
    const lInner = points[i].clone().sub(right.clone().multiplyScalar(halfRoadWidth));
    const lOuter = points[i].clone().sub(right.clone().multiplyScalar(halfRoadWidth + curbWidth));

    // Right curb: inner edge to outer edge
    const rInner = points[i].clone().add(right.clone().multiplyScalar(halfRoadWidth));
    const rOuter = points[i].clone().add(right.clone().multiplyScalar(halfRoadWidth + curbWidth));

    // Left curb top vertices (0, 1) and bottom edge (2, 3)
    vertices.push(
      lInner.x, curbHeight, lInner.z,
      lOuter.x, curbHeight, lOuter.z,
      rInner.x, curbHeight, rInner.z,
      rOuter.x, curbHeight, rOuter.z,
    );

    if (i < points.length - 1) {
      const b = i * 4;
      // Left curb top surface
      indices.push(b, b + 1, b + 4);
      indices.push(b + 1, b + 5, b + 4);
      // Right curb top surface
      indices.push(b + 2, b + 6, b + 3);
      indices.push(b + 3, b + 6, b + 7);
    }
  }

  const bg = new THREE.BufferGeometry();
  bg.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  bg.setIndex(indices);
  bg.computeVertexNormals();
  return bg;
}

export function Environment() {
  const routeWaypoints = useSimulationStore((state) => state.routeWaypoints);
  const routeGeom = useMemo(() => buildRouteGeometry(routeWaypoints), [routeWaypoints]);
  const ego = useSimulationStore((state) => state.ego);
  const density = useConsole((s) => s.density);
  const isFocus = density === 'focus';

  // Sky Mesh reference
  const skyRef = useRef<THREE.Mesh>(null);

  // Instanced Meshes references
  const glassMeshRef = useRef<THREE.InstancedMesh>(null);
  const concreteMeshRef = useRef<THREE.InstancedMesh>(null);
  const windowCyanMeshRef = useRef<THREE.InstancedMesh>(null);
  const windowWarmMeshRef = useRef<THREE.InstancedMesh>(null);

  const poleMeshRef = useRef<THREE.InstancedMesh>(null);
  const lampMeshRef = useRef<THREE.InstancedMesh>(null);
  const glowMeshRef = useRef<THREE.InstancedMesh>(null);

  // Procedural generation of buildings, windows, and streetlights
  const { buildings, windows, lights, curbGeometry } = useMemo(() => {
    if (!routeGeom || routeGeom.totalLength < 10) {
      return { buildings: [], windows: [], lights: [], curbGeometry: null };
    }

    const bList: BuildingData[] = [];
    const wList: WindowData[] = [];
    const lList: LightData[] = [];

    const totalLen = routeGeom.totalLength;
    const STATION_STEP = 24;
    const LIGHT_STEP = 42;

    // 1. Generate procedural buildings along route
    for (let s = 10; s < totalLen - 20; s += STATION_STEP) {
      const randLeft = mulberry32(s * 1000 + 1);
      const randRight = mulberry32(s * 1000 + 2);

      // Left building
      const dLeft = -(18 + randLeft() * 20);
      const hLeft = 9 + randLeft() * 34;
      const wLeft = 12 + randLeft() * 12;
      const dptLeft = 14 + randLeft() * 16;
      const isGlassLeft = randLeft() > 0.42;

      const { pos: posL, headingDeg: degL } = getWorldPosAtFrenet(routeGeom, s, dLeft);
      const rotYL = THREE.MathUtils.degToRad(-degL) + (randLeft() - 0.5) * 0.08;

      bList.push({
        s,
        x: posL.x,
        y: hLeft / 2,
        z: posL.z,
        rotY: rotYL,
        sx: wLeft,
        sy: hLeft,
        sz: dptLeft,
        isGlass: isGlassLeft,
      });

      // Window accent strip for left building
      if (randLeft() > 0.35) {
        const isCyan = randLeft() > 0.45;
        const winH = 0.8 + randLeft() * 1.5;
        const winW = wLeft * 0.8;
        const winY = 2.5 + randLeft() * (hLeft * 0.65);
        wList.push({
          s,
          x: posL.x,
          y: winY,
          z: posL.z,
          rotY: rotYL,
          sx: winW,
          sy: winH,
          sz: dptLeft * 1.02,
          isCyan,
        });
      }

      // Right building
      const dRight = 18 + randRight() * 20;
      const hRight = 9 + randRight() * 34;
      const wRight = 12 + randRight() * 12;
      const dptRight = 14 + randRight() * 16;
      const isGlassRight = randRight() > 0.42;

      const { pos: posR, headingDeg: degR } = getWorldPosAtFrenet(routeGeom, s, dRight);
      const rotYR = THREE.MathUtils.degToRad(-degR) + (randRight() - 0.5) * 0.08;

      bList.push({
        s,
        x: posR.x,
        y: hRight / 2,
        z: posR.z,
        rotY: rotYR,
        sx: wRight,
        sy: hRight,
        sz: dptRight,
        isGlass: isGlassRight,
      });

      // Window accent strip for right building
      if (randRight() > 0.35) {
        const isCyan = randRight() > 0.45;
        const winH = 0.8 + randRight() * 1.5;
        const winW = wRight * 0.8;
        const winY = 2.5 + randRight() * (hRight * 0.65);
        wList.push({
          s,
          x: posR.x,
          y: winY,
          z: posR.z,
          rotY: rotYR,
          sx: winW,
          sy: winH,
          sz: dptRight * 1.02,
          isCyan,
        });
      }
    }

    // 2. Generate streetlights alternating sides
    let lightIndex = 0;
    for (let s = 15; s < totalLen - 15; s += LIGHT_STEP) {
      const isEven = lightIndex % 2 === 0;
      const d = isEven ? -9.6 : 9.6;
      const { pos: polePos, headingDeg: deg } = getWorldPosAtFrenet(routeGeom, s, d);
      const { pos: lampPos } = getWorldPosAtFrenet(routeGeom, s, isEven ? -8.2 : 8.2);

      lList.push({
        s,
        poleX: polePos.x,
        poleZ: polePos.z,
        lampX: lampPos.x,
        lampZ: lampPos.z,
        rotY: THREE.MathUtils.degToRad(-deg),
      });
      lightIndex++;
    }

    const curb = buildCurbGeometry(routeGeom);

    return { buildings: bList, windows: wList, lights: lList, curbGeometry: curb };
  }, [routeGeom]);

  // Shared Materials & Shaders
  const skyUniforms = useMemo(
    () => ({
      topColor: { value: new THREE.Color('#05070C') },
      bottomColor: { value: new THREE.Color('#0B0F16') },
      offset: { value: 33 },
      exponent: { value: 0.6 },
    }),
    [],
  );

  const glassMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: '#0F172A',
        metalness: 0.9,
        roughness: 0.15,
        transparent: true,
        opacity: isFocus ? 0.6 : 1.0,
      }),
    [isFocus],
  );

  const concreteMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: '#334155',
        metalness: 0.25,
        roughness: 0.85,
        transparent: true,
        opacity: isFocus ? 0.6 : 1.0,
      }),
    [isFocus],
  );

  const winCyanMat = useMemo(
    () =>
      new THREE.MeshBasicMaterial({
        color: '#38BDF8',
        transparent: true,
        opacity: isFocus ? 0.25 : 0.55,
      }),
    [isFocus],
  );

  const winWarmMat = useMemo(
    () =>
      new THREE.MeshBasicMaterial({
        color: '#FEF08A',
        transparent: true,
        opacity: isFocus ? 0.25 : 0.55,
      }),
    [isFocus],
  );

  const poleMat = useMemo(() => new THREE.MeshStandardMaterial({ color: '#475569', metalness: 0.6, roughness: 0.4 }), []);
  const lampMat = useMemo(() => new THREE.MeshBasicMaterial({ color: '#FEF08A' }), []);
  const glowMat = useMemo(
    () =>
      new THREE.MeshBasicMaterial({
        color: '#FEF08A',
        transparent: true,
        opacity: 0.14,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    [],
  );

  // Per-frame distance culling without memory allocations
  useFrame((state) => {
    const egoStation = ego?.frenet?.s ?? 0;
    const camPos = state.camera.position;

    // Keep sky dome anchored to camera so horizon is infinite
    if (skyRef.current) {
      skyRef.current.position.set(camPos.x, 0, camPos.z);
    }

    // Cull buildings (|s - egoStation| < 260m)
    let glassIdx = 0;
    let concreteIdx = 0;

    for (let i = 0; i < buildings.length; i++) {
      const b = buildings[i];
      const isVisible = Math.abs(b.s - egoStation) < 260;

      SCRATCH_POS.set(b.x, b.y, b.z);
      SCRATCH_QUAT.setFromAxisAngle(THREE.Vector3.prototype.clone.call(THREE.Object3D.DEFAULT_UP), b.rotY);
      SCRATCH_SCALE.set(isVisible ? b.sx : 0, isVisible ? b.sy : 0, isVisible ? b.sz : 0);
      SCRATCH_MAT.compose(SCRATCH_POS, SCRATCH_QUAT, SCRATCH_SCALE);

      if (b.isGlass && glassMeshRef.current) {
        glassMeshRef.current.setMatrixAt(glassIdx++, SCRATCH_MAT);
      } else if (!b.isGlass && concreteMeshRef.current) {
        concreteMeshRef.current.setMatrixAt(concreteIdx++, SCRATCH_MAT);
      }
    }

    if (glassMeshRef.current) glassMeshRef.current.instanceMatrix.needsUpdate = true;
    if (concreteMeshRef.current) concreteMeshRef.current.instanceMatrix.needsUpdate = true;

    // Cull window accent strips
    let winCyanIdx = 0;
    let winWarmIdx = 0;

    for (let i = 0; i < windows.length; i++) {
      const w = windows[i];
      const isVisible = Math.abs(w.s - egoStation) < 260;

      SCRATCH_POS.set(w.x, w.y, w.z);
      SCRATCH_QUAT.setFromAxisAngle(THREE.Vector3.prototype.clone.call(THREE.Object3D.DEFAULT_UP), w.rotY);
      SCRATCH_SCALE.set(isVisible ? w.sx : 0, isVisible ? w.sy : 0, isVisible ? w.sz : 0);
      SCRATCH_MAT.compose(SCRATCH_POS, SCRATCH_QUAT, SCRATCH_SCALE);

      if (w.isCyan && windowCyanMeshRef.current) {
        windowCyanMeshRef.current.setMatrixAt(winCyanIdx++, SCRATCH_MAT);
      } else if (!w.isCyan && windowWarmMeshRef.current) {
        windowWarmMeshRef.current.setMatrixAt(winWarmIdx++, SCRATCH_MAT);
      }
    }

    if (windowCyanMeshRef.current) windowCyanMeshRef.current.instanceMatrix.needsUpdate = true;
    if (windowWarmMeshRef.current) windowWarmMeshRef.current.instanceMatrix.needsUpdate = true;

    // Cull streetlights (|s - egoStation| < 260m)
    if (poleMeshRef.current && lampMeshRef.current && glowMeshRef.current) {
      for (let i = 0; i < lights.length; i++) {
        const l = lights[i];
        const isVisible = Math.abs(l.s - egoStation) < 260;

        // Pole: 7m height
        SCRATCH_POS.set(l.poleX, 3.5, l.poleZ);
        SCRATCH_QUAT.setFromAxisAngle(THREE.Vector3.prototype.clone.call(THREE.Object3D.DEFAULT_UP), l.rotY);
        SCRATCH_SCALE.set(isVisible ? 1 : 0, isVisible ? 1 : 0, isVisible ? 1 : 0);
        SCRATCH_MAT.compose(SCRATCH_POS, SCRATCH_QUAT, SCRATCH_SCALE);
        poleMeshRef.current.setMatrixAt(i, SCRATCH_MAT);

        // Lamp fixture at top
        SCRATCH_POS.set(l.lampX, 7.0, l.lampZ);
        SCRATCH_MAT.compose(SCRATCH_POS, SCRATCH_QUAT, isVisible ? SCRATCH_SCALE : ZERO_SCALE);
        lampMeshRef.current.setMatrixAt(i, SCRATCH_MAT);

        // Ground glow disk (hidden in focus)
        const showGlow = isVisible && !isFocus;
        SCRATCH_POS.set(l.lampX, 0.04, l.lampZ);
        SCRATCH_QUAT.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
        SCRATCH_SCALE.set(showGlow ? 1 : 0, showGlow ? 1 : 0, showGlow ? 1 : 0);
        SCRATCH_MAT.compose(SCRATCH_POS, SCRATCH_QUAT, SCRATCH_SCALE);
        glowMeshRef.current.setMatrixAt(i, SCRATCH_MAT);
      }

      poleMeshRef.current.instanceMatrix.needsUpdate = true;
      lampMeshRef.current.instanceMatrix.needsUpdate = true;
      glowMeshRef.current.instanceMatrix.needsUpdate = true;
    }
  });

  const glassCount = useMemo(() => buildings.filter((b) => b.isGlass).length, [buildings]);
  const concreteCount = useMemo(() => buildings.filter((b) => !b.isGlass).length, [buildings]);
  const winCyanCount = useMemo(() => windows.filter((w) => w.isCyan).length, [windows]);
  const winWarmCount = useMemo(() => windows.filter((w) => !w.isCyan).length, [windows]);

  return (
    <group>
      {/* Sky Sphere */}
      <mesh ref={skyRef}>
        <sphereGeometry args={[650, 32, 15]} />
        <shaderMaterial
          vertexShader={SKY_VERTEX_SHADER}
          fragmentShader={SKY_FRAGMENT_SHADER}
          uniforms={skyUniforms}
          side={THREE.BackSide}
          depthWrite={false}
        />
      </mesh>

      {/* Raised Sidewalk / Curb */}
      {curbGeometry && (
        <mesh geometry={curbGeometry}>
          <meshStandardMaterial color="#1E293B" roughness={0.92} />
        </mesh>
      )}

      {/* Buildings Instanced Meshes */}
      {glassCount > 0 && (
        <instancedMesh ref={glassMeshRef} args={[undefined, undefined, glassCount]} material={glassMat}>
          <boxGeometry args={[1, 1, 1]} />
        </instancedMesh>
      )}

      {concreteCount > 0 && (
        <instancedMesh ref={concreteMeshRef} args={[undefined, undefined, concreteCount]} material={concreteMat}>
          <boxGeometry args={[1, 1, 1]} />
        </instancedMesh>
      )}

      {/* Window Accent Strips */}
      {winCyanCount > 0 && (
        <instancedMesh ref={windowCyanMeshRef} args={[undefined, undefined, winCyanCount]} material={winCyanMat}>
          <boxGeometry args={[1, 1, 1]} />
        </instancedMesh>
      )}

      {winWarmCount > 0 && (
        <instancedMesh ref={windowWarmMeshRef} args={[undefined, undefined, winWarmCount]} material={winWarmMat}>
          <boxGeometry args={[1, 1, 1]} />
        </instancedMesh>
      )}

      {/* Street Light Poles, Fixtures, and Ground Glows */}
      {lights.length > 0 && (
        <>
          <instancedMesh ref={poleMeshRef} args={[undefined, undefined, lights.length]} material={poleMat}>
            <cylinderGeometry args={[0.09, 0.12, 7, 8]} />
          </instancedMesh>

          <instancedMesh ref={lampMeshRef} args={[undefined, undefined, lights.length]} material={lampMat}>
            <boxGeometry args={[0.5, 0.15, 0.35]} />
          </instancedMesh>

          <instancedMesh ref={glowMeshRef} args={[undefined, undefined, lights.length]} material={glowMat}>
            <circleGeometry args={[3.2, 16]} />
          </instancedMesh>
        </>
      )}
    </group>
  );
}
