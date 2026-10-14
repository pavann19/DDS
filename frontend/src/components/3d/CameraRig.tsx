'use client';

import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { OrbitControls as DreiOrbitControls } from '@react-three/drei';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import * as THREE from 'three';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useCamera, CameraMode } from '../../store/useCamera';
import { useReducedMotion } from '../../hooks/useReducedMotion';

// Module-scope scratch vectors (zero per-frame allocation)
const CAM_POS = new THREE.Vector3();
const CAM_LOOK = new THREE.Vector3();
const FWD_A = new THREE.Vector3();
const FWD_B = new THREE.Vector3();

export interface SharedVehicleState {
  x: number;
  z: number;
  headingRad: number;
  initialized: boolean;
}

interface CameraRigProps {
  sharedVehicleState: SharedVehicleState;
  headingToForwardInto: (out: THREE.Vector3, headingRad: number) => THREE.Vector3;
}

export function CameraRig({ sharedVehicleState, headingToForwardInto }: CameraRigProps) {
  const mode = useCamera((s) => s.mode);
  const steering = useSimulationStore((s) => s.ego?.steering_angle ?? 0);
  const curvature = useSimulationStore((s) => s.planner?.curvature ?? 0);
  const speedMps = useSimulationStore((s) => s.ego?.velocity ?? 0);
  const reducedMotion = useReducedMotion();

  const orbitRef = useRef<OrbitControlsImpl>(null);
  const initialized = useRef(false);
  const anticip = useRef(0);
  const prevMode = useRef<CameraMode>(mode);
  const prevEpoch = useRef<string | null>(null);

  // Effective mode: cinematic falls back to chase under prefers-reduced-motion
  const effectiveMode: CameraMode = reducedMotion && mode === 'cinematic' ? 'chase' : mode;

  useFrame((state, delta) => {
    if (!sharedVehicleState.initialized) return;
    const sim = useSimulationStore.getState();
    const epochKey = `${sim.simulationId}:${sim.epoch}`;
    if (prevEpoch.current !== epochKey) {
      initialized.current = false;
      anticip.current = 0;
      prevEpoch.current = epochKey;
    }

    // Mutate camera properties (FOV, Up vector) strictly inside useFrame
    const persCam = state.camera as THREE.PerspectiveCamera;
    if (persCam.isPerspectiveCamera) {
      const targetFov = effectiveMode === 'cockpit' ? 62 : effectiveMode === 'top' ? 32 : 48;
      if (persCam.fov !== targetFov) {
        persCam.fov = targetFov;
        persCam.updateProjectionMatrix();
      }
    }

    if (effectiveMode === 'top') {
      state.camera.up.set(0, 0, -1);
    } else if (state.camera.up.z !== 0) {
      state.camera.up.set(0, 1, 0);
    }

    if (prevMode.current !== effectiveMode) {
      if (effectiveMode === 'orbit') {
        const fwd = headingToForwardInto(FWD_A, sharedVehicleState.headingRad);
        state.camera.position.set(
          sharedVehicleState.x - fwd.x * 16,
          8,
          sharedVehicleState.z - fwd.z * 16,
        );
        state.camera.lookAt(sharedVehicleState.x, 0.5, sharedVehicleState.z);
      }
      prevMode.current = effectiveMode;
    }

    const vX = sharedVehicleState.x;
    const vZ = sharedVehicleState.z;
    const spd = THREE.MathUtils.clamp(speedMps, 0, 25);

    // 1. Orbit mode — OrbitControls owns camera math
    if (effectiveMode === 'orbit') {
      if (orbitRef.current) {
        orbitRef.current.target.set(vX, 0.5, vZ);
        orbitRef.current.update();
      }
      return;
    }

    // 2. Cockpit mode — interior view looking forward
    if (effectiveMode === 'cockpit') {
      const fwd = headingToForwardInto(FWD_A, sharedVehicleState.headingRad);
      CAM_POS.set(vX + fwd.x * 0.5, 1.25, vZ + fwd.z * 0.5);
      CAM_LOOK.set(vX + fwd.x * 40, 1.25, vZ + fwd.z * 40);

      if (!initialized.current || reducedMotion) {
        state.camera.position.copy(CAM_POS);
        state.camera.lookAt(CAM_LOOK);
        initialized.current = true;
      } else {
        state.camera.position.lerp(CAM_POS, Math.min(1, delta * 8));
        state.camera.lookAt(CAM_LOOK);
      }
      return;
    }

    // 3. Top-down mode — overhead view looking down with up=(0,0,-1)
    if (effectiveMode === 'top') {
      CAM_POS.set(vX, 55 + spd * 1.2, vZ + 6);
      CAM_LOOK.set(vX, 0, vZ);

      if (!initialized.current || reducedMotion) {
        state.camera.position.copy(CAM_POS);
        state.camera.lookAt(CAM_LOOK);
        initialized.current = true;
      } else {
        state.camera.position.lerp(CAM_POS, Math.min(1, delta * 4));
        state.camera.lookAt(CAM_LOOK);
      }
      return;
    }

    // 4. Cinematic mode — slow auto-orbit around ego
    if (effectiveMode === 'cinematic') {
      const t = state.clock.elapsedTime;
      const radius = 18 + Math.sin(t * 0.15) * 6;
      const height = 4 + Math.sin(t * 0.23) * 2;
      const angle = t * 0.22;

      CAM_POS.set(vX + Math.sin(angle) * radius, height, vZ + Math.cos(angle) * radius);
      CAM_LOOK.set(vX, 0.8, vZ);

      state.camera.position.copy(CAM_POS);
      state.camera.lookAt(CAM_LOOK);
      return;
    }

    // 5. Chase mode — default anticipatory chase camera
    const targetAnticip = THREE.MathUtils.clamp(
      steering * 1.6 + Math.sign(steering || 1) * curvature * 14,
      -0.5,
      0.5,
    );
    anticip.current = reducedMotion
      ? targetAnticip
      : THREE.MathUtils.lerp(anticip.current, targetAnticip, Math.min(1, delta * 2.2));

    const camHeadingR = sharedVehicleState.headingRad + anticip.current * 0.6;
    const lookHeadingR = sharedVehicleState.headingRad + anticip.current;
    const back = headingToForwardInto(FWD_A, camHeadingR);
    const fwd = headingToForwardInto(FWD_B, lookHeadingR);

    const camDist = 10.8 + spd * 0.22;
    const camHeight = 4.5 + spd * 0.04;
    const lookAhead = 17 + spd * 0.45 + Math.abs(anticip.current) * 20;

    CAM_POS.set(vX - back.x * camDist, camHeight, vZ - back.z * camDist);
    CAM_LOOK.set(vX + fwd.x * lookAhead, 1.0, vZ + fwd.z * lookAhead);

    if (!initialized.current || reducedMotion) {
      state.camera.position.copy(CAM_POS);
      state.camera.lookAt(CAM_LOOK);
      initialized.current = true;
    } else {
      state.camera.position.lerp(CAM_POS, Math.min(1, delta * 3.2));
      state.camera.lookAt(CAM_LOOK);
    }
  });

  return (
    <>
      {effectiveMode === 'orbit' && (
        <DreiOrbitControls
          ref={orbitRef}
          enablePan={false}
          minDistance={8}
          maxDistance={80}
          maxPolarAngle={Math.PI * 0.49}
        />
      )}
    </>
  );
}
