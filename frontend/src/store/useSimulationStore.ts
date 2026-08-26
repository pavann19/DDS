import { create } from 'zustand';
import {
  TelemetryStatePayload,
  EgoState,
  EntityState,
  PerceptionObject,
  PlannerState,
  ShapResult,
  AnomalyResult,
  DriverScore,
  SafetyShieldState,
  RouteStep,
} from '../types/protocol';

interface SimulationStore {
  // Connection state
  isConnected: boolean;
  setConnected: (status: boolean) => void;

  // High-frequency telemetry
  ego: EgoState | null;
  traffic: EntityState[];
  perception: PerceptionObject[];
  planner: PlannerState | null;
  tick: number;
  simulationTime: number;

  // Explainability/safety -- restored 2026-08 after being computed every
  // tick server-side and silently discarded (never placed in the WS
  // payload, only logged to SQLite). See app/api/websockets.py.
  shap: ShapResult | null;
  anomaly: AnomalyResult | null;
  driverScore: DriverScore | null;
  safetyShield: SafetyShieldState | null;

  // Route: sent once per destination (app/api/websockets.py's "route"
  // message), NOT part of the 10Hz "state" tick -- [lat, lng] pairs, a
  // different coordinate space from ego/traffic's local (x, z) frame.
  routeWaypoints: [number, number][];
  routeSteps: RouteStep[];
  setRoute: (waypoints: [number, number][], steps: RouteStep[]) => void;

  // Actions for the Web Worker to call directly
  applyDelta: (payload: TelemetryStatePayload) => void;

  // Set once by the component that owns useTelemetry() (page.tsx), so any
  // component (e.g. a destination picker) can request a new route without
  // needing useTelemetry's hook instance threaded through props.
  sendCommand: (command: Record<string, unknown>) => void;
  setSendCommand: (fn: (command: Record<string, unknown>) => void) => void;
}

export const useSimulationStore = create<SimulationStore>((set) => ({
  isConnected: false,
  setConnected: (status) => set({ isConnected: status }),

  ego: null,
  traffic: [],
  perception: [],
  planner: null,
  tick: 0,
  simulationTime: 0,

  shap: null,
  anomaly: null,
  driverScore: null,
  safetyShield: null,

  routeWaypoints: [],
  routeSteps: [],
  setRoute: (waypoints, steps) => set({ routeWaypoints: waypoints, routeSteps: steps }),

  sendCommand: () => console.warn('sendCommand called before useTelemetry mounted'),
  setSendCommand: (fn) => set({ sendCommand: fn }),

  applyDelta: (payload) => set((state) => ({
    ego: payload.data.ego ?? state.ego,
    traffic: payload.data.traffic ?? state.traffic,
    perception: payload.data.perception ?? state.perception,
    planner: payload.data.planner ?? state.planner,
    shap: payload.data.shap ?? state.shap,
    anomaly: payload.data.anomaly ?? state.anomaly,
    driverScore: payload.data.driver_score ?? state.driverScore,
    safetyShield: payload.data.safety_shield ?? state.safetyShield,
    tick: payload.tick,
    simulationTime: payload.simulation_time_s,
  })),
}));
