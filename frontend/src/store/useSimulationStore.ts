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
  ScenarioState,
  ScenarioSummary,
  SurroundTrack,
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

  // Phase 6: 360-degree surround perception -- confirmed tracks only.
  surroundPerception: SurroundTrack[];

  // Route: sent once per destination (app/api/websockets.py's "route"
  // message), NOT part of the 10Hz "state" tick -- [lat, lng] pairs, a
  // different coordinate space from ego/traffic's local (x, z) frame.
  routeWaypoints: [number, number][];
  routeSteps: RouteStep[];
  setRoute: (waypoints: [number, number][], steps: RouteStep[]) => void;

  // Scenario Engine state & controls (Phase 5)
  scenario: ScenarioState | null;
  scenariosList: ScenarioSummary[];
  setScenariosList: (list: ScenarioSummary[]) => void;
  loadScenario: (scenarioId: string, density?: string, initialSpeedKmh?: number) => void;
  togglePause: () => void;
  stepSimulation: () => void;
  resetSimulation: () => void;

  // Actions for the Web Worker to call directly
  applyDelta: (payload: TelemetryStatePayload) => void;

  // Set once by the component that owns useTelemetry() (page.tsx), so any
  // component (e.g. a destination picker) can request a new route without
  // needing useTelemetry's hook instance threaded through props.
  sendCommand: (command: Record<string, unknown>) => void;
  setSendCommand: (fn: (command: Record<string, unknown>) => void) => void;
}

export const useSimulationStore = create<SimulationStore>((set, get) => ({
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
  surroundPerception: [],

  scenario: null,
  scenariosList: [],
  setScenariosList: (list) => set({ scenariosList: list }),
  loadScenario: (scenarioId, density, initialSpeedKmh) => {
    get().sendCommand({
      type: 'load_scenario',
      scenario_id: scenarioId,
      traffic_density: density,
      initial_speed_kmh: initialSpeedKmh,
    });
  },
  togglePause: () => {
    const isPaused = get().scenario?.is_paused ?? false;
    get().sendCommand({
      type: isPaused ? 'resume_simulation' : 'pause_simulation',
    });
  },
  stepSimulation: () => {
    get().sendCommand({ type: 'step_simulation' });
  },
  resetSimulation: () => {
    get().sendCommand({ type: 'reset_simulation' });
  },

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
    surroundPerception: payload.data.surround_perception ?? state.surroundPerception,
    scenario: payload.data.scenario ?? state.scenario,
    tick: payload.tick,
    simulationTime: payload.simulation_time_s,
  })),
}));
