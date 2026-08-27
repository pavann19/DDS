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
  PredictionState,
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

  // Phase 7: per-agent forecasts + intent + proactive cut-in response.
  prediction: PredictionState | null;

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
  prediction: null,

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

  applyDelta: (payload) => set((state) => {
    // Protocol v3 layered channels (ADR-001 item 7).
    const { pose, semantic, heavy } = payload.channels;
    return {
      ego: pose.ego ?? state.ego,
      traffic: semantic.traffic ?? state.traffic,
      perception: semantic.perception ?? state.perception,
      planner: semantic.planner ?? state.planner,
      shap: semantic.driver_analytics?.shap ?? state.shap,
      anomaly: semantic.driver_analytics?.anomaly ?? state.anomaly,
      driverScore: semantic.driver_analytics?.driver_score ?? state.driverScore,
      safetyShield: semantic.safety_shield ?? state.safetyShield,
      scenario: semantic.scenario ?? state.scenario,
      surroundPerception: heavy.surround_perception ?? state.surroundPerception,
      prediction: heavy.prediction ?? state.prediction,
      tick: payload.tick,
      simulationTime: payload.simulation_time_s,
    };
  }),
}));
