export type Coordinate = { x: number; y: number; z: number };
export type FrenetCoordinate = { s: number; d: number };
export type Quaternion = { x: number; y: number; z: number; w: number };

export interface EntityState {
  id: string;
  pose: Coordinate;
  rotation?: Quaternion; // Only if 3D orientation is sent, else compute from yaw
  yaw: number;
  velocity: number;
  acceleration: number;
  frenet: FrenetCoordinate;
}

// Real decision strings from app/services/inference.py's label encoder --
// NOT an abstract enum invented for this protocol. Do not "clean these up"
// to SCREAMING_SNAKE_CASE without changing the backend to match; they are
// the literal class names the deployed classifier was trained on.
export type Decision = "Accelerate" | "Decelerate" | "Maintain Speed";

export interface EgoState extends EntityState {
  decision: Decision;
  confidence: number; // 0..1, the classifier's max class probability
  // Which physical constraint is currently binding the speed controller --
  // "car_following" is the IDM braking-for-traffic case, "lateral_accel_limit"
  // is cornering, etc. See app/services/physics_engine.py's speed_limit_reason.
  speed_limit_reason: string;
  target_velocity: number;
  steering_angle: number;
  throttle: number;
  brake: number;
}

export interface PerceptionObject extends EntityState {
  type: "VEHICLE" | "PEDESTRIAN" | "BICYCLE" | "UNKNOWN";
  distance: number;
  rel_velocity: number;
  confidence: number;
}

// app/services/planner.py's LateralCandidate, as serialised by
// PhysicsEngine.get_planner_candidates(). Empty until a real route exists
// (candidates only get generated once app.services.frenet has a frame to
// project onto) -- an empty array on an early tick is expected, not a bug.
export interface PlannerCandidate {
  d_target: number;
  cost: number;
  is_chosen: boolean;
  is_lane_change: boolean;
}

export interface PlannerState {
  trajectory: Coordinate[];
  lookahead_point: Coordinate;
  lane_center: number;
  curvature: number;
  candidates: PlannerCandidate[];
  is_changing_lane: boolean;
}

// app/services/explainability.py's ExplainabilityEngine.explain_prediction()
export interface ShapContribution {
  feature: string;
  value: number;
  contribution: number;
}
export interface ShapResult {
  base_value: number;
  contributions: ShapContribution[];
}

// app/services/anomaly_detector.py's AnomalyDetector.detect()
export interface AnomalyResult {
  is_anomaly: boolean;
  type: string; // "NONE" | "OUT_OF_RANGE" | "OVERHEAT" | "RPM_SPIKE" | "HIGH_EMISSION" | "ERRATIC" | "UNKNOWN" | "INCOMPLETE_INPUT" | "FEATURE_MISMATCH"
  severity: "NONE" | "LOW" | "MEDIUM" | "HIGH";
  message: string;
}

// app/services/safety_shield.py's evaluate() -- an INDEPENDENT check run
// after the planner/IDM decision, not part of it. See that module's
// docstring for why it's a separate system rather than more planner logic.
export interface SafetyShieldState {
  approved: boolean;
  risk_level: "NONE" | "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";
  reasons: string[];
  override_action: "EMERGENCY_BRAKE" | "RECOVER_LOW_SPEED" | null;
  ttc_s: number | null;
}

// app/services/driver_scoring.py's DriverScorer.calculate_score()
export interface DriverScoreBreakdown {
  smoothness: number;
  efficiency: number;
  safety: number;
}
export interface DriverScore {
  score: number;
  rating: "A+" | "A" | "B" | "C" | "D" | "F";
  green_driving_index?: number;
  green_driving_rating?: "A+" | "A" | "B" | "C" | "D" | "F";
  breakdown: DriverScoreBreakdown;
}

// app/services/scenario_engine.py's ScenarioEngine.get_state()
export interface ScenarioState {
  id: string | null;
  name: string;
  category: "normal" | "traffic" | "maneuver" | "safety_critical" | string;
  description: string;
  is_paused: boolean;
  tick: number;
  elapsed_s: number;
  density: "low" | "medium" | "high" | string;
  initial_speed_kmh: number;
  status: "idle" | "running" | "completed" | "error" | string;
  milestone: string | null;
}

export interface ScenarioSummary {
  id: string;
  name: string;
  category: "normal" | "traffic" | "maneuver" | "safety_critical" | string;
  description: string;
  seed: number;
  default_initial_speed_kmh: number;
  default_density: "low" | "medium" | "high" | string;
}

// app/services/perception/perception_engine.py's SurroundPerceptionEngine.get_state()
// -- Phase 6's 360-degree surround perception. Confirmed tracks only (see
// that method's docstring); a track a real sensor rig hasn't detected in
// several ticks simply isn't in this list, same perception/control
// boundary as PerceptionObject/sensed_lead_vehicle.
export interface SurroundTrack {
  id: string;
  class: "SEDAN" | "SUV" | "TRUCK" | "MOTORCYCLE" | "BICYCLE" | "PEDESTRIAN" | "TRAFFIC_CONE" | string;
  status: "CONFIRMED" | string;
  x: number;
  z: number;
  vx: number;
  vz: number;
  range_m: number;
  azimuth_deg: number;
  sensors: string[];
  dims: [number, number, number]; // [length_m, width_m, height_m]
}

// V2 Protocol Payload
export interface TelemetryStatePayload {
  protocol_version: "2.0";
  simulation_id: string;
  run_id: string;
  tick: number;
  simulation_time_s: number;
  timestamp: string;
  coordinate_frame: "dds_world_v1";
  units: "SI";
  type: "state";
  data: {
    ego: EgoState;
    traffic: EntityState[];
    perception: PerceptionObject[];
    planner: PlannerState;
    shap: ShapResult;
    anomaly: AnomalyResult;
    driver_score: DriverScore;
    safety_shield: SafetyShieldState;
    scenario?: ScenarioState;
    surround_perception?: SurroundTrack[];
  };
}

// Sent once per set_route()/set_destination() -- app/api/websockets.py's
// fetch_and_apply_route(). waypoints/steps are [lat, lng] pairs (real
// geographic coordinates), NOT the dds_world_v1 local frame the "state"
// messages use -- SimulationScene is responsible for projecting them into
// the same local (x, z) frame the ego/traffic entities render in.
export interface RouteStep {
  type: string;
  modifier: string;
  instruction: string;
  location: [number, number]; // [lat, lng]
  distance: number;
}
export interface RoutePayload {
  type: "route";
  waypoints: [number, number][]; // [lat, lng] pairs
  steps: RouteStep[];
}

export interface SimulationEvent {
  event_id: string;
  simulation_id: string;
  run_id: string;
  tick: number;
  timestamp: string;
  type: string;
  actor: string;
  cause: string;
  decision: string;
  confidence: number;
  metadata: Record<string, any>;
}

export interface EventStreamPayload {
  type: "event";
  event: SimulationEvent;
}
