export type Coordinate = { x: number; y: number; z: number };
export type FrenetCoordinate = { s: number; d: number };
export type Quaternion = { x: number; y: number; z: number; w: number };

export interface SceneControl {node_id:number; kind:string; station_m:number; stop_station_m:number; phase:string|null;}

export interface EntityState {
  id: string;
  pose: Coordinate;
  rotation?: Quaternion; // Only if 3D orientation is sent, else compute from yaw
  yaw: number;
  velocity: number;
  acceleration: number;
  frenet: FrenetCoordinate;
  relative_heading_deg?: number;
  lateral_speed_mps?: number;
}

// Real decision strings from app/services/inference.py's label encoder --
// NOT an abstract enum invented for this protocol. Do not "clean these up"
// to SCREAMING_SNAKE_CASE without changing the backend to match; they are
// the literal class names the deployed classifier was trained on.
export type Decision = "Accelerate" | "Decelerate" | "Maintain Speed";

export interface SensorHealthState {
  camera_required?: boolean;
  camera_age_s?: number | null;
  camera_recovery_samples?: number;
  camera_view_health?: [string, number | null, number, boolean][];
  monitored: boolean;
  ready: boolean;
  reason: string;
  forward_age_s: number | null;
  surround_age_s: number | null;
  recovery_samples: number;
  faults?: Record<string, string | null>;
}

export interface MapFeatureEvidence {
  source: string; way_id: number; map_timestamp: string | null; map_version: number | null;
  recorded_width_m: number | null; width_tag: string | null; lane_width_verified: false; live_verified: false;
  access: { status: 'not_tagged' | 'recorded_allowance' | 'recorded_restriction' | 'conditional_unresolved'; live_verified: false; mission_eligible: boolean; tags: Record<string,string> };
}
export interface ParkingMapEvidence {
  lot: MapFeatureEvidence; aisle: MapFeatureEvidence; parking_type_tag: string | null;
  surface_tag: string | null; capacity_tag: string | null; individual_bay_geometry: 'unavailable';
  vacancy_live_verified: false; goal_pose_source: string;
}

export interface EgoState extends EntityState {
  control_perception?: {mode: string; source: string; fresh: boolean; capture_time_s?: number | null; age_s?: number | null; detections?: number; confirmed_controls?: number; fault?: string | null; camera_views?: {camera_id:string;capture_time_s:number;width:number;height:number;hfov_deg:number;detections:number}[]};
  camera_view_health?: [string, number | null, number, boolean][];
  sensor_health?: SensorHealthState;
  sensed_lead_gap_m?: number | null;
  configured_cruise_speed_kmh?: number;
  driving_state?: string;
  has_arrived?: boolean;
  remaining_distance_m?: number;
  parking_status?: string;
  parking_area?: [number, number][] | null;
  parking?: { parked: boolean; active: boolean; maneuvering: boolean; contained: boolean; mapped_bay: boolean; map_evidence?: ParkingMapEvidence } | null;
  junction?: { visually_observed?: boolean; kind: string; phase: string | null; phase_source: string; distance_to_stop_m: number; sensor_clear: boolean } | null;
  road_semantics?: {
    status: "mapped_rules_partial" | "unknown_or_ambiguous";
    posted_speed_limit_kmh: number | null;
    lane_count: number | null;
    oneway: string | null;
    way_id: number | null;
    map_timestamp: string | null;
    recorded_width_m?: number | null;
    width_tag?: string | null;
    map_version?: number | null;
    lane_width_verified: boolean;
    parking_verified: boolean;
  };
  geographic_position?: {lat: number; lng: number};
  decision: Decision;
  confidence: number; // 0..1, the classifier's max class probability
  // Which physical constraint is currently binding the speed controller --
  // "car_following" is the IDM braking-for-traffic case, "lateral_accel_limit"
  // is cornering, etc. See app/services/physics_engine.py's speed_limit_reason.
  speed_limit_reason: string;
  target_velocity: number | null;
  control_source?: string;
  requested_acceleration?: number | null;
  steering_angle: number;
  throttle: number;
  brake: number;
  world_pos?: Coordinate;
  world_heading_deg?: number;
  road_heading_deg?: number;
  relative_heading_deg?: number;
  steering_angle_rad?: number;
  wheels?: {
    steering_rad: [number, number, number, number];
    rotation_rad: [number, number, number, number];
    wheelbase_m: number; track_width_m: number; radius_m: number;
    pose_reference: 'vehicle_center';
  };
  grip_budget?: {
    scope: string;
    limit_mps2?: number;
    lateral_bound_mps2?: number;
    total_bound_mps2?: number;
    already_infeasible: boolean;
    recovery_active?: boolean;
    within_modeled_budget?: boolean;
    steering_limited?: boolean;
    acceleration_limited?: boolean;
  } | null;
  kinematic_sideslip_rad?: number;
  lateral_accel_mps2?: number;
  signed_lateral_accel_mps2?: number;
  yaw_rate_degps?: number;
  signals?: { left: boolean; right: boolean; hazards: boolean; brake: boolean };
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
  continuous_modeled_corridor_verified?: boolean;
  continuous_route_station_verified?: boolean;
  initial_actual_steering_checked?: boolean;
  sampled_modeled_vehicle_envelope_verified?: boolean;
  full_cartesian_swept_corridor_verified?: boolean;
  modeled_straight_vehicle_width_m?: number | null;
  continuous_min_half_width_margin_m?: number | null;
  joint_plan_status?: 'unavailable' | 'committed' | 'lane_following';
  checked_horizon_s?: number | null;
  feasibility_friction_mu?: number | null;
  peak_combined_lateral_accel_mps2?: number | null;
  initial_speed_recovery?: boolean;
  trajectory: Coordinate[];
  trajectory_source?: string;
  source_tick?: number | null;
  lateral_target_m?: number;
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
  overlap_detected?: boolean;
  body_contained?: boolean;
  swept_body_contained?: boolean;
  footprint_collision_risk?: boolean;
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
  params?: Record<string, number | boolean>;
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

// app/services/prediction/* -- Phase 7. Per-agent 3 s / 0.1 s forecast
// trails, intent distributions, and the current proactive cut-in response.
// Consumes the ego's sensor-resolved SurroundTrack picture, never raw NPCs.
export interface PredictedTrailPoint {
  t: number; // seconds ahead
  x: number;
  z: number;
}
export interface IntentWeight {
  label:
    | "LANE_KEEP"
    | "MERGE_LEFT"
    | "MERGE_RIGHT"
    | "DECELERATING"
    | "STOPPING"
    | string;
  p: number; // 0..1
}
export interface AgentPredictionState {
  track_id: number;
  intent: IntentWeight[]; // sorted, most likely first
  trail: PredictedTrailPoint[]; // 30 points (3.0 s @ 0.1 s)
}
export interface CutInState {
  active: boolean; // the ego is currently easing off for this cut-in
  probability: number; // 0..1, highest p_cut_in among intruding agents
  track_id: number | null;
  time_to_cross_s: number | null;
}
export interface PredictionState {
  agents: AgentPredictionState[];
  cut_in: CutInState;
  proactive_decel_mps2: number; // 0.0 when no proactive action this tick
}

// ADR-001 item 5: the learned model as an analytics channel, not a policy.
export interface DriverAnalytics {
  decision: Decision;
  confidence: number;
  shap: ShapResult;
  anomaly: AnomalyResult;
  driver_score: DriverScore;
}

// Protocol v3 (ADR-001 item 7) -- one message, layered channels:
//   pose      small, would-be-high-rate ego kinematics
//   semantic  everything the HMI needs to explain a decision
//   heavy     large payloads (surround tracks, per-agent predictions),
//             candidates for on-demand / delta-encoding in a later phase.
export interface TelemetryStatePayload {
  epoch: number;
  route_version: number;
  route_status: string;
  protocol_version: "3.0";
  simulation_id: string;
  run_id: string;
  tick: number;
  simulation_time_s: number;
  timestamp: string;
  coordinate_frame: "dds_world_v1";
  units: "SI";
  type: "state";
  channels: {
    pose: {
      ego: EgoState;
    };
    semantic: {
      traffic: EntityState[];
      perception: PerceptionObject[];
      planner: PlannerState;
      safety_shield: SafetyShieldState;
      scenario?: ScenarioState;
      driver_analytics: DriverAnalytics;
    };
    heavy: {
      scene_controls?: SceneControl[];
      surround_perception?: SurroundTrack[];
      prediction?: PredictionState;
    };
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
  epoch?: number;
  route_version?: number;
  type: "route";
  road_context?: [number, number][];
  waypoints: [number, number][]; // [lat, lng] pairs
  steps: RouteStep[];
}

export interface CommandResult {
  type: 'command_result';
  command_id: string | null;
  status: string;
  message?: string | null;
  epoch?: number;
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
  metadata: Record<string, unknown>;
}

export interface EventStreamPayload {
  type: "event";
  event: SimulationEvent;
}
