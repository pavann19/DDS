import type {
  EgoState,
  SafetyShieldState,
  PredictionState,
  PlannerState,
  PerceptionObject,
  SensorHealthState,
} from '../types/protocol';
import type { Tone } from '../components/primitives';

/**
 * Derived console state (Phase 7.5+ §12, §13, §15) — pure functions over
 * the real protocol channels. Nothing here invents state; it only reads
 * `speed_limit_reason`, `safety_shield`, `cut_in`, and `is_changing_lane`
 * and names the situation the way an operator would.
 */

export type AutonomyId =
  | 'cruising'
  | 'following'
  | 'easing'
  | 'cornering'
  | 'lane_change'
  | 'tracking'
  | 'override'
  | 'holding'
  | 'arriving'
  | 'limited'
  | 'arrived';

export interface AutonomyState {
  id: AutonomyId;
  label: string;
  sub: string;
  tone: Tone; // nominal → ok, caution → warn, critical → crit
}

export interface ConsoleCtx {
  ego: EgoState | null;
  shield: SafetyShieldState | null;
  prediction: PredictionState | null;
  planner: PlannerState | null;
}

/** Recovery approvals permit restricted motion, not a cleared safety condition. */
export function isCriticalSafety(shield: SafetyShieldState | null): boolean {
  return Boolean(shield && (!shield.approved || shield.risk_level === 'CRITICAL' ||
    shield.body_contained === false || shield.swept_body_contained === false));
}

export function displayedCameraCurrent(ready: boolean, telemetryCapture: number | undefined, telemetryAge: number | null | undefined, displayedCapture: number | undefined): boolean {
  if (!ready || !Number.isFinite(telemetryCapture) || !Number.isFinite(displayedCapture) || telemetryAge == null || !Number.isFinite(telemetryAge)) return false;
  const age = telemetryAge + telemetryCapture! - displayedCapture!;
  return age >= -1e-6 && age <= .35 + 1e-9;
}

export function deriveDriveHeading(ctx: ConsoleCtx, connected: boolean, paused: boolean): AutonomyState {
  if (!connected) return {id:'holding',label:'Waiting for connection',sub:'Vehicle information unavailable',tone:'warn'};
  const state = deriveAutonomy(ctx);
  if (isCriticalSafety(ctx.shield)) return state;
  if (paused) return {id:'holding',label:'Drive paused',sub:'Ready when you are',tone:'ok'};
  return state.id === 'tracking' ? {...state,label:'Following the road'} : state;
}

export function deriveAutonomy(ctx: ConsoleCtx): AutonomyState {
  const { ego, shield, prediction, planner } = ctx;
  const reason = ego?.speed_limit_reason ?? 'cruise';
  const speed = (ego?.velocity ?? 0) * 3.6;

  if (isCriticalSafety(shield)) {
    return {
      id: 'override',
      label: 'Safety override',
      sub: shield!.reasons?.[0] ?? (shield!.override_action === 'EMERGENCY_BRAKE' ? 'Emergency braking' : 'Restricted safety recovery'),
      tone: 'crit',
    };
  }
  if (ego?.sensor_health && (!ego.sensor_health.monitored || !ego.sensor_health.ready)) {
    const healthReason = typeof ego.sensor_health.reason === 'string' ? ego.sensor_health.reason : 'sensor_missing';
    const camera = healthReason.startsWith('sensor_camera_');
    const recovering = healthReason.endsWith('_recovering');
    return { id: speed < 1 ? 'holding' : 'limited',
      label: !ego.sensor_health.monitored ? 'Sensor checks pending' : camera ? recovering ? 'Checking camera' : 'Camera unavailable' : recovering ? 'Checking sensors' : 'Sensors unavailable',
      sub: recovering ? camera ? 'Confirming distinct camera frames' : 'Confirming fresh readings'
        : speed < 1 ? 'Stopped · awaiting fresh readings' : 'Slowing · readings missing or stale',
      tone: 'warn' };
  }
  if (ego?.grip_budget?.recovery_active || ego?.grip_budget?.within_modeled_budget === false) {
    return { id: speed < 1 ? 'holding' : 'limited', label: 'Grip recovery',
      sub: 'Slowing and reducing steering demand', tone: 'warn' };
  }
  if (ego?.has_arrived) {
    return speed < 1
      ? { id: 'arrived', label: ego.parking?.parked ? 'Parked' : 'Destination reached', sub: ego.parking?.parked ? 'Mapped parking area · simulated vacancy' : 'Stopped · parking location unverified', tone: 'ok' }
      : { id: 'arriving', label: 'Arriving', sub: 'Slowing to stop', tone: 'ok' };
  }
  if (reason === 'joint_trajectory_infeasible') {
    return { id: speed < 1 ? 'holding' : 'limited', label: 'Waiting for a safe path',
      sub: speed < 1 ? 'Stopped · no feasible maneuver' : 'Slowing down · no feasible maneuver',
      tone: 'warn' };
  }
  if (reason === 'turn_reference_infeasible' || reason === 'turn_geometry_infeasible') {
    return { id: speed < 1 ? 'holding' : 'limited', label: 'Unable to follow route',
      sub: speed < 1 ? 'Stopped · turn needs a new plan' : 'Stopping safely · turn needs a new plan',
      tone: 'warn' };
  }
  if (reason === 'steering_demand_friction_limit') {
    return { id: 'cornering', label: 'Adapting to grip',
      sub: 'Steering demand limited by available grip', tone: 'warn' };
  }
  if ((reason === 'turn_geometry_speed_limit' || reason === 'turn_steering_rate') &&
      typeof ego?.target_velocity === 'number' && ego.target_velocity <= .05) {
    return { id: speed < 1 ? 'holding' : 'limited', label: speed < 1 ? 'Waiting for a safe turn' : 'Slowing for a safe turn',
      sub: speed < 1 ? 'Stopped · steering constraints require a new path' : 'Slowing · steering constraints require a new path', tone: 'warn' };
  }
  if (['camera_prototype','scene_camera_prototype'].includes(ego?.control_perception?.mode ?? '') && ego?.junction && !ego.junction.visually_observed) {
    return {id: speed < 1 ? 'holding' : 'limited', label: 'Checking roadside controls',
      sub: 'Visual evidence missing, stale or uncertain', tone: 'warn'};
  }
  if (ego?.junction?.phase === 'GREEN' && reason === 'signal_green_approach') {
    return { id: 'easing', label: 'Approaching signal',
      sub: 'Green observed · maintaining stopping distance', tone: 'ok' };
  }
  if (ego?.junction && (reason.startsWith('signal_') || reason === 'stop_sign' || reason === 'yield_conflict')) {
    return { id: speed < 1 ? 'holding' : 'easing',
      label: ego.junction.kind === 'traffic_signals' ? `Signal ${ego.junction.phase?.toLowerCase() ?? 'unknown'}` : ego.junction.kind === 'stop' ? 'Stop sign' : 'Yielding',
      sub: `${Math.max(0, Math.round(ego.junction.distance_to_stop_m))} m to modeled stop line · simulated sensors`, tone: 'warn' };
  }
  if (ego?.parking?.active) {
    return { id: 'arriving', label: ego.parking.maneuvering ? 'Parking' : 'Parking approach',
      sub: 'Mapped area · planner-selected pose · simulated sensors', tone: 'ok' };
  }
  if (ego?.driving_state === 'FINAL_APPROACH') {
    return { id: 'arriving', label: 'Approaching destination', sub: `${Math.round(ego.remaining_distance_m ?? 0)} m remaining`, tone: 'ok' };
  }
  if (prediction?.cut_in.active) {
    const p = Math.round((prediction.cut_in.probability ?? 0) * 100);
    return { id: 'easing', label: 'Easing off', sub: `predicted cut-in · P ${p}%`, tone: 'warn' };
  }
  if (planner?.is_changing_lane) {
    return { id: 'lane_change', label: 'Lane change', sub: 'executing lateral maneuver', tone: 'warn' };
  }
  if (ego?.road_semantics?.status === 'unknown_or_ambiguous') {
    return { id: 'limited', label: 'Limited map coverage',
      sub: reason === 'map_uncertainty_hold' ? 'Map match uncertain · holding previous speed cap' : 'Road rules unverified · conservative speed',
      tone: 'warn' };
  }
  if (reason === 'car_following') {
    return { id: 'following', label: 'Following traffic', sub: 'IDM gap control', tone: 'ok' };
  }
  if (reason === 'lateral_accel_limit' || reason === 'low_friction_curvature') {
    return { id: 'cornering', label: reason === 'low_friction_curvature' ? 'Slick curve' : 'Cornering',
      sub: reason === 'low_friction_curvature' ? 'friction-limited speed' : 'lateral-accel limited', tone: 'warn' };
  }
  if (reason === 'turn_geometry_speed_limit' || reason === 'turn_steering_rate') {
    return { id: 'cornering', label: 'Preparing for turn',
      sub: reason === 'turn_steering_rate' ? 'Steering-rate limited speed' : 'Turn geometry limited speed', tone: 'warn' };
  }
  if (reason === 'tracking_correction') {
    return { id: 'tracking', label: 'Correcting line', sub: 'heading-error limited', tone: 'ok' };
  }
  if (speed < 1) {
    return { id: 'holding', label: 'Holding', sub: 'stopped', tone: 'ok' };
  }
  return { id: 'cruising', label: 'Cruising', sub: `target ${Math.round((ego?.target_velocity ?? 0) * 3.6)} km/h`, tone: 'ok' };
}

export type PathClearance = 'clear' | 'caution' | 'hazard' | 'unavailable';

export function pathClearance(shield: SafetyShieldState | null, prediction: PredictionState | null, health?: SensorHealthState): PathClearance {
  const risk = shield?.risk_level ?? 'NONE';
  if (!shield) return 'unavailable';
  if (isCriticalSafety(shield) || risk === 'HIGH') return 'hazard';
  if (health && (!health.monitored || !health.ready)) return 'unavailable';
  if (risk === 'MEDIUM' || prediction?.cut_in.active) return 'caution';
  return 'clear';
}

export const PATH_CLEARANCE_LABEL: Record<PathClearance, string> = {
  unavailable: 'Sensor readings unavailable',
  clear: 'Path clear',
  caution: 'Caution ahead',
  hazard: 'Hazard in path',
};

export type Salience = 'quiet' | 'active' | 'alert';

/** Per-panel salience so quiet subsystems recede and the reacting one
 *  stands out (§12). Keyed by the panel's short id. */
export function panelSalience(id: string, ctx: ConsoleCtx): Salience {
  const { ego, shield, prediction, planner } = ctx;
  const reason = ego?.speed_limit_reason ?? 'cruise';

  switch (id) {
    case 'ego':
      return reason !== 'cruise' && reason !== 'tracking_correction' ? 'active' : 'quiet';
    case 'perception': {
      return 'quiet';
    }
    case 'prediction':
      return prediction?.cut_in.active ? 'alert' : (prediction?.agents.length ?? 0) > 0 ? 'active' : 'quiet';
    case 'safety':
      if (isCriticalSafety(shield)) return 'alert';
      return (shield?.risk_level ?? 'NONE') !== 'NONE' ? 'active' : 'quiet';
    case 'planner':
      return planner?.is_changing_lane ? 'alert' : 'quiet';
    case 'analytics':
      return 'quiet';
    default:
      return 'quiet';
  }
}

/* --------------------------------------------------------------------------
 * Maneuver card (§6, §15) — the bottom-centre "what is the planner doing
 * and why". This is the DETERMINISTIC planner (Frenet candidates + IDM +
 * safety shield); the learned model is not in this loop (ADR-001), so
 * there is deliberately no "confidence %". Title + factors are all read
 * from real channels.
 * ----------------------------------------------------------------------- */
export interface ManeuverCtx extends ConsoleCtx {
  perception: PerceptionObject[];
  adjacentOccupied: boolean;
}

export interface Maneuver {
  title: string;
  tone: Tone;
  factors: string[];
  producer: string;
}

export function deriveManeuver(ctx: ManeuverCtx): Maneuver {
  const { ego, shield, prediction, planner, perception } = ctx;
  if (!ego || !shield) return {
    title: 'Telemetry unavailable', tone: 'warn',
    factors: ['Waiting for current vehicle and safety telemetry'], producer: 'Telemetry stream',
  };
  const reason = ego?.speed_limit_reason ?? 'cruise';
  const lead = [...perception].sort((a, b) => a.distance - b.distance)[0];
  const factors: string[] = [];

  let title = 'Hold lane & cruise';
  let tone: Tone = 'ok';
  let producer = 'Velocity Governor · Cruise Tracking';

  if (isCriticalSafety(shield)) {
    title = shield.override_action === 'EMERGENCY_BRAKE' ? 'Emergency brake' : 'Safety override';
    tone = 'crit';
    producer = `Safety Shield · ${shield.override_action === 'EMERGENCY_BRAKE' ? 'Emergency Brake' : 'Veto Override'}`;
  } else if (ego.sensor_health && (!ego.sensor_health.monitored || !ego.sensor_health.ready)) {
    title = 'Hold for sensor readings';
    tone = 'warn';
    producer = 'Sensor Freshness Guard';
    factors.push(ego.sensor_health.reason === 'sensor_recovering'
      ? 'Confirming fresh measurements before resuming' : 'Sensor readings are missing or stale');
  } else if (reason === 'steering_demand_friction_limit') {
    title = 'Adapting to grip';
    tone = 'warn';
    producer = 'Vehicle Controller · Steering Demand & Grip';
  } else if (prediction?.cut_in.active) {
    title = 'Ease off — predicted cut-in';
    tone = 'warn';
    producer = 'Cut-In Predictor · Proactive Decel';
  } else if (planner?.is_changing_lane) {
    title = 'Change lane — overtake';
    tone = 'warn';
    producer = 'Frenet Lattice · Lane Change Maneuver';
  } else if (reason === 'car_following') {
    title = 'Follow lead vehicle';
    tone = 'ok';
    producer = 'IDM · Car-Following Gap Control';
  } else if (reason === 'lateral_accel_limit') {
    title = 'Slow for curve';
    tone = 'warn';
    producer = 'Curvature Governor · Lateral Accel Cap';
  } else if (reason === 'low_friction_curvature') {
    title = 'Slow for slick curve';
    tone = 'warn';
    producer = 'Friction Adaptation · Curvature De-Rate';
  } else if (reason === 'tracking_correction') {
    title = 'Correct lane line';
    tone = 'ok';
    producer = 'Lateral Controller · Line Correction';
  } else if (reason === 'spatiotemporal_plan') {
    title = 'Trajectory Profile';
    tone = 'ok';
    producer = 'Spatiotemporal Planner · Trajectory Profile';
  } else if (reason === 'predictive_cut_in') {
    title = 'Ease off — cut-in risk';
    tone = 'warn';
    producer = 'Prediction Monitor · Proactive Decel';
  } else if ((ego?.velocity ?? 0) * 3.6 < 1) {
    title = 'Hold — stopped';
    tone = 'ok';
    producer = 'Stationary Hold Controller';
  }

  // Future-proof architecture compatibility: if an explicit learned policy
  // or controller provenance is provided in telemetry, display it directly.
  const explicitPolicy = 'policy_source' in ego && typeof ego.policy_source === 'string' ? ego.policy_source : undefined;
  if (explicitPolicy) {
    producer = `${explicitPolicy} · ${producer}`;
  }

  // Contributing factors — only real, observed conditions.
  if (reason === 'steering_demand_friction_limit') factors.push('Steering-demand grip constraint active');
  if (lead) {
    factors.push(
      `Lead vehicle at ${lead.distance.toFixed(1)} m, closing ${lead.rel_velocity >= 0 ? '+' : ''}${lead.rel_velocity.toFixed(1)} m/s`,
    );
  }
  if (prediction?.cut_in.active) {
    factors.push(
      `Cut-in P ${Math.round((prediction.cut_in.probability ?? 0) * 100)}%` +
        (prediction.cut_in.time_to_cross_s != null ? `, ${prediction.cut_in.time_to_cross_s.toFixed(1)} s to cross` : ''),
    );
  }
  if (reason === 'lateral_accel_limit' && planner) {
    factors.push(`Lateral-accel cap: curvature ${planner.curvature.toFixed(4)} /m ahead`);
  }
  if (reason === 'low_friction_curvature' && planner) {
    factors.push(`Low-friction curvature cap: ${planner.curvature.toFixed(4)} /m ahead`);
  }
  if (planner?.is_changing_lane) {
    factors.push('Quintic lateral maneuver in progress');
  }
  for (const r of shield?.reasons ?? []) factors.push(r);
  if (shield?.approved) {
    factors.push(`Safety shield: approved, TTC ${shield.ttc_s != null ? shield.ttc_s.toFixed(1) + ' s' : '> 9.9 s'}`);
  }
  if (!ctx.adjacentOccupied && !planner?.is_changing_lane) {
    factors.push('Adjacent lanes clear of lateral incursions');
  }

  return { title, tone, factors: factors.slice(0, 4), producer };
}
