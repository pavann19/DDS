import type {
  EgoState,
  SafetyShieldState,
  PredictionState,
  PlannerState,
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
  | 'holding';

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

export function deriveAutonomy(ctx: ConsoleCtx): AutonomyState {
  const { ego, shield, prediction, planner } = ctx;
  const reason = ego?.speed_limit_reason ?? 'cruise';
  const speed = (ego?.velocity ?? 0) * 3.6;

  if (shield && !shield.approved) {
    return {
      id: 'override',
      label: 'Safety override',
      sub: shield.override_action === 'EMERGENCY_BRAKE' ? 'emergency braking' : 'shield engaged',
      tone: 'crit',
    };
  }
  if (prediction?.cut_in.active) {
    const p = Math.round((prediction.cut_in.probability ?? 0) * 100);
    return { id: 'easing', label: 'Easing off', sub: `predicted cut-in · P ${p}%`, tone: 'warn' };
  }
  if (planner?.is_changing_lane) {
    return { id: 'lane_change', label: 'Lane change', sub: 'executing lateral maneuver', tone: 'warn' };
  }
  if (reason === 'car_following') {
    return { id: 'following', label: 'Following traffic', sub: 'IDM gap control', tone: 'ok' };
  }
  if (reason === 'lateral_accel_limit') {
    return { id: 'cornering', label: 'Cornering', sub: 'lateral-accel limited', tone: 'warn' };
  }
  if (reason === 'tracking_correction') {
    return { id: 'tracking', label: 'Correcting line', sub: 'heading-error limited', tone: 'ok' };
  }
  if (speed < 1) {
    return { id: 'holding', label: 'Holding', sub: 'stopped', tone: 'ok' };
  }
  return { id: 'cruising', label: 'Cruising', sub: `target ${Math.round((ego?.target_velocity ?? 0) * 3.6)} km/h`, tone: 'ok' };
}

export type PathClearance = 'clear' | 'caution' | 'hazard';

export function pathClearance(shield: SafetyShieldState | null, prediction: PredictionState | null): PathClearance {
  const risk = shield?.risk_level ?? 'NONE';
  if (!shield?.approved || risk === 'CRITICAL' || risk === 'HIGH') return 'hazard';
  if (risk === 'MEDIUM' || prediction?.cut_in.active) return 'caution';
  return 'clear';
}

export const PATH_CLEARANCE_LABEL: Record<PathClearance, string> = {
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
      if (shield && !shield.approved) return 'alert';
      return (shield?.risk_level ?? 'NONE') !== 'NONE' ? 'active' : 'quiet';
    case 'planner':
      return planner?.is_changing_lane ? 'alert' : 'quiet';
    case 'analytics':
      return 'quiet';
    default:
      return 'quiet';
  }
}
