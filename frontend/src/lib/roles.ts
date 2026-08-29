import type { SurroundTrack, PerceptionObject, PredictionState } from '../types/protocol';

/**
 * Honest semantic-role derivation (Phase 7.5+ §8, §9).
 *
 * Every role here is a consequence of real streamed state — a track's own
 * class / kinematics, the sensor's `sensed_lead_vehicle` call, the Phase 7
 * `cut_in.track_id`, or the planner's `is_changing_lane`. Nothing is a
 * capability the backend doesn't have (no map, no crosswalk model, no
 * traffic-signal state). A track that matches no rule is just "tracked".
 */

export type TrackRole =
  | 'cut-in'
  | 'lead'
  | 'oncoming'
  | 'adjacent'
  | 'pedestrian'
  | 'cyclist'
  | 'static'
  | 'tracked';

export type Relevance = 'primary' | 'secondary' | 'ambient';

export interface RoleCtx {
  egoHeadingRad: number;
  prediction: PredictionState | null;
  perception: PerceptionObject[];
  /** ego lateral offset (frenet d), metres — for the adjacent-lane test. */
  egoLateralM: number;
  laneChanging: boolean;
}

export interface AnnotatedTrack {
  track: SurroundTrack;
  num: number;
  role: TrackRole;
  relevance: Relevance;
  /** + = closing on the ego, − = opening. Projected onto the bearing. */
  closingMps: number;
  speedMps: number;
}

export const ROLE_LABEL: Record<TrackRole, string> = {
  'cut-in': 'Cut-in',
  lead: 'Lead vehicle',
  oncoming: 'Oncoming',
  adjacent: 'Adjacent lane',
  pedestrian: 'Pedestrian',
  cyclist: 'Cyclist',
  static: 'Static obstacle',
  tracked: 'Tracked',
};

const trackNum = (id: string) => {
  const m = id.match(/(\d+)/);
  return m ? parseInt(m[1], 10) : NaN;
};

export function annotateTracks(tracks: SurroundTrack[], ctx: RoleCtx): AnnotatedTrack[] {
  const cutInId =
    ctx.prediction?.cut_in.active ? ctx.prediction.cut_in.track_id : null;
  const hasSensedLead = ctx.perception.some((p) => p.id === 'sensed_lead_vehicle');

  // "lead" = the nearest track roughly straight ahead, but only claimed
  // when the sensor itself reported a lead vehicle this tick.
  let leadCandidate: SurroundTrack | null = null;
  if (hasSensedLead) {
    let best = Infinity;
    for (const t of tracks) {
      const ahead = Math.abs(((t.azimuth_deg + 180) % 360) - 180) < 25;
      if (ahead && t.range_m < best) {
        best = t.range_m;
        leadCandidate = t;
      }
    }
  }

  return tracks.map((track) => {
    const num = trackNum(track.id);
    const speedMps = Math.hypot(track.vx, track.vz);
    const bearingRad = (track.azimuth_deg * Math.PI) / 180;
    const closingMps = -(track.vx * Math.sin(bearingRad) + track.vz * Math.cos(bearingRad));

    // velocity heading vs ego heading — honest oncoming test from real vx/vz
    const vHeading = Math.atan2(track.vx, -track.vz);
    const rel = Math.cos(vHeading - ctx.egoHeadingRad);

    let role: TrackRole = 'tracked';
    if (cutInId != null && num === cutInId) role = 'cut-in';
    else if (leadCandidate && track.id === leadCandidate.id) role = 'lead';
    else if (track.class === 'PEDESTRIAN') role = 'pedestrian';
    else if (track.class === 'BICYCLE' || track.class === 'MOTORCYCLE') role = 'cyclist';
    else if (track.class === 'TRAFFIC_CONE' || speedMps < 0.5) role = 'static';
    else if (speedMps > 3 && rel < -0.4) role = 'oncoming';
    else if (Math.abs(track.azimuth_deg) > 35 && Math.abs(track.azimuth_deg) < 145 && track.range_m < 30)
      role = 'adjacent';

    let relevance: Relevance = 'ambient';
    if (role === 'cut-in' || role === 'lead' || role === 'pedestrian' || track.range_m < 15)
      relevance = 'primary';
    else if (track.range_m < 40 || (ctx.laneChanging && role === 'adjacent'))
      relevance = 'secondary';

    return { track, num, role, relevance, closingMps, speedMps };
  });
}
