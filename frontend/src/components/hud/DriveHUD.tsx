'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { Panel, Stat, Chip } from '../primitives';
import { useTween } from '../../hooks/useTween';

/** Which physical constraint is currently binding the speed controller.
 *  Traceable 1:1 to app/services/physics_engine.py's speed_limit_reason
 *  (post-ADR-001: no `ai_decelerate` — the model does not set speed). */
const CONSTRAINT: Record<string, string> = {
  cruise: 'cruising',
  car_following: 'following traffic (IDM)',
  lateral_accel_limit: 'lateral-accel cap',
  tracking_correction: 'correcting heading',
  predictive_cut_in: 'predicted cut-in',
  safety_shield_override: 'safety-shield override',
};

/** The one HUD. Frosted, minimal, values glide (Tesla-smooth). */
export function DriveHUD() {
  const ego = useSimulationStore((s) => s.ego);
  const cutIn = useSimulationStore((s) => s.prediction?.cut_in);

  const speedKmh = useTween((ego?.velocity ?? 0) * 3.6, 260);
  const targetKmh = Math.round((ego?.target_velocity ?? 0) * 3.6);
  const steerDeg = ((ego?.steering_angle ?? 0) * (180 / Math.PI));
  const reason = ego?.speed_limit_reason ?? 'cruise';
  const constraintLabel = CONSTRAINT[reason] ?? reason;

  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'flex-end',
        justifyContent: 'space-between',
        gap: 'var(--space-4)',
        pointerEvents: 'none',
      }}
    >
      <Panel frost style={{ padding: 'var(--space-3) var(--space-4)', pointerEvents: 'auto' }}>
        <Stat value={Math.round(speedKmh)} unit="KM/H" size="lg" />
        <div
          className="font-mono"
          style={{ fontSize: 11, color: 'var(--text-muted)', marginTop: 6, whiteSpace: 'nowrap' }}
        >
          target {targetKmh}
          <span style={{ color: 'var(--text-faint)' }}> &middot; </span>
          binding <b style={{ color: 'var(--brand)', fontWeight: 600 }}>{constraintLabel}</b>
          <span style={{ color: 'var(--text-faint)' }}> &middot; </span>
          steer {steerDeg >= 0 ? '+' : ''}{steerDeg.toFixed(1)}&deg;
        </div>
      </Panel>

      {cutIn?.active && (
        <div style={{ pointerEvents: 'auto', animation: 'dds-grow var(--dur) var(--ease-out)' }}>
          <Chip tone="warn">
            easing off &nbsp; P&nbsp;{Math.round((cutIn.probability ?? 0) * 100)}%
            {cutIn.time_to_cross_s != null && <> &middot; {cutIn.time_to_cross_s.toFixed(1)}s</>}
          </Chip>
        </div>
      )}
    </div>
  );
}
