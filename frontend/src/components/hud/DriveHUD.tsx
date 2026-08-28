'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { Panel, Stat, toneColor, toneMuted } from '../primitives';
import { useTween } from '../../hooks/useTween';
import {
  deriveAutonomy,
  pathClearance,
  PATH_CLEARANCE_LABEL,
} from '../../lib/consoleState';

/**
 * The one HUD (§13). Frosted, minimal, Tesla-restraint. It answers
 * "what is the vehicle doing right now?" without the operator reading the
 * rail: a big gliding speed, one autonomous-state pill (derived from the
 * real speed_limit_reason + safety_shield + cut_in — not an invented AI
 * decision), a path-clearance badge, and a subtle steering readout.
 */
export function DriveHUD() {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);

  const speedKmh = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 260));
  const targetKmh = Math.round((ego?.target_velocity ?? 0) * 3.6);
  const steerDeg = (ego?.steering_angle ?? 0) * (180 / Math.PI);

  const state = deriveAutonomy({ ego: ego ?? null, shield: shield ?? null, prediction: prediction ?? null, planner: planner ?? null });
  const clr = pathClearance(shield ?? null, prediction ?? null);
  const clrTone = clr === 'hazard' ? 'crit' : clr === 'caution' ? 'warn' : 'ok';

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
      <Panel frost style={{ padding: 'var(--space-3) var(--space-4)', pointerEvents: 'auto', minWidth: 240 }}>
        <div style={{ display: 'flex', alignItems: 'baseline', justifyContent: 'space-between', gap: 'var(--space-4)' }}>
          <Stat value={Math.round(speedKmh)} unit="KM/H" size="lg" />
          {/* steering readout — small, always visible (§13) */}
          <span className="font-mono" style={{ fontSize: 10.5, color: 'var(--text-faint)' }}>
            <span
              aria-hidden
              style={{
                display: 'inline-block',
                width: 12,
                height: 12,
                marginRight: 5,
                borderRadius: '50%',
                border: '1.5px solid var(--text-faint)',
                borderTopColor: 'var(--brand)',
                transform: `rotate(${steerDeg * 3}deg)`,
                transition: 'transform var(--dur-fast) linear',
                verticalAlign: '-2px',
              }}
            />
            {steerDeg >= 0 ? '+' : ''}
            {steerDeg.toFixed(1)}°
          </span>
        </div>

        {/* autonomous-state pill — crossfades on change via keyed remount */}
        <div
          key={state.id}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 8,
            marginTop: 8,
            animation: 'dds-grow var(--dur) var(--ease-out)',
          }}
        >
          <span
            style={{
              width: 7,
              height: 7,
              borderRadius: '50%',
              background: toneColor[state.tone],
              boxShadow: `0 0 8px ${toneColor[state.tone]}`,
              flex: 'none',
            }}
          />
          <span
            className="font-mono"
            style={{ fontSize: 12, fontWeight: 600, color: toneColor[state.tone], letterSpacing: '0.02em' }}
          >
            {state.label}
          </span>
          <span className="font-mono" style={{ fontSize: 10.5, color: 'var(--text-faint)' }}>
            {state.sub}
          </span>
        </div>

        <div
          className="font-mono"
          style={{ fontSize: 10.5, color: 'var(--text-faint)', marginTop: 5, whiteSpace: 'nowrap' }}
        >
          target {targetKmh} km/h
          <span style={{ margin: '0 6px', opacity: 0.5 }}>·</span>
          <span style={{ color: toneColor[clrTone], background: toneMuted[clrTone], padding: '1px 6px', borderRadius: 'var(--radius-xs)' }}>
            {PATH_CLEARANCE_LABEL[clr]}
          </span>
        </div>
      </Panel>
    </div>
  );
}
