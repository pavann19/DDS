'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { toneColor } from '../primitives';
import { useTween } from '../../hooks/useTween';
import { deriveAutonomy, pathClearance, PATH_CLEARANCE_LABEL } from '../../lib/consoleState';

/**
 * The automotive HUD bar (§13) — three clusters, cockpit visual language:
 *   left    speed (display face) · target roundel
 *   centre  autonomous-state pill (beacon + real state) · steering wheel
 *   right   forward radar (lead dist / rel speed) · path-clearance badge
 *
 * Every value traces to a real channel: `speed_limit_reason` +
 * `safety_shield` + `cut_in` drive the state; `sensed_lead_vehicle` drives
 * the radar. No learned-model decision / confidence anywhere (ADR-001).
 */
const mono: React.CSSProperties = { fontFamily: 'var(--font-mono)' };

export function DriveHUD() {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  const perception = useSimulationStore((s) => s.perception);

  const speedKmh = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 260));
  const targetKmh = Math.round((ego?.target_velocity ?? 0) * 3.6);
  const steerDeg = (ego?.steering_angle ?? 0) * (180 / Math.PI);

  const state = deriveAutonomy({ ego: ego ?? null, shield: shield ?? null, prediction: prediction ?? null, planner: planner ?? null });
  const clr = pathClearance(shield ?? null, prediction ?? null);
  const clrTone = clr === 'hazard' ? 'crit' : clr === 'caution' ? 'warn' : 'ok';

  const lead = perception.find((p) => p.id === 'sensed_lead_vehicle');

  return (
    <div
      className="dds-glass"
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        gap: 'var(--space-5)',
        height: 74,
        padding: '0 var(--space-5)',
        borderRadius: 'var(--radius-xl)',
      }}
    >
      {/* ---- left: speed cluster ---- */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 'var(--space-4)' }}>
        <div style={{ display: 'flex', alignItems: 'baseline', gap: 6 }}>
          <span
            style={{
              fontFamily: 'var(--font-display)',
              fontSize: 46,
              fontWeight: 800,
              lineHeight: 1,
              letterSpacing: '-0.03em',
              color: 'var(--text-bright)',
              textShadow: '0 0 16px rgba(255,255,255,0.15)',
            }}
          >
            {Math.round(speedKmh)}
          </span>
          <span style={{ ...mono, fontSize: 12, fontWeight: 600, color: 'var(--text-muted)', letterSpacing: '0.08em' }}>
            KM/H
          </span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 10, paddingLeft: 'var(--space-4)', borderLeft: '1px solid var(--border-subtle)' }}>
          <span
            style={{
              ...mono,
              fontSize: 14,
              fontWeight: 700,
              padding: '3px 9px',
              background: 'var(--brand-muted)',
              color: 'var(--brand)',
              border: '1px solid var(--brand-dim)',
              borderRadius: 'var(--radius-sm)',
            }}
          >
            D
          </span>
          <div
            style={{
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              width: 40,
              height: 40,
              borderRadius: '50%',
              background: '#fff',
              lineHeight: 1,
            }}
          >
            <span style={{ fontSize: 7, fontWeight: 800, color: '#000', fontFamily: 'var(--font-display)' }}>TGT</span>
            <span style={{ fontSize: 15, fontWeight: 800, color: '#000', fontFamily: 'var(--font-display)' }}>{targetKmh}</span>
          </div>
        </div>
      </div>

      {/* ---- centre: autonomous-state pill + steering ---- */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 'var(--space-3)' }}>
        <div
          key={state.id}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 10,
            padding: '9px 18px',
            borderRadius: 'var(--radius-pill)',
            background: `color-mix(in srgb, ${toneColor[state.tone]} 12%, transparent)`,
            border: `1px solid color-mix(in srgb, ${toneColor[state.tone]} 45%, transparent)`,
            animation: 'dds-grow var(--dur) var(--ease-out)',
          }}
        >
          <span
            aria-hidden
            style={{
              width: 8,
              height: 8,
              borderRadius: '50%',
              background: toneColor[state.tone],
              boxShadow: `0 0 10px ${toneColor[state.tone]}`,
              animation: 'dds-beacon 1.8s infinite ease-in-out',
            }}
          />
          <span style={{ fontFamily: 'var(--font-display)', fontSize: 13, fontWeight: 700, letterSpacing: '0.06em', color: toneColor[state.tone], textTransform: 'uppercase' }}>
            {state.label}
          </span>
          <span style={{ fontSize: 12, color: 'var(--text-primary)', paddingLeft: 8, borderLeft: '1px solid var(--border-strong)' }}>
            {state.sub}
          </span>
        </div>

        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 7,
            padding: '7px 12px',
            borderRadius: 'var(--radius-pill)',
            background: 'var(--bg-inset)',
            border: '1px solid var(--border-subtle)',
          }}
        >
          <svg
            width="17"
            height="17"
            viewBox="0 0 24 24"
            fill="none"
            stroke="var(--text-muted)"
            strokeWidth="1.8"
            style={{ transform: `rotate(${steerDeg * 1.5}deg)`, transition: 'transform var(--dur-fast) ease-out' }}
            aria-hidden
          >
            <circle cx="12" cy="12" r="9" />
            <path d="M12 3v6M4 12h6M14 12h6M8 17l4-5 4 5" />
          </svg>
          <span style={{ ...mono, fontSize: 12, fontWeight: 600, color: 'var(--text-muted)' }}>
            {steerDeg >= 0 ? '+' : ''}
            {steerDeg.toFixed(1)}°
          </span>
        </div>
      </div>

      {/* ---- right: forward radar + path clearance ---- */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 'var(--space-4)' }}>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ ...mono, fontSize: 9, fontWeight: 600, color: 'var(--text-faint)', letterSpacing: '0.06em' }}>LEAD DIST</span>
          <span style={{ ...mono, fontSize: 15, fontWeight: 700, color: 'var(--text-bright)' }}>
            {lead ? lead.distance.toFixed(1) : '>100'} <span style={{ fontSize: 11, color: 'var(--text-faint)' }}>m</span>
          </span>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ ...mono, fontSize: 9, fontWeight: 600, color: 'var(--text-faint)', letterSpacing: '0.06em' }}>REL SPEED</span>
          <span style={{ ...mono, fontSize: 15, fontWeight: 700, color: 'var(--text-bright)' }}>
            {lead ? `${lead.rel_velocity >= 0 ? '+' : ''}${lead.rel_velocity.toFixed(1)}` : '0.0'}{' '}
            <span style={{ fontSize: 11, color: 'var(--text-faint)' }}>m/s</span>
          </span>
        </div>
        <span
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 6,
            padding: '6px 12px',
            borderRadius: 'var(--radius-sm)',
            ...mono,
            fontSize: 11,
            fontWeight: 700,
            letterSpacing: '0.04em',
            color: toneColor[clrTone],
            background: `color-mix(in srgb, ${toneColor[clrTone]} 14%, transparent)`,
            border: `1px solid color-mix(in srgb, ${toneColor[clrTone]} 40%, transparent)`,
            animation: clr === 'hazard' ? 'dds-hazard 1s infinite alternate' : undefined,
            textTransform: 'uppercase',
          }}
        >
          <span style={{ width: 6, height: 6, borderRadius: '50%', background: 'currentColor' }} />
          {PATH_CLEARANCE_LABEL[clr]}
        </span>
      </div>
    </div>
  );
}
