'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { toneColor } from '../primitives';
import { deriveManeuver } from '../../lib/consoleState';

/**
 * The bottom-centre maneuver card (§6, §15) — "what is the planner doing,
 * and why". This is the DETERMINISTIC planner (Frenet candidates + IDM +
 * safety shield); the learned model is not in this loop (ADR-001), so
 * there is deliberately no confidence gauge. Title, factors and the
 * dynamics badges all read from real channels. Hidden in `focus` body.
 */
export function ManeuverCard() {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  const perception = useSimulationStore((s) => s.perception);
  const surround = useSimulationStore((s) => s.surroundPerception);
  const density = useConsole((s) => s.density);

  const adjacentOccupied = surround.some(
    (t) => Math.abs(t.azimuth_deg) > 35 && Math.abs(t.azimuth_deg) < 145 && t.range_m < 25,
  );

  const m = deriveManeuver({
    ego: ego ?? null,
    shield: shield ?? null,
    prediction: prediction ?? null,
    planner: planner ?? null,
    perception,
    adjacentOccupied,
  });

  const accent = toneColor[m.tone];
  const compact = density === 'focus';
  const steerDeg = (ego?.steering_angle ?? 0) * (180 / Math.PI);
  const accel = ego?.acceleration ?? 0;
  const targetKmh = Math.round((ego?.target_velocity ?? 0) * 3.6);

  return (
    <div
      className="dds-glass"
      style={{
        position: 'relative',
        display: 'flex',
        flexDirection: 'column',
        gap: 'var(--space-3)',
        padding: compact ? '12px 16px' : '16px 20px',
        borderRadius: 'var(--radius-xl)',
        boxShadow: `var(--elev-card), 0 0 24px color-mix(in srgb, ${accent} 14%, transparent)`,
      }}
    >
      {/* top hairline glow */}
      <span
        aria-hidden
        style={{
          position: 'absolute',
          top: 0,
          left: '22%',
          right: '22%',
          height: 1,
          background: `linear-gradient(90deg, transparent, ${accent}, transparent)`,
          opacity: 0.8,
        }}
      />

      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 'var(--space-4)' }}>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 3, minWidth: 0 }}>
          <span
            className="font-mono"
            style={{ fontSize: 9.5, fontWeight: 700, letterSpacing: '0.14em', color: 'var(--brand)', textTransform: 'uppercase' }}
          >
            Planner · deterministic
          </span>
          <h1
            style={{
              fontFamily: 'var(--font-display)',
              fontSize: compact ? 18 : 22,
              fontWeight: 800,
              letterSpacing: '-0.01em',
              color: 'var(--text-bright)',
              textShadow: `0 0 20px color-mix(in srgb, ${accent} 25%, transparent)`,
              textTransform: 'uppercase',
              transition: 'color var(--dur) var(--ease-out)',
            }}
          >
            {m.title}
          </h1>
        </div>
        <span
          className="font-mono"
          style={{
            flex: 'none',
            fontSize: 10,
            fontWeight: 700,
            letterSpacing: '0.06em',
            padding: '6px 12px',
            borderRadius: 'var(--radius-md)',
            color: shield?.approved === false ? 'var(--critical)' : 'var(--success)',
            background: shield?.approved === false ? 'var(--critical-muted)' : 'var(--success-muted)',
            border: `1px solid color-mix(in srgb, ${shield?.approved === false ? 'var(--critical)' : 'var(--success)'} 40%, transparent)`,
            textTransform: 'uppercase',
          }}
        >
          {shield?.approved === false ? 'Shield override' : 'Shield approved'}
        </span>
      </div>

      {!compact && (
        <div style={{ display: 'grid', gridTemplateColumns: '1.3fr 1fr', gap: 'var(--space-5)', paddingTop: 'var(--space-3)', borderTop: '1px solid var(--border-subtle)' }}>
          <div>
            <span className="font-mono" style={{ fontSize: 9, fontWeight: 700, letterSpacing: '0.08em', color: 'var(--text-faint)', textTransform: 'uppercase', display: 'block', marginBottom: 6 }}>
              Contributing factors
            </span>
            <ul style={{ listStyle: 'none', display: 'flex', flexDirection: 'column', gap: 5 }}>
              {m.factors.map((f, i) => (
                <li key={i} style={{ display: 'flex', alignItems: 'baseline', gap: 8, fontSize: 11.5, color: 'var(--text-muted)', lineHeight: 1.35 }}>
                  <span style={{ flex: 'none', width: 4, height: 4, borderRadius: '50%', background: accent, boxShadow: `0 0 6px ${accent}`, transform: 'translateY(-1px)' }} />
                  {f}
                </li>
              ))}
            </ul>
          </div>
          <div>
            <span className="font-mono" style={{ fontSize: 9, fontWeight: 700, letterSpacing: '0.08em', color: 'var(--text-faint)', textTransform: 'uppercase', display: 'block', marginBottom: 6 }}>
              Maneuver dynamics
            </span>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 8 }}>
              {[
                { l: 'ACCEL', v: `${accel >= 0 ? '+' : ''}${accel.toFixed(2)}`, hi: false },
                { l: 'STEER', v: `${steerDeg >= 0 ? '+' : ''}${steerDeg.toFixed(1)}°`, hi: false },
                { l: 'TARGET', v: `${targetKmh}`, hi: true },
              ].map((b) => (
                <div
                  key={b.l}
                  style={{
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    gap: 2,
                    padding: '8px 4px',
                    borderRadius: 'var(--radius-md)',
                    background: b.hi ? 'var(--brand-muted)' : 'var(--bg-card)',
                    border: `1px solid ${b.hi ? 'var(--brand-dim)' : 'var(--border-subtle)'}`,
                  }}
                >
                  <span className="font-mono" style={{ fontSize: 8, fontWeight: 700, letterSpacing: '0.06em', color: 'var(--text-faint)' }}>
                    {b.l}
                  </span>
                  <span className="font-mono" style={{ fontSize: 13, fontWeight: 700, color: b.hi ? 'var(--brand)' : 'var(--text-primary)' }}>
                    {b.v}
                  </span>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
