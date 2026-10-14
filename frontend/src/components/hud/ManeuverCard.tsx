'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { toneColor } from '../primitives';
import { deriveManeuver } from '../../lib/consoleState';

const mono: React.CSSProperties = { fontFamily: 'var(--font-mono)' };

/**
 * The bottom-left maneuver card — "what is the planner doing, and why".
 * Displays the authoritative deterministic lattice decision, causal factors,
 * and real-time physical vehicle dynamics.
 */
export function ManeuverCard() {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  const perception = useSimulationStore((s) => s.perception);
  const surround = useSimulationStore((s) => s.surroundPerception);

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
  const steerDeg = ego?.steering_angle_rad !== undefined
    ? ego.steering_angle_rad * (180 / Math.PI)
    : (ego?.steering_angle ?? 0) * (180 / Math.PI);
  const accel = ego?.acceleration ?? 0;
  const latAccel = ego?.lateral_accel_mps2 ?? 0;
  const sideslipDeg = (ego?.kinematic_sideslip_rad ?? 0) * (180 / Math.PI);

  const isShieldApproved = shield?.approved === true;

  return (
    <div
      className="dds-glass"
      style={{
        position: 'relative',
        display: 'flex',
        flexDirection: 'column',
        gap: 'var(--space-2)',
        padding: '12px 16px',
        borderRadius: 'var(--radius-lg)',
        boxShadow: `var(--elev-card), 0 0 20px color-mix(in srgb, ${accent} 12%, transparent)`,
        minWidth: 380,
      }}
    >
      {/* Top subtle accent glow hairline */}
      <span
        aria-hidden
        style={{
          position: 'absolute',
          top: 0,
          left: '20%',
          right: '20%',
          height: 1,
          background: `linear-gradient(90deg, transparent, ${accent}, transparent)`,
          opacity: 0.7,
        }}
      />

      {/* Header: Causal Attribution & Decision Title */}
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 'var(--space-3)' }}>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 2, minWidth: 0 }}>
          <span
            style={{
              ...mono,
              fontSize: 8.5,
              fontWeight: 700,
              letterSpacing: '0.12em',
              color: accent,
              textTransform: 'uppercase',
            }}
          >
            {m.producer}
          </span>
          <h2
            style={{
              margin: 0,
              fontFamily: 'var(--font-display)',
              fontSize: 16,
              fontWeight: 800,
              letterSpacing: '-0.01em',
              color: 'var(--text-bright)',
              textShadow: `0 0 16px color-mix(in srgb, ${accent} 22%, transparent)`,
              textTransform: 'uppercase',
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}
          >
            {m.title}
          </h2>
        </div>

        {/* Safety Shield Status Pill */}
        <span
          style={{
            ...mono,
            flex: 'none',
            fontSize: 9.5,
            fontWeight: 700,
            letterSpacing: '0.04em',
            padding: '4px 9px',
            borderRadius: 'var(--radius-sm)',
            color: isShieldApproved ? 'var(--success)' : 'var(--critical)',
            background: isShieldApproved ? 'var(--success-muted)' : 'var(--critical-muted)',
            border: `1px solid color-mix(in srgb, ${isShieldApproved ? 'var(--success)' : 'var(--critical)'} 35%, transparent)`,
            textTransform: 'uppercase',
          }}
        >
          {!shield ? 'Shield Unavailable' : isShieldApproved ? 'Shield Approved' : 'Shield Veto'}
        </span>
      </div>

      {/* Body: Contributing Factors & Dynamics Grid */}
      <div
        style={{
          display: 'grid',
          gridTemplateColumns: '1.2fr 1fr',
          gap: 'var(--space-3)',
          paddingTop: 'var(--space-2)',
          borderTop: '1px solid var(--border-subtle)',
        }}
      >
        {/* Left: Causal Factors */}
        <div>
          <span
            style={{
              ...mono,
              fontSize: 8.5,
              fontWeight: 700,
              letterSpacing: '0.08em',
              color: 'var(--text-faint)',
              textTransform: 'uppercase',
              display: 'block',
              marginBottom: 4,
            }}
          >
            Causal Factors
          </span>
          <ul style={{ listStyle: 'none', padding: 0, margin: 0, display: 'flex', flexDirection: 'column', gap: 4 }}>
            {m.factors.map((f, i) => (
              <li
                key={i}
                style={{
                  display: 'flex',
                  alignItems: 'baseline',
                  gap: 6,
                  fontSize: 11,
                  color: 'var(--text-muted)',
                  lineHeight: 1.3,
                }}
              >
                <span
                  style={{
                    flex: 'none',
                    width: 3.5,
                    height: 3.5,
                    borderRadius: '50%',
                    background: accent,
                    boxShadow: `0 0 5px ${accent}`,
                    transform: 'translateY(-1px)',
                  }}
                />
                {f}
              </li>
            ))}
          </ul>
        </div>

        {/* Right: Dynamics Telemetry Grid */}
        <div>
          <span
            style={{
              ...mono,
              fontSize: 8.5,
              fontWeight: 700,
              letterSpacing: '0.08em',
              color: 'var(--text-faint)',
              textTransform: 'uppercase',
              display: 'block',
              marginBottom: 4,
            }}
          >
            Dynamics
          </span>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
            {[
              { l: 'ACCEL', v: `${accel >= 0 ? '+' : ''}${accel.toFixed(1)} m/s²` },
              { l: 'LAT ACC', v: `${latAccel.toFixed(1)} m/s²` },
              { l: 'STEER', v: `${steerDeg >= 0 ? '+' : ''}${steerDeg.toFixed(1)}°` },
              { l: 'SIDESLIP', v: `${sideslipDeg.toFixed(2)}°` },
            ].map((b) => (
              <div
                key={b.l}
                style={{
                  display: 'flex',
                  flexDirection: 'column',
                  padding: '4px 6px',
                  borderRadius: 'var(--radius-xs)',
                  background: 'var(--bg-inset)',
                  border: '1px solid var(--border-subtle)',
                }}
              >
                <span style={{ ...mono, fontSize: 7.5, fontWeight: 700, color: 'var(--text-faint)', letterSpacing: '0.06em' }}>
                  {b.l}
                </span>
                <span style={{ ...mono, fontSize: 11, fontWeight: 700, color: 'var(--text-bright)' }}>
                  {b.v}
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
