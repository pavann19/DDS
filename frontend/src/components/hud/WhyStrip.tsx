'use client';

import React from 'react';
import { HelpCircle } from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { toneColor } from '../primitives';
import { deriveManeuver } from '../../lib/consoleState';

export function WhyStrip() {
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
  const primaryFactor = m.factors[0] ?? 'Nominal route tracking';

  return (
    <div
      className="dds-glass"
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: 'var(--space-3)',
        padding: '8px 14px',
        borderRadius: 'var(--radius-pill)',
        pointerEvents: 'auto',
        maxWidth: 'min(480px, 90vw)',
        border: `1px solid color-mix(in srgb, ${accent} 35%, var(--border-subtle))`,
        boxShadow: `var(--elev-1), 0 0 16px color-mix(in srgb, ${accent} 12%, transparent)`,
      }}
    >
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, flex: 'none' }}>
        <span
          aria-hidden
          style={{
            width: 7,
            height: 7,
            borderRadius: '50%',
            background: accent,
            boxShadow: `0 0 8px ${accent}`,
          }}
        />
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 4,
            fontFamily: 'var(--font-mono)',
            fontSize: 10,
            fontWeight: 800,
            letterSpacing: '0.1em',
            color: 'var(--brand)',
            textTransform: 'uppercase',
          }}
        >
          <HelpCircle className="h-3 w-3" aria-hidden="true" />
          <span>WHY:</span>
        </div>
      </div>

      <div
        style={{
          display: 'flex',
          alignItems: 'baseline',
          gap: 6,
          minWidth: 0,
          overflow: 'hidden',
          textOverflow: 'ellipsis',
          whiteSpace: 'nowrap',
        }}
      >
        <span
          style={{
            fontFamily: 'var(--font-display)',
            fontSize: 12,
            fontWeight: 700,
            color: 'var(--text-bright)',
            flex: 'none',
          }}
        >
          {m.title}
        </span>
        <span
          style={{
            fontSize: 11,
            color: 'var(--text-muted)',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
            whiteSpace: 'nowrap',
          }}
        >
          — {primaryFactor}
        </span>
      </div>
    </div>
  );
}
