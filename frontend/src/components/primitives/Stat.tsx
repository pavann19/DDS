'use client';

import React from 'react';
import { Tone, toneColor } from './tokens';

interface StatProps {
  /** Already-formatted display value (caller tweens if it wants to). */
  value: React.ReactNode;
  unit?: string;
  label?: string;
  tone?: Tone;
  /** Big HUD speed vs a compact rail figure. */
  size?: 'lg' | 'md' | 'sm';
}

const SIZE = {
  lg: { v: 40, weight: 300, u: 11, l: 10 },
  md: { v: 22, weight: 400, u: 10, l: 9 },
  sm: { v: 15, weight: 500, u: 9, l: 8.5 },
} as const;

/** A number + its unit + an optional caption. */
export function Stat({ value, unit, label, tone = 'default', size = 'md' }: StatProps) {
  const s = SIZE[size];
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {label && (
        <span
          style={{
            fontSize: s.l,
            letterSpacing: '0.1em',
            textTransform: 'uppercase',
            color: 'var(--text-faint)',
            fontFamily: 'var(--font-mono)',
          }}
        >
          {label}
        </span>
      )}
      <span
        style={{
          fontSize: s.v,
          fontWeight: s.weight,
          lineHeight: 1,
          letterSpacing: '-0.02em',
          color: toneColor[tone === 'default' ? 'default' : tone],
        }}
      >
        {tone === 'default' ? <span style={{ color: 'var(--text-bright)' }}>{value}</span> : value}
        {unit && (
          <span
            className="font-mono"
            style={{
              fontSize: s.u,
              fontWeight: 500,
              letterSpacing: '0.1em',
              color: 'var(--text-faint)',
              marginLeft: 6,
            }}
          >
            {unit}
          </span>
        )}
      </span>
    </div>
  );
}
