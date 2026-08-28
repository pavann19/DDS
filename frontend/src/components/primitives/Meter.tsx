'use client';

import React from 'react';
import { Tone, toneColor } from './tokens';

interface MeterProps {
  label: React.ReactNode;
  /** 0..1. Clamped. */
  value: number;
  tone?: Tone;
  /** Show the value as `.NN` on the right. */
  showValue?: boolean;
}

/** A labelled horizontal bar — for probabilities (intent distribution,
 *  p_cut_in). The fill width transitions on --ease-out (Tesla-smooth). */
export function Meter({ label, value, tone = 'brand', showValue = true }: MeterProps) {
  const v = Math.max(0, Math.min(1, Number.isFinite(value) ? value : 0));
  return (
    <div
      className="font-mono"
      style={{
        display: 'grid',
        gridTemplateColumns: '84px 1fr 30px',
        alignItems: 'center',
        gap: 7,
        fontSize: 10,
        color: 'var(--text-muted)',
      }}
    >
      <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
        {label}
      </span>
      <span
        role="meter"
        aria-valuemin={0}
        aria-valuemax={1}
        aria-valuenow={Number(v.toFixed(2))}
        style={{
          height: 6,
          borderRadius: 3,
          background: 'var(--bg-surface)',
          overflow: 'hidden',
        }}
      >
        <span
          style={{
            display: 'block',
            height: '100%',
            width: `${v * 100}%`,
            borderRadius: 3,
            background: toneColor[tone === 'default' ? 'brand' : tone],
            transition: 'width var(--dur) var(--ease-out)',
          }}
        />
      </span>
      {showValue && (
        <span style={{ textAlign: 'right', color: 'var(--text-primary)' }}>
          {v.toFixed(2).replace(/^0/, '')}
        </span>
      )}
    </div>
  );
}
