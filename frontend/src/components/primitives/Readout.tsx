'use client';

import React from 'react';
import { Tone, toneColor } from './tokens';

interface ReadoutProps {
  /** Key / label (left, muted). */
  k: React.ReactNode;
  /** Value (right, mono, primary or toned). */
  v: React.ReactNode;
  tone?: Tone;
  /** Wrap the row in a faint tinted box (for the "primary target" row). */
  boxed?: boolean;
}

/** One metric row — label left, mono value right. The atom of every card. */
export function Readout({ k, v, tone = 'default', boxed = false }: ReadoutProps) {
  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        gap: 'var(--space-3)',
        fontSize: 11.5,
        padding: boxed ? '6px 9px' : '2px 0',
        borderRadius: boxed ? 'var(--radius-xs)' : undefined,
        background: boxed ? 'var(--brand-muted)' : undefined,
        border: boxed ? '1px solid var(--brand-dim)' : undefined,
        marginTop: boxed ? 2 : undefined,
      }}
    >
      <span style={{ color: 'var(--text-muted)', fontWeight: 400 }}>{k}</span>
      <span
        className="font-mono"
        style={{
          color: tone === 'default' ? 'var(--text-primary)' : toneColor[tone],
          fontWeight: tone === 'default' ? 600 : 700,
          textAlign: 'right',
        }}
      >
        {v}
      </span>
    </div>
  );
}
