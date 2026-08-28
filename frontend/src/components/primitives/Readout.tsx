'use client';

import React from 'react';
import { Tone, toneColor } from './tokens';

interface ReadoutProps {
  /** Key / label (left, faint). */
  k: React.ReactNode;
  /** Value (right, primary or toned). */
  v: React.ReactNode;
  tone?: Tone;
}

/** One mono key/value row — the atom of every panel body. */
export function Readout({ k, v, tone = 'default' }: ReadoutProps) {
  return (
    <div
      className="font-mono"
      style={{
        display: 'flex',
        justifyContent: 'space-between',
        gap: 'var(--space-3)',
        fontSize: 11,
        padding: '3px 0',
      }}
    >
      <span style={{ color: 'var(--text-faint)' }}>{k}</span>
      <span style={{ color: toneColor[tone], textAlign: 'right' }}>{v}</span>
    </div>
  );
}
