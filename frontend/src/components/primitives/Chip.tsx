'use client';

import React from 'react';
import { Tone, toneColor, toneMuted } from './tokens';

interface ChipProps {
  tone?: Tone;
  icon?: React.ReactNode;
  /** Show a small leading status dot (glows in the tone colour). */
  dot?: boolean;
  children: React.ReactNode;
}

/** A pill. State-in-form: colour + optional glowing dot. */
export function Chip({ tone = 'brand', icon, dot = true, children }: ChipProps) {
  return (
    <span
      className="font-mono"
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: 7,
        fontSize: 11,
        letterSpacing: '0.04em',
        padding: '6px 11px',
        borderRadius: 'var(--radius-pill)',
        color: toneColor[tone],
        background: toneMuted[tone],
        border: `1px solid ${toneColor[tone]}`,
        borderColor:
          tone === 'default' ? 'var(--border-default)' : toneColor[tone],
      }}
    >
      {dot && (
        <span
          aria-hidden
          style={{
            width: 6,
            height: 6,
            borderRadius: '50%',
            background: 'currentColor',
            boxShadow: '0 0 7px currentColor',
          }}
        />
      )}
      {icon}
      {children}
    </span>
  );
}
