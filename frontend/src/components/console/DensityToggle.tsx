'use client';

import React from 'react';
import { useConsole, DENSITY_ORDER } from '../../store/useConsole';

/** The one control that replaces mode-switching. A 3-segment toggle. */
export function DensityToggle() {
  const density = useConsole((s) => s.density);
  const setDensity = useConsole((s) => s.setDensity);

  return (
    <div
      role="group"
      aria-label="Console density"
      style={{
        display: 'inline-flex',
        border: '1px solid var(--border-strong)',
        borderRadius: 'var(--radius-sm)',
        overflow: 'hidden',
      }}
    >
      {DENSITY_ORDER.map((d) => {
        const on = d === density;
        return (
          <button
            key={d}
            type="button"
            aria-pressed={on}
            onClick={() => setDensity(d)}
            className="font-mono"
            style={{
              all: 'unset',
              boxSizing: 'border-box',
              padding: '3px 10px',
              fontSize: 10.5,
              letterSpacing: '0.06em',
              textTransform: 'uppercase',
              cursor: 'pointer',
              color: on ? 'var(--brand-ink)' : 'var(--text-faint)',
              background: on ? 'var(--brand)' : 'transparent',
              fontWeight: on ? 600 : 400,
              transition: 'background var(--dur-fast) var(--ease-out), color var(--dur-fast) var(--ease-out)',
            }}
          >
            {d}
          </button>
        );
      })}
    </div>
  );
}
