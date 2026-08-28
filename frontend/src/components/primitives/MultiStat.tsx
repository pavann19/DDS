'use client';

import React from 'react';

interface MultiStatProps {
  items: { n: React.ReactNode; label: string }[];
}

/** A row of compact count cells in a faint inset — e.g. perception class
 *  tallies (cars / peds / cyclists). */
export function MultiStat({ items }: MultiStatProps) {
  return (
    <div
      style={{
        display: 'grid',
        gridTemplateColumns: `repeat(${items.length}, 1fr)`,
        gap: 6,
        background: 'var(--bg-inset)',
        padding: 6,
        borderRadius: 'var(--radius-xs)',
      }}
    >
      {items.map((it) => (
        <div key={it.label} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 1 }}>
          <span className="font-mono" style={{ fontSize: 14, fontWeight: 700, color: 'var(--text-bright)' }}>
            {it.n}
          </span>
          <span style={{ fontSize: 8.5, color: 'var(--text-faint)', textTransform: 'uppercase', letterSpacing: '0.04em' }}>
            {it.label}
          </span>
        </div>
      ))}
    </div>
  );
}
