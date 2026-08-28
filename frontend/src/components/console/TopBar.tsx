'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { DensityToggle } from './DensityToggle';

/** Connection · sim clock · scenario · density. In `focus` density this
 *  collapses to just the live dot + the density toggle. */
export function TopBar() {
  const isConnected = useSimulationStore((s) => s.isConnected);
  const tick = useSimulationStore((s) => s.tick);
  const simTime = useSimulationStore((s) => s.simulationTime);
  const scenario = useSimulationStore((s) => s.scenario);
  const density = useConsole((s) => s.density);

  const compact = density === 'focus';

  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: 'var(--space-4)',
        height: 42,
        padding: '0 var(--space-4)',
        borderBottom: compact ? 'none' : '1px solid var(--border-default)',
        background: compact ? 'transparent' : 'var(--bg-panel)',
        fontFamily: 'var(--font-mono)',
        fontSize: 11.5,
        color: 'var(--text-muted)',
        pointerEvents: 'auto',
      }}
    >
      <span
        aria-hidden
        style={{
          width: 7,
          height: 7,
          borderRadius: '50%',
          background: isConnected ? 'var(--success)' : 'var(--critical)',
          boxShadow: `0 0 7px ${isConnected ? 'var(--success)' : 'var(--critical)'}`,
        }}
      />
      <span className="sr-only">{isConnected ? 'Connected' : 'Disconnected'}</span>

      {!compact && (
        <>
          <span style={{ color: 'var(--text-bright)', fontWeight: 600, letterSpacing: '0.02em' }}>DDS</span>
          <span style={{ width: 1, height: 16, background: 'var(--border-strong)' }} />
          <span>
            tick <b style={{ color: 'var(--text-primary)' }}>{tick.toLocaleString()}</b>
          </span>
          <span>t+{simTime.toFixed(1)}s</span>
          <span style={{ width: 1, height: 16, background: 'var(--border-strong)' }} />
          <span>
            scenario{' '}
            <b style={{ color: 'var(--text-primary)' }}>{scenario?.name ?? 'Free Drive'}</b>
          </span>
        </>
      )}

      <span style={{ flex: 1 }} />
      <DensityToggle />
    </div>
  );
}
