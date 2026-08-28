'use client';

import React from 'react';
import { useConsole } from '../../store/useConsole';
import { useSimulationStore } from '../../store/useSimulationStore';
import { DensityToggle } from './DensityToggle';

interface ConsoleLayoutProps {
  /** The automotive HUD bar — top, over the stage, every density. */
  hud?: React.ReactNode;
  /** Right rail of channel-bound cards — slides away in `focus`. */
  rail?: React.ReactNode;
  /** Bottom-centre maneuver card — compacts in `focus`. */
  card?: React.ReactNode;
  /** Bottom scenario strip — full width. */
  strip?: React.ReactNode;
}

const RAIL_W = 372;

/**
 * The one console surface (ADR-002), cockpit layout: floating glass panels
 * over the always-mounted <SimulationScene/>. Nothing is a solid slab —
 * the 3D stage reads through every gap.
 *
 *   focus     HUD (full width) + compact card + stage
 *   standard  HUD + right rail + card + scenario strip
 *   inspect   same frame; cards render denser (raw rows)
 */
export function ConsoleLayout({ hud, rail, card, strip }: ConsoleLayoutProps) {
  const density = useConsole((s) => s.density);
  const isConnected = useSimulationStore((s) => s.isConnected);
  const tick = useSimulationStore((s) => s.tick);
  const focus = density === 'focus';

  return (
    <div style={{ position: 'absolute', inset: 0, zIndex: 10, pointerEvents: 'none' }}>
      {/* top HUD bar */}
      <div
        style={{
          position: 'absolute',
          top: 'var(--space-4)',
          left: 'var(--space-5)',
          right: focus ? 'var(--space-5)' : `calc(${RAIL_W}px + var(--space-6))`,
          pointerEvents: 'auto',
          transition: 'right var(--dur) var(--ease-out)',
        }}
      >
        {hud}
      </div>

      {/* top-right quick controls: connection + tick + density */}
      <div
        style={{
          position: 'absolute',
          top: `calc(var(--space-4) + 74px + var(--space-2))`,
          right: focus ? 'var(--space-5)' : `calc(${RAIL_W}px + var(--space-6))`,
          display: 'flex',
          alignItems: 'center',
          gap: 'var(--space-2)',
          pointerEvents: 'auto',
          fontFamily: 'var(--font-mono)',
          fontSize: 10.5,
          color: 'var(--text-muted)',
          transition: 'right var(--dur) var(--ease-out)',
        }}
      >
        <div
          className="dds-glass"
          style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '5px 10px', borderRadius: 'var(--radius-md)' }}
        >
          <span
            aria-hidden
            style={{
              width: 7,
              height: 7,
              borderRadius: '50%',
              background: isConnected ? 'var(--success)' : 'var(--critical)',
              boxShadow: `0 0 8px ${isConnected ? 'var(--success)' : 'var(--critical)'}`,
              animation: isConnected ? 'dds-beacon 1.8s infinite ease-in-out' : undefined,
            }}
          />
          <span className="sr-only">{isConnected ? 'Connected' : 'Disconnected'}</span>
          <span style={{ color: 'var(--text-bright)', fontWeight: 700, fontFamily: 'var(--font-display)', letterSpacing: '0.06em' }}>
            DDS
          </span>
          <span style={{ color: 'var(--text-faint)' }}>t{tick.toLocaleString()}</span>
        </div>
        <DensityToggle />
      </div>

      {/* right rail */}
      <aside
        aria-label="Subsystem panels"
        className="dds-glass"
        style={{
          position: 'absolute',
          top: 'var(--space-4)',
          right: 'var(--space-5)',
          bottom: `calc(52px + var(--space-6))`,
          width: RAIL_W,
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden',
          borderRadius: 'var(--radius-xl)',
          pointerEvents: 'auto',
          transform: focus ? `translateX(calc(${RAIL_W}px + var(--space-8)))` : 'translateX(0)',
          opacity: focus ? 0 : 1,
          transition: 'transform var(--dur-slow) var(--ease-out), opacity var(--dur) var(--ease-out)',
        }}
      >
        {rail}
      </aside>

      {/* bottom-centre maneuver card */}
      {card && (
        <div
          style={{
            position: 'absolute',
            bottom: `calc(52px + var(--space-6))`,
            left: focus ? '50%' : `calc((100% - ${RAIL_W}px - var(--space-6)) / 2 + var(--space-5))`,
            transform: 'translateX(-50%)',
            width: focus ? 'min(460px, 92vw)' : `min(680px, calc(100vw - ${RAIL_W}px - 120px))`,
            pointerEvents: 'auto',
            transition: 'left var(--dur) var(--ease-out), width var(--dur) var(--ease-out)',
          }}
        >
          {card}
        </div>
      )}

      {/* bottom scenario strip */}
      <div
        className="dds-glass"
        style={{
          position: 'absolute',
          bottom: 'var(--space-4)',
          left: 'var(--space-5)',
          right: 'var(--space-5)',
          height: 52,
          borderRadius: 'var(--radius-lg)',
          pointerEvents: 'auto',
          overflow: 'hidden',
        }}
      >
        {strip}
      </div>
    </div>
  );
}
