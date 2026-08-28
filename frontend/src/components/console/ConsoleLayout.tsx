'use client';

import React from 'react';
import { useConsole } from '../../store/useConsole';
import { TopBar } from './TopBar';

interface ConsoleLayoutProps {
  /** The HUD overlay — mounts over the stage in every density. */
  hud?: React.ReactNode;
  /** Right rail of channel-bound panels — hidden in `focus`. */
  rail?: React.ReactNode;
  /** Bottom scenario / timeline strip — hidden in `focus`. */
  strip?: React.ReactNode;
}

/**
 * The one console surface (ADR-002). An absolute overlay above the
 * always-mounted <SimulationScene/>: top bar / stage passthrough / right
 * rail / bottom strip. The grid itself is click-through; only the chrome
 * takes pointer events.
 *
 *   focus     topbar(mini) + stage + HUD
 *   standard  topbar + stage + HUD + rail + strip
 *   inspect   same layout as standard; panels render denser (item 8)
 */
export function ConsoleLayout({ hud, rail, strip }: ConsoleLayoutProps) {
  const density = useConsole((s) => s.density);
  const showChrome = density !== 'focus';

  return (
    <div
      style={{
        position: 'absolute',
        inset: 0,
        zIndex: 10,
        display: 'grid',
        gridTemplateColumns: showChrome ? '1fr 300px' : '1fr',
        gridTemplateRows: showChrome ? '42px 1fr 92px' : '42px 1fr',
        gridTemplateAreas: showChrome
          ? '"topbar topbar" "stage rail" "strip strip"'
          : '"topbar" "stage"',
        pointerEvents: 'none',
        transition: 'grid-template-columns var(--dur) var(--ease-out)',
      }}
    >
      <div style={{ gridArea: 'topbar' }}>
        <TopBar />
      </div>

      {/* stage area is a passthrough to the canvas behind; the HUD sits in it */}
      <div style={{ gridArea: 'stage', position: 'relative', overflow: 'hidden' }}>
        {hud && (
          <div
            style={{
              position: 'absolute',
              left: 'var(--space-5)',
              right: 'var(--space-5)',
              bottom: 'var(--space-4)',
              pointerEvents: 'none',
            }}
          >
            {hud}
          </div>
        )}
      </div>

      {showChrome && (
        <aside
          aria-label="Subsystem panels"
          style={{
            gridArea: 'rail',
            borderLeft: '1px solid var(--border-default)',
            background: 'var(--bg-panel)',
            overflowY: 'auto',
            padding: 'var(--space-2)',
            display: 'flex',
            flexDirection: 'column',
            gap: 'var(--space-2)',
            pointerEvents: 'auto',
          }}
        >
          {rail}
        </aside>
      )}

      {showChrome && (
        <div
          style={{
            gridArea: 'strip',
            borderTop: '1px solid var(--border-default)',
            background: 'var(--bg-panel)',
            pointerEvents: 'auto',
          }}
        >
          {strip}
        </div>
      )}
    </div>
  );
}
