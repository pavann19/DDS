'use client';

import React from 'react';
import { useConsole } from '../../store/useConsole';
import { useRail } from '../../store/useRail';
import { RailIconStrip } from './RailPanels';

interface ConsoleLayoutProps {
  /** The automotive HUD bar — top, full width. */
  hud?: React.ReactNode;
  /** Right rail of channel-bound cards — single card in standard, stack in inspect. */
  rail?: React.ReactNode;
  /** Maneuver & planner decision card. */
  card?: React.ReactNode;
  /** Bottom scenario strip — full width. */
  strip?: React.ReactNode;
}

/**
 * The unified console surface (ADR-002), cockpit layout:
 * - Top: 72px unified HUD header spanning full width
 * - Right: Non-overlapping subsystem telemetry rail
 * - Bottom: Full-width scenario & transport dock
 * - Open stage: Maximized 3D viewport area
 */
export function ConsoleLayout({ hud, rail, card, strip }: ConsoleLayoutProps) {
  const density = useConsole((s) => s.density);
  const expanded = useRail((s) => s.expanded);
  const activeCard = useRail((s) => s.activeCard);

  const focus = density === 'focus';
  const inspect = density === 'inspect';
  const standardExpanded = density === 'standard' && expanded && activeCard !== null;

  // Rail effective width in pixels
  const railWidth = focus ? 0 : inspect ? 372 : standardExpanded ? 340 : 44;

  return (
    <div style={{ position: 'absolute', inset: 0, zIndex: 10, pointerEvents: 'none' }}>
      {/* ==================== TOP HUD COMMAND BAR ==================== */}
      <div
        style={{
          position: 'absolute',
          top: 'var(--space-4)',
          left: 'var(--space-4)',
          right: 'var(--space-4)',
          height: 72,
          pointerEvents: 'auto',
          zIndex: 20,
        }}
      >
        {hud}
      </div>

      {/* ==================== RIGHT SUBSYSTEM TELEMETRY RAIL ==================== */}
      {!focus && (
        <aside
          aria-label="Subsystem telemetry panels"
          className={standardExpanded || inspect ? 'dds-glass' : undefined}
          style={{
            position: 'absolute',
            top: 'calc(72px + var(--space-4) + var(--space-3))',
            bottom: 'calc(52px + var(--space-4) + var(--space-3))',
            right: 'var(--space-4)',
            width: railWidth,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            borderRadius: 'var(--radius-xl)',
            pointerEvents: 'auto',
            transition: 'width var(--dur-slow) var(--ease-out), transform var(--dur-slow) var(--ease-out), opacity var(--dur) var(--ease-out)',
            zIndex: 15,
            boxShadow: standardExpanded || inspect ? 'var(--elev-card)' : 'none',
          }}
        >
          {standardExpanded || inspect ? rail : <RailIconStrip />}
        </aside>
      )}

      {/* ==================== BOTTOM-LEFT MANEUVER & PLANNER CARD ==================== */}
      {card && !focus && (
        <div
          style={{
            position: 'absolute',
            bottom: 'calc(52px + var(--space-4) + var(--space-3))',
            left: 'var(--space-4)',
            maxWidth: 'min(480px, calc(100vw - 400px))',
            pointerEvents: 'auto',
            zIndex: 14,
            transition: 'opacity var(--dur) var(--ease-out), transform var(--dur) var(--ease-out)',
          }}
        >
          {card}
        </div>
      )}

      {/* ==================== BOTTOM SCENARIO & TRANSPORT DOCK ==================== */}
      <div
        className="dds-glass"
        style={{
          position: 'absolute',
          bottom: 'var(--space-4)',
          left: 'var(--space-4)',
          right: 'var(--space-4)',
          height: 52,
          borderRadius: 'var(--radius-lg)',
          pointerEvents: 'auto',
          overflow: 'hidden',
          zIndex: 12,
          boxShadow: 'var(--elev-frost)',
        }}
      >
        {strip}
      </div>
    </div>
  );
}
