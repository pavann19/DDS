'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Tone, toneColor } from './tokens';

interface PanelSectionProps {
  title: string;
  /** Optional leading glyph for the card header. */
  icon?: React.ReactNode;
  /** The protocol v3 channel this card is bound to — a mono tag in the
   *  header so a reviewer sees the label↔field coupling at a glance. */
  channel: string;
  tone?: Tone;
  /** Bump on a subsystem state change → one-shot border pulse. */
  pulseKey?: string | number;
  /** Stagger index for the mount slide-in. */
  index?: number;
  /** Context-aware emphasis (§12): quiet cards recede, the reacting one
   *  is full-strength with a tone-accent border. */
  salience?: 'quiet' | 'active' | 'alert';
  children: React.ReactNode;
}

/** A rail card (ADR-002 rule 1 + cockpit visual language): always-open,
 *  icon + heading + bound-channel tag, bodies are metric rows. There is
 *  one card shape for every subsystem. */
export function PanelSection({
  title,
  icon,
  channel,
  tone = 'default',
  pulseKey,
  index = 0,
  salience = 'active',
  children,
}: PanelSectionProps) {
  const [pulsing, setPulsing] = useState(false);
  const first = useRef(true);

  useEffect(() => {
    if (first.current) {
      first.current = false;
      return;
    }
    setPulsing(true);
    const t = setTimeout(() => setPulsing(false), 460);
    return () => clearTimeout(t);
  }, [pulseKey]);

  const accent = tone !== 'default' && (salience === 'alert' || salience === 'active');

  return (
    <section
      style={{
        background: 'var(--bg-card)',
        border: '1px solid var(--border-subtle)',
        borderLeft: accent ? `2px solid ${toneColor[tone]}` : '1px solid var(--border-subtle)',
        borderRadius: 'var(--radius-md)',
        padding: 'var(--space-3)',
        display: 'flex',
        flexDirection: 'column',
        gap: 'var(--space-2)',
        opacity: salience === 'quiet' ? 0.62 : 1,
        animation: pulsing
          ? 'dds-pulse 0.46s var(--ease-out)'
          : `dds-slide-in var(--dur) var(--ease-out) both`,
        animationDelay: pulsing ? undefined : `${index * 40}ms`,
        transition: 'opacity var(--dur) var(--ease-out)',
        ['--pulse-color' as string]:
          tone === 'default' ? 'var(--brand-muted)' : `color-mix(in srgb, ${toneColor[tone]} 30%, transparent)`,
      }}
    >
      <header style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
        {icon && <span aria-hidden style={{ fontSize: 13, lineHeight: 1 }}>{icon}</span>}
        <h3
          style={{
            flex: 1,
            minWidth: 0,
            fontFamily: 'var(--font-display)',
            fontSize: 11,
            fontWeight: 700,
            letterSpacing: '0.06em',
            textTransform: 'uppercase',
            color: 'var(--text-muted)',
            whiteSpace: 'nowrap',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
          }}
        >
          {title}
        </h3>
        <span
          className="font-mono"
          style={{
            fontSize: 9,
            fontWeight: 500,
            color: 'var(--brand)',
            background: 'var(--brand-muted)',
            border: '1px solid var(--brand-dim)',
            borderRadius: 'var(--radius-xs)',
            padding: '1px 5px',
            whiteSpace: 'nowrap',
          }}
        >
          {channel}
        </span>
      </header>
      <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>{children}</div>
    </section>
  );
}
