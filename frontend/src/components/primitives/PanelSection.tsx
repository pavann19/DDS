'use client';

import React from 'react';
import { Panel } from './Panel';
import { Disclosure } from './Disclosure';
import { Tone } from './tokens';

interface PanelSectionProps {
  title: string;
  /** The protocol v3 channel this panel is bound to — rendered as a mono
   *  tag so a reviewer sees the label↔field coupling at a glance. */
  channel: string;
  tone?: Tone;
  defaultOpen?: boolean;
  /** Bump on a subsystem state change → one-shot border pulse. */
  pulseKey?: string | number;
  children: React.ReactNode;
}

/** A rail panel: a toned Panel wrapping a Disclosure whose header carries
 *  the bound-channel tag. This is the single shape every subsystem panel
 *  uses (ADR-002 rule 1). */
export function PanelSection({
  title,
  channel,
  tone = 'default',
  defaultOpen = false,
  pulseKey,
  children,
}: PanelSectionProps) {
  return (
    <Panel tone={tone} frost pulseKey={pulseKey}>
      <Disclosure
        defaultOpen={defaultOpen}
        summary={title}
        aside={
          <span
            className="font-mono"
            style={{
              fontSize: 9.5,
              fontWeight: 400,
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
        }
      >
        {children}
      </Disclosure>
    </Panel>
  );
}
