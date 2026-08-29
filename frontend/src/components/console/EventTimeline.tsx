'use client';

import React from 'react';
import { useEvents } from '../../store/useEvents';
import type { Tone } from '../primitives';
import { toneColor } from '../primitives';

/* Operational event timeline (§17) — driven ONLY by the backend's real
 * scenario event stream. Free drive emits none; the empty state says so
 * rather than inventing activity. */

const TONE_FOR: Record<string, Tone> = {
  SAFETY_SHIELD_OVERRIDE: 'crit',
  VEHICLE_CUT_IN: 'warn',
  LANE_CHANGE_INITIATED: 'warn',
  LANE_CHANGE_COMPLETED: 'ok',
  QUEUE_STANDSTILL: 'warn',
  QUEUE_RESUMED: 'ok',
  CRUISING_STABLE: 'ok',
  SCENARIO_LOADED: 'brand',
};

const humanize = (t: string) =>
  t.toLowerCase().replace(/_/g, ' ').replace(/\b\w/g, (c) => c.toUpperCase());

function tPlus(tick: number) {
  const s = tick * 0.1;
  return `t+${s.toFixed(1)}s`;
}

/** `ticker` = single latest line for the strip. `list` = scrollable log. */
export function EventTimeline({ variant = 'list' }: { variant?: 'ticker' | 'list' }) {
  const events = useEvents((s) => s.events);

  if (variant === 'ticker') {
    const last = events[events.length - 1];
    if (!last) return null;
    const tone = TONE_FOR[last.type] ?? 'default';
    return (
      <span
        className="font-mono"
        style={{
          fontSize: 10,
          color: 'var(--text-faint)',
          whiteSpace: 'nowrap',
          overflow: 'hidden',
          textOverflow: 'ellipsis',
          maxWidth: 320,
        }}
      >
        <span style={{ color: toneColor[tone] }}>◆</span> {tPlus(last.tick)}{' '}
        <span style={{ color: 'var(--text-muted)' }}>{humanize(last.type)}</span>
        {last.cause ? ` — ${last.cause}` : ''}
      </span>
    );
  }

  if (events.length === 0) {
    return (
      <div style={{ fontSize: 10, color: 'var(--text-faint)', padding: '4px 0', lineHeight: 1.5 }}>
        No scenario events. The operational log records backend milestone
        transitions — load a scenario from the strip to populate it.
      </div>
    );
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 2, maxHeight: 168, overflowY: 'auto' }}>
      {[...events].reverse().map((e) => {
        const tone = TONE_FOR[e.type] ?? 'default';
        return (
          <div
            key={e.event_id}
            className="font-mono"
            style={{ display: 'grid', gridTemplateColumns: '52px 1fr', gap: 8, fontSize: 10, padding: '2px 0' }}
          >
            <span style={{ color: 'var(--text-faint)' }}>{tPlus(e.tick)}</span>
            <span style={{ color: 'var(--text-muted)', lineHeight: 1.35 }}>
              <span style={{ color: toneColor[tone], fontWeight: 600 }}>{humanize(e.type)}</span>
              {e.cause ? <span style={{ color: 'var(--text-faint)' }}> · {e.cause}</span> : null}
            </span>
          </div>
        );
      })}
    </div>
  );
}
