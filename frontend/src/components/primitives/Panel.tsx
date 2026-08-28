'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Tone, toneColor, toneMuted } from './tokens';

interface PanelProps extends React.HTMLAttributes<HTMLDivElement> {
  /** Left severity stripe + border accent. `default` = no stripe. */
  tone?: Tone;
  /** Frosted (over the 3D stage) vs solid. */
  frost?: boolean;
  /** Bump this on a state change to fire a one-shot border pulse in
   *  the tone colour (Tesla-style acknowledge, then settle). */
  pulseKey?: string | number;
  children: React.ReactNode;
}

/** The one container primitive. HUD blocks and rail panels are both
 *  Panels — there is no second implementation (ADR-002 Problem 1). */
export function Panel({
  tone = 'default',
  frost = false,
  pulseKey,
  children,
  style,
  className = '',
  ...rest
}: PanelProps) {
  const [pulsing, setPulsing] = useState(false);
  const firstRef = useRef(true);

  useEffect(() => {
    if (firstRef.current) {
      firstRef.current = false;
      return;
    }
    setPulsing(true);
    const t = setTimeout(() => setPulsing(false), 460);
    return () => clearTimeout(t);
  }, [pulseKey]);

  return (
    <div
      {...rest}
      className={className}
      style={{
        position: 'relative',
        background: frost ? 'var(--bg-frost)' : 'var(--bg-panel)',
        backdropFilter: frost ? 'blur(10px)' : undefined,
        WebkitBackdropFilter: frost ? 'blur(10px)' : undefined,
        border: '1px solid var(--border-default)',
        borderLeft:
          tone === 'default'
            ? '1px solid var(--border-default)'
            : `2px solid ${toneColor[tone]}`,
        borderRadius: 'var(--radius-lg)',
        boxShadow: frost ? 'var(--elev-frost)' : 'var(--elev-1)',
        animation: pulsing ? 'dds-pulse 0.46s var(--ease-out)' : undefined,
        // consumed by the @keyframes
        ['--pulse-color' as string]: toneMuted[tone],
        ...style,
      }}
    >
      {children}
    </div>
  );
}
