'use client';

import React, { useId, useState } from 'react';

interface DisclosureProps {
  /** Header row content (left-aligned). */
  summary: React.ReactNode;
  /** Optional right-aligned adornment in the header (e.g. a channel tag). */
  aside?: React.ReactNode;
  defaultOpen?: boolean;
  /** Controlled open state; omit for uncontrolled. */
  open?: boolean;
  onToggle?: (open: boolean) => void;
  children: React.ReactNode;
}

/** Keyboard-operable collapsible. Slide+fade on the body (Tesla-smooth),
 *  handled by CSS grid-rows transition; reduced-motion collapses it. */
export function Disclosure({
  summary,
  aside,
  defaultOpen = false,
  open: controlled,
  onToggle,
  children,
}: DisclosureProps) {
  const [uncontrolled, setUncontrolled] = useState(defaultOpen);
  const open = controlled ?? uncontrolled;
  const bodyId = useId();

  const toggle = () => {
    const next = !open;
    if (controlled === undefined) setUncontrolled(next);
    onToggle?.(next);
  };

  return (
    <div>
      <button
        type="button"
        aria-expanded={open}
        aria-controls={bodyId}
        onClick={toggle}
        style={{
          all: 'unset',
          boxSizing: 'border-box',
          display: 'flex',
          alignItems: 'center',
          gap: 'var(--space-2)',
          width: '100%',
          padding: 'var(--space-2) var(--space-3)',
          cursor: 'pointer',
          fontSize: 12,
          fontWeight: 600,
          color: 'var(--text-bright)',
        }}
      >
        <span
          aria-hidden
          style={{
            display: 'inline-block',
            fontSize: 9,
            color: 'var(--text-faint)',
            transform: open ? 'rotate(90deg)' : 'rotate(0deg)',
            transition: 'transform var(--dur-fast) var(--ease-out)',
          }}
        >
          &#9654;
        </span>
        <span style={{ flex: 1, minWidth: 0 }}>{summary}</span>
        {aside}
      </button>
      <div
        id={bodyId}
        role="region"
        style={{
          display: 'grid',
          gridTemplateRows: open ? '1fr' : '0fr',
          transition: 'grid-template-rows var(--dur) var(--ease-out)',
        }}
      >
        <div style={{ overflow: 'hidden' }}>
          <div style={{ padding: '0 var(--space-3) var(--space-3)' }}>{children}</div>
        </div>
      </div>
    </div>
  );
}
