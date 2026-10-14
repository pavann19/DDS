'use client';

import { memo } from 'react';
import { ROLE_LABEL } from '../../lib/roles';
import { sameSurroundLabel, type SurroundLabelProps } from '../../lib/surroundLabel';

// Telemetry pose changes at stream frequency; badge text changes only when
// its displayed rounding or role changes. Keep those updates independent.
export const SurroundTrackLabel = memo(function SurroundTrackLabel({
  role, relevance, color, rangeText, speedText, closingText,
}: SurroundLabelProps) {
  return (
          <div
            className="pointer-events-none select-none"
            style={{
              position: 'relative',
              transform: 'translateY(-100%)',
              display: 'flex',
              flexDirection: 'column',
              gap: 2,
              padding: '4px 8px',
              fontFamily: 'var(--font-mono)',
              whiteSpace: 'nowrap',
              background: 'rgba(10, 15, 24, 0.85)',
              backdropFilter: 'blur(12px) saturate(160%)',
              border: `1px solid ${color}`,
              borderRadius: 6,
              boxShadow: '0 4px 12px rgba(0,0,0,0.5)',
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
              <span style={{ fontSize: 10, fontWeight: 700, color }}>{ROLE_LABEL[role]}</span>
              <span style={{ fontSize: 10, fontWeight: 600, color: 'var(--text-bright)' }}>{rangeText} m</span>
            </div>
            {relevance === 'primary' && (
              <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8, fontSize: 9, color: 'var(--text-muted)' }}>
                <span>{speedText} km/h</span>
                {closingText !== null && (
                  <span>{closingText}</span>
                )}
              </div>
            )}
            <span
              aria-hidden
              style={{
                position: 'absolute',
                bottom: -5,
                left: '50%',
                transform: 'translateX(-50%)',
                width: 0,
                height: 0,
                borderLeft: '5px solid transparent',
                borderRight: '5px solid transparent',
                borderTop: '5px solid rgba(10, 15, 24, 0.85)',
              }}
            />
          </div>
  );
}, sameSurroundLabel);
