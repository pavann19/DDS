'use client';

import React, { useEffect } from 'react';
import { BarChart3 } from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { toneColor } from '../primitives';
import { useTween } from '../../hooks/useTween';
import { deriveAutonomy, pathClearance, PATH_CLEARANCE_LABEL } from '../../lib/consoleState';
import { useMediaQuery } from '../../hooks/useMediaQuery';
import { CameraSwitcher } from '../console/CameraSwitcher';
import { DensityToggle } from '../console/DensityToggle';

const mono: React.CSSProperties = { fontFamily: 'var(--font-mono)' };

/**
 * The unified cockpit command header:
 *   Left:   Velocity, Gear, Target Speed
 *   Center: Autonomous State Pill, Steering, Forward Radar, Path Clearance
 *   Right:  Camera Viewports (1-5), Density Switcher, DDS Telemetry Heartbeat
 */
export function DriveHUD() {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  const perception = useSimulationStore((s) => s.perception);
  const isConnected = useSimulationStore((s) => s.isConnected);
  const isPaused = useSimulationStore((s) => s.scenario?.is_paused ?? false);
  const tick = useSimulationStore((s) => s.tick);

  const isEvalOpen = useConsole((s) => s.isEvalOpen);
  const toggleEval = useConsole((s) => s.toggleEval);

  // Keyboard shortcut 'e' / 'E' to toggle evaluation dashboard
  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;
      if (e.key === 'e' || e.key === 'E') {
        toggleEval();
      }
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, [toggleEval]);

  const isCompact = useMediaQuery('(max-width: 1400px)');
  const isNarrow = useMediaQuery('(max-width: 1300px)');

  const speedKmh = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 260));
  const targetKmh = Math.round((ego?.target_velocity ?? 0) * 3.6);
  const steerDeg = (ego?.steering_angle_rad !== undefined
    ? ego.steering_angle_rad * (180 / Math.PI)
    : (ego?.steering_angle ?? 0) * (180 / Math.PI));

  const state = deriveAutonomy({
    ego: ego ?? null,
    shield: shield ?? null,
    prediction: prediction ?? null,
    planner: planner ?? null,
  });

  const clr = pathClearance(shield ?? null, prediction ?? null, ego?.sensor_health);
  const clrTone = !isConnected || !shield ? 'warn' : clr === 'hazard' ? 'crit' : (clr === 'caution' || clr === 'unavailable') ? 'warn' : 'ok';
  const lead = perception.find((p) => p.id === 'sensed_lead_vehicle');

  return (
    <header
      className="dds-glass"
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        gap: isCompact ? 'var(--space-2)' : 'var(--space-4)',
        height: 72,
        padding: isCompact ? '0 var(--space-3)' : '0 var(--space-4)',
        borderRadius: 'var(--radius-xl)',
        minWidth: 0,
        boxShadow: 'var(--elev-frost)',
      }}
    >
      {/* ==================== LEFT: VELOCITY & POWERTRAIN ==================== */}
      <div style={{ display: 'flex', alignItems: 'center', gap: isCompact ? 'var(--space-2)' : 'var(--space-3)', flex: 'none' }}>
        <div style={{ display: 'flex', alignItems: 'baseline', gap: 5 }}>
          <span
            style={{
              fontFamily: 'var(--font-display)',
              fontSize: isCompact ? 34 : 42,
              fontWeight: 800,
              lineHeight: 1,
              letterSpacing: '-0.03em',
              color: 'var(--text-bright)',
              textShadow: '0 0 16px rgba(255,255,255,0.18)',
            }}
          >
            {isConnected && ego ? Math.round(speedKmh) : '—'}
          </span>
          <span style={{ ...mono, fontSize: 11, fontWeight: 700, color: 'var(--text-muted)', letterSpacing: '0.08em' }}>
            KM/H
          </span>
        </div>

        <div style={{ display: 'flex', alignItems: 'center', gap: 8, paddingLeft: isCompact ? 'var(--space-2)' : 'var(--space-3)', borderLeft: '1px solid var(--border-subtle)' }}>
          <span
            style={{
              ...mono,
              fontSize: 13,
              fontWeight: 800,
              padding: '3px 8px',
              background: 'var(--brand-muted)',
              color: 'var(--brand)',
              border: '1px solid var(--brand-dim)',
              borderRadius: 'var(--radius-sm)',
            }}
          >
            D
          </span>
          <div
            style={{
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              width: 36,
              height: 36,
              borderRadius: '50%',
              background: 'linear-gradient(135deg, #ffffff, #e2e8f0)',
              lineHeight: 1,
              boxShadow: '0 2px 8px rgba(0,0,0,0.3)',
            }}
          >
            <span style={{ fontSize: 6.5, fontWeight: 800, color: '#0f172a', fontFamily: 'var(--font-display)', letterSpacing: '0.04em' }}>TGT</span>
            <span style={{ fontSize: 14, fontWeight: 800, color: '#0f172a', fontFamily: 'var(--font-display)' }}>{isConnected && ego?.target_velocity != null ? targetKmh : '—'}</span>
          </div>
        </div>
      </div>

      {/* ==================== CENTER: AUTONOMY & GUIDANCE ==================== */}
      <div style={{ display: 'flex', alignItems: 'center', gap: isCompact ? 'var(--space-2)' : 'var(--space-3)', minWidth: 0, justifyContent: 'center' }}>
        {/* Autonomy State Pill */}
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 7,
            padding: isCompact ? '6px 10px' : '7px 14px',
            borderRadius: 'var(--radius-pill)',
            background: `color-mix(in srgb, ${toneColor[state.tone]} 12%, transparent)`,
            border: `1px solid color-mix(in srgb, ${toneColor[state.tone]} 42%, transparent)`,
            transition: 'background var(--dur) var(--ease-out), border-color var(--dur) var(--ease-out)',
            minWidth: 0,
          }}
        >
          <span
            aria-hidden
            style={{
              width: 7,
              height: 7,
              borderRadius: '50%',
              background: toneColor[state.tone],
              boxShadow: `0 0 10px ${toneColor[state.tone]}`,
              animation: 'dds-beacon 1.8s infinite ease-in-out',
              flex: 'none',
            }}
          />
          <span
            style={{
              fontFamily: 'var(--font-display)',
              fontSize: 12,
              fontWeight: 700,
              letterSpacing: '0.05em',
              color: toneColor[state.tone],
              textTransform: 'uppercase',
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}
          >
            {isConnected && ego ? (isPaused ? 'Paused' : state.label) : 'Offline'}
          </span>
          {!isNarrow && (
            <span
              style={{
                fontSize: 11,
                color: 'var(--text-muted)',
                paddingLeft: 6,
                borderLeft: '1px solid var(--border-subtle)',
                whiteSpace: 'nowrap',
                overflow: 'hidden',
                textOverflow: 'ellipsis',
                maxWidth: 150,
              }}
            >
              {isConnected && ego ? (isPaused ? 'simulation stopped' : state.sub) : 'waiting for current state'}
            </span>
          )}
        </div>

        {/* Steer Indicator */}
        {!isNarrow && (
          <div
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: 5,
              padding: '5px 10px',
              borderRadius: 'var(--radius-pill)',
              background: 'var(--bg-inset)',
              border: '1px solid var(--border-subtle)',
              flex: 'none',
            }}
          >
            <svg
              width="15"
              height="15"
              viewBox="0 0 24 24"
              fill="none"
              stroke="var(--text-muted)"
              strokeWidth="1.8"
              style={{ transform: `rotate(${steerDeg * 1.5}deg)`, transition: 'transform var(--dur-fast) ease-out' }}
              aria-hidden
            >
              <circle cx="12" cy="12" r="9" />
              <path d="M12 3v6M4 12h6M14 12h6M8 17l4-5 4 5" />
            </svg>
            <span style={{ ...mono, fontSize: 11, fontWeight: 600, color: 'var(--text-muted)' }}>
              {steerDeg >= 0 ? '+' : ''}
              {steerDeg.toFixed(1)}°
            </span>
          </div>
        )}

        {/* Forward Radar Track */}
        <div style={{ display: 'flex', alignItems: 'center', gap: 10, padding: '4px 10px', background: 'var(--bg-inset)', borderRadius: 'var(--radius-md)', border: '1px solid var(--border-subtle)', flex: 'none' }}>
          <div style={{ display: 'flex', flexDirection: 'column', lineHeight: 1.1 }}>
            <span style={{ ...mono, fontSize: 8.5, fontWeight: 700, color: 'var(--text-faint)', letterSpacing: '0.06em' }}>LEAD</span>
            <span style={{ ...mono, fontSize: 13, fontWeight: 700, color: 'var(--text-bright)' }}>
              {!isConnected ? '—' : lead ? lead.distance.toFixed(1) : '>100'}<span style={{ fontSize: 9.5, color: 'var(--text-faint)' }}>m</span>
            </span>
          </div>
          {!isCompact && (
            <div style={{ display: 'flex', flexDirection: 'column', lineHeight: 1.1, paddingLeft: 8, borderLeft: '1px solid var(--border-subtle)' }}>
              <span style={{ ...mono, fontSize: 8.5, fontWeight: 700, color: 'var(--text-faint)', letterSpacing: '0.06em' }}>ΔV</span>
              <span style={{ ...mono, fontSize: 13, fontWeight: 700, color: 'var(--text-bright)' }}>
                {lead ? `${lead.rel_velocity >= 0 ? '+' : ''}${lead.rel_velocity.toFixed(1)}` : '0.0'}<span style={{ fontSize: 9.5, color: 'var(--text-faint)' }}>m/s</span>
              </span>
            </div>
          )}
        </div>

        {/* Path Clearance Badge */}
        <span
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 5,
            padding: '5px 9px',
            borderRadius: 'var(--radius-sm)',
            ...mono,
            fontSize: 10.5,
            fontWeight: 700,
            letterSpacing: '0.04em',
            color: toneColor[clrTone],
            background: `color-mix(in srgb, ${toneColor[clrTone]} 14%, transparent)`,
            border: `1px solid color-mix(in srgb, ${toneColor[clrTone]} 40%, transparent)`,
            animation: clr === 'hazard' ? 'dds-hazard 1s infinite alternate' : undefined,
            textTransform: 'uppercase',
            whiteSpace: 'nowrap',
            flex: 'none',
          }}
        >
          <span style={{ width: 5, height: 5, borderRadius: '50%', background: 'currentColor' }} />
          {isConnected && shield ? PATH_CLEARANCE_LABEL[clr] : 'Path unknown'}
        </span>
      </div>

      {/* ==================== RIGHT: INTEGRATED UTILITY CONTROLS ==================== */}
      <div style={{ display: 'flex', alignItems: 'center', gap: isCompact ? 6 : 10, flex: 'none' }}>
        {/* Camera Switcher */}
        <div style={{ display: 'flex', alignItems: 'center' }}>
          <CameraSwitcher />
        </div>

        <span style={{ width: 1, height: 24, background: 'var(--border-subtle)' }} />

        {/* Density Toggle */}
        <DensityToggle />

        {/* Autonomy Evaluation Dashboard Button */}
        <button
          type="button"
          onClick={toggleEval}
          title="Toggle Autonomy Evaluation Dashboard (Hotkey: E)"
          aria-label="Toggle Autonomy Evaluation Dashboard"
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 5,
            padding: '4px 9px',
            borderRadius: 'var(--radius-sm)',
            background: isEvalOpen ? 'var(--brand-muted)' : 'var(--bg-inset)',
            border: `1px solid ${isEvalOpen ? 'var(--brand)' : 'var(--border-subtle)'}`,
            color: isEvalOpen ? 'var(--brand)' : 'var(--text-muted)',
            cursor: 'pointer',
            ...mono,
            fontSize: 10,
            fontWeight: 700,
            transition: 'all var(--dur-fast) ease-out',
            boxShadow: isEvalOpen ? '0 0 12px rgba(0, 242, 254, 0.25)' : 'none',
          }}
        >
          <BarChart3 style={{ width: 13, height: 13 }} />
          <span>EVAL</span>
        </button>

        {/* System Heartbeat & Tick Pill */}
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 6,
            padding: '4px 8px',
            borderRadius: 'var(--radius-sm)',
            background: 'var(--bg-inset)',
            border: '1px solid var(--border-subtle)',
            ...mono,
            fontSize: 10,
            color: 'var(--text-muted)',
          }}
          title={isConnected ? 'Telemetry connected (10 Hz simulation rate)' : 'Disconnected'}
        >
          <span
            aria-hidden
            style={{
              width: 6,
              height: 6,
              borderRadius: '50%',
              background: isConnected ? 'var(--success)' : 'var(--critical)',
              boxShadow: `0 0 6px ${isConnected ? 'var(--success)' : 'var(--critical)'}`,
              animation: isConnected ? 'dds-beacon 1.8s infinite ease-in-out' : undefined,
            }}
          />
          <span style={{ color: 'var(--text-bright)', fontWeight: 700, fontFamily: 'var(--font-display)' }}>DDS</span>
          <span style={{ color: 'var(--text-faint)' }}>t{tick.toLocaleString()}</span>
        </div>
      </div>
    </header>
  );
}
