'use client';

import React from 'react';
import {
  Gauge,
  ScanEye,
  Waypoints,
  Route,
  ShieldCheck,
  Activity,
  List,
  Pin,
  PinOff,
  X,
} from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { useRail } from '../../store/useRail';
import { PanelSection, Readout, Meter, MultiStat } from '../primitives';
import type { Tone } from '../primitives';
import { useTween } from '../../hooks/useTween';
import { panelSalience, type ConsoleCtx, type Salience } from '../../lib/consoleState';
import { EventTimeline } from './EventTimeline';

/* ----------------------------------------------------------------------------
 * The rail reads as one autonomous-system story (§11): a numbered pipeline
 *   01 EGO → 02 PERCEPTION → 03 PREDICTION → 04 PLANNER → 05 SAFETY → 06 ANALYTICS → 07 EVENTS
 * Each card is bound 1:1 to a protocol v3 channel (header tag); nothing
 * renders state the backend doesn't send. Context-aware (§12):
 * panelSalience() dims quiet cards and accents the reacting one.
 * In `standard` density, defaults to a 44px icon strip, expanding to a
 * 340px single card on click with localStorage pinning. `inspect` force-expands
 * the full 7-card stack.
 * ------------------------------------------------------------------------- */

const fmt = (n: number | null | undefined, d = 1) =>
  n == null || !Number.isFinite(n) ? '—' : n.toFixed(d);

function useInspect() {
  return useConsole((s) => s.density) === 'inspect';
}

function useCtx(): ConsoleCtx {
  const ego = useSimulationStore((s) => s.ego);
  const shield = useSimulationStore((s) => s.safetyShield);
  const prediction = useSimulationStore((s) => s.prediction);
  const planner = useSimulationStore((s) => s.planner);
  return { ego, shield, prediction, planner };
}

export const RAIL_SUBSYSTEMS = [
  { id: 'ego', num: '01', title: '01 · Ego kinematics', name: 'Ego', icon: Gauge, channel: 'pose.ego' },
  { id: 'perception', num: '02', title: '02 · Perception engine', name: 'Perception', icon: ScanEye, channel: 'semantic.perception' },
  { id: 'prediction', num: '03', title: '03 · Prediction matrix', name: 'Prediction', icon: Waypoints, channel: 'heavy.prediction' },
  { id: 'planner', num: '04', title: '04 · Planner', name: 'Planner', icon: Route, channel: 'semantic.planner' },
  { id: 'safety', num: '05', title: '05 · Safety system', name: 'Safety', icon: ShieldCheck, channel: 'semantic.safety_shield' },
  { id: 'analytics', num: '06', title: '06 · Driver analytics', name: 'Analytics', icon: Activity, channel: 'semantic.driver_analytics' },
  { id: 'events', num: '07', title: '07 · AI activity stream', name: 'Events', icon: List, channel: 'event.stream' },
] as const;

/** 01 — pose.ego: kinematics + realised control outputs. */
function EgoControlPanel({ index, salience }: { index: number; salience: Salience }) {
  const ego = useSimulationStore((s) => s.ego);
  const speed = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 240));
  const d = ego?.frenet?.d ?? 0;
  const laneDesc = Math.abs(d) < 1 ? 'lane 1 (centre)' : d < -1.6 ? 'lane 2 (left / overtake)' : d > 1.6 ? 'right lane' : 'lane transition';
  const steerDeg = ego?.steering_angle_rad !== undefined ? (ego.steering_angle_rad * (180 / Math.PI)) : ((ego?.steering_angle ?? 0) * (180 / Math.PI));
  return (
    <PanelSection title="01 · Ego kinematics" icon="⚡" channel="pose.ego" index={index} salience={salience}>
      <Readout k="current velocity" v={`${fmt(speed)} km/h`} tone="brand" />
      <Readout k="target velocity" v={`${fmt((ego?.target_velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="longitudinal accel" v={`${(ego?.acceleration ?? 0) >= 0 ? '+' : ''}${fmt(ego?.acceleration, 2)} m/s²`} />
      <Readout k="lateral accel" v={`${fmt(ego?.lateral_accel_mps2, 2)} m/s²`} />
      <Readout k="steering angle" v={`${fmt(steerDeg, 1)}°`} />
      <Readout k="yaw rate" v={`${fmt(ego?.yaw_rate_degps, 1)}°/s`} />
      <Readout k="sideslip (β)" v={`${fmt((ego?.kinematic_sideslip_rad ?? 0) * (180 / Math.PI), 2)}°`} />
      <Readout k="lane position" v={laneDesc} />
      <Readout k="station (frenet s)" v={`${fmt(ego?.frenet?.s, 0)} m`} />
      <Readout k="binding constraint" v={ego?.speed_limit_reason ?? '—'} tone="brand" />
      <Meter label="throttle" value={ego?.throttle ?? 0} tone="ok" />
      <Meter label="brake" value={ego?.brake ?? 0} tone="crit" />
    </PanelSection>
  );
}

/** 02 — semantic.perception + heavy.surround_perception. */
function PerceptionPanel({ index, salience }: { index: number; salience: Salience }) {
  const perception = useSimulationStore((s) => s.perception);
  const surround = useSimulationStore((s) => s.surroundPerception);
  const inspect = useInspect();
  const lead = [...perception].sort((a, b) => a.distance - b.distance)[0];

  const isPed = (c: string) => c === 'PEDESTRIAN';
  const isCyc = (c: string) => c === 'BICYCLE' || c === 'MOTORCYCLE';
  const veh = surround.filter((t) => !isPed(t.class) && !isCyc(t.class)).length;
  const ped = surround.filter((t) => isPed(t.class)).length;
  const cyc = surround.filter((t) => isCyc(t.class)).length;

  return (
    <PanelSection title="02 · Perception engine" icon="👁" channel="semantic.perception" index={index} salience={salience} pulseKey={surround.length}>
      <Readout k="active tracks" v={`${surround.length} objects`} tone="brand" />
      <MultiStat items={[{ n: veh, label: 'cars' }, { n: ped, label: 'peds' }, { n: cyc, label: 'cyclists' }]} />
      <Readout k="forward objects" v={perception.length} />
      <Readout
        k="primary target"
        v={lead ? `lead · ${fmt(lead.distance)} m` : 'none (nominal)'}
        tone={lead && lead.distance < 12 ? 'warn' : 'brand'}
        boxed
      />
      {inspect &&
        surround.map((t) => (
          <Readout key={t.id} k={`${t.class.toLowerCase()} ${t.id}`} v={`${fmt(t.range_m)} m · ${fmt(t.azimuth_deg, 0)}°`} />
        ))}
    </PanelSection>
  );
}

/** 03 — heavy.prediction: per-agent forecasts, intent, proactive cut-in. */
function PredictionPanel({ index, salience }: { index: number; salience: Salience }) {
  const prediction = useSimulationStore((s) => s.prediction);
  const inspect = useInspect();
  const cutIn = prediction?.cut_in;
  const agents = prediction?.agents ?? [];
  const decel = useTween(prediction?.proactive_decel_mps2 ?? 0, 240);
  const shown = inspect ? agents : agents.slice(0, 3);
  return (
    <PanelSection
      title="03 · Prediction matrix"
      icon="🔮"
      channel="heavy.prediction"
      tone={cutIn?.active ? 'warn' : 'default'}
      index={index}
      salience={salience}
      pulseKey={cutIn?.active ? 'active' : 'idle'}
    >
      <Readout
        k="cut-in probability"
        v={cutIn?.active ? `${Math.round((cutIn.probability ?? 0) * 100)}% — easing off` : `${Math.round((cutIn?.probability ?? 0) * 100)}% (low)`}
        tone={cutIn?.active ? 'warn' : 'ok'}
      />
      <Readout k="time to cross" v={cutIn?.time_to_cross_s != null ? `${fmt(cutIn.time_to_cross_s)} s` : '—'} />
      <Readout k="proactive decel" v={`${fmt(decel)} m/s²`} />
      <Readout k="agents forecast" v={agents.length} tone="brand" />
      {shown.map((a) => {
        const top = a.intent[0];
        return <Meter key={a.track_id} label={`#${a.track_id} ${top?.label.toLowerCase() ?? '—'}`} value={top?.p ?? 0} tone="brand" />;
      })}
    </PanelSection>
  );
}

/** 04 — semantic.planner: Frenet lateral planner candidates. */
function PlannerPanel({ index, salience }: { index: number; salience: Salience }) {
  const planner = useSimulationStore((s) => s.planner);
  const inspect = useInspect();
  const chosen = planner?.candidates.find((c) => c.is_chosen);
  return (
    <PanelSection
      title="04 · Planner"
      icon="🧭"
      channel="semantic.planner"
      tone={planner?.is_changing_lane ? 'warn' : 'default'}
      index={index}
      salience={salience}
      pulseKey={planner?.is_changing_lane ? 'lc' : 'hold'}
    >
      <Readout k="maneuver" v={planner?.is_changing_lane ? 'lane change' : 'lane keep'} tone={planner?.is_changing_lane ? 'warn' : 'ok'} />
      <Readout k="lane centre d" v={`${fmt(planner?.lane_center)} m`} />
      <Readout k="path curvature" v={fmt(planner?.curvature, 4)} />
      <Readout k="candidates scored" v={planner?.candidates.length ?? 0} />
      {chosen && <Readout k="chosen d_target" v={`${fmt(chosen.d_target)} m · cost ${fmt(chosen.cost, 2)}`} tone="brand" boxed />}
      {inspect &&
        planner?.candidates.map((c, i) => (
          <Readout key={i} k={`cand ${fmt(c.d_target, 1)} m`} v={`${fmt(c.cost, 2)}${c.is_chosen ? ' ✓' : ''}${c.is_lane_change ? ' ⇄' : ''}`} />
        ))}
    </PanelSection>
  );
}

/** 05 — semantic.safety_shield: the independent post-decision check. */
function SafetyPanel({ index, salience }: { index: number; salience: Salience }) {
  const shield = useSimulationStore((s) => s.safetyShield);
  const risk = shield?.risk_level ?? 'UNKNOWN';
  const overridden = shield?.approved === false;
  const badgeTone: Tone = overridden || risk === 'CRITICAL' || risk === 'HIGH' ? 'crit' : risk === 'MEDIUM' ? 'warn' : 'ok';
  return (
    <PanelSection
      title="05 · Safety system"
      icon="🛡"
      channel="semantic.safety_shield"
      tone={overridden ? 'crit' : 'default'}
      index={index}
      salience={salience}
      pulseKey={shield?.override_action ?? 'ok'}
    >
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: 8,
          padding: '7px 10px',
          borderRadius: 'var(--radius-xs)',
          fontFamily: 'var(--font-display)',
          fontSize: 11.5,
          fontWeight: 700,
          letterSpacing: '0.05em',
          color: `var(--${badgeTone === 'crit' ? 'critical' : badgeTone === 'warn' ? 'warning' : 'success'})`,
          background: `var(--${badgeTone === 'crit' ? 'critical' : badgeTone === 'warn' ? 'warning' : 'success'}-muted)`,
          border: `1px solid color-mix(in srgb, var(--${badgeTone === 'crit' ? 'critical' : badgeTone === 'warn' ? 'warning' : 'success'}) 40%, transparent)`,
        }}
      >
        <span style={{ width: 8, height: 8, borderRadius: '50%', background: 'currentColor' }} />
        {!shield ? 'TELEMETRY UNAVAILABLE' : overridden ? 'SHIELD OVERRIDE' : `SYSTEM ${risk === 'NONE' ? 'NOMINAL' : risk}`}
      </div>
      <Readout k="risk level" v={risk} tone={badgeTone} />
      <Readout k="time-to-collision" v={shield?.ttc_s != null ? `${fmt(shield.ttc_s)} s` : '> 9.9 s'} tone="brand" />
      <Readout k="override action" v={shield?.override_action ?? 'none (standby)'} tone={shield?.override_action ? 'crit' : 'default'} />
      {(shield?.reasons ?? []).map((r, i) => (
        <Readout key={i} k={i === 0 ? 'reasons' : ''} v={r} />
      ))}
    </PanelSection>
  );
}

/** 06 — semantic.driver_analytics: the learned model as analytics, NOT control. */
function DriverAnalyticsPanel({ index, salience }: { index: number; salience: Salience }) {
  const shap = useSimulationStore((s) => s.shap);
  const anomaly = useSimulationStore((s) => s.anomaly);
  const score = useSimulationStore((s) => s.driverScore);
  const top = [...(shap?.contributions ?? [])]
    .sort((a, b) => Math.abs(b.contribution) - Math.abs(a.contribution))
    .slice(0, 3);
  return (
    <PanelSection
      title="06 · Driver analytics"
      icon="📈"
      channel="semantic.driver_analytics"
      tone={anomaly?.is_anomaly ? 'warn' : 'default'}
      index={index}
      salience={salience}
      pulseKey={anomaly?.type ?? 'none'}
    >
      <div style={{ fontSize: 9.5, color: 'var(--text-faint)', lineHeight: 1.45 }}>
        Observational only — the learned model does not drive the vehicle (ADR-001).
      </div>
      <Readout k="score" v={score ? `${fmt(score.score, 0)} · ${score.rating}` : '—'} tone="brand" />
      {score && (
        <>
          <Meter label="smoothness" value={score.breakdown.smoothness / 100} tone="ok" />
          <Meter label="efficiency" value={score.breakdown.efficiency / 100} tone="ok" />
          <Meter label="safety" value={score.breakdown.safety / 100} tone="ok" />
        </>
      )}
      <Readout k="anomaly" v={anomaly?.is_anomaly ? `${anomaly.type} (${anomaly.severity})` : 'none'} tone={anomaly?.is_anomaly ? 'warn' : 'ok'} />
      {top.map((c) => (
        <Readout key={c.feature} k={c.feature} v={`${c.contribution >= 0 ? '+' : ''}${fmt(c.contribution, 3)}`} />
      ))}
    </PanelSection>
  );
}

/** 07 — event.stream: the real scenario milestone log (empty in free drive). */
function EventsPanel({ index }: { index: number }) {
  return (
    <PanelSection title="07 · AI activity stream" icon="📋" channel="event.stream" index={index} salience="quiet">
      <EventTimeline variant="list" />
    </PanelSection>
  );
}

/** 44px vertical icon strip rendered when the rail is collapsed */
export function RailIconStrip() {
  const ctx = useCtx();
  const toggle = useRail((s) => s.toggle);
  const activeCard = useRail((s) => s.activeCard);

  return (
    <nav
      aria-label="Subsystem launcher"
      className="dds-glass"
      style={{
        width: 44,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        gap: 6,
        padding: '10px 4px',
        borderRadius: 'var(--radius-xl)',
        pointerEvents: 'auto',
      }}
    >
      {RAIL_SUBSYSTEMS.map((sub) => {
        const Icon = sub.icon;
        const salience = panelSalience(sub.id, ctx);
        const isAlert = salience === 'alert';
        const isActive = activeCard === sub.id;

        return (
          <button
            key={sub.id}
            type="button"
            onClick={() => toggle(sub.id)}
            title={`${sub.title} (${sub.channel})`}
            aria-label={`Open ${sub.name} telemetry panel`}
            aria-pressed={isActive}
            style={{
              position: 'relative',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: 36,
              height: 36,
              borderRadius: 'var(--radius-md)',
              border: isActive
                ? '1px solid var(--brand)'
                : isAlert
                ? '1px solid var(--warning)'
                : '1px solid transparent',
              background: isActive
                ? 'var(--brand-muted)'
                : isAlert
                ? 'var(--warning-muted)'
                : 'transparent',
              color: isActive
                ? 'var(--brand)'
                : isAlert
                ? 'var(--warning)'
                : 'var(--text-muted)',
              cursor: 'pointer',
              transition: 'all var(--dur-fast) var(--ease-out)',
              boxShadow: isActive ? 'var(--glow-brand)' : isAlert ? 'var(--glow-warning)' : 'none',
            }}
          >
            <Icon className="h-4 w-4" aria-hidden="true" />
            {isAlert && (
              <span
                aria-hidden
                style={{
                  position: 'absolute',
                  top: 5,
                  right: 5,
                  width: 6,
                  height: 6,
                  borderRadius: '50%',
                  background: 'var(--warning)',
                  boxShadow: '0 0 6px var(--warning)',
                }}
              />
            )}
          </button>
        );
      })}
    </nav>
  );
}

/** The right rail: single card drawer (in standard) or full card stack (in inspect). */
export function Rail() {
  const connected = useSimulationStore(s=>s.isConnected);
  const ctx = useCtx();
  const density = useConsole((s) => s.density);
  const { activeCard, pinned, toggle, collapse, setPinned } = useRail();

  const isInspect = density === 'inspect';
  const selectedSub = RAIL_SUBSYSTEMS.find((s) => s.id === activeCard) ?? RAIL_SUBSYSTEMS[0];

  const renderSinglePanel = (id: string) => {
    switch (id) {
      case 'ego':
        return <EgoControlPanel index={0} salience={panelSalience('ego', ctx)} />;
      case 'perception':
        return <PerceptionPanel index={1} salience={panelSalience('perception', ctx)} />;
      case 'prediction':
        return <PredictionPanel index={2} salience={panelSalience('prediction', ctx)} />;
      case 'planner':
        return <PlannerPanel index={3} salience={panelSalience('planner', ctx)} />;
      case 'safety':
        return <SafetyPanel index={4} salience={panelSalience('safety', ctx)} />;
      case 'analytics':
        return <DriverAnalyticsPanel index={5} salience={panelSalience('analytics', ctx)} />;
      case 'events':
        return <EventsPanel index={6} />;
      default:
        return <EgoControlPanel index={0} salience={panelSalience('ego', ctx)} />;
    }
  };

  return (
    <>
      {/* Rail Header */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '12px 16px',
          borderBottom: '1px solid var(--border-subtle)',
          background: 'var(--bg-surface)',
          gap: 8,
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: 9, minWidth: 0 }}>
          <span
            aria-hidden
            style={{
              width: 7,
              height: 7,
              borderRadius: '50%',
              background: 'var(--brand)',
              boxShadow: '0 0 8px var(--brand)',
              animation: 'dds-beacon 1.5s infinite',
              flex: 'none',
            }}
          />
          <h2
            style={{
              fontFamily: 'var(--font-display)',
              fontSize: 11.5,
              fontWeight: 700,
              letterSpacing: '0.08em',
              color: 'var(--text-bright)',
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}
          >
            {isInspect ? 'Vehicle information' : selectedSub.title}
          </h2>
        </div>

        <div style={{ display: 'flex', alignItems: 'center', gap: 6, flex: 'none' }}>
          <span
            className="font-mono"
            style={{
              fontSize: 10,
              fontWeight: 600,
              color: 'var(--brand)',
              background: 'var(--brand-muted)',
              padding: '2px 7px',
              borderRadius: 'var(--radius-xs)',
              border: '1px solid var(--brand-dim)',
            }}
          >
            {connected ? 'Connected' : 'Offline'}
          </span>

          {!isInspect && (
            <>
              {/* Pin toggle */}
              <button
                type="button"
                onClick={() => setPinned(!pinned)}
                title={pinned ? 'Unpin panel' : 'Pin panel open'}
                aria-pressed={pinned}
                aria-label={pinned ? 'Unpin panel' : 'Pin panel open'}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  width: 26,
                  height: 26,
                  borderRadius: 'var(--radius-xs)',
                  border: pinned ? '1px solid var(--brand)' : '1px solid var(--border-subtle)',
                  background: pinned ? 'var(--brand-muted)' : 'var(--bg-inset)',
                  color: pinned ? 'var(--brand)' : 'var(--text-muted)',
                  cursor: 'pointer',
                }}
              >
                {pinned ? <Pin className="h-3.5 w-3.5" aria-hidden="true" /> : <PinOff className="h-3.5 w-3.5" aria-hidden="true" />}
              </button>

              {/* Close / Collapse button */}
              <button
                type="button"
                onClick={collapse}
                title="Collapse panel"
                aria-label="Collapse panel"
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  width: 26,
                  height: 26,
                  borderRadius: 'var(--radius-xs)',
                  border: '1px solid var(--border-subtle)',
                  background: 'var(--bg-inset)',
                  color: 'var(--text-muted)',
                  cursor: 'pointer',
                }}
              >
                <X className="h-3.5 w-3.5" aria-hidden="true" />
              </button>
            </>
          )}
        </div>
      </div>

      {/* Subsystem quick-switch tabs when in single-card standard mode */}
      {!isInspect && (
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: 4,
            padding: '6px 12px',
            borderBottom: '1px solid var(--border-subtle)',
            background: 'rgba(0,0,0,0.1)',
            overflowX: 'auto',
          }}
        >
          {RAIL_SUBSYSTEMS.map((sub) => {
            const Icon = sub.icon;
            const isSel = sub.id === (activeCard ?? 'ego');
            return (
              <button
                key={sub.id}
                type="button"
                onClick={() => toggle(sub.id)}
                title={sub.name}
                aria-label={`Switch to ${sub.name}`}
                aria-pressed={isSel}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: 4,
                  padding: '4px 8px',
                  borderRadius: 'var(--radius-xs)',
                  border: isSel ? '1px solid var(--brand)' : '1px solid transparent',
                  background: isSel ? 'var(--brand-muted)' : 'transparent',
                  color: isSel ? 'var(--brand)' : 'var(--text-muted)',
                  fontSize: 10,
                  fontFamily: 'var(--font-mono)',
                  cursor: 'pointer',
                  whiteSpace: 'nowrap',
                }}
              >
                <Icon className="h-3 w-3" aria-hidden="true" />
                <span>{sub.num}</span>
              </button>
            );
          })}
        </div>
      )}

      {/* Card Body */}
      <div style={{ flex: 1, overflowY: 'auto', padding: 14, display: 'flex', flexDirection: 'column', gap: 12 }}>
        {isInspect ? (
          <>
            <EgoControlPanel index={0} salience={panelSalience('ego', ctx)} />
            <PerceptionPanel index={1} salience={panelSalience('perception', ctx)} />
            <PredictionPanel index={2} salience={panelSalience('prediction', ctx)} />
            <PlannerPanel index={3} salience={panelSalience('planner', ctx)} />
            <SafetyPanel index={4} salience={panelSalience('safety', ctx)} />
            <DriverAnalyticsPanel index={5} salience={panelSalience('analytics', ctx)} />
            <EventsPanel index={6} />
          </>
        ) : (
          renderSinglePanel(activeCard ?? 'ego')
        )}
      </div>
    </>
  );
}
