'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { PanelSection, Readout, Meter, MultiStat } from '../primitives';
import type { Tone } from '../primitives';
import { useTween } from '../../hooks/useTween';
import { panelSalience, type ConsoleCtx, type Salience } from '../../lib/consoleState';
import { EventTimeline } from './EventTimeline';

/* ----------------------------------------------------------------------------
 * The rail reads as one autonomous-system story (§11): a numbered pipeline
 *   01 EGO → 02 PERCEPTION → 03 PREDICTION → 04 PLANNER → 05 SAFETY → 06 ANALYTICS
 * Each card is bound 1:1 to a protocol v3 channel (header tag); nothing
 * renders state the backend doesn't send. Context-aware (§12):
 * panelSalience() dims quiet cards and accents the reacting one. Density
 * governs depth — `inspect` adds raw per-track / per-candidate rows.
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

/** 01 — pose.ego: kinematics + realised control outputs. */
function EgoControlPanel({ index, salience }: { index: number; salience: Salience }) {
  const ego = useSimulationStore((s) => s.ego);
  const speed = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 240));
  const d = ego?.frenet?.d ?? 0;
  const laneDesc = Math.abs(d) < 1 ? 'lane 1 (centre)' : d < -1.6 ? 'lane 2 (left / overtake)' : d > 1.6 ? 'right lane' : 'lane transition';
  return (
    <PanelSection title="01 · Ego kinematics" icon="⚡" channel="pose.ego" index={index} salience={salience}>
      <Readout k="current velocity" v={`${fmt(speed)} km/h`} tone="brand" />
      <Readout k="target velocity" v={`${fmt((ego?.target_velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="longitudinal accel" v={`${(ego?.acceleration ?? 0) >= 0 ? '+' : ''}${fmt(ego?.acceleration, 2)} m/s²`} />
      <Readout k="steering angle" v={`${fmt((ego?.steering_angle ?? 0) * (180 / Math.PI))}°`} />
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
  const risk = shield?.risk_level ?? 'NONE';
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
        {overridden ? 'SHIELD OVERRIDE' : `SYSTEM ${risk === 'NONE' ? 'NOMINAL' : risk}`}
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

/** The right rail: header + the numbered autonomy pipeline, own scroll. */
export function Rail() {
  const ctx = useCtx();
  return (
    <>
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '14px 16px',
          borderBottom: '1px solid var(--border-subtle)',
          background: 'rgba(0,0,0,0.2)',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: 9 }}>
          <span
            aria-hidden
            style={{ width: 7, height: 7, borderRadius: '50%', background: 'var(--brand)', boxShadow: '0 0 8px var(--brand)', animation: 'dds-beacon 1.5s infinite' }}
          />
          <h2 style={{ fontFamily: 'var(--font-display)', fontSize: 12, fontWeight: 700, letterSpacing: '0.08em', color: 'var(--text-bright)' }}>
            AV OPERATIONAL TELEMETRY
          </h2>
        </div>
        <span
          className="font-mono"
          style={{ fontSize: 10, fontWeight: 600, color: 'var(--brand)', background: 'var(--brand-muted)', padding: '2px 7px', borderRadius: 'var(--radius-xs)', border: '1px solid var(--brand-dim)' }}
        >
          10 Hz
        </span>
      </div>
      <div style={{ flex: 1, overflowY: 'auto', padding: 14, display: 'flex', flexDirection: 'column', gap: 12 }}>
        <EgoControlPanel index={0} salience={panelSalience('ego', ctx)} />
        <PerceptionPanel index={1} salience={panelSalience('perception', ctx)} />
        <PredictionPanel index={2} salience={panelSalience('prediction', ctx)} />
        <PlannerPanel index={3} salience={panelSalience('planner', ctx)} />
        <SafetyPanel index={4} salience={panelSalience('safety', ctx)} />
        <DriverAnalyticsPanel index={5} salience={panelSalience('analytics', ctx)} />
        <EventsPanel index={6} />
      </div>
    </>
  );
}
