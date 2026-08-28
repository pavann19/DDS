'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { PanelSection, Readout, Meter } from '../primitives';
import type { Tone } from '../primitives';
import { useTween } from '../../hooks/useTween';
import { panelSalience, type ConsoleCtx, type Salience } from '../../lib/consoleState';
import { EventTimeline } from './EventTimeline';

/* ----------------------------------------------------------------------------
 * The rail reads as one autonomous-system story (§11): a numbered pipeline
 *   01 EGO → 02 PERCEPTION → 03 PREDICTION → 04 PLANNER → 05 SAFETY → 06 ANALYTICS
 * Each panel is bound 1:1 to a protocol v3 channel (header tag) and nothing
 * renders state the backend doesn't send.
 *
 * Context-aware (§12): panelSalience() dims quiet subsystems and pushes the
 * reacting one to `alert` (full strength + auto-open). Density still governs
 * how much each panel shows — `inspect` forces all open + raw rows.
 * ------------------------------------------------------------------------- */

const fmt = (n: number | null | undefined, d = 1) =>
  n == null || !Number.isFinite(n) ? '—' : n.toFixed(d);

function useForcedOpen() {
  return useConsole((s) => s.density) === 'inspect' ? true : undefined;
}
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
  const open = useForcedOpen();
  const speed = Math.max(0, useTween((ego?.velocity ?? 0) * 3.6, 240));
  const d = ego?.frenet?.d ?? 0;
  const laneDesc = Math.abs(d) < 1 ? 'lane centre' : d < -1.6 ? 'left / overtake lane' : d > 1.6 ? 'right lane' : 'lane transition';
  return (
    <PanelSection title="01 · Ego" channel="pose.ego" defaultOpen open={open} index={index} salience={salience}>
      <Readout k="speed" v={`${fmt(speed)} km/h`} tone="brand" />
      <Readout k="target" v={`${fmt((ego?.target_velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="lane" v={laneDesc} />
      <Readout k="accel" v={`${fmt(ego?.acceleration)} m/s²`} />
      <Readout k="steering" v={`${fmt((ego?.steering_angle ?? 0) * (180 / Math.PI))}°`} />
      <Readout k="binding" v={ego?.speed_limit_reason ?? '—'} />
      <Meter label="throttle" value={ego?.throttle ?? 0} tone="ok" />
      <Meter label="brake" value={ego?.brake ?? 0} tone="crit" />
    </PanelSection>
  );
}

/** 02 — semantic.perception + heavy.surround_perception. */
function PerceptionPanel({ index, salience }: { index: number; salience: Salience }) {
  const perception = useSimulationStore((s) => s.perception);
  const surround = useSimulationStore((s) => s.surroundPerception);
  const open = useForcedOpen();
  const inspect = useInspect();
  const lead = [...perception].sort((a, b) => a.distance - b.distance)[0];
  const byClass = surround.reduce<Record<string, number>>((m, t) => {
    m[t.class] = (m[t.class] ?? 0) + 1;
    return m;
  }, {});
  return (
    <PanelSection
      title="02 · Perception"
      channel="semantic.perception"
      open={open}
      index={index}
      salience={salience}
      pulseKey={surround.length}
    >
      <Readout k="objects (fwd)" v={perception.length} />
      <Readout
        k="lead"
        v={lead ? `${fmt(lead.distance)} m · ${fmt(lead.rel_velocity)} m/s` : 'none'}
        tone={lead && lead.distance < 12 ? 'warn' : 'default'}
      />
      <Readout k="surround tracks" v={surround.length} />
      {!inspect &&
        Object.entries(byClass).map(([c, n]) => <Readout key={c} k={c.toLowerCase()} v={n} />)}
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
  const open = useForcedOpen();
  const inspect = useInspect();
  const cutIn = prediction?.cut_in;
  const agents = prediction?.agents ?? [];
  const decel = useTween(prediction?.proactive_decel_mps2 ?? 0, 240);
  const shown = inspect ? agents : agents.slice(0, 3);
  return (
    <PanelSection
      title="03 · Prediction"
      channel="heavy.prediction"
      tone={cutIn?.active ? 'warn' : 'default'}
      open={open}
      index={index}
      salience={salience}
      pulseKey={cutIn?.active ? 'active' : 'idle'}
    >
      <Readout
        k="cut-in"
        v={cutIn?.active ? `easing off · P ${Math.round((cutIn.probability ?? 0) * 100)}%` : 'clear'}
        tone={cutIn?.active ? 'warn' : 'ok'}
      />
      <Readout k="time to cross" v={cutIn?.time_to_cross_s != null ? `${fmt(cutIn.time_to_cross_s)} s` : '—'} />
      <Readout k="proactive decel" v={`${fmt(decel)} m/s²`} />
      <Readout k="agents forecast" v={agents.length} />
      {shown.map((a) => {
        const top = a.intent[0];
        return (
          <Meter key={a.track_id} label={`#${a.track_id} ${top?.label.toLowerCase() ?? '—'}`} value={top?.p ?? 0} tone="brand" />
        );
      })}
    </PanelSection>
  );
}

/** 04 — semantic.planner: Frenet lateral planner candidates. */
function PlannerPanel({ index, salience }: { index: number; salience: Salience }) {
  const planner = useSimulationStore((s) => s.planner);
  const open = useForcedOpen();
  const inspect = useInspect();
  const chosen = planner?.candidates.find((c) => c.is_chosen);
  return (
    <PanelSection
      title="04 · Planner"
      channel="semantic.planner"
      tone={planner?.is_changing_lane ? 'warn' : 'default'}
      open={open}
      index={index}
      salience={salience}
      pulseKey={planner?.is_changing_lane ? 'lc' : 'hold'}
    >
      <Readout k="lane center d" v={`${fmt(planner?.lane_center)} m`} />
      <Readout k="curvature" v={fmt(planner?.curvature, 4)} />
      <Readout k="lane change" v={planner?.is_changing_lane ? 'yes' : 'no'} tone={planner?.is_changing_lane ? 'warn' : 'default'} />
      <Readout k="candidates" v={planner?.candidates.length ?? 0} />
      {chosen && <Readout k="chosen d_target" v={`${fmt(chosen.d_target)} m · cost ${fmt(chosen.cost, 2)}`} tone="brand" />}
      {inspect &&
        planner?.candidates.map((c, i) => (
          <Readout key={i} k={`cand ${fmt(c.d_target, 1)}m`} v={`${fmt(c.cost, 2)}${c.is_chosen ? ' ✓' : ''}${c.is_lane_change ? ' ⇄' : ''}`} />
        ))}
    </PanelSection>
  );
}

/** 05 — semantic.safety_shield: the independent post-decision check. */
function SafetyPanel({ index, salience }: { index: number; salience: Salience }) {
  const shield = useSimulationStore((s) => s.safetyShield);
  const open = useForcedOpen();
  const risk = shield?.risk_level ?? 'NONE';
  const tone: Tone = risk === 'CRITICAL' || risk === 'HIGH' ? 'crit' : risk === 'MEDIUM' ? 'warn' : 'ok';
  return (
    <PanelSection
      title="05 · Safety"
      channel="semantic.safety_shield"
      tone={shield && !shield.approved ? 'crit' : 'default'}
      open={open}
      index={index}
      salience={salience}
      pulseKey={shield?.override_action ?? 'ok'}
      defaultOpen
    >
      <Readout k="verdict" v={shield?.approved === false ? 'OVERRIDE' : 'approved'} tone={shield?.approved === false ? 'crit' : 'ok'} />
      <Readout k="risk" v={risk} tone={tone} />
      <Readout k="ttc" v={shield?.ttc_s != null ? `${fmt(shield.ttc_s)} s` : '—'} />
      {shield?.override_action && <Readout k="action" v={shield.override_action} tone="crit" />}
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
  const open = useForcedOpen();
  const top = [...(shap?.contributions ?? [])]
    .sort((a, b) => Math.abs(b.contribution) - Math.abs(a.contribution))
    .slice(0, 3);
  return (
    <PanelSection
      title="06 · Analytics"
      channel="semantic.driver_analytics"
      tone={anomaly?.is_anomaly ? 'warn' : 'default'}
      open={open}
      index={index}
      salience={salience}
      pulseKey={anomaly?.type ?? 'none'}
    >
      <div style={{ fontSize: 10, color: 'var(--text-faint)', padding: '2px 0 6px', lineHeight: 1.4 }}>
        Observational only — does not drive the vehicle (ADR-001).
      </div>
      <Readout k="score" v={score ? `${fmt(score.score, 0)} · ${score.rating}` : '—'} />
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
  const open = useForcedOpen();
  return (
    <PanelSection title="07 · Events" channel="event.stream" open={open} index={index} salience="quiet">
      <EventTimeline variant="list" />
    </PanelSection>
  );
}

/** The right rail: the numbered autonomy pipeline. */
export function Rail() {
  const ctx = useCtx();
  return (
    <>
      <EgoControlPanel index={0} salience={panelSalience('ego', ctx)} />
      <PerceptionPanel index={1} salience={panelSalience('perception', ctx)} />
      <PredictionPanel index={2} salience={panelSalience('prediction', ctx)} />
      <PlannerPanel index={3} salience={panelSalience('planner', ctx)} />
      <SafetyPanel index={4} salience={panelSalience('safety', ctx)} />
      <DriverAnalyticsPanel index={5} salience={panelSalience('analytics', ctx)} />
      <EventsPanel index={6} />
    </>
  );
}
