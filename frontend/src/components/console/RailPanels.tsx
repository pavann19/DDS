'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { PanelSection, Readout, Meter } from '../primitives';
import type { Tone } from '../primitives';

/* ----------------------------------------------------------------------------
 * Channel-aligned panels (ADR-002 rule 1): one panel per protocol v3 channel,
 * its header tag naming the exact field it reads. Nothing here invents state
 * the backend does not send.
 * ------------------------------------------------------------------------- */

const fmt = (n: number | null | undefined, d = 1) =>
  n == null || !Number.isFinite(n) ? '—' : n.toFixed(d);

/** pose.ego — kinematics + the realised control outputs. */
function EgoControlPanel() {
  const ego = useSimulationStore((s) => s.ego);
  return (
    <PanelSection title="Ego / Control" channel="pose.ego" defaultOpen>
      <Readout k="speed" v={`${fmt((ego?.velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="target" v={`${fmt((ego?.target_velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="accel" v={`${fmt(ego?.acceleration)} m/s²`} />
      <Readout k="steering" v={`${fmt((ego?.steering_angle ?? 0) * (180 / Math.PI))}°`} />
      <Readout k="binding" v={ego?.speed_limit_reason ?? '—'} tone="brand" />
      <Meter label="throttle" value={ego?.throttle ?? 0} tone="ok" />
      <Meter label="brake" value={ego?.brake ?? 0} tone="crit" />
    </PanelSection>
  );
}

/** semantic.perception + heavy.surround_perception — what the sensor rig
 *  resolves, never raw NPC ground truth. */
function PerceptionPanel() {
  const perception = useSimulationStore((s) => s.perception);
  const surround = useSimulationStore((s) => s.surroundPerception);
  const lead = [...perception].sort((a, b) => a.distance - b.distance)[0];
  const byClass = surround.reduce<Record<string, number>>((m, t) => {
    m[t.class] = (m[t.class] ?? 0) + 1;
    return m;
  }, {});
  return (
    <PanelSection
      title="Perception"
      channel="semantic.perception+heavy"
      pulseKey={surround.length}
    >
      <Readout k="objects (fwd)" v={perception.length} />
      <Readout
        k="lead"
        v={lead ? `${fmt(lead.distance)} m · ${fmt(lead.rel_velocity)} m/s` : 'none'}
        tone={lead && lead.distance < 12 ? 'warn' : 'default'}
      />
      <Readout k="surround tracks" v={surround.length} />
      {Object.entries(byClass).map(([c, n]) => (
        <Readout key={c} k={c.toLowerCase()} v={n} />
      ))}
    </PanelSection>
  );
}

/** heavy.prediction — per-agent forecasts, intent, proactive cut-in response. */
function PredictionPanel() {
  const prediction = useSimulationStore((s) => s.prediction);
  const cutIn = prediction?.cut_in;
  const agents = prediction?.agents ?? [];
  return (
    <PanelSection
      title="Prediction"
      channel="heavy.prediction"
      tone={cutIn?.active ? 'warn' : 'default'}
      pulseKey={cutIn?.active ? 'active' : 'idle'}
    >
      <Readout
        k="cut-in"
        v={cutIn?.active ? `easing off · P ${Math.round((cutIn.probability ?? 0) * 100)}%` : 'clear'}
        tone={cutIn?.active ? 'warn' : 'ok'}
      />
      <Readout k="time to cross" v={cutIn?.time_to_cross_s != null ? `${fmt(cutIn.time_to_cross_s)} s` : '—'} />
      <Readout k="proactive decel" v={`${fmt(prediction?.proactive_decel_mps2)} m/s²`} />
      <Readout k="agents forecast" v={agents.length} />
      {agents.slice(0, 3).map((a) => {
        const top = a.intent[0];
        return (
          <Meter
            key={a.track_id}
            label={`#${a.track_id} ${top?.label.toLowerCase() ?? '—'}`}
            value={top?.p ?? 0}
            tone="brand"
          />
        );
      })}
    </PanelSection>
  );
}

/** semantic.safety_shield — the independent post-decision check. */
function SafetyPanel() {
  const shield = useSimulationStore((s) => s.safetyShield);
  const risk = shield?.risk_level ?? 'NONE';
  const tone: Tone =
    risk === 'CRITICAL' || risk === 'HIGH' ? 'crit' : risk === 'MEDIUM' ? 'warn' : 'ok';
  return (
    <PanelSection
      title="Safety Shield"
      channel="semantic.safety_shield"
      tone={shield && !shield.approved ? 'crit' : 'default'}
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

/** semantic.planner — Frenet lateral planner candidates. */
function PlannerPanel() {
  const planner = useSimulationStore((s) => s.planner);
  const chosen = planner?.candidates.find((c) => c.is_chosen);
  return (
    <PanelSection title="Planner" channel="semantic.planner" pulseKey={planner?.is_changing_lane ? 'lc' : 'hold'}>
      <Readout k="lane center d" v={`${fmt(planner?.lane_center)} m`} />
      <Readout k="curvature" v={fmt(planner?.curvature, 4)} />
      <Readout k="lane change" v={planner?.is_changing_lane ? 'yes' : 'no'} tone={planner?.is_changing_lane ? 'warn' : 'default'} />
      <Readout k="candidates" v={planner?.candidates.length ?? 0} />
      {chosen && <Readout k="chosen d_target" v={`${fmt(chosen.d_target)} m · cost ${fmt(chosen.cost, 2)}`} tone="brand" />}
    </PanelSection>
  );
}

/** semantic.driver_analytics — the learned model as analytics, NOT control. */
function DriverAnalyticsPanel() {
  const shap = useSimulationStore((s) => s.shap);
  const anomaly = useSimulationStore((s) => s.anomaly);
  const score = useSimulationStore((s) => s.driverScore);
  const top = [...(shap?.contributions ?? [])]
    .sort((a, b) => Math.abs(b.contribution) - Math.abs(a.contribution))
    .slice(0, 3);
  return (
    <PanelSection
      title="Driver Analytics"
      channel="semantic.driver_analytics"
      tone={anomaly?.is_anomaly ? 'warn' : 'default'}
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
      <Readout
        k="anomaly"
        v={anomaly?.is_anomaly ? `${anomaly.type} (${anomaly.severity})` : 'none'}
        tone={anomaly?.is_anomaly ? 'warn' : 'ok'}
      />
      {top.map((c) => (
        <Readout key={c.feature} k={c.feature} v={`${c.contribution >= 0 ? '+' : ''}${fmt(c.contribution, 3)}`} />
      ))}
    </PanelSection>
  );
}

/** The right rail: every subsystem, one channel-bound panel each. */
export function Rail() {
  return (
    <>
      <EgoControlPanel />
      <PerceptionPanel />
      <PredictionPanel />
      <SafetyPanel />
      <PlannerPanel />
      <DriverAnalyticsPanel />
    </>
  );
}
