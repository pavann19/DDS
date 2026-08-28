'use client';

import React from 'react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useConsole } from '../../store/useConsole';
import { PanelSection, Readout, Meter } from '../primitives';
import type { Tone } from '../primitives';
import { useTween } from '../../hooks/useTween';

/* ----------------------------------------------------------------------------
 * Channel-aligned panels (ADR-002 rule 1): one panel per protocol v3 channel,
 * its header tag naming the exact field it reads. Nothing here invents state
 * the backend does not send.
 *
 * Density (item 8): `inspect` forces every panel open and reveals the raw
 * per-track rows; `standard` uses each panel's own default; `focus` hides the
 * rail entirely (handled in ConsoleLayout).
 * ------------------------------------------------------------------------- */

const fmt = (n: number | null | undefined, d = 1) =>
  n == null || !Number.isFinite(n) ? '—' : n.toFixed(d);

function useForcedOpen() {
  const density = useConsole((s) => s.density);
  return density === 'inspect' ? true : undefined;
}
function useInspect() {
  return useConsole((s) => s.density) === 'inspect';
}

/** pose.ego — kinematics + the realised control outputs. */
function EgoControlPanel({ index }: { index: number }) {
  const ego = useSimulationStore((s) => s.ego);
  const open = useForcedOpen();
  const speed = useTween((ego?.velocity ?? 0) * 3.6, 240);
  return (
    <PanelSection title="Ego / Control" channel="pose.ego" defaultOpen open={open} index={index}>
      <Readout k="speed" v={`${fmt(speed)} km/h`} tone="brand" />
      <Readout k="target" v={`${fmt((ego?.target_velocity ?? 0) * 3.6)} km/h`} />
      <Readout k="accel" v={`${fmt(ego?.acceleration)} m/s²`} />
      <Readout k="steering" v={`${fmt((ego?.steering_angle ?? 0) * (180 / Math.PI))}°`} />
      <Readout k="binding" v={ego?.speed_limit_reason ?? '—'} />
      <Meter label="throttle" value={ego?.throttle ?? 0} tone="ok" />
      <Meter label="brake" value={ego?.brake ?? 0} tone="crit" />
    </PanelSection>
  );
}

/** semantic.perception + heavy.surround_perception — what the sensor rig
 *  resolves, never raw NPC ground truth. */
function PerceptionPanel({ index }: { index: number }) {
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
      title="Perception"
      channel="semantic.perception+heavy"
      open={open}
      index={index}
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
        Object.entries(byClass).map(([c, n]) => (
          <Readout key={c} k={c.toLowerCase()} v={n} />
        ))}
      {inspect &&
        surround.map((t) => (
          <Readout
            key={t.id}
            k={`${t.class.toLowerCase()} ${t.id}`}
            v={`${fmt(t.range_m)} m · ${fmt(t.azimuth_deg, 0)}°`}
          />
        ))}
    </PanelSection>
  );
}

/** heavy.prediction — per-agent forecasts, intent, proactive cut-in response. */
function PredictionPanel({ index }: { index: number }) {
  const prediction = useSimulationStore((s) => s.prediction);
  const open = useForcedOpen();
  const inspect = useInspect();
  const cutIn = prediction?.cut_in;
  const agents = prediction?.agents ?? [];
  const decel = useTween(prediction?.proactive_decel_mps2 ?? 0, 240);
  const shown = inspect ? agents : agents.slice(0, 3);
  return (
    <PanelSection
      title="Prediction"
      channel="heavy.prediction"
      tone={cutIn?.active ? 'warn' : 'default'}
      open={open}
      index={index}
      pulseKey={cutIn?.active ? 'active' : 'idle'}
    >
      <Readout
        k="cut-in"
        v={cutIn?.active ? `easing off · P ${Math.round((cutIn.probability ?? 0) * 100)}%` : 'clear'}
        tone={cutIn?.active ? 'warn' : 'ok'}
      />
      <Readout
        k="time to cross"
        v={cutIn?.time_to_cross_s != null ? `${fmt(cutIn.time_to_cross_s)} s` : '—'}
      />
      <Readout k="proactive decel" v={`${fmt(decel)} m/s²`} />
      <Readout k="agents forecast" v={agents.length} />
      {shown.map((a) => {
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
function SafetyPanel({ index }: { index: number }) {
  const shield = useSimulationStore((s) => s.safetyShield);
  const open = useForcedOpen();
  const risk = shield?.risk_level ?? 'NONE';
  const tone: Tone =
    risk === 'CRITICAL' || risk === 'HIGH' ? 'crit' : risk === 'MEDIUM' ? 'warn' : 'ok';
  return (
    <PanelSection
      title="Safety Shield"
      channel="semantic.safety_shield"
      tone={shield && !shield.approved ? 'crit' : 'default'}
      open={open}
      index={index}
      pulseKey={shield?.override_action ?? 'ok'}
      defaultOpen
    >
      <Readout
        k="verdict"
        v={shield?.approved === false ? 'OVERRIDE' : 'approved'}
        tone={shield?.approved === false ? 'crit' : 'ok'}
      />
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
function PlannerPanel({ index }: { index: number }) {
  const planner = useSimulationStore((s) => s.planner);
  const open = useForcedOpen();
  const inspect = useInspect();
  const chosen = planner?.candidates.find((c) => c.is_chosen);
  return (
    <PanelSection
      title="Planner"
      channel="semantic.planner"
      open={open}
      index={index}
      pulseKey={planner?.is_changing_lane ? 'lc' : 'hold'}
    >
      <Readout k="lane center d" v={`${fmt(planner?.lane_center)} m`} />
      <Readout k="curvature" v={fmt(planner?.curvature, 4)} />
      <Readout
        k="lane change"
        v={planner?.is_changing_lane ? 'yes' : 'no'}
        tone={planner?.is_changing_lane ? 'warn' : 'default'}
      />
      <Readout k="candidates" v={planner?.candidates.length ?? 0} />
      {chosen && (
        <Readout
          k="chosen d_target"
          v={`${fmt(chosen.d_target)} m · cost ${fmt(chosen.cost, 2)}`}
          tone="brand"
        />
      )}
      {inspect &&
        planner?.candidates.map((c, i) => (
          <Readout
            key={i}
            k={`cand ${fmt(c.d_target, 1)}m`}
            v={`${fmt(c.cost, 2)}${c.is_chosen ? ' ✓' : ''}${c.is_lane_change ? ' ⇄' : ''}`}
          />
        ))}
    </PanelSection>
  );
}

/** semantic.driver_analytics — the learned model as analytics, NOT control. */
function DriverAnalyticsPanel({ index }: { index: number }) {
  const shap = useSimulationStore((s) => s.shap);
  const anomaly = useSimulationStore((s) => s.anomaly);
  const score = useSimulationStore((s) => s.driverScore);
  const open = useForcedOpen();
  const top = [...(shap?.contributions ?? [])]
    .sort((a, b) => Math.abs(b.contribution) - Math.abs(a.contribution))
    .slice(0, 3);
  return (
    <PanelSection
      title="Driver Analytics"
      channel="semantic.driver_analytics"
      tone={anomaly?.is_anomaly ? 'warn' : 'default'}
      open={open}
      index={index}
      pulseKey={anomaly?.type ?? 'none'}
    >
      <div
        style={{ fontSize: 10, color: 'var(--text-faint)', padding: '2px 0 6px', lineHeight: 1.4 }}
      >
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
        <Readout
          key={c.feature}
          k={c.feature}
          v={`${c.contribution >= 0 ? '+' : ''}${fmt(c.contribution, 3)}`}
        />
      ))}
    </PanelSection>
  );
}

/** The right rail: every subsystem, one channel-bound panel each. */
export function Rail() {
  return (
    <>
      <EgoControlPanel index={0} />
      <PerceptionPanel index={1} />
      <PredictionPanel index={2} />
      <SafetyPanel index={3} />
      <PlannerPanel index={4} />
      <DriverAnalyticsPanel index={5} />
    </>
  );
}
