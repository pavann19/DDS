'use client';

import React, { useEffect, useState } from 'react';
import { Play, Pause, StepForward, RotateCcw, MapPin } from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';
import { useEvents } from '../../store/useEvents';
import type { ScenarioSummary } from '../../types/protocol';
import { EventTimeline } from './EventTimeline';

/* The bottom strip (ADR-002 item 7): scenario state · transport · quick
 * scenario pick · destination. One mono row, DDS tokens only — folds in
 * the old ScenarioControlRoom + DestinationInput. */

const FALLBACK: { id: string; name: string }[] = [
  { id: 'normal_cruising', name: 'Normal Cruising' },
  { id: 'traffic_overtake', name: 'Traffic Overtake' },
  { id: 'emergency_cut_in', name: 'Cut-In & Brake' },
  { id: 'queue_stop_and_go', name: 'Stop & Go Queue' },
];

// US-101 / I-280 Peninsula freeway waypoints — OSRM keeps the route on the
// highway (long straights, gentle sweepers, no city junctions).
const DESTINATIONS: { label: string; lat: number; lng: number }[] = [
  { label: '101 · Burlingame', lat: 37.585, lng: -122.352 },
  { label: '101 · Redwood City', lat: 37.4849, lng: -122.228 },
  { label: '280 · Daly City', lat: 37.665, lng: -122.47 },
];

const pill: React.CSSProperties = {
  all: 'unset',
  boxSizing: 'border-box',
  display: 'inline-flex',
  alignItems: 'center',
  gap: 6,
  padding: '6px 13px',
  fontSize: 11.5,
  fontWeight: 600,
  fontFamily: 'var(--font-sans)',
  color: 'var(--text-muted)',
  background: 'var(--bg-inset)',
  border: '1px solid var(--border-subtle)',
  borderRadius: 'var(--radius-pill)',
  cursor: 'pointer',
  whiteSpace: 'nowrap',
  transition: 'all var(--dur-fast) var(--ease-out)',
};

const iconBtn: React.CSSProperties = {
  all: 'unset',
  boxSizing: 'border-box',
  display: 'inline-flex',
  alignItems: 'center',
  justifyContent: 'center',
  width: 32,
  height: 32,
  color: 'var(--text-primary)',
  background: 'var(--bg-card)',
  border: '1px solid var(--border-subtle)',
  borderRadius: 'var(--radius-md)',
  cursor: 'pointer',
  transition: 'all var(--dur-fast) var(--ease-out)',
};

export function ScenarioStrip() {
  const scenario = useSimulationStore((s) => s.scenario);
  const scenariosList = useSimulationStore((s) => s.scenariosList);
  const setScenariosList = useSimulationStore((s) => s.setScenariosList);
  const loadScenario = useSimulationStore((s) => s.loadScenario);
  const togglePause = useSimulationStore((s) => s.togglePause);
  const stepSimulation = useSimulationStore((s) => s.stepSimulation);
  const resetSimulation = useSimulationStore((s) => s.resetSimulation);
  const sendCommand = useSimulationStore((s) => s.sendCommand);

  const hasEvents = useEvents((s) => s.events.length > 0);
  const [dest, setDest] = useState<string | null>(null);

  useEffect(() => {
    const base = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
    let cancelled = false;
    (async () => {
      for (const url of [`${base}/api/scenarios`, '/api/scenarios']) {
        try {
          const res = await fetch(url);
          if (res.ok) {
            const data: ScenarioSummary[] = await res.json();
            if (!cancelled) setScenariosList(data);
            return;
          }
        } catch {
          /* try next */
        }
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [setScenariosList]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (
        e.code === 'Space' &&
        !(e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement)
      ) {
        e.preventDefault();
        togglePause();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [togglePause]);

  const cards = scenariosList?.length
    ? scenariosList.map((s) => ({ id: s.id, name: s.name }))
    : FALLBACK;
  const activeId = scenario?.id ?? 'normal_cruising';
  const paused = scenario?.is_paused ?? false;

  const activePill = (on: boolean): React.CSSProperties =>
    on
      ? {
          color: '#fff',
          background: 'linear-gradient(135deg, rgba(0,242,254,0.28), rgba(56,189,248,0.16))',
          borderColor: 'var(--brand)',
          boxShadow: '0 0 14px rgba(0,242,254,0.32)',
        }
      : {};

  return (
    <div
      style={{
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        gap: 'var(--space-4)',
        padding: '0 var(--space-4)',
      }}
    >
      {/* scenario tag + running dot */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, flex: 'none' }}>
        <span
          aria-hidden
          style={{
            width: 7,
            height: 7,
            borderRadius: '50%',
            background: paused ? 'var(--warning)' : 'var(--success)',
            boxShadow: `0 0 8px ${paused ? 'var(--warning)' : 'var(--success)'}`,
            animation: paused ? undefined : 'dds-beacon 1.8s infinite ease-in-out',
          }}
        />
        <span
          style={{
            fontFamily: 'var(--font-display)',
            fontSize: 11,
            fontWeight: 800,
            letterSpacing: '0.1em',
            color: 'var(--text-faint)',
            whiteSpace: 'nowrap',
          }}
        >
          🎬 SCENARIO
        </span>
      </div>

      {/* scenario pills — scrollable */}
      <div
        style={{ flex: 1, display: 'flex', alignItems: 'center', gap: 8, overflowX: 'auto', scrollbarWidth: 'none' }}
      >
        {cards.map((c) => {
          const on = c.id === activeId;
          return (
            <button key={c.id} style={{ ...pill, ...activePill(on) }} onClick={() => loadScenario(c.id)}>
              {c.name}
            </button>
          );
        })}
      </div>

      {/* latest operational event */}
      {hasEvents ? (
        <EventTimeline variant="ticker" />
      ) : scenario?.milestone ? (
        <span
          style={{
            fontFamily: 'var(--font-mono)',
            color: 'var(--brand)',
            fontSize: 10,
            maxWidth: 220,
            whiteSpace: 'nowrap',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
            flex: 'none',
          }}
        >
          ◆ {scenario.milestone}
        </span>
      ) : null}

      <span style={{ width: 1, height: 26, background: 'var(--border-subtle)', flex: 'none' }} />

      {/* playback */}
      <div style={{ display: 'flex', gap: 6, flex: 'none' }}>
        <button style={iconBtn} onClick={togglePause} aria-label={paused ? 'Resume' : 'Pause'} title={paused ? 'Resume' : 'Pause'}>
          {paused ? <Play className="h-4 w-4" /> : <Pause className="h-4 w-4" />}
        </button>
        <button style={iconBtn} onClick={stepSimulation} aria-label="Step one tick" title="Step +1 tick">
          <StepForward className="h-4 w-4" />
        </button>
        <button style={iconBtn} onClick={resetSimulation} aria-label="Reset scenario" title="Reset">
          <RotateCcw className="h-4 w-4" />
        </button>
      </div>

      <span style={{ width: 1, height: 26, background: 'var(--border-subtle)', flex: 'none' }} />

      {/* destination */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, flex: 'none' }}>
        <MapPin className="h-3.5 w-3.5" style={{ color: 'var(--brand)' }} aria-hidden />
        {DESTINATIONS.map((d) => (
          <button
            key={d.label}
            style={{ ...pill, ...(dest === d.label ? { color: 'var(--brand)', borderColor: 'var(--brand-dim)' } : {}) }}
            onClick={() => {
              sendCommand({ type: 'set_destination', lat: d.lat, lng: d.lng });
              setDest(d.label);
            }}
          >
            {d.label}
          </button>
        ))}
      </div>
    </div>
  );
}
