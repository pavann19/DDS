'use client';

import React, { useEffect, useState } from 'react';
import { Play, Pause, StepForward, RotateCcw, Navigation } from 'lucide-react';
import { useSimulationStore } from '../../store/useSimulationStore';
import type { ScenarioSummary } from '../../types/protocol';
import { Chip } from '../primitives';

/* The bottom strip (ADR-002 item 7): scenario state · transport · quick
 * scenario pick · destination. One mono row, DDS tokens only — folds in
 * the old ScenarioControlRoom + DestinationInput. */

const FALLBACK: { id: string; name: string }[] = [
  { id: 'normal_cruising', name: 'Normal Cruising' },
  { id: 'traffic_overtake', name: 'Traffic Overtake' },
  { id: 'emergency_cut_in', name: 'Cut-In & Brake' },
  { id: 'queue_stop_and_go', name: 'Stop & Go Queue' },
];

const DESTINATIONS: { label: string; lat: number; lng: number }[] = [
  { label: 'Golden Gate', lat: 37.8199, lng: -122.4783 },
  { label: 'Ferry Bldg', lat: 37.7955, lng: -122.3937 },
  { label: 'Twin Peaks', lat: 37.7544, lng: -122.4477 },
];

const btn: React.CSSProperties = {
  all: 'unset',
  boxSizing: 'border-box',
  display: 'inline-flex',
  alignItems: 'center',
  gap: 6,
  padding: '5px 10px',
  fontSize: 11,
  fontFamily: 'var(--font-mono)',
  color: 'var(--text-primary)',
  background: 'var(--bg-surface)',
  border: '1px solid var(--border-default)',
  borderRadius: 'var(--radius-sm)',
  cursor: 'pointer',
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

  return (
    <div
      style={{
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        gap: 'var(--space-4)',
        padding: '0 var(--space-4)',
        fontFamily: 'var(--font-mono)',
        fontSize: 11,
        color: 'var(--text-muted)',
        overflowX: 'auto',
      }}
    >
      {/* status */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: 3, minWidth: 132 }}>
        <Chip tone={paused ? 'warn' : 'ok'} dot>
          {paused ? 'paused' : 'running'}
        </Chip>
        <span style={{ color: 'var(--text-faint)', fontSize: 10 }}>
          {scenario?.name ?? 'Free Drive'} · t{scenario?.tick ?? 0}
        </span>
      </div>

      {/* transport */}
      <div style={{ display: 'flex', gap: 6 }}>
        <button style={btn} onClick={togglePause} aria-label={paused ? 'Resume' : 'Pause'}>
          {paused ? <Play className="h-3.5 w-3.5" /> : <Pause className="h-3.5 w-3.5" />}
          {paused ? 'resume' : 'pause'}
        </button>
        <button style={btn} onClick={stepSimulation} aria-label="Step one tick">
          <StepForward className="h-3.5 w-3.5" /> step
        </button>
        <button style={btn} onClick={resetSimulation} aria-label="Reset scenario">
          <RotateCcw className="h-3.5 w-3.5" /> reset
        </button>
      </div>

      <span style={{ width: 1, height: 28, background: 'var(--border-strong)' }} />

      {/* scenario quick-pick */}
      <div style={{ display: 'flex', gap: 6 }}>
        {cards.map((c) => {
          const on = c.id === activeId;
          return (
            <button
              key={c.id}
              style={{
                ...btn,
                color: on ? 'var(--brand-ink)' : 'var(--text-muted)',
                background: on ? 'var(--brand)' : 'var(--bg-surface)',
                borderColor: on ? 'var(--brand)' : 'var(--border-default)',
                fontWeight: on ? 600 : 400,
              }}
              onClick={() => loadScenario(c.id)}
            >
              {c.name}
            </button>
          );
        })}
      </div>

      <span style={{ flex: 1 }} />

      {/* milestone */}
      {scenario?.milestone && (
        <span style={{ color: 'var(--brand)', fontSize: 10, maxWidth: 260, whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>
          ◆ {scenario.milestone}
        </span>
      )}

      {/* destination */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
        <Navigation className="h-3.5 w-3.5" style={{ color: 'var(--brand)' }} />
        {DESTINATIONS.map((d) => (
          <button
            key={d.label}
            style={{
              ...btn,
              color: dest === d.label ? 'var(--brand)' : 'var(--text-muted)',
              borderColor: dest === d.label ? 'var(--brand-dim)' : 'var(--border-default)',
            }}
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
