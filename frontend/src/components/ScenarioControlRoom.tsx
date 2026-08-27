'use client';

import React, { useState, useEffect, useMemo } from 'react';
import {
  Play,
  Pause,
  RotateCcw,
  StepForward,
  Sliders,
  ShieldAlert,
  Gauge,
  Sparkles,
  Layers,
  ChevronDown,
  ChevronUp,
} from 'lucide-react';
import { useSimulationStore } from '../store/useSimulationStore';
import { ScenarioSummary } from '../types/protocol';

const BUILTIN_SCENARIOS: {
  id: string;
  name: string;
  category: string;
  description: string;
  badgeColor: string;
}[] = [
  {
    id: 'normal_cruising',
    name: 'Normal Cruising',
    category: 'NORMAL',
    description: 'Open road cruise at target speed with stable lane-centring.',
    badgeColor: 'border-emerald-500/30 text-emerald-400 bg-emerald-500/10',
  },
  {
    id: 'traffic_overtake',
    name: 'Traffic Overtake',
    category: 'MANEUVER',
    description: 'Slow lead vehicle triggers candidate planner passing maneuver.',
    badgeColor: 'border-cyan-500/30 text-cyan-400 bg-cyan-500/10',
  },
  {
    id: 'emergency_cut_in',
    name: 'Cut-In & Brake',
    category: 'SAFETY CRITICAL',
    description: 'Abrupt vehicle intrusion triggers TTC Safety Shield emergency braking.',
    badgeColor: 'border-rose-500/30 text-rose-400 bg-rose-500/10',
  },
  {
    id: 'queue_stop_and_go',
    name: 'Stop & Go Queue',
    category: 'TRAFFIC',
    description: 'Congested stationary queue brings ego to standstill, then resumes.',
    badgeColor: 'border-amber-500/30 text-amber-400 bg-amber-500/10',
  },
];

export function ScenarioControlRoom() {
  const scenario = useSimulationStore((state) => state.scenario);
  const scenariosList = useSimulationStore((state) => state.scenariosList);
  const loadScenario = useSimulationStore((state) => state.loadScenario);
  const togglePause = useSimulationStore((state) => state.togglePause);
  const stepSimulation = useSimulationStore((state) => state.stepSimulation);
  const resetSimulation = useSimulationStore((state) => state.resetSimulation);
  const setScenariosList = useSimulationStore((state) => state.setScenariosList);

  const [showConfig, setShowConfig] = useState(false);
  const [selectedDensity, setSelectedDensity] = useState<string>('medium');
  const [initialSpeed, setInitialSpeed] = useState<number>(45);

  // Fetch scenarios metadata from REST API on mount (using absolute backend URL with proxy fallback)
  useEffect(() => {
    const apiBase = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
    let cancelled = false;

    async function loadScenarios() {
      try {
        const res = await fetch(`${apiBase}/api/scenarios`);
        if (res.ok) {
          const data: ScenarioSummary[] = await res.json();
          if (!cancelled) setScenariosList(data);
          return;
        }
      } catch {
        // Direct backend fetch failed, attempt Next.js rewritten proxy path
      }

      try {
        const res = await fetch('/api/scenarios');
        if (res.ok) {
          const data: ScenarioSummary[] = await res.json();
          if (!cancelled) setScenariosList(data);
        }
      } catch (err) {
        console.warn('Could not fetch /api/scenarios from backend:', err);
      }
    }

    loadScenarios();
    return () => {
      cancelled = true;
    };
  }, [setScenariosList]);

  // Spacebar shortcut to pause/resume simulation
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Don't intercept spacebar if user is focused on an input element
      if (
        e.code === 'Space' &&
        !(e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement)
      ) {
        e.preventDefault();
        togglePause();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [togglePause]);

  const activeId = scenario?.id || 'normal_cruising';
  const isPaused = scenario?.is_paused ?? false;

  const getCategoryBadgeColor = (category: string) => {
    switch (category?.toLowerCase()) {
      case 'normal':
        return 'border-emerald-500/30 text-emerald-400 bg-emerald-500/10';
      case 'maneuver':
        return 'border-cyan-500/30 text-cyan-400 bg-cyan-500/10';
      case 'safety_critical':
      case 'safety critical':
        return 'border-rose-500/30 text-rose-400 bg-rose-500/10';
      case 'traffic':
        return 'border-amber-500/30 text-amber-400 bg-amber-500/10';
      default:
        return 'border-white/20 text-white/70 bg-white/5';
    }
  };

  // Derive cards directly from the live backend scenarios registry when available
  const displayScenarios: {
    id: string;
    name: string;
    category: string;
    description: string;
    badgeColor: string;
  }[] = useMemo(() => {
    if (scenariosList && scenariosList.length > 0) {
      return scenariosList.map((sc: ScenarioSummary) => ({
        id: sc.id,
        name: sc.name,
        category: sc.category.toUpperCase().replace('_', ' '),
        description: sc.description,
        badgeColor: getCategoryBadgeColor(sc.category),
      }));
    }
    return BUILTIN_SCENARIOS;
  }, [scenariosList]);

  const handleSelectScenario = (scenarioId: string) => {
    loadScenario(scenarioId, selectedDensity, initialSpeed);
  };

  return (
    <div className="pointer-events-auto bg-black/70 backdrop-blur-2xl border border-white/10 rounded-2xl shadow-2xl p-4 w-full max-w-xl text-white select-none transition-all duration-300">
      {/* Top Header & Simulation Metrics */}
      <div className="flex items-center justify-between pb-3 border-b border-white/10">
        <div className="flex items-center gap-2.5">
          <div className="p-1.5 rounded-lg bg-[var(--brand)]/10 border border-[var(--brand)]/30 text-[var(--brand)]">
            <Sparkles className="w-4 h-4" />
          </div>
          <div>
            <h2 className="text-xs font-bold tracking-wider uppercase text-white/90">
              Scenario Control Room
            </h2>
            <div className="flex items-center gap-2 mt-0.5 text-[11px] text-white/50">
              <span>{scenario?.name || 'Deterministic Replay'}</span>
              <span>•</span>
              <span className="font-mono text-white/70">
                Tick {scenario?.tick ?? 0} ({scenario?.elapsed_s?.toFixed(1) ?? '0.0'}s)
              </span>
            </div>
          </div>
        </div>

        {/* Status Indicator */}
        <div className="flex items-center gap-2">
          <div
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[10px] font-bold tracking-wider uppercase border ${
              isPaused
                ? 'bg-amber-500/10 border-amber-500/30 text-amber-400'
                : 'bg-emerald-500/10 border-emerald-500/30 text-emerald-400'
            }`}
          >
            <span
              className={`w-1.5 h-1.5 rounded-full ${
                isPaused ? 'bg-amber-400 animate-pulse' : 'bg-emerald-400 animate-pulse'
              }`}
            />
            {isPaused ? 'Paused' : 'Running'}
          </div>

          <button
            onClick={() => setShowConfig((c) => !c)}
            title="Configure Scenario Parameters"
            className="p-1.5 rounded-lg bg-white/5 hover:bg-white/10 border border-white/10 text-white/70 hover:text-white transition-colors"
          >
            <Sliders className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      {/* Scenario Selection Cards */}
      <div className="grid grid-cols-2 gap-2 mt-3">
        {displayScenarios.map((sc) => {
          const isSelected = activeId === sc.id;
          return (
            <button
              key={sc.id}
              onClick={() => handleSelectScenario(sc.id)}
              className={`flex flex-col text-left p-2.5 rounded-xl border transition-all duration-200 relative overflow-hidden group ${
                isSelected
                  ? 'bg-white/[0.08] border-[var(--brand)] shadow-[0_0_15px_rgba(59,130,246,0.15)] ring-1 ring-[var(--brand)]/30'
                  : 'bg-white/[0.03] border-white/10 hover:bg-white/[0.06] hover:border-white/20'
              }`}
            >
              <div className="flex items-center justify-between w-full mb-1">
                <span className="text-xs font-semibold text-white/90 group-hover:text-white">
                  {sc.name}
                </span>
                <span
                  className={`text-[9px] font-bold px-1.5 py-0.5 rounded border uppercase tracking-wider ${sc.badgeColor}`}
                >
                  {sc.category}
                </span>
              </div>
              <p className="text-[10px] text-white/50 leading-relaxed line-clamp-2">
                {sc.description}
              </p>
            </button>
          );
        })}
      </div>

      {/* Configuration Drawer (Density & Speed) */}
      {showConfig && (
        <div className="mt-3 p-3 rounded-xl bg-white/[0.03] border border-white/10 space-y-3 animate-in fade-in duration-150">
          <div className="flex items-center justify-between text-xs">
            <span className="text-white/70 flex items-center gap-1.5">
              <Layers className="w-3.5 h-3.5 text-[var(--brand)]" />
              Traffic Density
            </span>
            <div className="flex items-center gap-1">
              {(['low', 'medium', 'high'] as const).map((density) => (
                <button
                  key={density}
                  onClick={() => {
                    setSelectedDensity(density);
                    loadScenario(activeId, density, initialSpeed);
                  }}
                  className={`px-2 py-0.5 rounded text-[10px] font-semibold uppercase tracking-wider border transition-all ${
                    selectedDensity === density
                      ? 'bg-[var(--brand)]/20 border-[var(--brand)] text-white'
                      : 'bg-white/5 border-white/10 text-white/60 hover:text-white'
                  }`}
                >
                  {density}
                </button>
              ))}
            </div>
          </div>

          <div className="space-y-1">
            <div className="flex items-center justify-between text-xs">
              <span className="text-white/70 flex items-center gap-1.5">
                <Gauge className="w-3.5 h-3.5 text-[var(--brand)]" />
                Initial Speed
              </span>
              <span className="font-mono text-white/90 font-semibold">{initialSpeed} km/h</span>
            </div>
            <input
              type="range"
              min={20}
              max={80}
              step={5}
              value={initialSpeed}
              onChange={(e) => {
                const val = Number(e.target.value);
                setInitialSpeed(val);
                loadScenario(activeId, selectedDensity, val);
              }}
              className="w-full h-1.5 bg-white/10 rounded-lg appearance-none cursor-pointer accent-[var(--brand)]"
            />
          </div>
        </div>
      )}

      {/* Active Milestone Banner */}
      {scenario?.milestone && (
        <div className="mt-3 flex items-center gap-2 px-3 py-2 rounded-xl bg-[var(--brand)]/10 border border-[var(--brand)]/30 text-white animate-in slide-in-from-top-1 duration-200">
          <ShieldAlert className="w-4 h-4 text-[var(--brand)] shrink-0" />
          <span className="text-xs text-white/90 font-medium leading-tight">
            {scenario.milestone}
          </span>
        </div>
      )}

      {/* Transport Controls Bar */}
      <div className="flex items-center justify-between mt-3 pt-3 border-t border-white/10">
        <div className="flex items-center gap-2">
          {/* Play / Pause */}
          <button
            onClick={togglePause}
            className="flex items-center gap-2 px-3 py-1.5 rounded-xl bg-white/10 hover:bg-white/15 border border-white/20 text-white text-xs font-semibold shadow-lg transition-all active:scale-95"
          >
            {isPaused ? (
              <>
                <Play className="w-3.5 h-3.5 text-emerald-400 fill-emerald-400" />
                <span>Resume</span>
              </>
            ) : (
              <>
                <Pause className="w-3.5 h-3.5 text-amber-400 fill-amber-400" />
                <span>Pause</span>
              </>
            )}
            <kbd className="hidden sm:inline text-[9px] px-1 py-0.2 rounded bg-black/40 text-white/50 border border-white/10 font-mono">
              Space
            </kbd>
          </button>

          {/* Step (+1 Tick) */}
          <button
            onClick={stepSimulation}
            title="Step simulation forward by 1 tick (0.1s)"
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-white/5 hover:bg-white/10 border border-white/10 text-white/80 hover:text-white text-xs font-semibold transition-all active:scale-95"
          >
            <StepForward className="w-3.5 h-3.5 text-cyan-400" />
            <span>Step</span>
          </button>

          {/* Reset */}
          <button
            onClick={resetSimulation}
            title="Reset scenario to tick 0"
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-white/5 hover:bg-white/10 border border-white/10 text-white/80 hover:text-white text-xs font-semibold transition-all active:scale-95"
          >
            <RotateCcw className="w-3.5 h-3.5 text-white/60" />
            <span>Reset</span>
          </button>
        </div>

        {/* Active Scenario Indicator Badge */}
        <div className="text-[10px] font-mono text-white/40 tracking-wider">
          SEED:{' '}
          <span className="text-white/70">
            {activeId === 'normal_cruising'
              ? '42'
              : activeId === 'traffic_overtake'
              ? '101'
              : activeId === 'emergency_cut_in'
              ? '202'
              : '303'}
          </span>
        </div>
      </div>
    </div>
  );
}
