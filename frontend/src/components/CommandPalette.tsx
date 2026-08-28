'use client';
import { useEffect, useState } from 'react';
import { Command } from 'cmdk';
import { useConsole, DENSITY_ORDER } from '../store/useConsole';
import { useSimulationStore } from '../store/useSimulationStore';
import { Search, LayoutDashboard, Play, Pause, StepForward, RotateCcw } from 'lucide-react';

const SCENARIOS: { id: string; label: string }[] = [
  { id: 'normal_cruising', label: 'Normal Cruising (open road)' },
  { id: 'traffic_overtake', label: 'Traffic Overtake (lane change)' },
  { id: 'emergency_cut_in', label: 'Cut-In & Brake (safety shield)' },
  { id: 'queue_stop_and_go', label: 'Stop & Go Queue (IDM follow)' },
];

const ITEM_CLASS =
  'flex items-center px-2 py-3 rounded cursor-pointer aria-selected:bg-[var(--bg-surface)] aria-selected:text-[var(--brand)] transition-colors data-[selected=true]:bg-[var(--bg-surface)] data-[selected=true]:text-[var(--brand)]';

export function CommandPalette() {
  const [open, setOpen] = useState(false);
  const setDensity = useConsole((s) => s.setDensity);
  const loadScenario = useSimulationStore((s) => s.loadScenario);
  const togglePause = useSimulationStore((s) => s.togglePause);
  const stepSimulation = useSimulationStore((s) => s.stepSimulation);
  const resetSimulation = useSimulationStore((s) => s.resetSimulation);

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if (e.key === 'k' && (e.metaKey || e.ctrlKey)) {
        e.preventDefault();
        setOpen((o) => !o);
      }
    };
    document.addEventListener('keydown', down);
    return () => document.removeEventListener('keydown', down);
  }, []);

  if (!open) return null;

  const run = (fn: () => void) => () => {
    fn();
    setOpen(false);
  };

  return (
    <div
      className="fixed inset-0 z-50 flex items-start justify-center pt-[20vh] bg-black/60 backdrop-blur-sm"
      onClick={() => setOpen(false)}
    >
      <Command
        className="w-full max-w-lg bg-[var(--bg-panel)] rounded-xl border border-[var(--border-default)] shadow-2xl overflow-hidden text-[var(--text-primary)]"
        onClick={(e) => e.stopPropagation()}
        loop
      >
        <div className="flex items-center border-b border-[var(--border-default)] px-3 py-2">
          <Search className="w-5 h-5 text-[var(--text-muted)] mr-2" />
          <Command.Input
            autoFocus
            placeholder="Type a command…  (density, scenario, pause)"
            className="w-full bg-transparent outline-none text-[var(--text-bright)] py-2 placeholder:text-[var(--text-muted)]"
          />
        </div>

        <Command.List className="max-h-[320px] overflow-y-auto p-2">
          <Command.Empty className="p-4 text-center text-sm text-[var(--text-muted)]">
            No results.
          </Command.Empty>

          <Command.Group
            heading={
              <div className="px-2 py-1 text-xs font-semibold text-[var(--text-muted)] uppercase tracking-wider">
                Console density
              </div>
            }
          >
            {DENSITY_ORDER.map((d) => (
              <Command.Item key={d} onSelect={run(() => setDensity(d))} className={ITEM_CLASS}>
                <LayoutDashboard className="w-4 h-4 mr-3" />
                Density: {d}
              </Command.Item>
            ))}
          </Command.Group>

          <Command.Group
            heading={
              <div className="px-2 py-1 text-xs font-semibold text-[var(--text-muted)] uppercase tracking-wider">
                Scenarios
              </div>
            }
          >
            {SCENARIOS.map((s) => (
              <Command.Item key={s.id} onSelect={run(() => loadScenario(s.id))} className={ITEM_CLASS}>
                <Play className="w-4 h-4 mr-3 text-[var(--success)]" />
                {s.label}
              </Command.Item>
            ))}
          </Command.Group>

          <Command.Group
            heading={
              <div className="px-2 py-1 text-xs font-semibold text-[var(--text-muted)] uppercase tracking-wider">
                Simulation
              </div>
            }
          >
            <Command.Item onSelect={run(togglePause)} className={ITEM_CLASS}>
              <Pause className="w-4 h-4 mr-3 text-[var(--warning)]" />
              Pause / Resume [Space]
            </Command.Item>
            <Command.Item onSelect={run(stepSimulation)} className={ITEM_CLASS}>
              <StepForward className="w-4 h-4 mr-3 text-[var(--brand)]" />
              Step Forward (+1 tick)
            </Command.Item>
            <Command.Item onSelect={run(resetSimulation)} className={ITEM_CLASS}>
              <RotateCcw className="w-4 h-4 mr-3 text-[var(--text-muted)]" />
              Reset Scenario
            </Command.Item>
          </Command.Group>
        </Command.List>
      </Command>
    </div>
  );
}
