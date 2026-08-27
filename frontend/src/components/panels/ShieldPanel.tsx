'use client';
import { useSimulationStore } from '../../store/useSimulationStore';
import { ShieldCheck, ShieldAlert } from 'lucide-react';

const RISK_COLOR: Record<string, string> = {
  NONE: 'var(--success)',
  LOW: 'var(--success)',
  MEDIUM: 'var(--warning)',
  HIGH: 'var(--warning)',
  CRITICAL: 'var(--critical)',
};

// app/services/safety_shield.py: an INDEPENDENT check, run AFTER the
// planner/IDM decision, that can override it. Rendered as the three-stage
// flow the architecture is actually built around -- AI decision -> safety
// validation -> final action -- not collapsed into one number, because the
// whole point of the shield is that those three things can disagree.
export function ShieldPanel({ compact = false }: { compact?: boolean }) {
  const { ego, safetyShield } = useSimulationStore();
  const overridden = !!safetyShield && !safetyShield.approved;

  if (compact) {
    return (
      <div
        className={`flex items-center gap-2 bg-black/40 backdrop-blur-xl border rounded-xl px-4 py-2.5 shadow-2xl pointer-events-auto transition-colors ${
          overridden ? 'border-[var(--critical)] animate-pulse' : 'border-white/10'
        }`}
      >
        {overridden ? (
          <ShieldAlert className="w-4 h-4 text-[var(--critical)]" />
        ) : (
          <ShieldCheck className="w-4 h-4 text-[var(--success)]" />
        )}
        <div className="flex flex-col">
          <span className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-muted)]">
            Safety Shield
          </span>
          <span
            className="text-xs font-mono font-bold"
            style={{ color: RISK_COLOR[safetyShield?.risk_level ?? 'NONE'] }}
          >
            {overridden ? `OVERRIDE: ${safetyShield?.override_action}` : 'Approved'}
          </span>
        </div>
      </div>
    );
  }

  return (
    <section className="bg-[var(--bg-panel)]/80 backdrop-blur-md border border-[var(--border-default)] rounded-lg p-6 shadow-xl">
      <div className="flex items-center justify-between mb-6">
        <h2 className="text-sm font-bold tracking-widest text-[var(--brand)] uppercase">Safety Shield</h2>
        <span className="text-xs text-[var(--text-muted)]">Independent check -- runs after the planner decides</span>
      </div>

      {/* AI decision -> safety validation -> final action */}
      <div className="grid grid-cols-3 gap-3 mb-4 text-center">
        <div className="p-3 rounded border border-[var(--border-default)] bg-[var(--bg-app)]">
          <span className="block text-[9px] text-[var(--text-muted)] uppercase tracking-wider mb-1">AI Decision</span>
          <span className="font-mono font-bold text-white">{ego?.decision ?? '--'}</span>
        </div>
        <div className="p-3 rounded border bg-[var(--bg-app)]"
             style={{ borderColor: RISK_COLOR[safetyShield?.risk_level ?? 'NONE'] }}>
          <span className="block text-[9px] text-[var(--text-muted)] uppercase tracking-wider mb-1">Safety Validation</span>
          <span className="font-mono font-bold" style={{ color: RISK_COLOR[safetyShield?.risk_level ?? 'NONE'] }}>
            {safetyShield ? (safetyShield.approved ? 'APPROVED' : 'REJECTED') : '--'}
          </span>
        </div>
        <div className="p-3 rounded border border-[var(--border-default)] bg-[var(--bg-app)]">
          <span className="block text-[9px] text-[var(--text-muted)] uppercase tracking-wider mb-1">Final Action</span>
          <span className={`font-mono font-bold ${overridden ? 'text-[var(--critical)]' : 'text-white'}`}>
            {overridden ? safetyShield?.override_action : (ego?.decision ?? '--')}
          </span>
        </div>
      </div>

      {/* Detail */}
      <div className="p-3 rounded border border-[var(--border-default)] bg-[var(--bg-app)] space-y-2 font-mono text-xs">
        <div className="flex justify-between">
          <span className="text-[var(--text-muted)]">Risk level</span>
          <span style={{ color: RISK_COLOR[safetyShield?.risk_level ?? 'NONE'] }}>
            {safetyShield?.risk_level ?? 'NONE'}
          </span>
        </div>
        {safetyShield?.ttc_s !== null && safetyShield?.ttc_s !== undefined && (
          <div className="flex justify-between">
            <span className="text-[var(--text-muted)]">Time-to-collision</span>
            <span className="text-white">{safetyShield.ttc_s.toFixed(1)}s</span>
          </div>
        )}
        {safetyShield?.reasons.length ? (
          <ul className="pt-1 border-t border-[var(--border-subtle)] space-y-1">
            {safetyShield.reasons.map((r, i) => (
              <li key={i} className="text-[var(--warning)]">- {r}</li>
            ))}
          </ul>
        ) : (
          <div className="text-[var(--success)] pt-1 border-t border-[var(--border-subtle)]">
            All checks passed -- TTC, road boundary, vehicle dynamics
          </div>
        )}
      </div>
    </section>
  );
}
