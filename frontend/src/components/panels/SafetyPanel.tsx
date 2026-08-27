'use client';
import { useSimulationStore } from '../../store/useSimulationStore';
import { AlertTriangle, ShieldCheck } from 'lucide-react';

const SEVERITY_COLOR: Record<string, string> = {
  HIGH: 'var(--critical)',
  MEDIUM: 'var(--warning)',
  LOW: 'var(--text-muted)',
  NONE: 'var(--success)',
};

// Real driver-scoring/anomaly/SHAP data (app/services/driver_scoring.py,
// anomaly_detector.py, explainability.py), streamed live -- restored 2026-08
// after being computed every tick server-side and silently discarded (see
// app/api/websockets.py's payload fix). Two render modes: `compact` for a
// small always-visible DriveMode corner card, full for DeveloperMode.
export function SafetyPanel({ compact = false }: { compact?: boolean }) {
  // NOT an object-literal selector ((state) => ({a: state.a, ...})) --
  // that allocates a new object every render, which Zustand v5 (no
  // `useShallow`) treats as "changed" every time and infinite-loops.
  // Matches the whole-store destructure pattern DriveMode/DeveloperMode
  // already use.
  const { shap, anomaly, driverScore } = useSimulationStore();

  if (compact) {
    return (
      <div className="bg-black/40 backdrop-blur-xl border border-white/10 rounded-xl px-4 py-3 shadow-2xl flex items-center gap-4 pointer-events-auto">
        <div className="flex flex-col items-center">
          <span className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-muted)]">Driver Score</span>
          <span className="text-lg font-mono font-bold text-white">
            {driverScore ? `${driverScore.score} (${driverScore.rating})` : '--'}
          </span>
        </div>
        <div className="h-8 w-[1px] bg-white/10" />
        <div className="flex items-center gap-2">
          {anomaly?.is_anomaly ? (
            <>
              <AlertTriangle className="w-4 h-4" style={{ color: SEVERITY_COLOR[anomaly.severity] }} />
              <span className="text-xs font-semibold" style={{ color: SEVERITY_COLOR[anomaly.severity] }}>
                {anomaly.type}
              </span>
            </>
          ) : (
            <>
              <ShieldCheck className="w-4 h-4 text-[var(--success)]" />
              <span className="text-xs font-semibold text-[var(--success)]">Nominal</span>
            </>
          )}
        </div>
      </div>
    );
  }

  return (
    <section className="bg-[var(--bg-panel)]/80 backdrop-blur-md border border-[var(--border-default)] rounded-lg p-6 shadow-xl">
      <div className="flex items-center justify-between mb-6">
        <h2 className="text-sm font-bold tracking-widest text-[var(--brand)] uppercase">Safety &amp; Explainability</h2>
      </div>

      {/* Anomaly */}
      <div
        className="p-3 rounded border mb-4"
        style={{ borderColor: SEVERITY_COLOR[anomaly?.severity ?? 'NONE'], background: 'var(--bg-app)' }}
      >
        <span className="block text-xs text-[var(--text-muted)] mb-1">ANOMALY DETECTOR</span>
        {anomaly?.is_anomaly ? (
          <>
            <span className="font-mono font-bold" style={{ color: SEVERITY_COLOR[anomaly.severity] }}>
              {anomaly.type} ({anomaly.severity})
            </span>
            <p className="text-xs text-[var(--text-muted)] mt-1">{anomaly.message}</p>
          </>
        ) : (
          <span className="font-mono text-[var(--success)]">Nominal -- no anomaly detected</span>
        )}
      </div>

      {/* Driver score */}
      <div className="p-3 rounded border border-[var(--border-default)] bg-[var(--bg-app)] mb-4">
        <span className="block text-xs text-[var(--text-muted)] mb-2">DRIVER SCORE (rolling 60-reading window)</span>
        {driverScore ? (
          <div className="grid grid-cols-4 gap-2 text-center font-mono">
            <div>
              <div className="text-lg font-bold text-white">{driverScore.score}</div>
              <div className="text-[9px] text-[var(--text-muted)] uppercase">Overall ({driverScore.rating})</div>
            </div>
            <div>
              <div className="text-lg font-bold text-white">{driverScore.breakdown.smoothness}</div>
              <div className="text-[9px] text-[var(--text-muted)] uppercase">Smoothness</div>
            </div>
            <div>
              <div className="text-lg font-bold text-white">{driverScore.breakdown.efficiency}</div>
              <div className="text-[9px] text-[var(--text-muted)] uppercase">Efficiency</div>
            </div>
            <div>
              <div className="text-lg font-bold text-white">{driverScore.breakdown.safety}</div>
              <div className="text-[9px] text-[var(--text-muted)] uppercase">Safety</div>
            </div>
          </div>
        ) : (
          <span className="text-xs text-[var(--text-muted)]">Waiting for telemetry...</span>
        )}
      </div>

      {/* SHAP */}
      <div className="p-3 rounded border border-[var(--border-default)] bg-[var(--bg-app)]">
        <span className="block text-xs text-[var(--text-muted)] mb-2">
          SHAP FEATURE CONTRIBUTIONS (why the classifier decided this)
        </span>
        {shap && shap.contributions.length > 0 ? (
          <div className="space-y-1.5">
            {shap.contributions.slice(0, 6).map((c) => {
              const magnitude = Math.min(1, Math.abs(c.contribution) / 0.5);
              const positive = c.contribution >= 0;
              return (
                <div key={c.feature} className="flex items-center gap-2 text-xs font-mono">
                  <span className="w-32 truncate text-[var(--text-muted)]">{c.feature}</span>
                  <div className="flex-1 h-3 bg-white/5 rounded overflow-hidden flex items-center">
                    <div
                      className="h-full rounded"
                      style={{
                        width: `${magnitude * 100}%`,
                        background: positive ? 'var(--brand)' : 'var(--critical)',
                      }}
                    />
                  </div>
                  <span className="w-14 text-right text-[var(--text-bright)]">{c.contribution.toFixed(3)}</span>
                </div>
              );
            })}
          </div>
        ) : (
          <span className="text-xs text-[var(--text-muted)]">Waiting for telemetry...</span>
        )}
      </div>
    </section>
  );
}
