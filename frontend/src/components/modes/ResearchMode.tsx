'use client';
import { AnalyticsChart } from '../charts/AnalyticsChart';

export function ResearchMode() {
  // Mock data for analytics placeholder
  const chartData = {
    labels: ['Run 1', 'Run 2', 'Run 3', 'Run 4', 'Run 5'],
    datasets: [
      {
        label: 'Safety Score (0-100)',
        data: [85, 88, 92, 90, 95],
        borderColor: '#00E5FF',
        backgroundColor: 'rgba(0, 229, 255, 0.1)',
        tension: 0.4,
      },
      {
        label: 'Comfort Score (0-100)',
        data: [70, 75, 72, 80, 85],
        borderColor: '#00FF88',
        backgroundColor: 'rgba(0, 255, 136, 0.1)',
        tension: 0.4,
      }
    ]
  };

  return (
    <div className="absolute inset-0 z-10 p-8 overflow-y-auto bg-[var(--bg-app)] pointer-events-auto">
      <header className="mb-8 border-b border-[var(--border-default)] pb-4">
        <h1 className="text-xl font-bold text-[var(--text-bright)] mb-1">Research Lab</h1>
        <p className="text-xs text-[var(--text-muted)]">Aggregate Run Analysis & Experiment Configurator</p>
      </header>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
        <section className="bg-[var(--bg-panel)] border border-[var(--border-default)] rounded-lg p-6 shadow-xl">
          <h2 className="text-sm font-bold tracking-widest text-[var(--brand)] uppercase mb-6">Macro Metrics</h2>
          <div className="h-80">
            <AnalyticsChart data={chartData} title="Model Performance over Runs" />
          </div>
        </section>

        <section className="bg-[var(--bg-panel)] border border-[var(--border-default)] rounded-lg p-6 shadow-xl">
          <h2 className="text-sm font-bold tracking-widest text-[var(--text-bright)] uppercase mb-6">Experiment Configurator</h2>
          <div className="space-y-4">
            <div className="bg-[var(--bg-surface)] p-4 rounded border border-[var(--border-default)]">
              <label className="block text-xs text-[var(--text-muted)] mb-2 uppercase tracking-wider">Target Speed (km/h)</label>
              <input type="range" min="30" max="120" defaultValue="80" className="w-full accent-[var(--brand)]" />
            </div>
            <div className="bg-[var(--bg-surface)] p-4 rounded border border-[var(--border-default)]">
              <label className="block text-xs text-[var(--text-muted)] mb-2 uppercase tracking-wider">Traffic Density</label>
              <input type="range" min="0" max="20" defaultValue="8" className="w-full accent-[var(--brand)]" />
            </div>
            <button className="w-full bg-[var(--brand)] text-[var(--bg-app)] font-bold py-3 rounded hover:opacity-90 transition-opacity">
              Deploy Experiment
            </button>
          </div>
        </section>
      </div>
    </div>
  );
}
