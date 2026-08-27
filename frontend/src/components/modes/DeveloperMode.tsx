'use client';
import { useSimulationStore } from '../../store/useSimulationStore';
import { ScenarioControlRoom } from '../ScenarioControlRoom';
import { SafetyPanel } from '../panels/SafetyPanel';
import { ShieldPanel } from '../panels/ShieldPanel';

export function DeveloperMode() {
  const { isConnected, ego, traffic, tick, simulationTime } = useSimulationStore();
  
  return (
    <div className="relative z-10 pointer-events-none h-full flex flex-col p-8 overflow-hidden">
      <header className="flex justify-between items-center mb-8 pb-4 pointer-events-auto border-b border-[var(--border-default)]">
        <div>
          <h1 className="text-xl font-bold text-[var(--text-bright)] mb-1">DDS Inspector</h1>
          <p className="text-xs text-[var(--text-muted)]">Developer Telemetry View</p>
        </div>
        <div className="flex items-center gap-3 bg-[var(--bg-panel)] px-4 py-2 rounded-md border border-[var(--border-default)]">
          <div className={`w-3 h-3 rounded-full ${isConnected ? 'bg-[var(--success)] shadow-[0_0_10px_var(--success)]' : 'bg-[var(--critical)] shadow-[0_0_10px_var(--critical)]'}`} />
          <span className="text-sm tracking-wider uppercase font-semibold text-[var(--text-bright)]">
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
          {isConnected && (
            <span className="ml-4 text-xs font-mono text-[var(--brand)]">
              Tick: {tick} | {simulationTime.toFixed(1)}s
            </span>
          )}
        </div>
      </header>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8 pointer-events-auto overflow-y-auto pb-20">
        <div className="col-span-full">
          <ScenarioControlRoom />
        </div>

        <section className="bg-[var(--bg-panel)]/80 backdrop-blur-md border border-[var(--border-default)] rounded-lg p-6 shadow-xl">
          <div className="flex items-center justify-between mb-6">
            <h2 className="text-sm font-bold tracking-widest text-[var(--brand)] uppercase">Ego Vehicle</h2>
            <div className="text-xs bg-[var(--bg-surface)] px-2 py-1 rounded font-mono text-[var(--text-primary)] border border-[var(--border-subtle)]">
              30Hz Stream
            </div>
          </div>
          
          {ego ? (
            <div className="space-y-4 text-sm font-mono">
              <div className="grid grid-cols-2 gap-4">
                <div className="bg-[var(--bg-app)]/90 p-3 rounded border border-[var(--border-default)]">
                  <span className="block text-xs text-[var(--text-muted)] mb-1">DECISION</span>
                  <span className="text-[var(--text-bright)] font-bold text-lg">{ego.decision}</span>
                </div>
                <div className="bg-[var(--bg-app)]/90 p-3 rounded border border-[var(--border-default)]">
                  <span className="block text-xs text-[var(--text-muted)] mb-1">VELOCITY</span>
                  <span className="text-[var(--text-bright)] font-bold text-lg">{ego.velocity.toFixed(2)} <span className="text-[var(--text-muted)] text-sm">m/s</span></span>
                </div>
                <div className="bg-[var(--bg-app)]/90 p-3 rounded border border-[var(--border-default)]">
                  <span className="block text-xs text-[var(--text-muted)] mb-1">TARGET VELOCITY</span>
                  <span className="text-[var(--text-bright)] font-bold text-lg">{ego.target_velocity.toFixed(2)} <span className="text-[var(--text-muted)] text-sm">m/s</span></span>
                </div>
                <div className="bg-[var(--bg-app)]/90 p-3 rounded border border-[var(--border-default)]">
                  <span className="block text-xs text-[var(--text-muted)] mb-1">ACCELERATION</span>
                  <span className="text-[var(--text-bright)] font-bold text-lg">{ego.acceleration.toFixed(2)} <span className="text-[var(--text-muted)] text-sm">m/s²</span></span>
                </div>
              </div>
              
              <div className="bg-[var(--bg-app)]/90 p-4 rounded border border-[var(--border-default)]">
                <h3 className="text-xs text-[var(--text-muted)] mb-2 border-b border-[var(--border-default)] pb-2 uppercase tracking-wider">Frenet Coordinates</h3>
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <span className="text-[var(--text-muted)]">S: </span>
                    <span className="text-[var(--text-bright)]">{ego.frenet.s.toFixed(2)}</span>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">D: </span>
                    <span className="text-[var(--text-bright)]">{ego.frenet.d.toFixed(2)}</span>
                  </div>
                </div>
              </div>
            </div>
          ) : (
            <div className="text-[var(--text-muted)] flex items-center justify-center h-48 font-mono">Waiting for telemetry...</div>
          )}
        </section>

        <section className="bg-[var(--bg-panel)]/80 backdrop-blur-md border border-[var(--border-default)] rounded-lg p-6 shadow-xl flex flex-col max-h-[60vh]">
          <div className="flex items-center justify-between mb-6">
            <h2 className="text-sm font-bold tracking-widest text-[var(--text-bright)] uppercase">Traffic Array</h2>
            <div className="text-xs bg-[var(--bg-surface)] px-2 py-1 rounded font-mono text-[var(--brand)] border border-[var(--brand-muted)]">
              {traffic.length} NPC(s)
            </div>
          </div>
          
          <div className="flex-1 bg-[var(--bg-app)]/90 border border-[var(--border-default)] rounded overflow-y-auto font-mono">
            {traffic.length > 0 ? (
              <table className="w-full text-left text-xs">
                <thead className="bg-[var(--bg-surface)] sticky top-0 border-b border-[var(--border-default)]">
                  <tr>
                    <th className="p-3 font-medium text-[var(--text-muted)]">ID</th>
                    <th className="p-3 font-medium text-[var(--text-muted)]">V (m/s)</th>
                    <th className="p-3 font-medium text-[var(--text-muted)]">S</th>
                    <th className="p-3 font-medium text-[var(--text-muted)]">D</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-[var(--border-default)]">
                  {traffic.map(npc => (
                    <tr key={npc.id} className="hover:bg-[var(--bg-surface)] transition-colors">
                      <td className="p-3 text-[var(--brand)]">{npc.id}</td>
                      <td className="p-3 text-[var(--text-bright)]">{npc.velocity.toFixed(2)}</td>
                      <td className="p-3 text-[var(--text-bright)]">{npc.frenet.s.toFixed(2)}</td>
                      <td className="p-3 text-[var(--text-bright)]">{npc.frenet.d.toFixed(2)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            ) : (
              <div className="flex items-center justify-center h-32 text-[var(--text-muted)]">
                No traffic detected.
              </div>
            )}
          </div>
        </section>

        <ShieldPanel />
        <SafetyPanel />
      </div>
    </div>
  );
}
