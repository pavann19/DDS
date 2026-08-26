'use client';

import { useEffect } from 'react';
import { useTelemetry } from '../hooks/useTelemetry';
import { useUISettings } from '../store/useUISettings';
import { useSimulationStore } from '../store/useSimulationStore';
import { SimulationScene } from '../components/3d/SimulationScene';
import { DriveMode } from '../components/modes/DriveMode';
import { DeveloperMode } from '../components/modes/DeveloperMode';
import { ResearchMode } from '../components/modes/ResearchMode';

export default function Home() {
  // Connect to the backend
  const { sendCommand } = useTelemetry('ws://localhost:8000/ws/telemetry');
  const setSendCommand = useSimulationStore((state) => state.setSendCommand);

  useEffect(() => {
    setSendCommand(sendCommand);
  }, [sendCommand, setSendCommand]);

  const { activeMode } = useUISettings();

  return (
    <main className="relative min-h-screen font-sans transition-colors overflow-hidden bg-[var(--bg-app)] text-[var(--text-primary)]">
      {/* 3D Canvas Background (render unless in full opaque Research Mode) */}
      {activeMode !== 'research' && <SimulationScene />}
      
      {/* UI Modes */}
      {activeMode === 'drive' && <DriveMode />}
      {activeMode === 'developer' && <DeveloperMode />}
      {activeMode === 'research' && <ResearchMode />}
      
      {/* Global Command Palette Hint */}
      <div className="absolute bottom-4 right-4 z-50 pointer-events-none">
        <p className="text-[10px] text-[var(--text-muted)] bg-[var(--bg-app)]/80 backdrop-blur rounded px-2 py-1 border border-[var(--border-default)]">
          Press <kbd className="font-mono text-[var(--brand)]">Ctrl+K</kbd> to change modes
        </p>
      </div>
    </main>
  );
}
