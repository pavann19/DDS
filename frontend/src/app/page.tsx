'use client';

import { useEffect } from 'react';
import { useTelemetry } from '../hooks/useTelemetry';
import { useSimulationStore } from '../store/useSimulationStore';
import { SimulationScene } from '../components/3d/SimulationScene';
import { ConsoleLayout } from '../components/console/ConsoleLayout';
import { Rail } from '../components/console/RailPanels';
import { ScenarioStrip } from '../components/console/ScenarioStrip';
import { DriveHUD } from '../components/hud/DriveHUD';
import { ManeuverCard } from '../components/hud/ManeuverCard';

export default function Home() {
  const { sendCommand } = useTelemetry('ws://localhost:8000/ws/telemetry');
  const setSendCommand = useSimulationStore((state) => state.setSendCommand);

  useEffect(() => {
    setSendCommand(sendCommand);
  }, [sendCommand, setSendCommand]);

  return (
    <main
      className="relative min-h-screen overflow-hidden"
      style={{ background: 'var(--bg-app)', color: 'var(--text-primary)' }}
    >
      {/* The 3D stage is always mounted, behind the console overlay. */}
      <SimulationScene />

      {/* One console surface (ADR-002): stage passthrough + HUD + rail + strip. */}
      <ConsoleLayout
        hud={<DriveHUD />}
        rail={<Rail />}
        card={<ManeuverCard />}
        strip={<ScenarioStrip />}
      />
    </main>
  );
}
