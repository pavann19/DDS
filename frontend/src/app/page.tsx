'use client';

import { useEffect } from 'react';
import { useTelemetry } from '../hooks/useTelemetry';
import { useSimulationStore } from '../store/useSimulationStore';
import { SimulationScene } from '../components/3d/SimulationScene';
import { DrivingCockpit } from '../components/console/DrivingCockpit';

const telemetryUrl = process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:8000/ws/telemetry';

export default function Home() {
  const { sendCommand } = useTelemetry(telemetryUrl);
  const setSendCommand = useSimulationStore((state) => state.setSendCommand);

  useEffect(() => {
    setSendCommand(sendCommand);
  }, [sendCommand, setSendCommand]);

  return (
    <main className="drive-app">
      <DrivingCockpit><SimulationScene /></DrivingCockpit>
    </main>
  );
}
