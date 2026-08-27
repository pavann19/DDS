import { useCallback, useEffect, useRef } from 'react';
import { useSimulationStore } from '../store/useSimulationStore';
import { RoutePayload, TelemetryStatePayload } from '../types/protocol';

export function useTelemetry(url: string) {
  const workerRef = useRef<Worker>(null);
  const { applyDelta, setConnected, setRoute } = useSimulationStore();

  useEffect(() => {
    // Initialize Web Worker
    workerRef.current = new Worker(new URL('../workers/telemetryWorker.ts', import.meta.url), {
      type: 'module',
    });

    workerRef.current.onmessage = (event: MessageEvent) => {
      const { type, payload, connected } = event.data;

      switch (type) {
        case 'STATUS':
          setConnected(connected);
          break;
        case 'STATE_DELTA':
          applyDelta(payload as TelemetryStatePayload);
          break;
        case 'ROUTE': {
          const route = payload as RoutePayload;
          setRoute(route.waypoints, route.steps);
          break;
        }
        case 'EVENT':
          // TODO: Dispatch to Event Store (Phase 5)
          console.log('Received Event:', payload);
          break;
      }
    };

    // Tell worker to connect
    workerRef.current.postMessage({ type: 'CONNECT', url });

    return () => {
      workerRef.current?.postMessage({ type: 'DISCONNECT' });
      workerRef.current?.terminate();
    };
  }, [url, applyDelta, setConnected, setRoute]);

  // Exposed so callers (e.g. a destination picker) can send a command back
  // over the same socket -- app/api/websockets.py's listen_for_commands()
  // currently understands only { type: 'set_destination', lat, lng }.
  const sendCommand = useCallback((command: Record<string, unknown>) => {
    workerRef.current?.postMessage({ type: 'SEND', command });
  }, []);

  return { sendCommand };
}
