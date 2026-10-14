import { runtimeCapture, epochNow } from '../lib/runtimeMetrics';
import { useCallback, useEffect, useRef } from 'react';
import { useSimulationStore } from '../store/useSimulationStore';
import { useEvents } from '../store/useEvents';
import { EventStreamPayload, RoutePayload, TelemetryStatePayload } from '../types/protocol';

export function useTelemetry(url: string) {
  const workerRef = useRef<Worker>(null);
  const applyDelta = useSimulationStore(s => s.applyDelta);
  const setConnected = useSimulationStore(s => s.setConnected);
  const setRoute = useSimulationStore(s => s.setRoute);

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
          if (!connected) useEvents.getState().clear();
          break;
        case 'COMMAND_RESULT':
          useSimulationStore.getState().setCommandResult(payload);
          break;
        case 'STATE_DELTA': {
          const started = runtimeCapture.enabled ? epochNow() : 0;
          applyDelta(payload as TelemetryStatePayload);
          if (runtimeCapture.enabled && event.data.timing) {
            const p = payload as TelemetryStatePayload;
            runtimeCapture.applied(`${p.simulation_id}:${p.epoch}:${p.tick}`, event.data.timing, started, epochNow(), p.timestamp);
          }
          workerRef.current?.postMessage({type:'STATE_APPLIED',deliveryId:event.data.deliveryId});
          break;
        }
        case 'ROUTE': {
          const route = payload as RoutePayload;
          setRoute(route.waypoints, route.steps, route.road_context);
          break;
        }
        case 'EVENT': {
          const evt = payload as EventStreamPayload;
          if (evt?.event) useEvents.getState().pushEvent(evt.event);
          break;
        }
      }
    };

    // Tell worker to connect
    workerRef.current.postMessage({ type: 'CONNECT', url, metrics: runtimeCapture.enabled });

    return () => {
      setConnected(false);
      workerRef.current?.postMessage({ type: 'DISCONNECT' });
      workerRef.current?.terminate();
    };
  }, [url, applyDelta, setConnected, setRoute]);

  // Exposed so callers (e.g. a destination picker) can send a command back
  // over the same socket -- app/api/websockets.py's listen_for_commands()
  // currently understands only { type: 'set_destination', lat, lng }.
  const sendCommand = useCallback((command: Record<string, unknown>) => {
    const command_id = crypto.randomUUID();
    // A new request replaces the displayed last command; responses cannot
    // overwrite it if they belong to an older request.
    useSimulationStore.setState({commandResult:{type:'command_result',command_id,status:'sending'}});
    workerRef.current?.postMessage({ type: 'SEND', command: {...command,command_id} });
  }, []);

  return { sendCommand };
}
