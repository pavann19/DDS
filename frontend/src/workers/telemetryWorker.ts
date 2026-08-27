/// <reference lib="webworker" />
import { TelemetryStatePayload, EventStreamPayload, RoutePayload } from '../types/protocol';

// Prevent TS errors with `self` being a worker global scope
declare const self: DedicatedWorkerGlobalScope;

let socket: WebSocket | null = null;

// Normalizes coordinates from Unreal/Other backends to canonical DDS Web frame
// Currently a pass-through, but ready for conversion logic.
function normalizeCoordinates(payload: TelemetryStatePayload): TelemetryStatePayload {
  // If we ever need to swap Y and Z, or negate X, we do it here.
  return payload;
}

self.onmessage = (e: MessageEvent) => {
  const { type, url, command } = e.data;

  if (type === 'SEND') {
    // e.g. { type: 'SEND', command: { type: 'set_destination', lat, lng } } --
    // app/api/websockets.py's listen_for_commands() expects exactly this
    // shape on the socket.
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(command));
    } else {
      console.warn('telemetryWorker: SEND requested but socket is not open');
    }
    return;
  }

  if (type === 'CONNECT') {
    if (socket) socket.close();
    
    socket = new WebSocket(url);
    
    socket.onopen = () => {
      self.postMessage({ type: 'STATUS', connected: true });
    };
    
    socket.onclose = () => {
      self.postMessage({ type: 'STATUS', connected: false });
    };
    
    socket.onerror = (error) => {
      console.error('WebSocket Error:', error);
    };
    
    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (data.type === 'state') {
          const payload = data as TelemetryStatePayload;
          // Validate & Normalize
          if (payload.protocol_version !== "2.0") {
             console.warn("Unsupported protocol version:", payload.protocol_version);
          }
          const normalized = normalizeCoordinates(payload);
          
          // Send to main thread
          self.postMessage({ type: 'STATE_DELTA', payload: normalized });
        } else if (data.type === 'event') {
          const payload = data as EventStreamPayload;
          // Route to Event Store/Timeline
          self.postMessage({ type: 'EVENT', payload });
        } else if (data.type === 'route') {
          // Was previously silently dropped here -- the real OSRM road
          // never reached the renderer, which is why SimulationScene fell
          // back to a generic infinite highway instead of the actual route.
          const payload = data as RoutePayload;
          self.postMessage({ type: 'ROUTE', payload });
        }
      } catch (err) {
        console.error('Failed to parse telemetry message', err);
      }
    };
  } else if (type === 'DISCONNECT') {
    if (socket) {
      socket.close();
      socket = null;
    }
  }
};
