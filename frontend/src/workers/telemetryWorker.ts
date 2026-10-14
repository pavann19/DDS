/// <reference lib="webworker" />
import { TelemetryStatePayload, EventStreamPayload, RoutePayload } from '../types/protocol';

// Prevent TS errors with `self` being a worker global scope
declare const self: DedicatedWorkerGlobalScope;

let collectMetrics = false;
const epochNow = () => performance.timeOrigin + performance.now();
let socket: WebSocket | null = null;
let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
let watchdog: ReturnType<typeof setTimeout> | null = null;
let endpoint: string | null = null;
let retryMs = 500;
let lastStamp: {id:string;epoch:number;tick:number} | null = null;
let reportedConnected: boolean | null = null;
let deliverySequence = 0;
let inFlightDelivery: number | null = null;
let pendingState: {payload: TelemetryStatePayload; timing?: {received_ms:number;received_wall_ms:number}} | null = null;

function reportConnected(connected:boolean) {
  if (reportedConnected === connected) return;
  reportedConnected = connected;
  self.postMessage({type:'STATUS',connected});
}

function clearStateHandoff() {
  inFlightDelivery = null;
  pendingState = null;
}

function forwardState(state: NonNullable<typeof pendingState>) {
  // One posted state plus one replaceable latest state. The worker must not
  // accumulate obsolete camera poses while the main thread is busy rendering.
  if (inFlightDelivery !== null) { pendingState = state; return; }
  inFlightDelivery = ++deliverySequence;
  self.postMessage({type:'STATE_DELTA',payload:state.payload,deliveryId:inFlightDelivery,
    timing:state.timing ? {...state.timing,forwarded_ms:epochNow()} : undefined});
}

function clearTimers() {
  if (reconnectTimer) clearTimeout(reconnectTimer);
  if (watchdog) clearTimeout(watchdog);
  reconnectTimer = watchdog = null;
}

function connect(url: string) {
  const current = new WebSocket(url);
  socket = current;
  const armWatchdog = () => {
    if (watchdog) clearTimeout(watchdog);
    watchdog = setTimeout(() => {
      if (socket !== current) return;
      reportConnected(false);
      current.close();
    }, 3000);
  };
  armWatchdog();
  current.onclose = () => {
    if (socket !== current) return;
    clearTimers();
    clearStateHandoff();
    reportConnected(false);
    socket = null;
    if (endpoint) {
      reconnectTimer = setTimeout(() => { if (endpoint) connect(endpoint); }, retryMs);
      retryMs = Math.min(retryMs * 2, 5000);
    }
  };
  current.onerror = () => current.close();
  current.onmessage = (event) => {
    if (socket !== current) return;
    const received_ms = collectMetrics ? epochNow() : 0;
    const received_wall_ms = collectMetrics ? Date.now() : 0;
    try {
      const data = JSON.parse(event.data);
      if (data.type === 'state') {
        const ego = data.channels?.pose?.ego;
        if (data.protocol_version !== '3.0' || data.coordinate_frame !== 'dds_world_v1' ||
            !data.channels?.semantic || !data.channels?.heavy || !ego ||
            ![data.tick, data.epoch, data.route_version, data.simulation_time_s,
              ego.velocity, ego.acceleration, ego.pose?.x, ego.pose?.z].every(Number.isFinite)) return;
        if (lastStamp && lastStamp.id === data.simulation_id &&
            (data.epoch < lastStamp.epoch || (data.epoch === lastStamp.epoch && data.tick < lastStamp.tick))) return;
        lastStamp = {id:data.simulation_id,epoch:data.epoch,tick:data.tick};
        retryMs = 500;
        armWatchdog();
        reportConnected(true);
        forwardState({payload:normalizeCoordinates(data),
          timing:collectMetrics ? {received_ms,received_wall_ms} : undefined});
      } else if (data.type === 'event') {
        self.postMessage({ type: 'EVENT', payload: data as EventStreamPayload });
      } else if (data.type === 'route') {
        self.postMessage({ type: 'ROUTE', payload: data as RoutePayload });
      } else if (data.type === 'command_result') {
        self.postMessage({ type: 'COMMAND_RESULT', payload: data });
      }
    } catch (err) {
      console.error('Failed to parse telemetry message', err);
    }
  };
}

// Normalizes coordinates from Unreal/Other backends to canonical DDS Web frame
// Currently a pass-through, but ready for conversion logic.
function normalizeCoordinates(payload: TelemetryStatePayload): TelemetryStatePayload {
  // If we ever need to swap Y and Z, or negate X, we do it here.
  return payload;
}

self.onmessage = (e: MessageEvent) => {
  const { type, url, command } = e.data;

  if (type === 'STATE_APPLIED') {
    if (e.data.deliveryId !== inFlightDelivery) return;
    inFlightDelivery = null;
    const latest = pendingState;
    pendingState = null;
    if (latest) forwardState(latest);
    return;
  }

  if (type === 'SEND') {
    // e.g. { type: 'SEND', command: { type: 'set_destination', lat, lng } } --
    // app/api/websockets.py's listen_for_commands() expects exactly this
    // shape on the socket.
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(command));
    } else {
      self.postMessage({type:'COMMAND_RESULT',payload:{type:'command_result',
        command_id:command.command_id,status:'rejected',message:'Telemetry disconnected; command not sent'}});
    }
    return;
  }

  if (type === 'CONNECT') {
    collectMetrics = e.data.metrics === true;
    endpoint = url;
    clearTimers();
    clearStateHandoff();
    const old = socket;
    socket = null;
    old?.close();
    reportConnected(false);
    connect(url);
    return;
    
  } else if (type === 'DISCONNECT') {
    endpoint = null;
    clearTimers();
    clearStateHandoff();
    if (socket) {
      socket.close();
      socket = null;
    }
  }
};
