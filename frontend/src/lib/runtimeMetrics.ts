/** Opt-in bounded diagnostics. No driving inputs or product controls depend on this. */
export interface WorkerTiming { received_ms: number; forwarded_ms: number; received_wall_ms: number }
export function createRuntimeCapture(enabled = false, capacity = 2048) {
  const samples = new Map<string, number[]>();
  const totals = new Map<string, number>();
  const invalid = new Map<string, number>();
  let pending: { key: string; applied: number; received: number } | null = null;
  let lastKey: string | null = null;
  let lastFrame: number | null = null;
  let superseded = 0;
  let duplicates = 0;
  let suspended = false;
  const record = (name: string, value: number) => {
    if (!enabled) return;
    if (!Number.isFinite(value) || value < 0) { invalid.set(name, (invalid.get(name) ?? 0) + 1); return; }
    const values = samples.get(name) ?? [];
    values.push(value);
    if (values.length > capacity) values.shift();
    samples.set(name, values); totals.set(name, (totals.get(name) ?? 0) + 1);
  };
  return {
    enabled, record,
    applied(key: string, timing: WorkerTiming, started: number, finished: number, serverTimestamp: string) {
      if (!enabled) return;
      record('worker_parse_validate_ms', timing.forwarded_ms - timing.received_ms);
      record('worker_to_main_queue_ms', started - timing.forwarded_ms);
      record('main_store_apply_ms', finished - started);
      // Wall clocks are comparable ONLY in the declared same-host benchmark.
      record('server_stamp_to_worker_wall_ms', timing.received_wall_ms - Date.parse(serverTimestamp));
      if (key === lastKey) { duplicates++; return; }
      if (pending) superseded++;
      lastKey = key;
      if (!suspended) pending = { key, applied: finished, received: timing.received_ms };
    },
    frame(now: number, visible: boolean) {
      if (!enabled) return;
      if (!visible) { suspended = true; lastFrame = null; pending = null; return; }
      if (suspended) { pending = null; suspended = false; }
      if (lastFrame !== null) record('scene_callback_interval_ms', now - lastFrame);
      lastFrame = now;
      if (pending) {
        record('main_apply_to_scene_callback_ms', now - pending.applied);
        record('worker_receive_to_scene_callback_ms', now - pending.received);
        pending = null;
      }
    },
    reset() { samples.clear(); totals.clear(); invalid.clear(); pending = null; lastKey = null; lastFrame = null; superseded = duplicates = 0; },
    snapshot() {
      const distributions = Object.fromEntries([...samples].map(([name, values]) => {
        const sorted = values.slice().sort((a,b) => a-b);
        const at = (p: number) => sorted[Math.floor((sorted.length-1)*p)];
        return [name, { sample_count: sorted.length, total_count: totals.get(name), p50_ms: at(.5), p95_ms: at(.95), p99_ms: at(.99), max_ms: sorted.at(-1), samples_ms: values.slice() }];
      }));
      return { enabled, capacity, distributions, invalid_samples: Object.fromEntries(invalid), superseded_before_scene: superseded, same_stamp_messages: duplicates,
        scope: 'CPU callback/receipt instrumentation; no painted FPS, GPU completion or cross-host clock calibration' };
    },
  };
}
export const epochNow = () => performance.timeOrigin + performance.now();
export const runtimeCapture = createRuntimeCapture(typeof window !== 'undefined' && new URLSearchParams(window.location.search).get('ddsMetrics') === '1');
if (typeof window !== 'undefined' && runtimeCapture.enabled) {
  Object.defineProperty(window, '__ddsRuntime', { value: runtimeCapture, configurable: true });
  if (typeof PerformanceObserver !== 'undefined' && PerformanceObserver.supportedEntryTypes.includes('longtask')) {
    const observer = new PerformanceObserver(list => { for (const entry of list.getEntries()) runtimeCapture.record('main_thread_long_task_ms', entry.duration); });
    observer.observe({ type: 'longtask', buffered: false });
  }
}
