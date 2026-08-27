'use client';
import { useState } from 'react';
import { Navigation } from 'lucide-react';
import { useSimulationStore } from '../store/useSimulationStore';

// A handful of real, useful destinations near the backend's hardcoded
// start point (app/api/websockets.py: PhysicsEngine(start_lat=37.7749,
// start_lng=-122.4194) -- San Francisco) so this is usable without typing
// raw coordinates, plus a manual lat/lng fallback for anywhere else.
const PRESETS: { label: string; lat: number; lng: number }[] = [
  { label: 'Golden Gate Bridge', lat: 37.8199, lng: -122.4783 },
  { label: 'Ferry Building', lat: 37.7955, lng: -122.3937 },
  { label: 'Twin Peaks', lat: 37.7544, lng: -122.4477 },
];

export function DestinationInput() {
  const sendCommand = useSimulationStore((state) => state.sendCommand);
  const [open, setOpen] = useState(false);
  const [lat, setLat] = useState('');
  const [lng, setLng] = useState('');
  const [lastSet, setLastSet] = useState<string | null>(null);

  const setDestination = (destLat: number, destLng: number, label?: string) => {
    // Matches app/api/websockets.py's SetDestinationCommand exactly --
    // { type: 'set_destination', lat, lng }.
    sendCommand({ type: 'set_destination', lat: destLat, lng: destLng });
    setLastSet(label ?? `${destLat.toFixed(4)}, ${destLng.toFixed(4)}`);
    setOpen(false);
  };

  const handleManualSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const parsedLat = parseFloat(lat);
    const parsedLng = parseFloat(lng);
    if (Number.isFinite(parsedLat) && Number.isFinite(parsedLng)) {
      setDestination(parsedLat, parsedLng);
      setLat('');
      setLng('');
    }
  };

  return (
    <div className="pointer-events-auto">
      <button
        onClick={() => setOpen((o) => !o)}
        className="flex items-center gap-2 bg-black/40 backdrop-blur-xl border border-white/10 rounded-xl px-4 py-2.5 shadow-2xl text-xs font-semibold text-white hover:border-[var(--brand)]/50 transition-colors"
      >
        <Navigation className="w-4 h-4 text-[var(--brand)]" />
        {lastSet ? `To: ${lastSet}` : 'Set Destination'}
      </button>

      {open && (
        <div className="mt-2 bg-black/60 backdrop-blur-xl border border-white/10 rounded-xl p-4 shadow-2xl w-64 space-y-3">
          <div className="space-y-1.5">
            <span className="text-[10px] font-bold uppercase tracking-wider text-[var(--text-muted)]">Presets</span>
            {PRESETS.map((p) => (
              <button
                key={p.label}
                onClick={() => setDestination(p.lat, p.lng, p.label)}
                className="block w-full text-left text-xs text-white/90 hover:text-[var(--brand)] px-2 py-1.5 rounded hover:bg-white/5 transition-colors"
              >
                {p.label}
              </button>
            ))}
          </div>

          <div className="h-[1px] bg-white/10" />

          <form onSubmit={handleManualSubmit} className="space-y-2">
            <span className="text-[10px] font-bold uppercase tracking-wider text-[var(--text-muted)]">Custom coordinates</span>
            <div className="flex gap-2">
              <input
                type="number" step="any" placeholder="Lat" value={lat}
                onChange={(e) => setLat(e.target.value)}
                className="w-1/2 bg-white/5 border border-white/10 rounded px-2 py-1.5 text-xs text-white outline-none focus:border-[var(--brand)]"
              />
              <input
                type="number" step="any" placeholder="Lng" value={lng}
                onChange={(e) => setLng(e.target.value)}
                className="w-1/2 bg-white/5 border border-white/10 rounded px-2 py-1.5 text-xs text-white outline-none focus:border-[var(--brand)]"
              />
            </div>
            <button
              type="submit"
              className="w-full bg-[var(--brand)] text-black font-bold text-xs py-2 rounded hover:opacity-90 transition-opacity"
            >
              Go
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
