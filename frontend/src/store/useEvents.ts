import { create } from 'zustand';
import type { SimulationEvent } from '../types/protocol';

/**
 * Operational event log (Phase 7.5+ §17).
 *
 * The ONLY source is the backend's real `{ type: "event" }` messages —
 * `app/services/scenario_engine.py::_create_event`, emitted at scenario
 * milestone transitions (LANE_CHANGE_INITIATED, VEHICLE_CUT_IN,
 * SAFETY_SHIELD_OVERRIDE, QUEUE_STANDSTILL, …). Free-drive emits none, so
 * an empty log is the correct, honest state — never synthesised.
 */
const CAP = 40;

interface EventsState {
  events: SimulationEvent[]; // newest last
  pushEvent: (e: SimulationEvent) => void;
  clear: () => void;
}

export const useEvents = create<EventsState>((set) => ({
  events: [],
  pushEvent: (e) =>
    set((s) => {
      // A new scenario resets the operational log to that first marker.
      if (e.type === 'SCENARIO_LOADED') return { events: [e] };
      // De-dupe on event_id (a reconnect can replay the last milestone).
      if (s.events.some((x) => x.event_id === e.event_id)) return s;
      const next = [...s.events, e];
      return { events: next.length > CAP ? next.slice(next.length - CAP) : next };
    }),
  clear: () => set({ events: [] }),
}));
