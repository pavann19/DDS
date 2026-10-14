import { create } from 'zustand';

const STORAGE_KEY = 'dds.rail';

interface SavedRailState {
  pinned: boolean;
  activeCard: string | null;
}

function loadSavedState(): SavedRailState {
  if (typeof window === 'undefined') {
    return { pinned: false, activeCard: null };
  }
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as Partial<SavedRailState>;
      return {
        pinned: Boolean(parsed.pinned),
        activeCard: typeof parsed.activeCard === 'string' ? parsed.activeCard : null,
      };
    }
  } catch {
    // localStorage read failure fallback
  }
  return { pinned: false, activeCard: null };
}

function saveState(pinned: boolean, activeCard: string | null) {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({ pinned, activeCard }));
  } catch {
    // localStorage write failure ignore
  }
}

export interface RailStore {
  expanded: boolean;
  activeCard: string | null;
  pinned: boolean;
  toggle: (id: string) => void;
  collapse: () => void;
  setPinned: (pinned: boolean) => void;
}

const initial = loadSavedState();

export const useRail = create<RailStore>((set, get) => ({
  expanded: initial.pinned && Boolean(initial.activeCard),
  activeCard: initial.activeCard,
  pinned: initial.pinned,

  toggle: (id: string) => {
    const { expanded, activeCard, pinned } = get();
    if (expanded && activeCard === id) {
      set({ expanded: false, activeCard: null });
      saveState(pinned, null);
    } else {
      set({ expanded: true, activeCard: id });
      saveState(pinned, id);
    }
  },

  collapse: () => {
    const { pinned } = get();
    set({ expanded: false, activeCard: null });
    saveState(pinned, null);
  },

  setPinned: (pinned: boolean) => {
    const { activeCard } = get();
    set({ pinned });
    saveState(pinned, activeCard);
  },
}));
