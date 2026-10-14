import { create } from 'zustand';

const STORAGE_KEY = 'dds.camera';

export type CameraMode = 'chase' | 'cockpit' | 'top' | 'orbit' | 'cinematic';

const VALID_MODES: CameraMode[] = ['chase', 'cockpit', 'top', 'orbit', 'cinematic'];

function loadSavedMode(): CameraMode {
  if (typeof window === 'undefined') return 'chase';
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw && VALID_MODES.includes(raw as CameraMode)) {
      return raw as CameraMode;
    }
  } catch {
    // localStorage fallback
  }
  return 'chase';
}

function saveMode(mode: CameraMode) {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(STORAGE_KEY, mode);
  } catch {
    // localStorage ignore
  }
}

interface CameraStore {
  mode: CameraMode;
  restoreSavedMode: () => void;
  setMode: (mode: CameraMode) => void;
}

export const useCamera = create<CameraStore>((set) => ({
  mode: 'chase',
  restoreSavedMode: () => set({mode: loadSavedMode()}),
  setMode: (mode: CameraMode) => {
    set({ mode });
    saveMode(mode);
  },
}));
