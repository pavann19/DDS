import { create } from 'zustand';

type AppMode = 'drive' | 'developer' | 'research';

interface UISettings {
  activeMode: AppMode;
  setMode: (mode: AppMode) => void;
}

export const useUISettings = create<UISettings>((set) => ({
  activeMode: 'drive', // Default
  setMode: (mode) => set({ activeMode: mode }),
}));
