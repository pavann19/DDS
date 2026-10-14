import { create } from 'zustand';
import type { Density } from '../components/primitives/tokens';

/** Console density — replaces the old drive / developer / research mode
 *  enum (ADR-002 rule 2). The 3D stage is always mounted; density only
 *  changes how much the one console tells you.
 *    focus    — stage + HUD only (layout-equivalent to the old Drive mode)
 *    standard — HUD + right rail with each panel's headline
 *    inspect  — every panel expanded, raw numbers, full `heavy` channel
 */
const ORDER: Density[] = ['focus', 'standard', 'inspect'];

interface ConsoleState {
  density: Density;
  isEvalOpen: boolean;
  setDensity: (d: Density) => void;
  cycleDensity: () => void;
  setEvalOpen: (open: boolean) => void;
  toggleEval: () => void;
}

export const useConsole = create<ConsoleState>((set) => ({
  density: 'standard',
  isEvalOpen: false,
  setDensity: (density) => set({ density }),
  cycleDensity: () =>
    set((s) => ({ density: ORDER[(ORDER.indexOf(s.density) + 1) % ORDER.length] })),
  setEvalOpen: (isEvalOpen) => set({ isEvalOpen }),
  toggleEval: () => set((s) => ({ isEvalOpen: !s.isEvalOpen })),
}));

export const DENSITY_ORDER = ORDER;
