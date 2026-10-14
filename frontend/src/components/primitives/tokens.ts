/** Shared prop unions + token lookups for the console primitives.
 *  Every colour here resolves to a globals.css custom property — no
 *  component ever names a hex literal (ADR-002 rule 4). */

export type Tone = 'default' | 'brand' | 'ok' | 'warn' | 'crit';

/** Foreground / accent colour for a tone. */
export const toneColor: Record<Tone, string> = {
  default: 'var(--text-primary)',
  brand: 'var(--brand)',
  ok: 'var(--success)',
  warn: 'var(--warning)',
  crit: 'var(--critical)',
};

/** Faint fill for a tone (stripes, pulses, meter tracks). */
export const toneMuted: Record<Tone, string> = {
  default: 'var(--border-default)',
  brand: 'var(--brand-muted)',
  ok: 'var(--success-muted)',
  warn: 'var(--warning-muted)',
  crit: 'var(--critical-muted)',
};

export type Density = 'focus' | 'standard' | 'inspect';
