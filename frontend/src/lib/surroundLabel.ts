import type { TrackRole } from './roles';

export type SurroundLabelProps = {
  role: TrackRole;
  relevance: 'primary' | 'secondary';
  color: string;
  rangeText: string;
  speedText: string;
  closingText: string | null;
};

export function surroundLabelText(primary: boolean, range: number, speed: number, closing: number) {
  return {
    rangeText: range.toFixed(0),
    speedText: primary ? (speed * 3.6).toFixed(0) : '',
    closingText: primary && Number.isFinite(closing)
      ? `${closing > 0 ? '▼' : '▲'} ${Math.abs(closing).toFixed(1)} m/s`
      : null,
  };
}

export function sameSurroundLabel(a: SurroundLabelProps, b: SurroundLabelProps) {
  return a.role === b.role && a.relevance === b.relevance && a.color === b.color
    && a.rangeText === b.rangeText && a.speedText === b.speedText
    && a.closingText === b.closingText;
}
