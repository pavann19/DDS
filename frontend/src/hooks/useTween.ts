import { useEffect, useRef, useState } from 'react';
import { useReducedMotion } from './useReducedMotion';

/** Smoothly eases a displayed number toward `target` (Tesla-style: values
 *  glide, they never snap). Under prefers-reduced-motion it returns
 *  `target` unchanged. `duration` is the approximate settle time in ms. */
export function useTween(target: number, duration = 300): number {
  const reduced = useReducedMotion();
  const [animated, setAnimated] = useState(target);
  const fromRef = useRef(target);
  const rafRef = useRef<number | null>(null);

  useEffect(() => {
    if (reduced || !Number.isFinite(target)) return;

    const from = Number.isFinite(fromRef.current) ? fromRef.current : target;
    const start = performance.now();

    const tick = (now: number) => {
      // RAF timestamps can precede an effect's performance.now() within a
      // busy frame. Negative progress extrapolates away from the target and
      // can poison subsequent animations. Bound both ends of progress.
      const t = duration > 0 && Number.isFinite(duration)
        ? Math.max(0, Math.min(1, (now - start) / duration))
        : 1;
      const eased = 1 - Math.pow(1 - t, 3); // ease-out cubic
      const next = from + (target - from) * eased;
      setAnimated(next);
      fromRef.current = next;
      if (t < 1) rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);

    return () => {
      if (rafRef.current != null) cancelAnimationFrame(rafRef.current);
    };
  }, [target, duration, reduced]);

  if (reduced || !Number.isFinite(target)) return target;
  return Number.isFinite(animated) ? animated : target;
}
