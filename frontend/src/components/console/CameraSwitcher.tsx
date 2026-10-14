'use client';

import React, { useEffect } from 'react';
import { Camera, Car, Grid2x2, Orbit, Clapperboard } from 'lucide-react';
import { useCamera, CameraMode } from '../../store/useCamera';

interface CameraOption {
  id: CameraMode;
  label: string;
  key: string;
  icon: React.ComponentType<{ className?: string; 'aria-hidden'?: boolean | 'true' | 'false' }>;
}

const OPTIONS: CameraOption[] = [
  { id: 'chase', label: 'Chase', key: '1', icon: Camera },
  { id: 'cockpit', label: 'Cockpit', key: '2', icon: Car },
  { id: 'top', label: 'Top-down', key: '3', icon: Grid2x2 },
  { id: 'orbit', label: 'Orbit', key: '4', icon: Orbit },
  { id: 'cinematic', label: 'Cinematic', key: '5', icon: Clapperboard },
];

export function CameraSwitcher() {
  const mode = useCamera((s) => s.mode);
  const setMode = useCamera((s) => s.setMode);

  useEffect(() => {
    useCamera.getState().restoreSavedMode();
    const onKeyDown = (e: KeyboardEvent) => {
      const target = e.target as HTMLElement | null;
      if (
        target &&
        (target.tagName === 'INPUT' ||
          target.tagName === 'TEXTAREA' ||
          target.tagName === 'SELECT' ||
          target.isContentEditable)
      ) {
        return;
      }

      switch (e.key) {
        case '1':
          setMode('chase');
          break;
        case '2':
          setMode('cockpit');
          break;
        case '3':
          setMode('top');
          break;
        case '4':
          setMode('orbit');
          break;
        case '5':
          setMode('cinematic');
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [setMode]);

  return (
    <nav
      aria-label="Camera perspective switcher"
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: 2,
        padding: 3,
        background: 'var(--bg-inset)',
        border: '1px solid var(--border-subtle)',
        borderRadius: 'var(--radius-md)',
        pointerEvents: 'auto',
      }}
    >
      {OPTIONS.map((opt) => {
        const Icon = opt.icon;
        const isActive = mode === opt.id;
        return (
          <button
            key={opt.id}
            type="button"
            onClick={() => setMode(opt.id)}
            title={`${opt.label} (${opt.key})`}
            aria-pressed={isActive}
            aria-label={`${opt.label} camera view (press ${opt.key})`}
            style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: 28,
              height: 28,
              borderRadius: 'var(--radius-xs)',
              border: isActive ? '1px solid var(--brand)' : '1px solid transparent',
              background: isActive ? 'var(--brand-muted)' : 'transparent',
              color: isActive ? 'var(--brand)' : 'var(--text-muted)',
              cursor: 'pointer',
              transition: 'all var(--dur-fast) var(--ease-out)',
              boxShadow: isActive ? '0 0 8px rgba(0,242,254,0.3)' : 'none',
            }}
          >
            <Icon className="h-3.5 w-3.5" aria-hidden="true" />
            <span className="sr-only">{opt.label} view</span>
          </button>
        );
      })}
    </nav>
  );
}
