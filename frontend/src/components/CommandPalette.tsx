'use client';
import { useEffect, useState } from 'react';
import { Command } from 'cmdk';
import { useUISettings } from '../store/useUISettings';
import { Search, Monitor, TerminalSquare, FlaskConical } from 'lucide-react';

export function CommandPalette() {
  const [open, setOpen] = useState(false);
  const { setMode } = useUISettings();

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if (e.key === 'k' && (e.metaKey || e.ctrlKey)) {
        e.preventDefault();
        setOpen((open) => !open);
      }
    };
    document.addEventListener('keydown', down);
    return () => document.removeEventListener('keydown', down);
  }, []);

  if (!open) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-start justify-center pt-[20vh] bg-black/60 backdrop-blur-sm" onClick={() => setOpen(false)}>
      <Command 
        className="w-full max-w-lg bg-[var(--bg-panel)] rounded-xl border border-[var(--border-default)] shadow-2xl overflow-hidden text-[var(--text-primary)]"
        onClick={(e) => e.stopPropagation()}
        loop
      >
        <div className="flex items-center border-b border-[var(--border-default)] px-3 py-2">
          <Search className="w-5 h-5 text-[var(--text-muted)] mr-2" />
          <Command.Input 
            autoFocus 
            placeholder="Type a command or search... (e.g. 'Drive')" 
            className="w-full bg-transparent outline-none text-[var(--text-bright)] py-2 placeholder:text-[var(--text-muted)]"
          />
        </div>

        <Command.List className="max-h-[300px] overflow-y-auto p-2">
          <Command.Empty className="p-4 text-center text-sm text-[var(--text-muted)]">No results found.</Command.Empty>

          <Command.Group heading={<div className="px-2 py-1 text-xs font-semibold text-[var(--text-muted)] uppercase tracking-wider">Modes</div>}>
            <Command.Item 
              onSelect={() => { setMode('drive'); setOpen(false); }}
              className="flex items-center px-2 py-3 rounded cursor-pointer aria-selected:bg-[var(--bg-surface)] aria-selected:text-[var(--brand)] transition-colors data-[selected=true]:bg-[var(--bg-surface)] data-[selected=true]:text-[var(--brand)]"
            >
              <Monitor className="w-4 h-4 mr-3" />
              Switch to Drive Mode
            </Command.Item>
            <Command.Item 
              onSelect={() => { setMode('developer'); setOpen(false); }}
              className="flex items-center px-2 py-3 rounded cursor-pointer aria-selected:bg-[var(--bg-surface)] aria-selected:text-[var(--brand)] transition-colors data-[selected=true]:bg-[var(--bg-surface)] data-[selected=true]:text-[var(--brand)]"
            >
              <TerminalSquare className="w-4 h-4 mr-3" />
              Switch to Developer Mode
            </Command.Item>
            <Command.Item 
              onSelect={() => { setMode('research'); setOpen(false); }}
              className="flex items-center px-2 py-3 rounded cursor-pointer aria-selected:bg-[var(--bg-surface)] aria-selected:text-[var(--brand)] transition-colors data-[selected=true]:bg-[var(--bg-surface)] data-[selected=true]:text-[var(--brand)]"
            >
              <FlaskConical className="w-4 h-4 mr-3" />
              Switch to Research Lab
            </Command.Item>
          </Command.Group>
        </Command.List>
      </Command>
    </div>
  );
}
