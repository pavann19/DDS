#!/usr/bin/env node
/**
 * Phase 7.5 item 10 — UI contract gate.
 *
 * No test runner is installed for the frontend, so this is a standalone
 * static check with two jobs:
 *
 *  1. Panel contract: every <PanelSection> in the rail must declare a
 *     `channel=` prop bound to a real protocol v3 channel
 *     (pose. / semantic. / heavy.). A panel with no channel, or a channel
 *     that is not a real one, is a labelling bug — the whole point of
 *     ADR-002 rule 1 is that the header names the field it reads.
 *
 *  2. Capability-claim denylist: the live component tree must not carry
 *     stale framing that overclaims what the system does — "AI Decision"
 *     as a control signal, XGBoost "driving" / "deciding speed", raw
 *     "self-driving", etc. (ADR-001: the learned model is analytics, not
 *     policy; ADR-002 item 9.)
 *
 * Exit non-zero on any violation so it can gate CI later.
 */
import { readFileSync, readdirSync } from 'node:fs';
import { join, extname } from 'node:path';
import { fileURLToPath } from 'node:url';

const SRC = join(fileURLToPath(new URL('.', import.meta.url)), '..', 'src');

/** Files that make up the live console (everything page.tsx pulls in). */
const LIVE_DIRS = ['app', 'components/console', 'components/hud', 'components/3d', 'components/primitives'];

// pose./semantic./heavy. are the protocol v3 channels; event. is the
// separate top-level {type:"event"} scenario stream (also real backend data).
const VALID_CHANNEL = /^(pose|semantic|heavy|event)\./;

const DENYLIST = [
  { re: /\bAI[\s-]?decision\b/i, why: 'model output is analytics, not a control/decision label (ADR-001)' },
  { re: /XGBoost[^.\n]{0,40}(driv|decid|control|steer|speed)/i, why: 'implies the classifier drives the car' },
  { re: /\bself[\s-]?driving\b/i, why: 'use "autonomous-driving simulation" — no capability claim' },
  { re: /\bfully autonomous\b/i, why: 'unqualified autonomy claim' },
  { re: /\bconfidence\b[^.\n]{0,30}\b(throttle|brake|speed|steer)/i, why: 'classifier confidence is not a control input' },
];

function walk(dir) {
  const out = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const p = join(dir, entry.name);
    if (entry.isDirectory()) out.push(...walk(p));
    else if (['.ts', '.tsx'].includes(extname(entry.name))) out.push(p);
  }
  return out;
}

const liveFiles = LIVE_DIRS.flatMap((d) => {
  try {
    return walk(join(SRC, d));
  } catch {
    return [];
  }
});

let errors = 0;

// --- 1. panel contract -----------------------------------------------------
const railSrc = readFileSync(join(SRC, 'components/console/RailPanels.tsx'), 'utf8');
const sections = [...railSrc.matchAll(/<PanelSection\b[^>]*?(?:\/>|>)/gs)];
if (sections.length === 0) {
  console.error('✗ panel contract: no <PanelSection> found in RailPanels.tsx');
  errors++;
}
for (const m of sections) {
  const tag = m[0].replace(/\s+/g, ' ');
  const title = /title=(?:"([^"]*)"|{`([^`]*)`})/.exec(tag);
  const name = title?.[1] ?? title?.[2] ?? '(untitled)';
  const ch = /channel="([^"]+)"/.exec(tag);
  if (!ch) {
    console.error(`✗ panel "${name}" has no channel= prop`);
    errors++;
  } else if (!VALID_CHANNEL.test(ch[1])) {
    console.error(`✗ panel "${name}" channel "${ch[1]}" is not a protocol v3 channel`);
    errors++;
  }
}

// --- 2. capability-claim denylist ---------------------------------------
for (const file of liveFiles) {
  const text = readFileSync(file, 'utf8');
  text.split('\n').forEach((line, i) => {
    for (const { re, why } of DENYLIST) {
      if (re.test(line)) {
        console.error(`✗ ${file.replace(SRC, 'src')}:${i + 1}  overclaim — ${why}\n    ${line.trim()}`);
        errors++;
      }
    }
  });
}

if (errors) {
  console.error(`\nUI contract: ${errors} violation(s).`);
  process.exit(1);
}
console.log(`UI contract OK — ${sections.length} panels, all channel-bound; ${liveFiles.length} live files clean.`);
