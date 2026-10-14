// Keep the security override compatible with the APIs Next's ESLint plugin uses.
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import path from 'node:path';
import { ESLint } from 'eslint';

const require = createRequire(import.meta.url);
const pluginRoot = path.dirname(require.resolve('@next/eslint-plugin-next/package.json'));
const { getRootDirs } = require(path.join(pluginRoot, 'dist/utils/get-root-dirs.js'));
const roots = getRootDirs({
  cwd: process.cwd(),
  settings: { next: { rootDir: ['src/{app,components}', 'src/lib'] } },
}).map(p => p.replaceAll('\\', '/'));
assert.deepEqual(roots.sort(), ['src/app', 'src/components', 'src/lib']);
const eslint = new ESLint();
const [result] = await eslint.lintText(
  'export default function Probe() { return <img src="/photo.png" alt="Probe" />; }',
  { filePath: 'src/app/lint-probe.tsx' },
);
assert.ok(result.messages.some(m => m.ruleId === '@next/next/no-img-element'),
  'Next.js lint rules must remain active after replacing its glob dependency');
console.log('Next.js root-directory globs and lint rules verified.');
