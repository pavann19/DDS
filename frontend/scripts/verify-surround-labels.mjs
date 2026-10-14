import assert from 'node:assert/strict';
import fs from 'node:fs';
import { sameSurroundLabel, surroundLabelText } from '../src/lib/surroundLabel.ts';

const base = { role: 'lead', relevance: 'primary', color: '#fff',
  ...surroundLabelText(true, 24.1, 10.01, 1.01) };
assert.deepEqual(base, {role:'lead',relevance:'primary',color:'#fff',
  rangeText:'24',speedText:'36',closingText:'▼ 1.0 m/s'});
// Moving poses and sub-display precision must not schedule a label update.
assert.equal(sameSurroundLabel(base, {...base,
  ...surroundLabelText(true,24.2,10.02,1.02)}), true);
for (const patch of [{rangeText:'25'}, {speedText:'37'}, {closingText:'▲ 1.0 m/s'},
  {closingText:null}, {role:'cut-in'}, {relevance:'secondary'}, {color:'#f00'}]) {
  assert.equal(sameSurroundLabel(base,{...base,...patch}),false);
}
// Secondary cards never show speeds: hidden precision cannot churn the DOM.
const secondary = {role:'adjacent',relevance:'secondary',color:'#fff',
  ...surroundLabelText(false,24.1,10,1)};
assert.equal(sameSurroundLabel(secondary,{...secondary,
  ...surroundLabelText(false,24.2,12,-3)}),true);
assert.equal(surroundLabelText(true,10,2,NaN).closingText,null);
assert.equal(surroundLabelText(true,10,2,Infinity).closingText,null);
assert.equal(surroundLabelText(true,10,2,-1.04).closingText,'▲ 1.0 m/s');
const component = fs.readFileSync(new URL('../src/components/3d/SurroundTrackLabel.tsx',import.meta.url),'utf8');
assert.match(component,/memo\(function SurroundTrackLabel/);
assert.match(component,/}, sameSurroundLabel\)/);
assert.match(component,/backdropFilter: 'blur\(12px\) saturate\(160%\)'/);
console.log('Surround label display changes, role/color invalidation, and invisible-value bailout passed.');
