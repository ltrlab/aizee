import assert from 'node:assert/strict';
import {headSetupView} from '../../src/robot_platform/helm/static/head_setup.js';

// Helm on a PC: the head is set up where its servo board is.
assert.deepEqual(headSetupView(null), {available: false,
  note: 'The head is set up in Helm on Minerva, beside its servo board.'});

const servo = (name, label, steps, extra = {}) => ({name, label, steps, voltage_v: 12.1, temperature_c: 27,
  alarms: [], torque: false, ...extra});
const base = {available: true, reading: false, fresh: false, error: null, message: null, rows: [], pending: 0,
  reference: null, limits: null, servos: [servo('head_pan', 'Head pan', null), servo('head_tilt', 'Head tilt', null)]};

// Not reading yet: nothing can be set; the reference step says how to begin.
const idle = headSetupView(base);
assert.equal(idle.readLabel, 'Read the head');
assert.equal(idle.readingText, 'Not reading. The servos stay limp either way.');
assert.equal(idle.referenceEnabled, false);
assert.equal(idle.referenceLabel, 'Set the reference…');
assert.match(idle.referenceText, /^No reference yet: read the head, hold it straight ahead and level/);
assert.equal(idle.saveEnabled, false);
assert.equal(idle.limitsText, 'No head limits saved yet.');

// Reading, fresh: the servos' readings and health; the reference can be set (its prompt says what it writes).
const reading = headSetupView({...base, reading: true, fresh: true,
  servos: [servo('head_pan', 'Head pan', 2224), servo('head_tilt', 'Head tilt', 1307)]});
assert.equal(reading.readLabel, 'Stop reading');
assert.equal(reading.readingText, 'Head pan 2224 · 12.1 V · 27 °C   Head tilt 1307 · 12.1 V · 27 °C');
assert.equal(reading.readingTone, 'ok');
assert.equal(reading.referenceEnabled, true);
assert.match(reading.referenceConfirm, /straight ahead and level.*zero is written to their memory\. Set/);
assert.equal(headSetupView({...base, reading: true, fresh: false}).readingText, 'Waiting for the servos…');
const error = headSetupView({...base, reading: true, error: 'Reading the head: no reply from servo 2'});
assert.deepEqual([error.readingText, error.readingTone], ['Reading the head: no reply from servo 2', 'bad']);
// A servo holding torque or in alarm: no reference from it.
const alarmed = headSetupView({...base, reading: true, fresh: true,
  servos: [servo('head_pan', 'Head pan', 2048, {alarms: ['overload']}), servo('head_tilt', 'Head tilt', 2048)]});
assert.deepEqual([alarmed.readingTone, alarmed.referenceEnabled], ['warn', false]);
assert.match(alarmed.readingText, /Head pan 2048 · 12\.1 V · 27 °C · overload/);

// With a reference and both joints confirmed: save; setting the reference again warns the limits go stale.
const rows = [{name: 'head_pan'}, {name: 'head_tilt'}];
const swept = headSetupView({...reading, ...base, reading: true, fresh: true, rows, pending: 2,
  reference: {id: 'r1', created_at: '2026-10-29T12:00:00Z', directions_verified: false},
  limits: {created_at: '2026-10-29T12:05:00Z', matches_reference: true}, message: 'Head limits saved.'});
assert.equal(swept.referenceLabel, 'Set the reference again…');
assert.match(swept.referenceText, /directions not checked yet: turn each joint and watch the 3D view$/);
assert.match(swept.referenceConfirm, /saved head limits will need sweeping again/);
assert.deepEqual([swept.saveEnabled, swept.saveLabel], [true, 'Save head limits (2/2)']);
assert.match(swept.limitsText, /^Head limits saved /);
assert.equal(swept.status, 'Head limits saved.');
const half = headSetupView({...base, reading: true, fresh: true, rows, pending: 1});
assert.deepEqual([half.saveEnabled, half.saveLabel], [false, 'Save head limits (1/2)']);
const stale = headSetupView({...base, reading: true, fresh: true, rows,
  reference: {id: 'r2', created_at: '2026-10-29T12:10:00Z', directions_verified: true},
  limits: {created_at: '2026-10-29T12:05:00Z', matches_reference: false}});
assert.match(stale.limitsText, /older reference: sweep both joints again and save/);
assert.match(stale.referenceText, /directions checked$/);

console.log('Head setup: steps follow the reader, the reference and the sweep; stale limits are named.');
