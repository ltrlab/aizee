import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../src/robot_platform/helm/static/helm.js', import.meta.url), 'utf8');
// Exercise the actual page lifecycle functions without importing its device/action UI.
const start = source.indexOf('const modelViewerSource =');
const end = source.indexOf('function renderTask(state)', start);
assert.ok(start >= 0 && end > start);
const navigations = [];
const frame = {hidden: false, getAttribute: () => 'viewer?embed=1',
  set src(value) { navigations.push(value); }};
const elements = {'model-view': frame, 'task-window-note': {hidden: true}, 'model-heading': {textContent: ''}};
const context = {$: id => elements[id]};
vm.createContext(context);
vm.runInContext(source.slice(start, end) + ';globalThis.api = {taskOwnsViewer, renderTaskViewer};', context);
const {taskOwnsViewer, renderTaskViewer} = context.api;
const state = (mode = 'simulation', configured = true, phase = 'running') =>
  ({mode, teleop: {task_configured: configured, phase}});
for (const phase of ['starting', 'running']) assert.equal(taskOwnsViewer(state('simulation', true, phase)), true);
for (const mode of ['robot', 'unknown']) assert.equal(taskOwnsViewer(state(mode)), false);
for (const phase of ['stopped', 'stopping', 'fault', 'unknown']) assert.equal(taskOwnsViewer(state('simulation', true, phase)), false);
assert.equal(taskOwnsViewer(state('simulation', false)), false);
assert.equal(taskOwnsViewer({mode: 'simulation'}), false);
assert.equal(taskOwnsViewer(null), false);

renderTaskViewer(state('robot'));
assert.deepEqual(navigations, []); // Physical/calibration viewer never reloads.
renderTaskViewer(state('simulation', true, 'starting'));
assert.deepEqual(navigations, ['about:blank']);
assert.equal(frame.hidden, true);
assert.equal(elements['task-window-note'].hidden, false);
assert.equal(elements['model-heading'].textContent, 'LIVE TELEMETRY');
renderTaskViewer(state()); renderTaskViewer(state());
assert.deepEqual(navigations, ['about:blank']); // Polling cannot repeatedly navigate.
renderTaskViewer(state('simulation', true, 'stopped'));
assert.deepEqual(navigations, ['about:blank', 'viewer?embed=1']);
assert.equal(frame.hidden, false);
assert.equal(elements['task-window-note'].hidden, true);
assert.equal(elements['model-heading'].textContent, 'MODEL & TELEMETRY');
renderTaskViewer(state('simulation', false)); renderTaskViewer(state('robot'));
assert.equal(navigations.length, 2);
renderTaskViewer(state()); renderTaskViewer(state('robot'));
assert.deepEqual(navigations.slice(-2), ['about:blank', 'viewer?embed=1']);
console.log('Helm task viewer predicate and unload/restore transitions passed.');
