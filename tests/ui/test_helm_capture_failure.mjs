import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../src/robot_platform/helm/static/helm.js', import.meta.url), 'utf8');
const section = (from, to) => source.slice(source.indexOf(from), source.indexOf(to, source.indexOf(from)));
const elements = new Map();
const $ = id => {
  if (!elements.has(id)) elements.set(id, {hidden: false, disabled: false, textContent: '',
    classList: {toggle() {}}, setAttribute() {}, append() {}, replaceChildren() {}});
  return elements.get(id);
};
const lifecycle = ['I', 'E', 'T', 'H', 'X', undefined].map(key => ({dataset: {key}, disabled: false}));
const context = {$, ENGAGED: ['ENGAGING', 'TELEOP'], CONTROL_LAWS: {}, teleopPhase: 'running',
  last: {mode: 'simulation'}, pendingConfirm: null, tcards: new Map(), renderManual() {},
  document: {body: {dataset: {}, classList: {toggle() {}}},
    querySelector: () => $('teleop-tab'), querySelectorAll: selector => selector === '.lifecycle button' ? lifecycle : []}};
vm.createContext(context);
vm.runInContext(section('function renderTeleop(t)', 'function onKey(event)')
  + section('function renderTask(state)', 'for (const [id, action]')
  + section('function renderGlobalControls(state)', "$('global-hold').onclick"), context);
const running = {phase: 'running', state: 'HOLD', recording_fault: 'camera worker exited', fault: null, joints: [],
  task_configured: true, task: {available: true, capture_ready: true, recording: {state: 'idle'}}};
const render = t => {
  context.renderTeleop(t);
  context.renderTask({mode: 'simulation', teleop: t});
  context.renderGlobalControls({teleop: t});
};
for (const state of ['HOLD', 'ENGAGING', 'TELEOP', 'DISABLED']) {
  render({...running, state});
  for (const button of lifecycle) assert.equal(button.disabled, !['H', 'X'].includes(button.dataset.key));
  assert.equal($('teleop-session').disabled, false);
  assert.equal($('teleop-session').textContent, 'Stop session');
  assert.equal($('global-hold').disabled, true);
  assert.equal($('global-shutdown').disabled, false);
  assert.equal($('global-disable').disabled, false);
  assert.equal(Boolean($('task-record').disabled), true);
  assert.equal($('task-reset').disabled, true);
  assert.equal($('task-error').textContent, 'camera worker exited');
}
render({...running, task: {...running.task, recording: {active: true, state: 'recording'}}});
assert.equal($('task-stop').disabled, true); // Cannot finish/publish after the abort latch.
render({...running, recording_fault: null});
assert.equal(Boolean($('task-record').disabled), false);
assert.equal($('global-hold').disabled, false);
assert.ok(lifecycle.every(button => !button.disabled));
render({...running, fault: 'receiver watchdog expired'});
assert.equal($('global-shutdown').disabled, true); // Real receiver fault retains its existing recovery path.
assert.equal($('global-disable').disabled, false);
assert.equal(lifecycle.find(button => button.dataset.key === 'H').disabled, false);
assert.equal(lifecycle.find(button => button.dataset.key === 'X').disabled, true);
console.log('Helm capture-fault lifecycle, task publication, and emergency controls passed.');
