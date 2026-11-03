import assert from 'node:assert/strict';

// The headset's ring of controls (xr_ring.js), worked with a controller's A or X and thumbstick, and what it offers
// from Helm's state, the alerts with it closed, and the requests that start, step and stop a session (xr_session.js).
const root = new URL('../../', import.meta.url);
const {DEAD_ZONE, HOLD_S, Ring, SLIDE_PER_S, aimed} = await import(new URL('src/robot_platform/helm/static/xr_ring.js', root).href);
const {SessionControl, alertOf, receiverWord, ringMenu, statusLines} = await import(
  new URL('src/robot_platform/helm/static/xr_session.js', root).href);

// Aimed clockwise from the top; nothing inside the dead zone.
assert.equal(aimed([0, -1], 4), 0);
assert.equal(aimed([1, 0], 4), 1);
assert.equal(aimed([0, 1], 4), 2);
assert.equal(aimed([-1, 0], 4), 3);
assert.equal(aimed([-.1, -1], 4), 0);  // Just left of the top: still the top's.
assert.equal(aimed([DEAD_ZONE * .9, 0], 4), null);
assert.equal(aimed([1, 0], 0), null);

// A ring over a small menu: a submenu, an item that asks for the hold, one out of reach, a slider, a switch that
// keeps the ring open, and a plain item.
let gone = false;
const menu = path => {
  if (!path.length) {
    return {title: 'Top', items: [{id: 'sub', label: 'Sub', submenu: true}, {id: 'move', label: 'Move',
      confirm: ['It moves.']}, {id: 'locked', label: 'Locked', disabled: 'Not now'}, {id: 'plain', label: 'Plain'}]};
  }
  if (path[0] === 'sub' && !gone) {
    return {title: 'Sub', items: [{id: 'slider:x', label: 'X', slider: .5}, {id: 'flag', label: 'Flag', stay: true}]};
  }
  return null;
};
const ring = new Ring(menu);
const right = (button, stick = [0, 0]) => ({hand: 'right', button, stick});
const left = (button, stick = [0, 0]) => ({hand: 'left', button, stick});
const UP = [0, -1], RIGHT = [1, 0], DOWN = [0, 1], LEFT = [-1, 0];
assert.equal(ring.drawing(0), null);

// A or X opens it on that controller; the other is left alone while it is open.
assert.deepEqual(ring.step([right(true)], 0), [{id: 'opened'}]);
assert.equal(ring.hand, 'right');
assert.deepEqual(ring.step([right(false)], .1), []);
assert.deepEqual(ring.step([left(true, LEFT)], .2), []);
assert.equal(ring.drawing(.2).aim, null);
// Chosen with the stick centred at the top: it closes.
ring.step([right(true), left(false)], .3);
assert.equal(ring.open, false);

// Opened with X, an item aimed and chosen: it is sent, and the ring closes.
ring.step([left(true)], 1);
ring.step([left(false, LEFT)], 1.1);
assert.equal(ring.drawing(1.1).aim, 3);
assert.deepEqual(ring.step([left(true, LEFT)], 1.2), [{id: 'plain'}]);
assert.equal(ring.open, false);

// Out of reach: why, and nothing sent.
ring.step([left(false)], 2);
ring.step([left(true)], 2.1);
ring.step([left(false, DOWN)], 2.2);
assert.deepEqual(ring.step([left(true, DOWN)], 2.3), []);
assert.equal(ring.drawing(2.3).notice, 'Not now');
ring.step([left(false, RIGHT)], 2.4);
assert.equal(ring.drawing(2.4).notice, null);  // Aimed elsewhere: the notice goes.

// An item that moves the robot: its confirmation in the middle, then held HOLD_S on it to choose.
assert.deepEqual(ring.drawing(2.4).confirm, ['It moves.']);
assert.deepEqual(ring.step([left(true, RIGHT)], 3), []);
assert.equal(ring.drawing(3 + HOLD_S / 2).progress, .5);
assert.equal(ring.drawing(3 + HOLD_S / 3).progress, .3);  // In twentieths.
// Let go too soon: nothing, and why.
assert.deepEqual(ring.step([left(false, RIGHT)], 3 + HOLD_S / 2), []);
assert.equal(ring.drawing(3.6).notice, 'Hold to confirm: let go too soon');
assert.equal(ring.drawing(3.6).progress, 0);
// Aimed away while held: cancelled.
ring.step([left(true, RIGHT)], 4);
ring.step([left(true, UP)], 4.2);
assert.deepEqual(ring.step([left(true, RIGHT)], 4 + HOLD_S + .1), []);
// Held on it the whole time: chosen once, and the ring closes.
ring.step([left(false, RIGHT)], 6);
ring.step([left(true, RIGHT)], 6.1);
assert.deepEqual(ring.step([left(true, RIGHT)], 6.1 + HOLD_S / 2), []);
assert.deepEqual(ring.step([left(true, RIGHT)], 6.1 + HOLD_S), [{id: 'move'}]);
assert.equal(ring.open, false);
assert.deepEqual(ring.step([left(true, RIGHT)], 7.5), []);  // Still held from the hold: not a new press.

// A submenu: in, a switch that keeps the ring open, a slider, and back out with the stick centred.
ring.step([right(false)], 7.9);
ring.step([right(true)], 8);
ring.step([right(false, UP)], 8.1);
ring.step([right(true, UP)], 8.2);
assert.equal(ring.drawing(8.2).title, 'Sub');
assert.equal(ring.drawing(8.2).depth, 1);
ring.step([right(false, DOWN)], 8.3);
assert.deepEqual(ring.step([right(true, DOWN)], 8.4), [{id: 'flag'}]);
assert.ok(ring.open);
ring.step([right(false, UP)], 8.5);
assert.deepEqual(ring.step([right(true, UP)], 8.6), []);  // The slider takes the stick.
assert.deepEqual(ring.drawing(8.6).adjust, {id: 'slider:x', value: .5});
const changes = [];
for (let t = 8.7; t < 9.7; t += .05) changes.push(...ring.step([right(false, RIGHT)], t, .05));
assert.ok(changes.length >= 8 && changes.every(change => change.id === 'slider:x' && !change.done));
assert.equal(changes.at(-1).value, Number(Math.min(1, .5 + SLIDE_PER_S * 1).toFixed(2)));
assert.deepEqual(ring.step([right(true, LEFT)], 9.8, .05), [{id: 'slider:x', value: changes.at(-1).value, done: true}]);
assert.equal(ring.drawing(9.8).adjust, null);
ring.step([right(false)], 9.9);
ring.step([right(true)], 10);  // Centred: back to the top.
assert.equal(ring.drawing(10).title, 'Top');
// A submenu that goes (the session changed): back to the top.
ring.step([right(false, UP)], 10.1);
ring.step([right(true, UP)], 10.2);
gone = true;
assert.equal(ring.drawing(10.3).title, 'Top');
ring.close();
assert.equal(ring.open, false);

// What the ring offers from Helm's state.
const ids = level => level.items.map(item => item.id);
const item = (level, id) => level.items.find(each => each.id === id);
const VIEW = {follow: true, depth: false, model: true, hands: true, map: false, opacity: 1, wristOpacity: .6};
const CONTROLLED = {enabled: true, in_control: true, device: 'Quest', holder: {device: 'Quest'}};
const OTHER = {enabled: true, in_control: false, device: 'Quest', holder: {device: 'Operator PC'}};
const stopped = {phase: 'stopped', state: null, receiver_choices: ['virtual', 'left', 'right', 'both', 'head'],
  live_available: true, collision: {mode: 'advisory'}};
assert.deepEqual(item(ringMenu({teleop: null}, [], VIEW), 'none').disabled, 'Helm runs without a teleop profile');
let top = ringMenu({teleop: stopped, access: CONTROLLED}, [], VIEW);
assert.deepEqual(ids(top), ['start', 'view', 'zero']);
let start = ringMenu({teleop: stopped, access: CONTROLLED}, ['start'], VIEW);
assert.deepEqual(ids(start), ['start:virtual', 'start:left', 'start:right', 'start:both', 'start:head']);
assert.equal(item(start, 'start:virtual').confirm, undefined);  // No motors, and control is this device's.
assert.match(item(start, 'start:both').confirm.join(' '), /hands stop about 30 cm apart/);
assert.match(item(start, 'start:head').confirm.join(' '), /arms are left as they are/);
// Live needs Helm on Minerva; the receivers are the profile's.
start = ringMenu({teleop: {...stopped, live_available: false, receiver_choices: ['virtual', 'head']},
  access: CONTROLLED}, ['start'], VIEW);
assert.deepEqual(ids(start), ['start:virtual', 'start:head']);
assert.equal(item(start, 'start:head').disabled, 'Live sessions need Helm on Minerva');
// Another device has control: starting takes it, and says so; Take control is offered, held.
top = ringMenu({teleop: stopped, access: OTHER}, [], VIEW);
assert.deepEqual(ids(top), ['start', 'view', 'zero', 'claim']);
assert.match(item(top, 'claim').confirm[0], /from Operator PC: its page goes view-only/);
start = ringMenu({teleop: stopped, access: OTHER}, ['start'], VIEW);
assert.match(item(start, 'start:virtual').confirm[0], /takes Helm's control from Operator PC/);
// The View level: the switches as they are, the sliders at their values.
const viewLevel = ringMenu({teleop: stopped, access: CONTROLLED}, ['view'], VIEW);
assert.deepEqual(ids(viewLevel), ['view:follow', 'view:depth', 'view:model', 'view:hands', 'view:map',
  'slider:opacity', 'slider:wristOpacity', 'recenter']);
assert.deepEqual([item(viewLevel, 'view:depth').detail, item(viewLevel, 'view:hands').label], ['off', 'Wrists']);
assert.ok(item(viewLevel, 'view:follow').stay);
assert.equal(item(viewLevel, 'slider:wristOpacity').slider, .6);
assert.equal(ringMenu({teleop: stopped, access: CONTROLLED}, ['nothing'], VIEW), null);

// A running live session: its next step, held where it powers or moves the robot; Shut down, End and Disable.
const live = (state, extra = {}) => ({teleop: {...stopped, phase: 'running', state, live: {arm: 'both'},
  receiver_choice: {mode: 'live', arm: 'both'}, ...extra}, access: CONTROLLED});
const steps = {DISABLED: 'Idle (I)', IDLE: 'Enable (E)', HOLD: 'Engage (T)', ENGAGING: 'Hold (T)', TELEOP: 'Hold (T)'};
for (const [state, label] of Object.entries(steps)) {
  const level = ringMenu(live(state), [], VIEW);
  assert.equal(level.items[0].label, label);
  assert.equal(!!level.items[0].confirm, !['ENGAGING', 'TELEOP'].includes(state), state);  // Holding needs no hold.
}
assert.equal(ringMenu(live('DISABLED'), ['start'], VIEW), null);  // Running: no Start.
top = ringMenu(live('DISABLED'), [], VIEW);
assert.deepEqual(ids(top), ['key:I', 'key:X', 'stop', 'key:H', 'view', 'zero']);
assert.equal(item(top, 'key:X').disabled, 'Nothing to shut down');
assert.equal(item(top, 'stop').confirm, undefined);  // Unpowered: ended at once.
assert.equal(item(top, 'key:H').confirm, undefined);  // No torque to remove.
top = ringMenu(live('TELEOP'), [], VIEW);
assert.equal(item(top, 'key:X').disabled, undefined);
assert.match(item(top, 'stop').confirm.join(' '), /LIVE both arms is powered \(TELEOP\).*lowered slowly/);
assert.match(item(top, 'key:H').confirm[0], /the arms drop where they are/);
assert.match(ringMenu(live('HOLD'), [], VIEW).items[0].confirm.join(' '), /ready pose/);
// The head alone: its own words.
assert.match(ringMenu(live('IDLE', {live: {arm: 'head'}}), [], VIEW).items[0].confirm[0], /The head holds/);
// Virtual: nothing held but Disable's absence of torque.
const virtual = {teleop: {...stopped, phase: 'running', state: 'HOLD', live: null, receiver_choice: {mode: 'virtual'}},
  access: CONTROLLED};
top = ringMenu(virtual, [], VIEW);
assert.equal(top.items[0].confirm, undefined);
assert.equal(item(top, 'stop').confirm, undefined);
assert.equal(item(top, 'key:H').confirm, undefined);
// View only: the steps and End wait for control; Shut down and Disable stay (helm.access lets them through).
top = ringMenu({...live('TELEOP'), access: OTHER}, [], VIEW);
assert.match(top.items[0].disabled, /View only: Operator PC has control/);
assert.match(item(top, 'stop').disabled, /Take control first/);
assert.equal(item(top, 'key:X').disabled, undefined);
assert.equal(item(top, 'key:H').disabled, undefined);
assert.ok(ids(top).includes('claim'));
// Faulted: no step, and why.
top = ringMenu(live('TELEOP', {fault: 'left J4 limit'}), [], VIEW);
assert.equal(top.items[0].label, 'Faulted');
assert.match(top.items[0].disabled, /left J4 limit/);
// Starting: nothing to choose until it runs.
assert.equal(ringMenu({teleop: {...stopped, phase: 'starting'}, access: CONTROLLED}, [], VIEW).items[0].id, 'starting');

// The ring's middle, and the receiver as a word.
assert.equal(receiverWord(live('TELEOP').teleop), 'LIVE both arms');
assert.equal(receiverWord({live: {arm: 'left'}}), 'LIVE left arm');
assert.equal(receiverWord(virtual.teleop), 'Virtual');
assert.deepEqual(statusLines(live('TELEOP'), {connected: true}, [['Head at its zero', 'warn'], null]),
  [['LIVE both arms · TELEOP', 'ok'], ['Head at its zero', 'warn']]);
assert.deepEqual(statusLines({...live('HOLD', {fault: 'x'}), access: OTHER}, {connected: false}), [
  ['Helm: reconnecting…', 'warn'], ['LIVE both arms · HOLD', 'bad'], ['Fault: x', 'bad'],
  ['View only: Operator PC has control', 'warn']]);
assert.deepEqual(statusLines({teleop: stopped, access: CONTROLLED}, {connected: true}), [['No session', '']]);

// The alerts with the ring closed: a new fault stays; control lost during a session stays; a fall into a hold passes.
const up = {connected: true};
assert.deepEqual(alertOf(live('TELEOP', {fault: 'J4'}), live('TELEOP'), up), {text: 'Fault: J4', tone: 'bad', sticky: true});
assert.equal(alertOf(live('TELEOP', {fault: 'J4'}), live('TELEOP', {fault: 'J4'}), up), null);  // Already said.
assert.deepEqual(alertOf(live('TELEOP', {head: {fault: 'servo 2'}}), live('TELEOP'), up),
  {text: 'Head fault: servo 2', tone: 'bad', sticky: true});
assert.deepEqual(alertOf({...live('TELEOP'), access: OTHER}, live('TELEOP'), up),
  {text: 'View only: Operator PC took control', tone: 'warn', sticky: true});
assert.deepEqual(alertOf(live('HOLD', {message: 'Left wrist camera dropped'}), live('TELEOP'), up),
  {text: 'Holding: Left wrist camera dropped', tone: 'warn', sticky: false});
assert.equal(alertOf(live('HOLD'), live('IDLE'), up), null);  // Enabled into a hold: no alert.
assert.equal(alertOf(live('TELEOP'), null, up), null);
assert.deepEqual(alertOf(live('TELEOP'), live('TELEOP'), {connected: false, message: 'reconnecting…'}),
  {text: 'reconnecting…', tone: 'warn', sticky: false});

// The requests: as the Teleop tab sends them, with Helm's action token; taking control first where another has it.
function helm(access, refuse = {}) {
  const sent = [];
  const state = {teleop: stopped, access, action_token: 'tok'};
  const fetch = async (path, options) => {
    const body = options.body === undefined ? undefined : JSON.parse(options.body);
    sent.push([path, body, options.headers?.['X-Helm-Token']]);
    if (refuse[path]) return {ok: false, status: 400, json: async () => ({error: refuse[path]})};
    if (path === '/api/helm/control/claim') state.access = CONTROLLED;
    return {ok: true, json: async () => state};
  };
  return {sent, state, fetch};
}
let fake = helm(OTHER);
let seen = [];
let control = new SessionControl({fetch: fake.fetch, token: () => 'tok', onState: (state, previous) => seen.push(
  [state, previous])});
await control.refresh();
assert.equal(control.state.access.in_control, false);
await control.act('start:left');
assert.deepEqual(fake.sent.slice(1), [
  ['/api/helm/control/claim', {}, 'tok'],
  ['/api/helm/teleop/receiver', {mode: 'live', arm: 'left'}, 'tok'],
  ['/api/helm/teleop/start', {headset: true, confirm: true}, 'tok']]);
assert.equal(seen.length, 2);  // The refresh, then the start's reply.
fake = helm(CONTROLLED);
control = new SessionControl({fetch: fake.fetch, token: () => 'tok'});
await control.refresh();
await control.act('start:virtual');
assert.deepEqual(fake.sent.slice(1).map(([path, body]) => [path, body]), [
  ['/api/helm/teleop/receiver', {mode: 'virtual'}], ['/api/helm/teleop/start', {headset: true}]]);  // No claim.
await control.act('start:head');  // The head alone takes no headset input for the arms.
assert.deepEqual(fake.sent.at(-1).slice(0, 2), ['/api/helm/teleop/start', {confirm: true}]);
await control.act('key:T');
assert.deepEqual(fake.sent.at(-1).slice(0, 2), ['/api/helm/teleop/key', {key: 'T'}]);
await control.act('stop');
assert.deepEqual(fake.sent.at(-1).slice(0, 2), ['/api/helm/teleop/stop', {}]);
await assert.rejects(control.act('key:Q'), /No key Q/);
await assert.rejects(control.act('start:arm9'), /No receiver arm9/);
assert.equal(control.busy, false);
// Helm's refusal, in its words; one action at a time.
fake = helm(CONTROLLED, {'/api/helm/teleop/key': 'Enable (E) first'});
control = new SessionControl({fetch: fake.fetch, token: () => 'tok'});
await assert.rejects(control.act('key:T'), /Enable \(E\) first/);
const first = control.act('stop');
await assert.rejects(control.act('stop'), /still answering/);
await first;
fake = helm(OTHER);
control = new SessionControl({fetch: fake.fetch, token: () => 'tok'});
await control.refresh();
assert.equal((await control.act('claim')).access.in_control, true);
assert.deepEqual(fake.sent.map(([path]) => path), ['/api/helm/state', '/api/helm/control/claim', '/api/helm/state']);
console.log('headset ring and session control: ok');
