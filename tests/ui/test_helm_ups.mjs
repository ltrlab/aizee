import assert from 'node:assert/strict';
import {duration, upsView} from '../../src/robot_platform/helm/static/ups.js';

assert.equal(duration(42.4), '42 min');
assert.equal(duration(60), '1 h');
assert.equal(duration(100), '1 h 40 min');
assert.equal(duration(-1), null);
assert.equal(duration(undefined), null);

const base = {version: 1, sensor: 'ok', source: 'external', charge: 'charging', level: 'ok', percent: 41,
  voltage_v: 11.62, current_a: 1.21, power_w: 14.06, cell_v: 3.873, minutes_left: null, shutdown: null,
  thresholds: {low_percent: 25, shutdown_percent: 10, floor_cell_v: 3.35}};
const ok = state => ({status: 'ok', state: {...base, ...state}, age_s: 1, poweroff_in_s: null, error: null});

assert.equal(upsView(null).hidden, true);  // Simulation: no UPS.
assert.equal(upsView({status: 'starting'}).text, '…');

const charging = upsView(ok({}));
assert.deepEqual([charging.tone, charging.text, charging.level, charging.charging, charging.banner],
  ['good', '41%', 41, true, null]);
assert.match(charging.title, /^On the charger, charging: 11\.62 V · \+1\.21 A · 14\.1 W · 3\.87 V a cell · about 41% charge$/);
assert.equal(upsView(ok({charge: 'full', percent: 99})).text, 'Full');
const onPower = upsView(ok({charge: 'on_power'}));
assert.deepEqual([onPower.text, onPower.charging], ['41%', false]);

const battery = upsView(ok({source: 'battery', charge: 'discharging', percent: 63, minutes_left: 55, current_a: -1.4}));
assert.equal(battery.tone, 'warn');
assert.equal(battery.text, '63%');
assert.match(battery.title, /^On battery, about 55 min to shutdown: /);
assert.equal(battery.banner.tone, 'warn');
assert.match(battery.banner.text, /running on its UPS battery: about 63%, roughly 55 min before it shuts itself down at 10%\. Plug in the UPS charger\./);

const low = upsView(ok({source: 'battery', charge: 'discharging', level: 'low', percent: 22, minutes_left: 12}));
assert.deepEqual([low.tone, low.text, low.banner.tone], ['bad', '22%', 'bad']);
assert.match(low.banner.text, /low: about 22%, roughly 12 min .* Plug in the UPS charger now\./);

const shutdown = {stage: 'pending', reason: 'about 9% charge, at or under 10%', since: 1, poweroff_by: 91};
const pending = upsView({...ok({source: 'battery', level: 'critical', percent: 9, shutdown}), poweroff_in_s: 47});
assert.deepEqual([pending.tone, pending.text], ['bad', '47 s']);
assert.match(pending.title, /^Shutting down: /);
assert.match(pending.banner.text, /critical \(about 9% charge, at or under 10%\)\. Helm ends any session, lowering the arms first, and the Jetson powers off within 47 s\. Plug in the UPS charger now to cancel\./);
const powering = upsView(ok({source: 'battery', level: 'critical', shutdown: {...shutdown, stage: 'powering_off'}}));
assert.equal(powering.text, 'Off');
assert.match(powering.banner.text, /powering off now/);

const offline = upsView({status: 'unreachable', error: 'No route to host', last: {...base, source: 'battery', percent: 4, seen_s_ago: 120}});
assert.deepEqual([offline.tone, offline.text, offline.level], ['bad', '4%?', 4]);
assert.match(offline.banner.text, /went offline while running on its UPS battery \(last about 4%\)/);
assert.match(offline.title, /No route to host.*Last reading 2 min ago/);
const quiet = upsView({status: 'unreachable', error: 'timed out', last: {...base}});
assert.deepEqual([quiet.tone, quiet.text, quiet.banner], ['muted', '—', null]);

assert.match(upsView({status: 'missing'}).title, /sudo python3 -B -m robot_platform\.ops\.ups install/);
assert.equal(upsView({status: 'missing'}).tone, 'warn');
assert.match(upsView({status: 'stale', age_s: 31}).title, /stopped writing 31 s ago/);
const broken = upsView(ok({sensor: 'error', error: '[Errno 121] Remote I/O error'}));
assert.deepEqual([broken.tone, broken.text], ['warn', '!']);
assert.match(broken.title, /Remote I\/O error.*cannot warn or shut down/);
console.log('ups view checks passed');
