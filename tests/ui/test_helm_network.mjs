import assert from 'node:assert/strict';
import {bridgeView, networkView} from '../../src/robot_platform/helm/static/network.js';

const base = {installed: true, running: true, age_s: .4, version: 1, local: '10.42.0.1',
  ethernet: {carrier: true, connection: 'minerva-lan', addresses: ['192.168.8.43/24']},
  wifi: {connection: null, mode: 'off', addresses: []},
  ap: {wanted: false, on: false, ssid: 'minerva', address: '10.42.1.1'}};

// Helm on a PC: no network section from it.
assert.equal(networkView(null).hidden, true);
const missing = networkView({installed: false});
assert.equal(missing.mode, 'Not installed');
assert.match(missing.lines[0], /sudo python3 -B -m robot_platform\.ops\.network install/);

// Wired on a network.
const lan = networkView({...base, mode: 'lan'});
assert.deepEqual([lan.text, lan.tone, lan.apOn, lan.apEnabled], ['Wired', 'good', false, true]);
assert.match(lan.lines[0], /On a network by its cable: 192\.168\.8\.43\./);
assert.match(lan.lines.at(-1), /https:\/\/192\.168\.8\.43:8760 \(or https:\/\/aizee-rover\.local:8760\)/);
assert.equal(lan.apLabel, 'Turn the access point on');
assert.match(lan.apConfirm, /Join "minerva" and open https:\/\/10\.42\.0\.1:8760/);

// Tether, plus the access point.
const both = networkView({...base, mode: 'tether', ethernet: {carrier: true, addresses: ['10.42.0.1/24']},
  wifi: {connection: 'minerva-ap', mode: 'ap', addresses: ['10.42.1.1/24']},
  ap: {...base.ap, wanted: true, on: true}});
assert.equal(both.text, 'Tether + AP');
assert.ok(both.lines.some(line => /Access point "minerva" is on: its devices reach Minerva at 10\.42\.0\.1/.test(line)));
// 10.42.0.1 once, never the access point's own 10.42.1.1 (it listed both, twice, in a preview, 2026-10-25).
assert.equal(both.lines.at(-1), 'Open Helm at https://10.42.0.1:8760 (or https://aizee-rover.local:8760).');
assert.equal(both.apLabel, 'Turn the access point off');
assert.match(both.apConfirm, /Turn the access point off\?/);

// Unplugged, on Wi-Fi; unplugged with nothing; and the cable on a network whose DHCP did not answer.
const wifi = networkView({...base, mode: 'unplugged', ethernet: {carrier: false, addresses: []},
  wifi: {connection: 'birdnest', mode: 'client', addresses: ['192.168.8.180/24']}});
assert.deepEqual([wifi.text, wifi.tone], ['Wi-Fi', 'good']);
const alone = networkView({...base, mode: 'unplugged', ethernet: {carrier: false, addresses: []}});
assert.deepEqual([alone.text, alone.tone], ['No network', 'warn']);
const stuck = networkView({...base, mode: 'no_address', ethernet: {carrier: true, addresses: []}});
assert.deepEqual([stuck.text, stuck.tone], ['No address', 'bad']);
assert.match(stuck.lines[0], /Replug the cable/);

// A service that stopped writing: red, said first, and no switch.
const silent = networkView({...base, mode: 'lan', running: false, age_s: 42});
assert.equal(silent.tone, 'bad');
assert.match(silent.lines[0], /has not reported for 42 s/);
assert.equal(silent.apEnabled, false);

// The GELLO pill: where the bridge reached Minerva, and Wi-Fi said.
assert.equal(bridgeView(null).hidden, true);
assert.deepEqual([bridgeView({connected: false}).text, bridgeView({connected: false}).tone], ['No GELLO', 'warn']);
const wired = bridgeView({connected: true, host: 'soliloquy', peer: '192.168.8.225', robot_address: '192.168.8.43',
  path: 'wired'});
assert.deepEqual([wired.text, wired.tone], ['GELLO', 'good']);
assert.equal(wired.title, 'The GELLO bridge on soliloquy (192.168.8.225) serves the leader arms at 192.168.8.43, wired');
const air = bridgeView({connected: true, host: 'soliloquy', peer: '10.42.1.20', robot_address: '10.42.0.1',
  path: 'wifi'});
assert.deepEqual([air.text, air.tone], ['GELLO · Wi-Fi', 'warn']);
assert.match(air.title, /over Wi-Fi: leader teleop runs, but expect holds/);
