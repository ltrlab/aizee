import assert from 'node:assert/strict';
import {GRIP_SETTINGS, forceColor, gripLive, gripSettingText, gripSliderValue, gripView}
  from '../../src/robot_platform/helm/static/grip.js';

// Green at rest, yellow halfway, red at the maximum; out-of-range and missing levels clamp.
assert.equal(forceColor(0), 'hsl(120 85% 46%)');
assert.equal(forceColor(.5), 'hsl(60 85% 46%)');
assert.equal(forceColor(1), 'hsl(0 85% 46%)');
assert.equal(forceColor(3), 'hsl(0 85% 46%)');
assert.equal(forceColor(NaN), 'hsl(120 85% 46%)');
assert.equal(forceColor(undefined), 'hsl(120 85% 46%)');

const grip = (extra = {}) => ({enabled: false, available: true, cap_ma: 150, levels: {left: 0, right: .4},
  currents_ma: {left: 0, right: 0}, ...extra});
const teleop = (extra = {}) => ({phase: 'running', state: 'TELEOP', grip_settings: {strength: 1, cap_ma: 150},
  grip_feedback: grip(), ...extra});

assert.match(gripView({phase: 'stopped', grip_settings: {cap_ma: 150}}).text, /Start a session/);
assert.equal(gripView(null).usable, false);
assert.match(gripView(teleop({grip_feedback: grip({available: false})})).text, /no triggers configured/);
const off = gripView(teleop());
assert.equal(off.usable, true);
assert.equal(off.active, false);
assert.match(off.text, /Off: the triggers stay free.*150 mA/);
assert.equal(off.sides.right.percent, 40);  // The bar shows the squeeze even with feedback off.
assert.equal(off.sides.right.color, forceColor(.4));
const waiting = gripView(teleop({state: 'HOLD', grip_feedback: grip({enabled: true})}));
assert.equal(waiting.active, false);
assert.match(waiting.text, /once you engage \(T\), up to 150 mA/);
const pushing = gripView(teleop({grip_feedback: grip({enabled: true, levels: {left: 1.3, right: -1},
  currents_ma: {left: 149.6, right: 0}})}));
assert.equal(pushing.active, true);
assert.equal(pushing.sides.left.percent, 100);
assert.equal(pushing.sides.right.percent, 0);
assert.match(pushing.text, /left 150 mA · right 0 mA, up to 150 mA/);
assert.deepEqual(pushing.settings, {strength: 1, cap_ma: 150});

// Settings -> Grippers: a slider per setting, read in its units, set without float noise.
const setting = key => GRIP_SETTINGS.find(item => item.key === key);
assert.deepEqual(GRIP_SETTINGS.map(item => item.key).sort(), ['cap_ma', 'grip_nm', 'hold_ms', 'strength', 'wall_deg']);
assert.equal(gripSettingText(setting('wall_deg'), '2'), '2.0° past the wall');
assert.equal(gripSettingText(setting('grip_nm'), '0.5'), '0.50 Nm');
assert.equal(gripSettingText(setting('hold_ms'), '100'), '100 ms');
assert.equal(gripSettingText(setting('cap_ma'), '150'), '150 mA');
assert.equal(gripSettingText(setting('strength'), '60'), '60%');
assert.equal(gripSliderValue(setting('wall_deg'), 2.5), '2.5');
assert.equal(gripSliderValue(setting('strength'), .6), '60');
assert.equal(gripSliderValue(setting('grip_nm'), .55), '0.55');
assert.match(gripLive({phase: 'stopped'}), /Start a session/);
assert.equal(gripLive(teleop({grip_feedback: grip({squeeze_nm: {left: .31, right: 1.92},
  gripping: {left: false, right: true}, currents_ma: {left: 0, right: 149.6}})})),
  'Left 0.31 Nm · free  |  Right 1.92 Nm · holding · 150 mA');
console.log('grip view checks passed');
