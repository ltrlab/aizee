// Gripper force feedback's page logic without the DOM: the force bars' colour and the panel's words.

// Green at rest, through yellow, to red at the maximum force, like the golf swing meter in Wii Sports.
export function forceColor(level) {
  const clamped = Math.min(1, Math.max(0, Number.isFinite(level) ? level : 0));
  return `hsl(${Math.round(120 * (1 - clamped))} 85% 46%)`;
}

// Settings -> Grippers: force feedback's settings, one slider each. `scale` turns a setting into its slider's value
// (strength and the contact push are shares, shown in percent).
export const GRIP_SETTINGS = [
  { key: 'cap_ma', id: 'grip-cap', label: 'maximum trigger force', scale: 1, unit: ' mA', digits: 0 },
  { key: 'strength', id: 'grip-strength', label: 'strength', scale: 100, unit: '%', digits: 0 },
  { key: 'wall_deg', id: 'grip-wall', label: 'full force at', scale: 1, unit: '° past the wall', digits: 1 },
  { key: 'grip_nm', id: 'grip-threshold', label: 'grip threshold', scale: 1, unit: ' Nm', digits: 2 },
  { key: 'hold_ms', id: 'grip-hold', label: 'hold time', scale: 1, unit: ' ms', digits: 0 },
];

// A slider's value in its words, and a setting's value as its slider's.
export function gripSettingText(setting, sliderValue) {
  const value = Number(sliderValue);
  return `${Number.isFinite(value) ? value.toFixed(setting.digits) : '-'}${setting.unit}`;
}
export function gripSliderValue(setting, value) {
  return String(Number((value * setting.scale).toFixed(setting.digits)));
}

// The live line under the settings: each gripper's squeeze now, whether it counts as a grip, and its push.
export function gripLive(teleop) {
  const grip = teleop?.grip_feedback;
  if (teleop?.phase !== 'running' || !grip) return `Start a session to see each gripper's squeeze here.`;
  return ['left', 'right']
    .map((side) => {
      const squeeze = Number(grip.squeeze_nm?.[side]);
      const current = Number(grip.currents_ma?.[side]);
      const parts = [
        `${side === 'left' ? 'Left' : 'Right'} ${Number.isFinite(squeeze) ? squeeze.toFixed(2) : '-'} Nm`,
        grip.gripping?.[side] ? 'holding' : 'free',
      ];
      if (Number.isFinite(current) && current >= 1) parts.push(`${Math.round(current)} mA`);
      return parts.join(' · ');
    })
    .join('  |  ');
}

// What the Teleop panel shows for this state: bars for each gripper's squeeze against the maximum force (shown
// even with feedback off), whether the triggers are pushing now, and the saved maximum force and strength.
export function gripView(teleop) {
  const grip = teleop?.grip_feedback;
  const running = teleop?.phase === 'running';
  const usable = running && grip?.available === true;
  const settings = teleop?.grip_settings || {};
  const cap = Number.isFinite(grip?.cap_ma) ? grip.cap_ma : settings.cap_ma;
  const capText = Number.isFinite(cap) ? `${Math.round(cap)} mA` : 'the maximum';
  const engaged = teleop?.state === 'TELEOP';
  const sides = {};
  for (const side of ['left', 'right']) {
    const raw = Number(grip?.levels?.[side]);
    const level = Math.min(1, Math.max(0, Number.isFinite(raw) ? raw : 0));
    const current = Number(grip?.currents_ma?.[side]);
    sides[side] = {
      level,
      percent: Math.round(level * 100),
      color: forceColor(level),
      current: Number.isFinite(current) ? Math.round(current) : 0,
    };
  }
  const enabled = grip?.enabled === true;
  const text = !running
    ? 'Start a session to switch it on.'
    : !usable
      ? 'These leaders have no triggers configured for force feedback.'
      : !enabled
        ? `Off: the triggers stay free. The bars show each gripper's squeeze against ${capText}.`
        : !engaged
          ? `On: the triggers push back once you engage (T), up to ${capText}.`
          : `On · left ${sides.left.current} mA · right ${sides.right.current} mA, up to ${capText}.`;
  return { usable, enabled, active: usable && enabled && engaged, sides, text, settings };
}
