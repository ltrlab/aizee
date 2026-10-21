// Gripper force feedback's page logic without the DOM: the force bars' colour and the panel's words.

// Green at rest, through yellow, to red at the maximum force, like the golf swing meter in Wii Sports.
export function forceColor(level) {
  const clamped = Math.min(1, Math.max(0, Number.isFinite(level) ? level : 0));
  return `hsl(${Math.round(120 * (1 - clamped))} 85% 46%)`;
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
    sides[side] = {level, percent: Math.round(level * 100), color: forceColor(level),
      current: Number.isFinite(current) ? Math.round(current) : 0};
  }
  const enabled = grip?.enabled === true;
  const text = !running ? 'Start a session to switch it on.'
    : !usable ? 'These leaders have no triggers configured for force feedback.'
    : !enabled ? `Off: the triggers stay free. The bars show each gripper's squeeze against ${capText}.`
    : !engaged ? `On: the triggers push back once you engage (T), up to ${capText}.`
    : `On · left ${sides.left.current} mA · right ${sides.right.current} mA, up to ${capText}.`;
  return {usable, enabled, active: usable && enabled && engaged, sides, text, settings};
}
