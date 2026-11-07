// Helm's Joint zero without the DOM: what the Limits tab's Joint zero card shows (helm.joint_zero): the arm joints to
// choose with their live readings, the zeros waiting for the motors' power cycle, the reference Verify recorded, and
// the joints whose limits to sweep again.

const radians = (value) => (Number.isFinite(value) ? `${value < 0 ? '−' : '+'}${Math.abs(value).toFixed(3)} rad` : '—');
export const ARM_JOINTS = ['left', 'right'].flatMap((side) => [1, 2, 3, 4, 5, 6].map((n) => `${side}_arm_j${n}`));
const labels = (items) => items.map((item) => item.label || item.motor).join(', ');

// The arm joints to choose from, left arm first, each with its label and live reading from the Limits rows.
export function zeroChoices(rows) {
  const byName = new Map((rows || []).map((row) => [row.name, row]));
  return ARM_JOINTS.filter((name) => byName.has(name)).map((name) => {
    const row = byName.get(name);
    return {
      motor: name,
      label: row.label || name,
      arm: name.split('_')[0],
      reading: row.current_rad,
      text: radians(row.current_rad),
    };
  });
}

// Why neither the zero nor Verify can run now, or ''.
export function jointZeroBlocked(state) {
  const zero = state?.joint_zero;
  if (!zero) return 'Joint zero runs with Helm on Minerva.';
  if (!zero.available) return `${zero.reason || 'Joint zero is unavailable'}.`;
  if (zero.applied)
    return zero.restarts ? 'Helm is restarting to load the new reference.' : 'Restart Helm to load the new reference.';
  if (zero.running) return zero.running === 'verify' ? 'Verifying…' : 'Zeroing…';
  if (['starting', 'running'].includes(state?.teleop?.phase)) return 'Stop the teleop session first.';
  if (['starting', 'running'].includes(state?.tune?.phase)) return 'Finish or stop the servo sweep first.';
  if (state?.check?.running) return 'Wait for Minerva Check to finish.';
  if (state?.gripper_zero?.running) return 'Wait for the gripper zero to finish.';
  if (state?.robot?.connection && !['tether', 'robot'].includes(state.robot.connection)) return 'Choose Tether first.';
  return '';
}

// The card's status line: what waits, what was recorded, what to sweep again.
export function jointZeroStatus(zero) {
  if (!zero) return '';
  if (zero.applied) {
    return (
      `Recorded the arms' reference ${zero.applied.reference}. ` +
      (zero.restarts
        ? 'Helm restarts to load it: log in again when the page asks, then sweep the zeroed joints.'
        : 'Restart Helm to load it, then sweep the zeroed joints.')
    );
  }
  if (zero.pending?.length) {
    return (
      `Zeroed, waiting for the power cycle: ${labels(zero.pending)}. Switch the motors' power off and on, leave ` +
      'the arm where it is, then Verify. Nothing that moves the arms starts until then.'
    );
  }
  if (zero.resweep?.length) {
    return `Sweep ${labels(zero.resweep)} again below and save: ${
      zero.resweep.length === 1 ? 'its limits were' : 'their limits were'
    } cleared when ${zero.resweep.length === 1 ? 'it was' : 'they were'} zeroed.`;
  }
  return `Every joint keeps its zero (reference ${zero.reference}).`;
}

// The zeros waiting for the power cycle, each with what it read before its zero.
export function pendingLines(zero) {
  return (zero?.pending || []).map(
    (entry) =>
      `${entry.label || entry.motor}: read ${radians(entry.before_rad)} at the ` +
      `zero pose, now ${radians(entry.after_save_rad)}${entry.save_response_missing ? ' (its save gave no reply)' : ''}`,
  );
}

// The last zero or Verify, as the feedback line says it: {text, bad}, or null.
export function jointZeroResult(zero) {
  const last = zero?.last;
  if (!last) return null;
  if (last.action === 'zero') {
    const zeroed = (last.pending || []).length;
    if (last.success)
      return { text: `Zeroed and saved ${zeroed} joint${zeroed === 1 ? '' : 's'}. Now the power cycle.`, bad: false };
    return {
      text:
        `The zero failed: ${last.error || 'no reason given'}` +
        (zeroed
          ? ` (${zeroed} joint${zeroed === 1 ? ' was' : 's were'} zeroed before it stopped: they wait for the power cycle too)`
          : ''),
      bad: true,
    };
  }
  if (last.success)
    return { text: `Verified: every zeroed joint kept its zero. Recorded ${last.reference}.`, bad: false };
  return { text: `Verify did not pass: ${last.error || 'no reason given'}`, bad: true };
}
