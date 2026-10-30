// The Limits tab's Head section (helm/head_setup.py): what it says and which of its steps can run, from Helm's
// state. The joint cards themselves are the arms' (helm.js build/update), on the head's endpoints.
const date = text => (text ? new Date(text).toLocaleString() : '');

function servoText(servo) {
  if (servo.steps == null) return `${servo.label} —`;
  const parts = [`${servo.label} ${servo.steps}`];
  if (Number.isFinite(servo.voltage_v)) parts.push(`${servo.voltage_v.toFixed(1)} V`);
  if (Number.isFinite(servo.temperature_c)) parts.push(`${servo.temperature_c} °C`);
  if (servo.alarms?.length) parts.push(servo.alarms.join(', '));
  if (servo.torque) parts.push('holding torque');
  return parts.join(' · ');
}

export function headSetupView(setup) {
  if (!setup) {
    return {available: false, note: 'The head is set up in Helm on Minerva, beside its servo board.'};
  }
  const rows = setup.rows || [], fresh = !!setup.fresh, reading = !!setup.reading;
  const servos = setup.servos || [];
  const trouble = servos.some(servo => servo.alarms?.length || servo.torque);
  const reference = setup.reference, limits = setup.limits;
  let readingText, readingTone = '';
  if (!reading) readingText = 'Not reading. The servos stay limp either way.';
  else if (setup.error) [readingText, readingTone] = [setup.error, 'bad'];
  else if (!fresh) readingText = 'Waiting for the servos…';
  else [readingText, readingTone] = [servos.map(servoText).join('   '), trouble ? 'warn' : 'ok'];
  const referenceText = reference
    ? `Reference set ${date(reference.created_at)} · directions ${reference.directions_verified
      ? 'checked' : 'not checked yet: turn each joint and watch the 3D view'}`
    : 'No reference yet: read the head, hold it straight ahead and level, then set it.';
  const limitsText = !limits ? 'No head limits saved yet.'
    : limits.matches_reference ? `Head limits saved ${date(limits.created_at)}.`
      : 'The saved head limits belong to an older reference: sweep both joints again and save.';
  const pending = setup.pending || 0;
  return {
    available: true, reading, fresh, live: fresh, rows,
    readLabel: reading ? 'Stop reading' : 'Read the head', readingText, readingTone,
    referenceLabel: reference ? 'Set the reference again…' : 'Set the reference…',
    referenceEnabled: fresh && !trouble, referenceText,
    referenceConfirm: 'Hold the head straight ahead and level, and keep it there. Both servos\' zero is written '
      + 'to their memory' + (limits ? ', and the saved head limits will need sweeping again' : '')
      + '. Set the head\'s reference now?',
    limitsText,
    saveEnabled: fresh && rows.length > 0 && pending === rows.length,
    saveLabel: pending ? `Save head limits (${pending}/${rows.length})` : 'Save head limits',
    status: setup.message || '',
  };
}
