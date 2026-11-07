// The digital twin's correction in Helm without the DOM: what the Teleop tab's twin card shows (helm.twin): the
// proposal from the newest wrist calibration and the head camera's saved alignment, what is applied, and whether the
// 3D views draw it now.

const degrees = (value) => (Number.isFinite(value) ? `${value < 0 ? '−' : '+'}${Math.abs(value).toFixed(2)}°` : '—');
const millimetres = (value) => (Number.isFinite(value) ? `${value.toFixed(1)} mm` : '—');

// A joint as the card names it: left_arm_j1 → left J1, neck_z → pan.
export function jointLabel(name) {
  const arm = /^(left|right)_arm_j(\d)$/.exec(name);
  if (arm) return `${arm[1]} J${arm[2]}`;
  return { neck_z: 'pan', neck_y: 'tilt' }[name] || name;
}

const offsetsText = (part) =>
  Object.entries(part.offsets_deg || {})
    .map(([joint, value]) => {
      const sigma = part.sigma_deg?.[joint];
      return `${jointLabel(joint)} ${degrees(value)}${Number.isFinite(sigma) ? ` ±${sigma.toFixed(2)}` : ''}`;
    })
    .join(', ');

// Whether Apply has something new to save: a proposal with a part, unlike what is applied and drawn.
export function twinCanApply(twin) {
  const proposal = twin?.proposal;
  if (!twin || twin.busy || !(proposal?.arms || proposal?.head)) return false;
  return !twin.applied?.current || (twin.drawing?.notes || []).length > 0;
}

// The card's status line: whether the 3D views draw a correction, and from what.
export function twinStatusText(twin) {
  if (!twin) return '';
  const applied = twin.applied,
    drawing = twin.drawing || {};
  if (!applied) {
    return (
      'Not applied: the 3D views draw the measured angles' +
      (twin.proposal?.arms || twin.proposal?.head ? ' · Apply to draw the proposal below' : '')
    );
  }
  const when = String(applied.saved_at || '')
    .replace('T', ' ')
    .slice(0, 16);
  const drawn = [
    drawing.arms ? `${drawing.arms} arm joint${drawing.arms === 1 ? '' : 's'}` : '',
    drawing.head ? 'the pan' : '',
  ]
    .filter(Boolean)
    .join(' and ');
  return (
    `Applied ${when} UTC from wrist run ${applied.wrists_run || '—'}` +
    (drawn ? ` · the 3D views draw ${drawn} corrected` : ' · the 3D views draw none of it') +
    (applied.current ? '' : ' · the calibrations changed since: Apply again for the proposal below')
  );
}

// The proposal's parts, why a part is missing, and why an applied part is not drawn.
export function twinLines(twin) {
  if (!twin) return [];
  const proposal = twin.proposal || {},
    lines = [];
  if (proposal.arms) {
    lines.push({ title: `Arms (wrist run ${proposal.arms.run})`, text: offsetsText(proposal.arms), bad: false });
  }
  if (proposal.head) {
    const head = proposal.head;
    lines.push({
      title: `Head (alignment ${head.alignment_run || '—'})`,
      bad: false,
      text:
        `${offsetsText(head)} · the two boards ${millimetres(head.board_mm)} apart; beside the pan ` +
        `${millimetres(head.other_mm)} and ${Number.isFinite(head.other_deg) ? head.other_deg.toFixed(2) : '—'}°, ` +
        "the neck's place against CAD (not drawn)",
    });
  }
  for (const note of proposal.notes || []) lines.push({ title: '', text: note, bad: false });
  for (const note of twin.drawing?.notes || []) lines.push({ title: 'Applied, not drawn:', text: note, bad: true });
  return lines;
}
