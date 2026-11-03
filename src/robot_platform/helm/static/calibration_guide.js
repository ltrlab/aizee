// The guided camera calibration in Helm without the DOM (Teleop → Calibrate): its three steps (the wrist cameras, the
// head camera, the digital twin), what to do next, how far each camera has got, and what a solve's numbers mean.
import {ROLE_LABELS, captureBlock, headCaptureBlock} from './calibration.js';
import {twinCanApply} from './twin.js';

export const STEPS = ['wrists', 'head', 'twin'];
export const STEP_TITLES = {wrists: 'Wrist cameras', head: 'Head camera', twin: 'Digital twin'};
export const WRISTS = ['left_wrist', 'right_wrist'];
// Stills with tags a camera wants: a solve takes as few as 5, 9 to hold some out (docs/development/
// WRIST_CAMERA_CALIBRATION.md). The head's camera is placed by how far it turned and tilted across its stills: under
// 10° the solve warns (vision.head_alignment MIN_SPREAD_DEG).
export const WRIST_STILLS = 15, HEAD_STILLS = 10;
export const SPREAD_DEG = 20, MIN_SPREAD_DEG = 10;
// About 1 px is a good fit on these cameras (0.3–1 px with good kinematics); past 3 px the stills disagree.
export const GOOD_PX = 1.5, BAD_PX = 3;
// Stills further apart than this may not share one setup: the table or Minerva may have moved between them (a head
// run of 2026-11-02 mixed stills of two days and fitted at 21 px).
export const GAP_MINUTES = 30;
export const COVERAGE_CELLS = 12;  // helm.calibration COVERAGE: 4 columns, 3 rows.

const LEVELS = ['good', 'check', 'redo'];
const worst = levels => LEVELS[Math.max(0, ...levels.map(level => LEVELS.indexOf(level)))];
const fixed = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const signed = (value, digits = 1) => Number.isFinite(value) ? `${value < 0 ? '−' : '+'}${Math.abs(value).toFixed(digits)}` : '—';
const label = role => ROLE_LABELS[role] || role;
const lower = text => text.charAt(0).toLowerCase() + text.slice(1);

// Reasons as {level, text}, the worst first.
function verdictOf(reasons, extra = {}) {
  const sorted = [...reasons].sort((a, b) => LEVELS.indexOf(b.level) - LEVELS.indexOf(a.level));
  return {level: worst(reasons.map(reason => reason.level)), reasons: sorted.map(reason => reason.text), ...extra};
}

// -- what a solve's numbers mean --------------------------------------------------------------------------------

function wristCamera(camera, name) {
  const reasons = [], flag = (level, text) => reasons.push({level, text: `${name}: ${text}`});
  const px = camera.rms_px, held = camera.holdout_px;
  if (px > BAD_PX) flag('redo', `its stills disagree (${fixed(px, 2)} px; about 1 px is good): something moved between them, or its mount is not where the model has it (see the checks).`);
  else if (px > GOOD_PX) flag('check', `${fixed(px, 2)} px, more than a good fit's ${GOOD_PX}.`);
  if (held > Math.max(2 * px, GOOD_PX)) flag('check', `stills held out fit worse (${fixed(held, 2)} px): take more varied stills.`);
  const sigma = Math.max(0, ...(camera.mount_sigma_mm || []));
  if (sigma > 5) flag('check', `its mount is uncertain (±${fixed(sigma)} mm): add stills from more angles.`);
  if (camera.stills < 12) flag('check', `only ${camera.stills} stills: aim for ${WRIST_STILLS}.`);
  if (camera.cells_seen < 8) flag('check', `tags were seen in only ${camera.cells_seen} of ${COVERAGE_CELLS} parts of the image: put them near its corners and edges too.`);
  const outliers = camera.outliers || [];
  if (outliers.length > Math.max(1, camera.stills / 10)) flag('check', `${outliers.length} stills fit poorly (${outliers.join(', ')}).`);
  return verdictOf(reasons, {px, items: reasons});
}

// The wrist solve's recommended model: good, check or redo, with the reasons; null before a solve.
export function wristVerdict(result) {
  if (!result?.models) return null;
  const model = result.models[result.recommended];
  if (!model || model.error) {
    const error = model?.error || Object.values(result.models).map(m => m.error).find(Boolean);
    return {level: 'redo', reasons: [error || 'No model fitted the stills.'], cameras: {}};
  }
  const cameras = Object.fromEntries(Object.entries(model.cameras || {}).map(([role, camera]) =>
    [role, wristCamera(camera, label(role))]));
  const reasons = Object.values(cameras).flatMap(camera => camera.items);
  for (const role of WRISTS) {
    if (!cameras[role]) reasons.push({level: 'check', text: `${label(role)}: not in this solve. Take its stills and Solve again.`});
  }
  const apart = model.left_vs_right?.translation_mm;
  if (apart > 25) reasons.push({level: 'redo', text: `The two arms put the board ${fixed(apart)} mm apart: the table or Minerva moved between their stills. Start a new run and take both cameras' stills in one go.`});
  else if (apart > 10) reasons.push({level: 'check', text: `The two arms put the board ${fixed(apart)} mm apart (within about 5 mm is good).`});
  for (const note of result.notes || []) reasons.push({level: 'check', text: note});
  return verdictOf(reasons, {cameras});
}

// The head solve: good, check or redo, with the reasons, and notes that say what it found (not a verdict).
export function headVerdict(result) {
  if (!result) return null;
  if (result.error) return {level: 'redo', reasons: [result.error], notes: []};
  const reasons = [], notes = [], flag = (level, text) => reasons.push({level, text});
  const px = result.rms_px, turned = result.spread_deg?.neck_z, tilted = result.spread_deg?.neck_y;
  if (px > BAD_PX) flag('redo', `The stills disagree (${fixed(px, 2)} px; about 1 px is good): most likely the table or Minerva moved between them. Start a new run and take them in one go.`);
  else if (px > GOOD_PX) flag('check', `${fixed(px, 2)} px, more than a good fit's ${GOOD_PX}.`);
  const spread = Math.min(turned, tilted);
  if (spread < MIN_SPREAD_DEG) flag('redo', `The head turned ${fixed(turned, 0)}° and tilted ${fixed(tilted, 0)}° across the stills: too little to place its camera. Turn and tilt it about ${SPREAD_DEG}° each.`);
  else if (spread < SPREAD_DEG) flag('check', `The head turned ${fixed(turned, 0)}° and tilted ${fixed(tilted, 0)}°: about ${SPREAD_DEG}° each places its camera better.`);
  const away = result.cad?.translation_mm;
  if (away > 100) flag('redo', `The camera came out ${fixed(away, 0)} mm from where it is fixed on the head: the stills do not pin it down.`);
  else if (away > 40) flag('check', `The camera came out ${fixed(away, 0)} mm from its CAD place (expect within a few cm).`);
  const sigma = Math.max(0, ...(result.mount_sigma_mm || []));
  if (sigma > 10) flag('check', `Its place is uncertain (±${fixed(sigma, 0)} mm): add stills turned and tilted further.`);
  if (result.stills < 8) flag('check', `Only ${result.stills} stills: aim for ${HEAD_STILLS}.`);
  if (result.outliers?.length) flag('check', `${result.outliers.length} stills fit poorly (${result.outliers.join(', ')}).`);
  // The tilt's zero error turns the mount (vision.head_alignment), so a turn off CAD is a finding, not a fault.
  if (result.cad?.rotation_deg > 5) notes.push(`It looks ${fixed(result.cad.rotation_deg, 0)}° off its CAD direction: the camera's bracket or the head's tilt zero (the alignment cannot tell which).`);
  const arms = result.arms;
  if (!arms) notes.push('Solve the wrist cameras too: the head alone cannot tell its pan zero from where the board lies.');
  else if (arms.other_mm > 30 || arms.other_deg > 3) flag('check', `Against the arms' board, ${fixed(arms.other_mm)} mm and ${fixed(arms.other_deg)}° apart beside the pan: the board moved between the wrist and head stills, or the neck sits off its CAD place.`);
  else notes.push(`Against the arms' board: a pan zero of ${signed(arms.pan_deg, 2)}°, and the rest within ${fixed(arms.other_mm)} mm.`);
  return verdictOf(reasons, {notes});
}

// -- how far each camera has got --------------------------------------------------------------------------------

// Where a run's stills are far apart in time: the table or Minerva may have moved between them.
export function runGaps(taken, minutes = GAP_MINUTES) {
  const times = (taken || []).map(still => ({name: still.name, at: Date.parse(still.at)}))
    .filter(still => Number.isFinite(still.at)).sort((a, b) => a.at - b.at);
  const gaps = [];
  for (let i = 1; i < times.length; i++) {
    const gap = (times[i].at - times[i - 1].at) / 60000;
    if (gap > minutes) gaps.push({after: times[i - 1].name, before: times[i].name, minutes: gap});
  }
  return gaps;
}

export function duration(minutes) {
  if (minutes < 90) return `${Math.round(minutes)} min`;
  return minutes < 48 * 60 ? `${Math.round(minutes / 60)} h` : `${Math.round(minutes / 1440)} days`;
}

export function gapText(gaps) {
  if (!gaps?.length) return null;
  const longest = gaps.reduce((a, b) => b.minutes > a.minutes ? b : a);
  const more = gaps.length > 1 ? ` (and ${gaps.length - 1} more gap${gaps.length > 2 ? 's' : ''})` : '';
  return `${duration(longest.minutes)} passed between ${longest.after} and ${longest.before}${more}: if the table or `
    + 'Minerva moved in between, these stills do not fit together. Start a new run (New run) and take them in one go.';
}

// The parts of the image (helm.calibration's 4 x 3 grid of tag corners seen) to fill next: corners first, where
// lens distortion is largest, then the edges, then the middle.
const REGIONS = [['top-left corner', [[0, 0]]], ['top-right corner', [[0, 3]]], ['bottom-left corner', [[2, 0]]],
  ['bottom-right corner', [[2, 3]]], ['top edge', [[0, 1], [0, 2]]], ['bottom edge', [[2, 1], [2, 2]]],
  ['left edge', [[1, 0]]], ['right edge', [[1, 3]]], ['middle', [[1, 1], [1, 2]]]];
export function coverageGap(grid) {
  if (!grid) return null;
  for (const [name, cells] of REGIONS) if (cells.some(([row, column]) => !(grid[row]?.[column] > 0))) return name;
  return null;
}
export const cellsSeen = grid => (grid || []).flat().filter(count => count > 0).length;

// How far the head turned (pan, left positive) and tilted (down negative) across its stills with tags.
export function headSpread(taken) {
  const tagged = (taken || []).filter(still => still.tags > 0 && Number.isFinite(still.pan_deg)
    && Number.isFinite(still.tilt_deg));
  if (!tagged.length) return null;
  const pans = tagged.map(still => still.pan_deg), tilts = tagged.map(still => still.tilt_deg);
  const pan = [Math.min(...pans), Math.max(...pans)], tilt = [Math.min(...tilts), Math.max(...tilts)];
  return {count: tagged.length, pan, tilt, turned: pan[1] - pan[0], tilted: tilt[1] - tilt[0]};
}

// The head now ({pan, tilt} degrees) from the session's view: the neck's CAD angles in radians.
export function headNow(teleop) {
  const joints = teleop?.head?.joints || {};
  const degrees = value => Number.isFinite(value) ? value * 180 / Math.PI : null;
  const pan = degrees(joints.neck_z), tilt = degrees(joints.neck_y);
  return pan === null || tilt === null ? null : {pan, tilt};
}

// Which way the next head stills should go: toward the side of the stills' middle the head is on now (left and down
// without the head's angles).
const sides = (spread, now) => ({left: !now || now.pan >= (spread.pan[0] + spread.pan[1]) / 2,
  down: !now || now.tilt <= (spread.tilt[0] + spread.tilt[1]) / 2});

// What the next head still should add: more turn (the axis furthest from SPREAD_DEG first), toward the side the head
// is on now, or more tilt; null once both are wide enough.
export function headHint(spread, now) {
  if (!spread) return 'look at the table until the head camera sees three or more tags';
  const turn = SPREAD_DEG - spread.turned, tilt = SPREAD_DEG - spread.tilted, {left, down} = sides(spread, now);
  if (turn <= 0 && tilt <= 0) return null;
  if (turn >= tilt) return `turn the head about ${Math.ceil(turn)}° further ${left ? 'left' : 'right'}, still seeing tags`;
  return `tilt the head about ${Math.ceil(tilt)}° further ${down ? 'down, nearer' : 'up, farther'} on the table, `
    + 'still seeing tags';
}

// The pan and tilt ranges (degrees, low to high) the head's stills should cover: SPREAD_DEG from the farthest still
// toward the side headHint sends the head, or the stills' own range once it is wider.
export function headTarget(spread, now) {
  if (!spread) return null;
  const {left, down} = sides(spread, now);
  const axis = ([low, high], upward) => high - low >= SPREAD_DEG ? [low, high]
    : upward ? [low, low + SPREAD_DEG] : [high - SPREAD_DEG, high];
  return {pan: axis(spread.pan, left), tilt: axis(spread.tilt, !down)};
}

const tagged = (calibration, role) => calibration?.counts?.[role]?.tagged || 0;
// A solve is current while no still came after it.
export function solveCurrent(snapshot) {
  const solved = Date.parse(snapshot?.result?.solved_at);
  if (!snapshot?.result) return false;
  if (!Number.isFinite(solved)) return true;
  return !(snapshot.taken || []).some(still => Date.parse(still.at) > solved);
}
const since = snapshot => (snapshot?.taken || []).filter(still =>
  Date.parse(still.at) > Date.parse(snapshot.result?.solved_at)).length;

// -- the steps --------------------------------------------------------------------------------------------------

// Each step's status (todo, active, done, check or redo) and a few words for its tab.
export function stepStatus(state) {
  const calibration = state?.calibration, alignment = state?.head_alignment, twin = state?.twin;
  const wrists = (() => {
    if (!calibration) return {status: 'todo', summary: 'not on this Helm'};
    const verdict = solveCurrent(calibration) ? wristVerdict(calibration.result) : null;
    if (verdict?.level === 'good') {
      return {status: 'done', summary: `${WRISTS.map(role => fixed(verdict.cameras[role]?.px)).join(' · ')} px`};
    }
    if (verdict) return {status: verdict.level, summary: verdict.level === 'check' ? 'solved: check' : 'redo'};
    const counts = WRISTS.map(role => tagged(calibration, role));
    if (calibration.result) return {status: 'active', summary: 'Solve again'};
    return counts.some(Boolean) ? {status: 'active', summary: `left ${counts[0]}, right ${counts[1]} of ${WRIST_STILLS}`}
      : {status: 'todo', summary: 'not started'};
  })();
  const head = (() => {
    if (!alignment) return {status: 'todo', summary: 'not on this Helm'};
    const saved = alignment.saved, verdict = solveCurrent(alignment) ? headVerdict(alignment.result) : null;
    if (saved?.current && saved.run === alignment.run && verdict?.level !== 'redo') {
      return {status: 'done', summary: `saved · ${fixed(saved.fit?.rms_px)} px`};
    }
    if (verdict?.level === 'redo') return {status: 'redo', summary: 'redo'};
    if (verdict) return {status: verdict.level === 'good' ? 'active' : 'check', summary: 'solved: save it'};
    if (alignment.tagged) return {status: 'active', summary: alignment.result ? 'Solve again' : `${alignment.tagged} of ${HEAD_STILLS}`};
    return saved?.current ? {status: 'done', summary: `run ${saved.run} in use`} : {status: 'todo', summary: 'not started'};
  })();
  const twinStep = (() => {
    if (!twin) return {status: 'todo', summary: 'not on this Helm'};
    if (twin.applied?.current && !(twin.drawing?.notes || []).length) return {status: 'done', summary: 'applied'};
    if (twinCanApply(twin)) {
      const {arms, head: pan} = twin.proposal;
      return {status: 'active', summary: arms && pan ? 'ready to apply' : arms ? 'arms ready' : 'head ready'};
    }
    return {status: 'todo', summary: twin.applied ? 'applied: check' : 'after the cameras'};
  })();
  return {wrists, head, twin: twinStep};
}

// The step to open on: the first not done.
export function firstStep(statuses) {
  return STEPS.find(step => statuses[step].status !== 'done') || 'twin';
}

// The session's checks for a still on this step: each {text, ok}.
export function sessionChecks(teleop, step) {
  if (step === 'twin') return [];
  const live = teleop?.phase === 'running' && teleop?.receiver_choice?.mode === 'live';
  const recording = teleop?.task?.recording;
  const checks = [{text: step === 'head' ? 'A live session with the head' : 'A live session on the arms', ok: live}];
  if (step === 'head') {
    checks.push({text: 'The head enabled (E)', ok: live && ['HOLD', 'TELEOP'].includes(teleop?.head?.state)});
  } else {
    checks.push({text: 'The arms enabled (E)', ok: live && ['HOLD', 'TELEOP'].includes(teleop?.state)});
  }
  checks.push({text: 'Not recording', ok: !(recording?.active
    || ['starting', 'recording', 'stopping', 'finishing'].includes(recording?.state))});
  return checks;
}

// The one thing to do now on `step` (`role`: the wrist camera chosen): {text, tone, action, label}. `action` is
// 'capture', 'solve', 'save', 'apply' or 'new' (the card's own button for it is the one to press), {step} or
// {role} (the guide's button moves there), or null.
export function nextStep(state, step, role) {
  if (step === 'head') return headStep(state);
  if (step === 'twin') return twinStep(state);
  return wristStep(state, role);
}

const say = (text, tone = 'info', action = null, label = null) => ({text, tone, action, label});

function wristStep(state, role) {
  const calibration = state?.calibration;
  if (!calibration) return say('Calibration runs with Helm on Minerva.', 'muted');
  if (calibration.solving) return say('Solving the wrist cameras: this takes a minute or two.');
  if (calibration.busy) return say('Taking a still: hold the arm still until it is saved.');
  const verdict = solveCurrent(calibration) ? wristVerdict(calibration.result) : null;
  if (verdict?.level === 'good') {
    const px = WRISTS.map(role => fixed(verdict.cameras[role]?.px, 2)).join(' and ');
    return say(`Wrist cameras calibrated (${px} px). Next: the head camera.`, 'good', {step: 'head'}, 'Head camera →');
  }
  if (verdict?.level === 'check') {
    return say('Solved, with something to check below: add stills and Solve again, or go on to the head camera.',
      'warn', {step: 'head'}, 'Head camera →');
  }
  if (verdict) return say('This solve is not usable: see why below, then add stills or start a new run.', 'bad');
  const block = captureBlock(calibration, state.teleop);
  const other = WRISTS.find(wrist => wrist !== role) || WRISTS[0], count = tagged(calibration, role);
  if (count < WRIST_STILLS) {
    if (block) return say(block, 'warn');
    if (!count) {
      return say(`${label(role)}: bring its camera over the table until it sees tags, then pause (pedal B or T): `
        + 'Helm takes the still 1 s later. Or Capture (P) while it holds.', 'info', 'capture');
    }
    const gap = coverageGap(calibration.coverage?.[role]);
    return say(`${label(role)}: ${count} of ${WRIST_STILLS} stills. Next: ${gap ? `tags near the image's ${gap}`
      : 'another distance, tilt or roll'}.`, 'info', 'capture');
  }
  if (tagged(calibration, other) < WRIST_STILLS) {
    return say(`${label(role)} has its ${count} stills. Next: the ${lower(label(other))}.`, 'info', {role: other},
      `${label(other)} →`);
  }
  if (calibration.result) return say(`${since(calibration)} stills since the last solve: Solve again.`, 'info', 'solve');
  return say('Both wrist cameras have their stills: Solve (it takes a minute or two).', 'info', 'solve');
}

function headStep(state) {
  const alignment = state?.head_alignment;
  if (!alignment) return say('Head alignment runs with Helm on Minerva.', 'muted');
  if (alignment.solving) return say('Solving the head camera…');
  if (alignment.busy) return say('Taking a still: hold the head still until it is saved.');
  const verdict = solveCurrent(alignment) ? headVerdict(alignment.result) : null, saved = alignment.saved;
  if (verdict && verdict.level !== 'redo' && saved?.current && saved.run === alignment.run) {
    return say(`Head camera aligned and saved (${fixed(saved.fit?.rms_px, 2)} px). Next: the digital twin.`, 'good',
      {step: 'twin'}, 'Digital twin →');
  }
  if (verdict?.level === 'good') {
    return say(`Head camera aligned (${fixed(alignment.result.rms_px, 2)} px): save it for the headset's 3D view and `
      + 'model.', 'good', 'save');
  }
  if (verdict?.level === 'check') {
    return say('Aligned, with something to check below: save it, or add stills and Solve again.', 'warn', 'save');
  }
  if (verdict) return say('This solve is not usable: see why below, then add stills or start a new run.', 'bad');
  const block = headCaptureBlock(alignment, state.teleop), spread = headSpread(alignment.taken);
  const hint = headHint(spread, headNow(state.teleop)), count = alignment.tagged || 0;
  if (count < HEAD_STILLS || hint) {
    if (block) return say(block, 'warn');
    if (!count) {
      return say('Head: look at the table until the head camera sees three or more tags, then pause (pedal B or T): '
        + 'Helm takes the still 1 s later.', 'info', 'capture');
    }
    return say(`Head: ${count} of ${HEAD_STILLS} stills, turned ${fixed(spread?.turned, 0)}° and tilted `
      + `${fixed(spread?.tilted, 0)}° of ${SPREAD_DEG}°. Next: ${hint || 'another pose'}.`, 'info', 'capture');
  }
  if (alignment.result) return say(`${since(alignment)} stills since the last solve: Solve again.`, 'info', 'solve');
  return say('Head stills done: Solve.', 'info', 'solve');
}

function twinStep(state) {
  const twin = state?.twin;
  if (!twin) return say('The digital twin runs with Helm on Minerva.', 'muted');
  if (twin.busy) return say('Saving the correction…');
  if (twin.applied?.current && !(twin.drawing?.notes || []).length) {
    return say("Applied: the 3D view and the headset's model draw the correction. The calibration is done.", 'good');
  }
  if (twinCanApply(twin)) {
    const {arms, head} = twin.proposal;
    const parts = arms && head ? 'the arms and head' : arms ? 'the arms' : 'the head';
    const later = !head ? " The head's part comes once its alignment is saved (step 2)."
      : !arms ? " The arms' part comes from a usable wrist solve (step 1)." : '';
    return say(`Apply to the model: the 3D view and the headset's model draw ${parts} where the cameras put them `
      + `(drawing only).${later}`, 'info', 'apply');
  }
  const statuses = stepStatus(state), back = statuses.wrists.status === 'done' ? 'head' : 'wrists';
  return say("Nothing to apply yet: solve the wrist cameras (the arms' joint zeros) and save the head camera's "
    + 'alignment (its pan zero).', 'muted', {step: back}, `${STEP_TITLES[back]} →`);
}

// What a step's run warns of: stills far apart in time.
export function stepWarning(state, step) {
  if (step === 'wrists') return gapText(runGaps(state?.calibration?.taken));
  if (step === 'head') return gapText(runGaps(state?.head_alignment?.taken));
  return null;
}

// How each wrist camera has got on: its stills with tags against WRIST_STILLS, and its image coverage.
export function wristProgress(calibration) {
  return WRISTS.map(role => ({role, label: label(role), count: tagged(calibration, role),
    stills: calibration?.counts?.[role]?.stills || 0, target: WRIST_STILLS,
    grid: calibration?.coverage?.[role] || [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
    seen: cellsSeen(calibration?.coverage?.[role])}));
}

// The head's progress: its stills with tags, how far it turned and tilted, the ranges to cover (headTarget), each
// still's pose and the head now.
export function headProgress(alignment, teleop) {
  const spread = headSpread(alignment?.taken), now = headNow(teleop);
  return {count: alignment?.tagged || 0, stills: alignment?.stills || 0, target: HEAD_STILLS, spread,
    turned: spread?.turned || 0, tilted: spread?.tilted || 0, spreadTarget: SPREAD_DEG, box: headTarget(spread, now),
    points: (alignment?.taken || []).filter(still => Number.isFinite(still.pan_deg) && Number.isFinite(still.tilt_deg))
      .map(still => ({name: still.name, pan: still.pan_deg, tilt: still.tilt_deg, tags: still.tags})),
    now};
}

// How to take each step's stills, in a few lines.
export const HOW = {
  wrists: [
    'Live · both (or one arm at a time), the arms enabled (E) and following (T).',
    "Bring a wrist camera over the table until it sees one or more tags, then pause (pedal B or T) and keep still: Helm takes the still 1 s later and the arms hold until it is saved. Capture (P) takes one while the arms hold.",
    `${WRIST_STILLS} stills a camera: vary the distance (15–50 cm), tilt (up to about 45°) and roll (J6), and put tags near the image's corners and edges.`,
    'Keep the table and Minerva where they are until both cameras and the head are done.'],
  head: [
    'A live session with the head (Live · both with it, or Live · Head), the head enabled and following the headset.',
    'Look at the table until the head camera sees three or more tags, then pause (pedal B or T): Helm takes the still 1 s later.',
    `${HEAD_STILLS} stills, turned about ${SPREAD_DEG}° from side to side and tilted about ${SPREAD_DEG}° between nearer and farther on the table.`,
    "Solve, then Save alignment: the headset's 3D view and model use it from the next session."],
  twin: [
    "Made from the newest wrist solve (the arms' joint zeros, when its joint-offset model fits best) and the saved head alignment (its pan zero, against the same board).",
    "Drawing only: the 3D view and the headset's model use it at once; the motors' zeros, control and collision keep the measured angles."],
};
