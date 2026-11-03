// Teleop's mode: what a session is for (the user, 2026-11-01). Direct drives the arms with the leaders and records
// episodes (pedal A). Calibrate takes a camera calibration still a second after each pause (pedal B, or T while the
// arms follow), and records nothing: a still refuses to run beside a recording. Policy runs a trained policy with the
// leaders connected: the operator takes over and hands back (pedal B or O), and pedal A records both, each frame
// labelled (policy corrections). Nothing here moves a motor: the mode chooses Teleop's cards and when a still is taken.
// While a still is taken the arms hold: Resume (T, or a pedal's Resume or Engage) waits until it is saved, here and in
// Helm (helm.teleop STILL_PENDING). The first live still came 3.6 s after its pause, after the operator resumed.
export const MODES = ['direct', 'calibrate', 'policy'];
export const SETTLE_MS = 1000;  // From the pause reaching Hold to the still: the arm and head stop ringing first.
export const PAUSE_WAIT_MS = 3000;  // A pause that has not reached Hold by then was refused; it takes no still.
export const NOTE_MS = 10000;  // How long the card says what the last still found, unless the next pause comes first.
export const NO_RECORDING = 'Calibrate records nothing: choose Direct to record an episode';
export const STILL_PENDING = 'Taking a still: resume once it is saved';

export const knownMode = mode => (MODES.includes(mode) ? mode : 'direct');

// Teleop's input, its own switch beside the mode (docs/development/VR_ARM_TELEOP_PLAN.md): the GELLO leaders, or
// the headset's controllers by grab and drag (helm/teleop.py's headset source). Direct and Calibrate take either; a
// headset session is not recorded yet, so Direct with the headset only drives the arms.
export const INPUTS = ['gello', 'headset'];
export const knownInput = input => (INPUTS.includes(input) ? input : 'gello');

// The mode a session shows: a running policy session is Policy, whatever is chosen.
export function sessionMode(chosen, {policyMode = false} = {}) {
  return policyMode ? 'policy' : knownMode(chosen);
}

// The input a session shows: a running session's own (a headset session reports its arms), else the choice.
export function sessionInput(chosen, teleop) {
  const t = teleop || {};
  if (['running', 'starting'].includes(t.phase)) return t.headset ? 'headset' : 'gello';
  return knownInput(chosen);
}

// Teleop's cards for a mode. A running replay shows its own card; the head alone has no leaders and drives no arms.
// The recording card shows in Direct and in Policy, which records the policy's stretches and the corrections. The
// headset takes the leaders' place: its own card, no trigger feel, nothing recorded.
export function modeCards(mode, {replaying = false, policyMode = false, headOnly = false, input = 'gello'} = {}) {
  const driven = !replaying && !policyMode && !headOnly;
  const headset = driven && mode !== 'policy' && input === 'headset';
  return {
    task: (driven && !headset && mode === 'direct') || (!replaying && !headOnly && mode === 'policy'),
    calibration: !replaying && !policyMode && mode === 'calibrate',
    policyStart: !replaying && !headOnly && mode === 'policy',
    grip: driven && !headset,
    headset,
  };
}

// Whether the operator may pick `input` now, and if not, why. Between sessions, for Direct and Calibrate; a running
// session keeps the input it started with; a replay, Policy and the head alone take none.
export function inputChoice(input, teleop, mode, {headOnly = false} = {}) {
  const t = teleop || {}, active = ['running', 'starting'].includes(t.phase);
  if (t.replay && active) return {allowed: false, reason: 'A replay session runs: its episode drives the arms'};
  if (mode === 'policy') return {allowed: false, reason: 'Policy mode: the policy drives the arms'};
  if (headOnly) return {allowed: false, reason: 'Live · Head steers the head alone: choose Left, Right or Both for the arms'};
  if (active && input !== sessionInput(null, t)) return {allowed: false, reason: 'Stop this session to change the input'};
  return {allowed: true, reason: ''};
}

// Whether the operator may pick `mode` now, and if not, why. Between sessions any mode; a leader session switches
// between Direct and Calibrate (not while a recording runs: its card would hide); a policy or replay session keeps
// its own.
export function modeChoice(mode, teleop, chosen) {
  const t = teleop || {}, active = ['running', 'starting'].includes(t.phase);
  if (!active) return {allowed: true, reason: ''};
  if (t.replay) return {allowed: false, reason: 'A replay session runs; stop it to change the mode'};
  if (t.policy) return {allowed: mode === 'policy', reason: mode === 'policy' ? '' : 'Stop the policy session first'};
  if (mode === 'policy') return {allowed: false, reason: 'Stop this session, then start a policy session'};
  const recording = t.task?.recording || {};
  if (mode !== knownMode(chosen) && (recording.active || ['starting', 'recording', 'stopping', 'finishing']
    .includes(recording.state))) return {allowed: false, reason: 'Finish the recording first'};
  return {allowed: true, reason: ''};
}

// Calibrate's still after a pause. The operator's pause arms it: a pedal B pause the loop confirmed, T pressed
// while the arms follow (keyPause), or the headset's B/Y holding them (the session counts those presses). It is due
// once, SETTLE_MS after the session reports Hold; engaging again, a fault, another mode or the end of the session
// first takes no still. A Hold the operator did not ask for (a camera dropping off USB, a head fault) takes none
// either.
export class PauseStill {
  constructor({settleMs = SETTLE_MS, waitMs = PAUSE_WAIT_MS} = {}) {
    Object.assign(this, {settleMs, waitMs});
    this.session = undefined;
    this.seen = null;
    this.holds = null;  // The headset's B/Y holds counted so far.
    this.reset();
  }

  reset() {
    this.pausedAt = null;  // When the operator paused (pedal B confirmed, or T sent).
    this.holdSince = null;  // When the session first reported Hold after that pause.
  }

  keyPause(teleop, now) {
    if (['ENGAGING', 'TELEOP'].includes(teleop?.state)) this.pausedAt = now;
  }

  // Seconds until the still, or null when none is coming: for the card's countdown.
  remaining(now) {
    return this.holdSince === null ? null : Math.max(0, (this.settleMs - (now - this.holdSince)) / 1000);
  }

  // Called with each state and on a timer; true exactly once, when the still is due.
  due(teleop, now, mode) {
    const t = teleop || {}, session = t.pedals?.session_id ?? null, result = t.pedals?.last_result;
    const press = result ? `${result.press_id}` : null, holds = t.headset?.hold_requests ?? null;
    if (session !== this.session) {
      // A new session, or the first state this page sees: an earlier pause is not this operator's latest.
      this.session = session;
      this.seen = press;
      this.holds = holds;
      this.reset();
      return false;
    }
    if (press !== this.seen) {
      this.seen = press;
      if (result?.pedal === 'b' && result.action === 'pause' && result.accepted !== false) this.pausedAt = now;
    }
    if (holds !== this.holds) {
      if (holds !== null && this.holds !== null && holds > this.holds) this.pausedAt = now;
      this.holds = holds;
    }
    if (mode !== 'calibrate' || t.phase !== 'running' || t.fault || t.recording_fault || this.pausedAt === null) {
      this.reset();
      return false;
    }
    if (t.state !== 'HOLD') {
      // Still following after the pause: wait for Hold, but not forever; any other state ends it.
      if (this.holdSince !== null || !['ENGAGING', 'TELEOP'].includes(t.state) || now - this.pausedAt > this.waitMs)
        this.reset();
      return false;
    }
    if (this.holdSince === null) this.holdSince = now;
    if (now - this.holdSince < this.settleMs) return false;
    this.reset();
    return true;
  }
}

// Whether a still is being taken: this page's request is out, or Helm says a wrist or head still runs (another page's).
export function stillTaking({pending = false, calibration = null, headAlignment = null} = {}) {
  return Boolean(pending || calibration?.busy || headAlignment?.busy);
}

const cameraName = role => role === 'head' ? 'the head camera' : role === 'right_wrist' ? 'the right wrist camera'
  : 'the left wrist camera';

// What the last still found, for the card's line: the still Helm kept (its summary, helm.calibration
// _still_summary), or why there is none.
export function stillOutcome(last, error, role) {
  if (error) return `No still: ${error}`;
  if (!last) return null;
  if (!last.tags?.length) return `${last.name} saved, but it saw no board tags: aim ${cameraName(role)} at the board, `
    + 'then pause again (Undo drops it).';
  return `${last.name} saved: tag${last.tags.length === 1 ? '' : 's'} ${last.tags.join(', ')}. Resume for the next pose.`;
}

// The calibration card's line for the mode: the countdown while a still is coming, that it is being taken, what the
// last one found (`note`, for NOTE_MS), else what a pause does (`input`: the headset's B/Y pauses too).
export function pauseStillText(remaining, role, {taking = false, note = null, input = 'gello'} = {}) {
  const camera = cameraName(role), pauses = input === 'headset' ? 'B/Y, pedal B or T' : 'pedal B or T';
  if (remaining !== null) return `Still of ${camera} in ${remaining.toFixed(1)} s: hold still.`;
  if (taking) return `Taking a still of ${camera}: hold still. Resume once it is saved.`;
  return note || `Calibrate: each pause (${pauses}) takes a still of ${camera} ${SETTLE_MS / 1000} s later, and `
    + 'the arms hold until it is saved. Resume for the next pose.';
}

// The headset card's line for an arm (helm/teleop.py's headset view: the worker's latest answer for it). `tone` is
// '' when all is well, 'near' when the arm holds, moves on its own or is held back, 'bad' when it cannot follow the
// hand. `stage`: 'held' before Engage, 'aligning' while Engage aligns (`raising`: a hanging arm rises to its ready
// pose), 'following' once it follows. Until following begins nothing tracks: the worker is seeded then. A trigger is
// reported only for a fresh controller, so it tells one is there.
export function headsetArm(arm, {stage = 'following', raising = false} = {}) {
  const a = arm || {};
  const trigger = a.trigger == null ? '' : ` · gripper ${Math.round(a.trigger * 100)}% closed`;
  if (stage === 'aligning') return raising
    ? {state: 'Raising', detail: 'To its ready pose at the alignment speed: forearm forward, gripper level', tone: 'near'}
    : {state: 'Aligning', detail: `It stays where it is; the gripper goes to its trigger${trigger}`, tone: ''};
  if (stage !== 'following') return a.trigger == null
    ? {state: 'No controller', detail: 'Enter VR on the Quest: each controller drives its own arm.', tone: ''}
    : {state: 'Ready', detail: `Engage (T), then hold the grip to drag this arm${trigger}`, tone: ''};
  if (a.tracking == null) return {state: 'Waiting for the headset', detail: '', tone: ''};
  if (!a.tracking) return {state: 'No controller', detail: 'The arm holds where it is.', tone: 'near'};
  if (!a.gripped) return {state: 'Free', detail: `Hold the grip to drag this arm${trigger}`, tone: ''};
  const notes = [];
  if (a.converged === false) notes.push('out of reach: the arm stops short of the hand');
  if (a.near_singular) notes.push('nearly straight: lift the hand a little before reaching');
  if ((a.at_limit || []).length) notes.push(`at its limit: ${a.at_limit.map(name => name.replace(/^.*_j/, 'J')).join(', ')}`);
  if (a.clamped) notes.push('held back to its box or speed');
  return {state: 'Dragging', detail: (notes.join(' · ') || 'Following the hand') + trigger,
    tone: a.converged === false ? 'bad' : notes.length ? 'near' : ''};
}
