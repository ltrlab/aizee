// Teleop's mode: what a session is for (the user, 2026-11-01). Direct drives the arms with the leaders and records
// episodes (pedal A). Calibrate takes a camera calibration still a second after each pause (pedal B, or T while the
// arms follow), and records nothing: a still refuses to run beside a recording. Policy runs a trained policy in place
// of the leaders (policy mode). Nothing here moves a motor: the mode chooses Teleop's cards and when a still is taken.
export const MODES = ['direct', 'calibrate', 'policy'];
export const SETTLE_MS = 1000;  // From the pause reaching Hold to the still: the arm and head stop ringing first.
export const PAUSE_WAIT_MS = 3000;  // A pause that has not reached Hold by then was refused; it takes no still.
export const NO_RECORDING = 'Calibrate records nothing: choose Direct to record an episode';

export const knownMode = mode => (MODES.includes(mode) ? mode : 'direct');

// The mode a session shows: a running policy session is Policy, whatever is chosen.
export function sessionMode(chosen, {policyMode = false} = {}) {
  return policyMode ? 'policy' : knownMode(chosen);
}

// Teleop's cards for a mode. A running replay shows its own card; the head alone has no leaders and drives no arms.
export function modeCards(mode, {replaying = false, policyMode = false, headOnly = false} = {}) {
  const leaders = !replaying && !policyMode && !headOnly;
  return {
    task: leaders && mode === 'direct',
    calibration: !replaying && !policyMode && mode === 'calibrate',
    policyStart: !replaying && !headOnly && mode === 'policy',
    grip: leaders,
  };
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

// Calibrate's still after a pause. The operator's pause arms it: a pedal B pause the loop confirmed, or T pressed
// while the arms follow (keyPause). It is due once, SETTLE_MS after the session reports Hold; engaging again,
// a fault, another mode or the end of the session first takes no still. A Hold the operator did not ask for (a camera
// dropping off USB, a head fault) takes none either.
export class PauseStill {
  constructor({settleMs = SETTLE_MS, waitMs = PAUSE_WAIT_MS} = {}) {
    Object.assign(this, {settleMs, waitMs});
    this.session = undefined;
    this.seen = null;
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
    const press = result ? `${result.press_id}` : null;
    if (session !== this.session) {
      // A new session, or the first state this page sees: an earlier pause is not this operator's latest.
      this.session = session;
      this.seen = press;
      this.reset();
      return false;
    }
    if (press !== this.seen) {
      this.seen = press;
      if (result?.pedal === 'b' && result.action === 'pause' && result.accepted !== false) this.pausedAt = now;
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

// The calibration card's line for the mode: what a pause does, and the countdown while a still is coming.
export function pauseStillText(remaining, role) {
  const camera = role === 'head' ? 'the head camera' : role === 'right_wrist' ? 'the right wrist camera'
    : 'the left wrist camera';
  return remaining === null
    ? `Calibrate: each pause (pedal B or T) takes a still of ${camera} ${SETTLE_MS / 1000} s later. Resume for the next pose.`
    : `Still of ${camera} in ${remaining.toFixed(1)} s: hold still.`;
}
