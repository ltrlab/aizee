// Pedals are guarded backend offers, never a browser-side motion sequence.
export const STATE_MAX_AGE_MS = 750;
const running = (t) => ['running', 'starting'].includes(t?.phase);
// How old the loop's latest word is. Pedals act on its lifecycle state and offers, which its status carries at least
// every 250 ms; joint observations can lag (Idle has no time to rebuild them) without making those stale. An older
// Helm reports only the observation age.
export const stateAge = (t) => (Number.isFinite(t?.status_age_ms) ? t.status_age_ms : t?.observation_age_ms);
export function presentation(state, fresh = true) {
  const t = state?.teleop || {},
    fault = t.fault || t.recording_fault || (t.phase === 'fault' ? t.message || 'Session fault' : '');
  const stale = !fresh || (running(t) && Number.isFinite(stateAge(t)) && stateAge(t) >= STATE_MAX_AGE_MS);
  if (stale)
    return {
      tone: 'fault',
      label: 'State unavailable',
      detail: 'Connection lost or stale. Pedals locked; H / X remain available.',
      stage: '',
    };
  if (fault) return { tone: 'fault', label: 'Fault · reset required', detail: String(fault), stage: '' };
  if (!running(t))
    return {
      tone: 'off',
      label: 'Session stopped',
      detail: 'Touch Start session first. Pedals cannot start a session.',
      stage: 'off',
    };
  if (t.phase === 'starting')
    return {
      tone: 'off',
      label: 'Starting session',
      detail: 'Waiting for the control worker. No pedal action yet.',
      stage: 'off',
    };
  // A live replay follows a recorded episode, policy mode a policy, and the Headset input the controllers, in place
  // of the leaders. With takeover (policy corrections) the operator's leaders drive after a take over, until they
  // hand back. With the headset Engage raises a hanging arm to its ready pose; the grippers go to the triggers.
  const you = t.policy?.takeover && t.policy.driver === 'leaders',
    takeover = !!t.policy?.takeover;
  const followed = t.replay
    ? "the episode's pose"
    : t.policy && !you
      ? "the policy's first action"
      : t.headset
        ? 'the ready pose (a hanging arm rises) and the triggers'
        : 'the leaders';
  const states = {
    DISABLED: [
      'off',
      'Disabled',
      t.pedals?.actions?.c?.enabled === false && t.pedals.actions.c.reason
        ? t.pedals.actions.c.reason
        : 'C: Idle powers the motors with zero torque.',
      'off',
    ],
    IDLE: ['off', 'Idle', 'Motors on, zero torque. C: Enable holding.', 'idle'],
    HOLD: [
      'hold',
      t.pedals?.paused ? 'Hold · paused' : 'Hold · enabled',
      takeover
        ? `Holding position; motors stay enabled. ${you ? 'You drive' : 'The policy drives'} after Resume.`
        : 'Holding position; motors stay enabled.',
      'hold',
    ],
    ENGAGING: ['engaging', 'Engaging · aligning', `Aligning to ${followed}. Waiting for reported Teleop.`, 'align'],
    TELEOP: t.replay
      ? ['teleop', 'Replay · following', 'Following the recorded episode. B pauses in Hold.', 'teleop']
      : you
        ? [
            'teleop',
            'Correcting · you drive',
            'Following both leaders: a correction. B hands back through a Hold.',
            'teleop',
          ]
        : t.policy
          ? [
              'teleop',
              'Policy · driving',
              takeover
                ? 'The policy drives the arms. B takes over through a Hold.'
                : 'The policy drives the arms. B pauses in Hold.',
              'teleop',
            ]
          : t.headset
            ? ['teleop', 'Headset · following', 'A held grip drags its arm. B/Y or pedal B pauses in Hold.', 'teleop']
            : ['teleop', 'Teleop · following', 'Following both leaders. B pauses in Hold.', 'teleop'],
    SHUTDOWN: [
      'shutdown',
      'Shutdown · returning',
      'Returning to the reference, then disabling. Keep the path clear.',
      'shutdown',
    ],
    FAULT: ['fault', 'Fault · reset required', t.message || 'Stop the session and inspect the fault.', ''],
  };
  const [tone, label, detail, stage] = states[t.state] || [
    'off',
    'Waiting for state',
    'No lifecycle state reported.',
    '',
  ];
  return { tone, label, detail, stage };
}
export function recordingLabel(task) {
  task = task || {}; // Helm reports task: null until a recording task is set up.
  const r = task.recording || {},
    cmd = task.last_command || {};
  if (r.error || task.error || ['failed', 'aborted'].includes(r.state)) return 'Recording incomplete';
  if (cmd.state === 'pending') {
    if (cmd.action === 'stop_record') return 'Finishing recording…';
    if (cmd.action === 'start_record') return 'Starting recording…';
  }
  if (r.state === 'finishing') return 'Finishing recording…';
  if (r.active || r.state === 'recording') return 'Recording';
  if (r.complete === true) return 'Complete · saved';
  return 'Not recording';
}
// The recording indicator's mode: off, starting, recording, finishing (saving), saved or failed.
export function recordingMode(task) {
  task = task || {};
  const r = task.recording || {},
    cmd = task.last_command || {};
  if (r.error || task.error || ['failed', 'aborted'].includes(r.state)) return 'failed';
  if (cmd.state === 'pending' && cmd.action === 'stop_record') return 'finishing';
  if (cmd.state === 'pending' && cmd.action === 'start_record') return 'starting';
  if (['finishing', 'stopping'].includes(r.state)) return 'finishing';
  if (r.state === 'starting') return 'starting';
  if (r.active || r.state === 'recording') return 'recording';
  if (r.complete === true) return 'saved';
  return 'off';
}
export function editableTarget(target) {
  return !!target?.closest?.(
    'input, textarea, select, [contenteditable]:not([contenteditable="false"]), [role="textbox"]',
  );
}
export class PedalControls {
  constructor({ request, onState, onError, context, onChange, now = () => performance.now() }) {
    Object.assign(this, { request, onState, onError, context, onChange, now });
    this.state = null;
    this.receivedAt = -Infinity;
    this.connected = false;
    this.held = new Set();
    this.pending = new Map();
    this.pressId = 0;
    this.session = null;
    this.blockedUntilRefresh = false;
    this.testMode = false;
    this.testCounts = { a: 0, b: 0, c: 0 };
    this.testLast = '';
  }
  update(state, receivedAt = this.now()) {
    const session = state?.teleop?.pedals?.session_id;
    if (session !== this.session) {
      this.session = session;
      this.pressId = 0;
      this.pending.clear();
      // Held keys survive session changes: a new session still needs a release.
    }
    this.state = state;
    this.receivedAt = receivedAt;
    this.connected = true;
    if (!this.canTest()) this.setTest(false);
    this.blockedUntilRefresh = false;
    this.onChange?.();
  }
  canTest() {
    const s = this.state;
    return (
      s?.teleop?.phase === 'stopped' &&
      !s.teleop.fault &&
      !s.teleop.recording_fault &&
      s.teleop.state !== 'FAULT' &&
      !['starting', 'running', 'fault'].includes(s?.tune?.phase) &&
      !s?.check?.running &&
      !s?.gripper_zero?.running
    );
  }
  setTest(enabled) {
    this.testMode = enabled && this.canTest() && this.fresh();
    this.testCounts = { a: 0, b: 0, c: 0 };
    this.testLast = '';
    this.onChange?.();
  }
  disconnect() {
    this.connected = false;
    this.setTest(false);
    this.onChange?.();
  }
  fresh() {
    const t = this.state?.teleop,
      age = running(t) && Number.isFinite(stateAge(t)) ? Math.max(0, stateAge(t)) : 0;
    return this.connected && age + this.now() - this.receivedAt < STATE_MAX_AGE_MS;
  }
  offer(pedal) {
    const t = this.state?.teleop || {},
      p = t.pedals,
      action = p?.actions?.[pedal];
    const context = this.context();
    let reason = '';
    if (this.testMode) return { ...action, enabled: false, reason: 'Test only · no commands sent.' };
    if (!this.fresh() || presentation(this.state, this.fresh()).label === 'State unavailable')
      reason = 'State is stale; wait for a fresh update.';
    else if (context.tab !== 'teleop' || context.hidden || context.confirming)
      reason = 'Use pedals in the visible Teleop workspace.';
    else if (t.phase !== 'running') reason = 'Touch Start session first and wait for it to start.';
    else if (!p || !action || !Number.isSafeInteger(action.revision) || !p.session_id)
      reason = 'Pedal offers unavailable.';
    else if (this.blockedUntilRefresh) reason = 'Wait for a fresh state after the last response.';
    else if (this.pending.has(pedal) || (pedal !== 'a' && [...this.pending.keys()].some((k) => k !== 'a')))
      reason = 'Motion input pending.';
    // Teleop's mode may record nothing (Calibrate); finishing a recording is never refused.
    else if (pedal === 'a' && context.noRecording && action.action === 'start_record') reason = context.noRecording;
    // A camera still is being taken: the arms hold until it is saved, so nothing sets them moving (Helm refuses too).
    else if (context.stillPending && ['engage', 'resume'].includes(action.action)) reason = context.stillPending;
    else if (!action.enabled) reason = action.reason || 'Action unavailable.';
    return { ...action, enabled: !reason && action?.enabled === true, reason: reason || action?.reason || '' };
  }
  async press(pedal) {
    const offer = this.offer(pedal);
    if (!offer.enabled) return false;
    const session = this.session,
      id = ++this.pressId;
    const payload = { session_id: session, revision: offer.revision, press_id: id, pedal };
    this.pending.set(pedal, { id, action: offer.action });
    this.onChange?.();
    try {
      const state = await this.request('/api/helm/teleop/pedal', payload);
      // A late response from an old session must not replace the current display.
      if (this.session === session) {
        this.onState(state);
        const result = state?.teleop?.pedals?.last_result;
        if (result?.press_id === id && result.accepted === false)
          this.onError?.(result.error || 'Pedal action refused.');
      }
    } catch (error) {
      if (this.session === session) {
        this.blockedUntilRefresh = true;
        this.onError?.(error.message);
      }
    } finally {
      if (this.session === session && this.pending.get(pedal)?.id === id) this.pending.delete(pedal);
      this.onChange?.();
    }
    return true;
  }
  keydown(event) {
    const pedal = event.key?.toLowerCase();
    if (!['a', 'b', 'c'].includes(pedal)) return;
    const already = this.held.has(pedal);
    this.held.add(pedal);
    if (
      already ||
      event.repeat ||
      event.isComposing ||
      event.keyCode === 229 ||
      event.ctrlKey ||
      event.metaKey ||
      event.altKey ||
      event.shiftKey ||
      editableTarget(event.target)
    )
      return;
    const context = this.context();
    if (
      this.testMode &&
      this.canTest() &&
      this.fresh() &&
      context.pedalTest &&
      !context.hidden &&
      !context.confirming
    ) {
      event.preventDefault();
      this.testCounts[pedal]++;
      this.testLast = pedal;
      this.onChange?.();
      return;
    }
    if (!this.offer(pedal).enabled) return;
    event.preventDefault();
    void this.press(pedal);
  }
  keyup(event) {
    this.held.delete(event.key?.toLowerCase());
  }
  // Do not clear held keys on blur/visibility: reconnecting must not reinterpret a held pedal.
}

// Response order and worker time both matter: a delayed green snapshot cannot replace a fault.
export class StateOrder {
  constructor() {
    this.requestId = 0;
    this.session = null;
    this.statusAt = -Infinity;
    this.faulted = false;
  }
  accept(state, requestId) {
    if (requestId <= this.requestId) return false;
    const t = state?.teleop,
      session = t?.pedals?.session_id || null;
    const status = Number.isFinite(t?.status_at) ? t.status_at : t?.observed_at;
    const faulted = !!(t?.fault || t?.recording_fault || t?.phase === 'fault' || t?.state === 'FAULT');
    if (session && session === this.session) {
      if (Number.isFinite(status) && status < this.statusAt) return false;
      // Parent-side failure can retain the last worker timestamp. It still latches until a new session.
      if (this.faulted && running(t) && !faulted) return false;
    } else this.faulted = false;
    this.faulted = this.faulted || faulted;
    this.requestId = requestId;
    this.session = session;
    this.statusAt = Number.isFinite(status) ? status : -Infinity;
    return true;
  }
}
