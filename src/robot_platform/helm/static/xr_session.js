// The headset running Helm's teleop session (xr.js): what its ring offers (xr_ring.js) from Helm's state, the status in
// the ring's middle, the one-line alerts with the ring closed, and the requests that start, step and stop a session,
// as the Teleop tab's buttons send them (helm.js). Starting from the headset takes Helm's control from the device that
// has it (helm.access): that device's page goes view-only, still showing the session, and can still Disable (H) or
// Shut down (X). A live start, and each step that powers or moves the robot, asks for the ring's hold with the Teleop
// tab's own confirmation. No DOM and no three.js, so node tests it.

export const POLL_MS = 500;
export const ALERT_S = 5;  // A passing alert (a hold, a refusal) shows this long; a fault stays until the ring opens.
const ENGAGED = ['ENGAGING', 'TELEOP'];
const POWERED = ['HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN'];
const RECEIVERS = {virtual: 'Virtual', left: 'Left arm', right: 'Right arm', both: 'Both arms', head: 'Head'};

const running = teleop => ['running', 'starting'].includes(teleop?.phase);

// The receiver a running session drives, as a word: 'LIVE both arms', 'Virtual'.
export function receiverWord(teleop) {
  const live = teleop?.live?.arm || (teleop?.receiver_choice?.mode === 'live' ? teleop.receiver_choice.arm : null);
  if (!live) return 'Virtual';
  return `LIVE ${live === 'head' ? 'head' : live === 'both' ? 'both arms' : `${live} arm`}`;
}

// What taking control would take, for a confirmation: null when this device has it (or Helm has no logins).
function takes(access) {
  if (!access?.enabled || access.in_control) return null;
  return access.holder ? `This takes Helm's control from ${access.holder.device}: its page goes view-only.`
    : "This takes Helm's control.";
}

// The lines a live start confirms, as the Teleop tab asks them.
function startLines(choice, access, collision) {
  const control = takes(access);
  if (choice === 'virtual') return control ? [control, 'No motors move.'] : null;
  const lines = [];
  if (choice === 'head') {
    lines.push("The head's servos are powered at Idle (I); Engage (T) raises it slowly toward the headset.",
      "Nothing is in the head's way; Disable (H) removes its torque at once.", 'The arms are left as they are.');
  } else {
    const arms = choice === 'both' ? 'Both arms' : `The ${choice} arm`;
    lines.push(`${arms}' motors are powered at Idle (I). The foot switch (e-stop) is in reach.`,
      `${arms} hang straight down in the reference pose, grippers closed.`,
      collision === 'disabled' ? 'Collision detection is disabled.'
        : choice === 'both' ? 'One arm is checked against the other: the hands stop about 30 cm apart.'
          : 'Collision checks are advisory.');  // Engage's own confirmation says how it raises the arms.
  }
  return control ? [control, ...lines] : lines;
}

// The next step of a running session: [label, key, confirmation lines or null], or null for none.
function nextStep(teleop) {
  const live = !!teleop.live, head = teleop.live?.arm === 'head';
  switch (teleop.state) {
    case 'DISABLED':
      return ['Idle (I)', 'I', live ? [head ? "The head's servos are powered, limp." : 'The motors are powered at zero '
        + 'torque: the arms hang limp.', 'The foot switch (e-stop) is in reach.'] : null];
    case 'IDLE':
      return ['Enable (E)', 'E', live ? [head ? 'The head holds where it rests.' : 'The arms hold where they hang, '
        + 'under torque.'] : null];
    case 'HOLD':
      return ['Engage (T)', 'T', live ? (head ? ['The head rises slowly toward the headset, then follows it.']
        : ['Each arm hanging at the reference rises to its ready pose: the forearm swings forward, then up; the '
          + 'gripper ends level about 28 cm ahead and 24 cm higher.', 'Keep the space in front of the hands clear. '
          + 'Then a held grip drags its arm at 1:1; letting go holds it.']) : null];
    case 'ENGAGING':
    case 'TELEOP':
      return ['Hold (T)', 'T', null];
    default:
      return null;
  }
}

// The ring's level at `path` from Helm's state (`state`: /api/helm/state) and the headset's view (xr_view.js), or
// null for a submenu that no longer exists.
export function ringMenu(state, path, view) {
  const teleop = state?.teleop || null, access = state?.access || {enabled: false};
  const controlled = !access.enabled || access.in_control;
  const viewOnly = controlled ? null : `View only: ${access.holder ? `${access.holder.device} has control`
    : 'no device has control'}. Take control first.`;
  if (!path.length) {
    const items = [];
    if (!teleop) {
      items.push({id: 'none', label: 'Session', disabled: 'Helm runs without a teleop profile'});
    } else if (teleop.phase === 'starting') {
      items.push({id: 'starting', label: 'Starting…', disabled: 'The session is starting'});
    } else if (!running(teleop)) {
      items.push({id: 'start', label: 'Start', detail: 'session', submenu: true});
    } else {
      const step = teleop.fault ? null : nextStep(teleop);
      items.push(step ? {id: `key:${step[1]}`, label: step[0], confirm: step[2] || undefined,
        disabled: controlled ? undefined : viewOnly, tone: step[1] === 'T' && ENGAGED.includes(teleop.state) ? 'warn'
          : 'ok'} : {id: 'step', label: teleop.fault ? 'Faulted' : teleop.state || '…',
        disabled: teleop.fault ? `Fault: ${teleop.fault}. End the session and start again.` : 'No step from here'});
      const powered = !!teleop.live && POWERED.includes(teleop.state);
      items.push({id: 'key:X', label: 'Shut down (X)', detail: 'lowers first',
        disabled: ['IDLE', 'HOLD', 'ENGAGING', 'TELEOP'].includes(teleop.state) ? undefined : 'Nothing to shut down'});
      items.push({id: 'stop', label: 'End session', tone: 'warn', disabled: controlled ? undefined : viewOnly,
        confirm: powered ? [`${receiverWord(teleop)} is powered (${teleop.state}).`, teleop.live.arm === 'head'
          ? 'It returns to its zero first, then its torque goes.'
          : 'The arms are lowered slowly to the reference first, then disabled.'] : undefined});
      const torque = !['DISABLED', 'IDLE'].includes(teleop.state);
      items.push({id: 'key:H', label: 'Disable (H)', tone: 'bad', confirm: torque && teleop.live ? [
        'Removes torque at once: the arms drop where they are.', 'Hold (T, or B/Y) or Shut down (X) lower them '
          + 'gently instead.'] : undefined});
    }
    items.push({id: 'view', label: 'View', submenu: true});
    items.push({id: 'zero', label: 'Zero head', detail: 'where you look'});
    if (!controlled) items.push({id: 'claim', label: 'Take control', tone: 'warn', confirm: [takes(access)]});
    return {title: 'Minerva', items};
  }
  if (path.length === 1 && path[0] === 'start') {
    if (!teleop || running(teleop)) return null;
    const choices = teleop.receiver_choices || ['virtual'];
    const items = Object.entries(RECEIVERS).filter(([choice]) => choices.includes(choice)).map(([choice, label]) => {
      const live = choice !== 'virtual';
      return {id: `start:${choice}`, label, detail: live ? 'LIVE' : 'no motors', tone: live ? 'warn' : 'ok',
        disabled: live && !teleop.live_available ? 'Live sessions need Helm on Minerva' : undefined,
        confirm: startLines(choice, access, teleop.collision?.mode) || undefined};
    });
    return {title: 'Start', items};
  }
  if (path.length === 1 && path[0] === 'view') {
    const flag = (id, label, on) => ({id: `view:${id}`, label, detail: on ? 'on' : 'off', tone: on ? 'ok' : '',
      stay: true});
    return {title: 'View', items: [
      flag('follow', 'Follow head', view.follow), flag('depth', '3D', view.depth), flag('model', 'Model', view.model),
      flag('hands', 'Wrists', view.hands), flag('map', 'Map', view.map),
      {id: 'slider:opacity', label: 'Picture', detail: `${Math.round(view.opacity * 100)} %`, slider: view.opacity},
      {id: 'slider:wristOpacity', label: 'Wrists', detail: `${Math.round(view.wristOpacity * 100)} %`,
        slider: view.wristOpacity},
      {id: 'recenter', label: 'Recenter', detail: 'picture'},
    ]};
  }
  return null;
}

// The ring's middle: [text, tone] lines, the session first, then `more` (the head, the camera, the wrists).
export function statusLines(state, link, more = []) {
  const teleop = state?.teleop, access = state?.access || {enabled: false};
  const lines = [];
  if (!link.connected) lines.push([link.message || 'Helm: reconnecting…', 'warn']);
  else if (link.error) lines.push([link.error, 'bad']);
  if (!teleop || !running(teleop)) {
    lines.push([teleop?.phase === 'stopped' || !teleop ? 'No session' : teleop.message || 'No session', '']);
  } else {
    lines.push([`${receiverWord(teleop)} · ${teleop.state || teleop.phase}`, teleop.fault ? 'bad'
      : ENGAGED.includes(teleop.state) ? 'ok' : 'warn']);
    if (teleop.fault) lines.push([`Fault: ${teleop.fault}`, 'bad']);
  }
  if (access.enabled && !access.in_control) {
    lines.push([`View only: ${access.holder ? `${access.holder.device} has control` : 'no device has control'}`,
      'warn']);
  }
  lines.push(...more.filter(Boolean));
  return lines;
}

// The alert with the ring closed, as {text, tone, sticky} or null: a fault (it stays), control lost during a
// session, the arms falling into a hold from following (a trip or a camera drop: the loop's message says which).
// `previous`: the state before this one, for the hold.
export function alertOf(state, previous, link) {
  const teleop = state?.teleop, before = previous?.teleop;
  if (!link.connected) return {text: link.message || 'Helm: reconnecting…', tone: 'warn', sticky: false};
  if (teleop?.fault && teleop.fault !== before?.fault) return {text: `Fault: ${teleop.fault}`, tone: 'bad', sticky: true};
  if (teleop?.head?.fault && teleop.head.fault !== before?.head?.fault) {
    return {text: `Head fault: ${teleop.head.fault}`, tone: 'bad', sticky: true};
  }
  const access = state?.access, had = previous?.access;
  if (running(teleop) && access?.enabled && had?.in_control && !access.in_control) {
    return {text: `View only: ${access.holder?.device || 'another device'} took control`, tone: 'warn', sticky: true};
  }
  if (teleop?.state === 'HOLD' && ENGAGED.includes(before?.state)) {
    return {text: `Holding${teleop.message ? `: ${teleop.message}` : ''}`, tone: 'warn', sticky: false};
  }
  return null;
}

// Helm's requests from the headset: `fetch` (the browser's), its action token, and what to do with each new state.
export class SessionControl {
  constructor({fetch: get = (...args) => fetch(...args), token = () => null, onState = () => {}} = {}) {
    Object.assign(this, {get, token, onState});
    this.state = null;
    this.busy = false;  // A start, step or stop under way: the ring waits for it.
  }

  async request(path, payload) {
    const options = payload === undefined ? {cache: 'no-store'} : {method: 'POST', cache: 'no-store',
      body: JSON.stringify(payload), headers: {'Content-Type': 'application/json', 'X-Helm-Token': this.token() || ''}};
    const response = await this.get(path, options);
    let data = null;
    try {
      data = await response.json();
    } catch {
      data = null;
    }
    if (!response.ok) throw new Error(data?.error || `Helm returned HTTP ${response.status}`);
    return data;
  }

  async refresh() {
    this.take(await this.request('/api/helm/state'));
    return this.state;
  }

  take(state) {
    if (!state || typeof state !== 'object') return;
    const previous = this.state;
    this.state = state;
    this.onState(state, previous);
  }

  // A ring item's session action: start:<receiver>, key:<I|E|T|X|H>, stop or claim. Resolves to the state after it;
  // a refusal rejects with Helm's words.
  async act(id) {
    if (this.busy) throw new Error('Wait: Helm is still answering the last one');
    this.busy = true;
    try {
      const [kind, value] = id.split(':');
      const access = this.state?.access;
      const claim = async () => {
        if (access?.enabled && !access.in_control) await this.request('/api/helm/control/claim', {});
      };
      if (kind === 'claim') {
        await claim();
        return await this.refresh();
      }
      if (kind === 'start') {
        if (!RECEIVERS[value]) throw new Error(`No receiver ${value}`);
        await claim();
        await this.request('/api/helm/teleop/receiver', value === 'virtual' ? {mode: 'virtual'}
          : {mode: 'live', arm: value});
        const live = value !== 'virtual';
        this.take(await this.request('/api/helm/teleop/start', {...(value === 'head' ? {} : {headset: true}),
          ...(live ? {confirm: true} : {})}));
      } else if (kind === 'key') {
        if (!['I', 'E', 'T', 'X', 'H'].includes(value)) throw new Error(`No key ${value}`);
        this.take(await this.request('/api/helm/teleop/key', {key: value}));
      } else if (kind === 'stop') {
        this.take(await this.request('/api/helm/teleop/stop', {}));
      } else {
        throw new Error(`No session action ${id}`);
      }
      return this.state;
    } finally {
      this.busy = false;
    }
  }
}
