import {SettingsPanel} from './settings.js';

// Helm operator page. Reads /api/helm/state; every action needs the session token.
const $ = id => document.getElementById(id);
const DEG = 180 / Math.PI;
const cards = new Map();
let token = null, last = null, busy = false, lockNoticeShown = false;
let pendingConfirm = null;  // An open live confirmation: {kind: 'start' | 'stop', arm}

const signed = (q, digits = 3) => (q == null || !Number.isFinite(q)) ? '—' : `${q < 0 ? '−' : '+'}${Math.abs(q).toFixed(digits)}`;
const range = entry => `${signed(entry.min_rad, 2)} … ${signed(entry.max_rad, 2)} rad`;

async function request(path, payload, timeoutMs = 15000) {
  const options = payload === undefined
    ? {cache: 'no-store', signal: AbortSignal.timeout(3000)}
    : {method: 'POST', cache: 'no-store', signal: AbortSignal.timeout(timeoutMs), body: JSON.stringify(payload),
       headers: {'Content-Type': 'application/json', 'X-Helm-Token': token || ''}};
  const response = await fetch(path, options);
  let data = null;
  try { data = await response.json(); } catch { data = null; }
  if (!response.ok) throw new Error(data?.error || `Helm returned HTTP ${response.status}`);
  return data;
}

const STATUS = {check: 'check-status', limits: 'action-status', leaders: 'leaders-status', teleop: 'teleop-status',
  tune: 'tune-status', settings:'settings-status'};

function say(text, kind = '') {
  const status = $(STATUS[document.body.dataset.tab] || 'action-status');
  status.textContent = text;
  status.className = kind;
}

async function act(path, payload, done) {
  if (busy) return;
  busy = true;
  try {
    render(await request(path, payload));
    if (done) say(typeof done === 'function' ? done() : done, 'good');
  } catch (error) {
    say(error.message, 'bad');
  } finally {
    busy = false;
  }
}

function build(row) {
  const card = document.createElement('div');
  card.className = 'joint';
  card.innerHTML = `<div class="joint-top"><div><span class="joint-name"></span><span class="motor"></span></div>
      <span class="direction"><span class="chip" title="Motor-to-model direction"></span>
        <button class="flip" title="The model moves the opposite way: reverse it">Flip</button>
        <button class="accept" title="The model moves the same way as the arm">Looks right</button></span></div>
    <div class="bar" aria-hidden="true"><span class="zero"></span><span class="saved" hidden></span><span class="held" hidden></span><span class="now" hidden></span></div>
    <div class="numbers"><span>now<b class="v-now"></b></span><span>min<b class="v-min"></b></span><span>max<b class="v-max"></b></span><span>span<b class="v-span"></b></span></div>
    <div class="joint-foot"><span class="status" role="status"></span><button class="reset">Reset</button><button class="confirm primary">Confirm</button></div>`;
  card.querySelector('.joint-name').textContent = row.label;
  card.querySelector('.motor').textContent = row.name;
  card.querySelector('.reset').onclick = () =>
    act('/api/helm/limits', {action: 'reset', motor: row.name}, `Cleared the sweep for ${row.label}.`);
  card.querySelector('.confirm').onclick = () => {
    const undo = ['confirmed', 'changed'].includes(card.dataset.status);
    act('/api/helm/limits', {action: undo ? 'unconfirm' : 'confirm', motor: row.name},
      undo ? `${row.label} is no longer confirmed.` : `${row.label} confirmed. Save when you are done.`);
  };
  if (row.direction && row.direction.editable !== false) {
    card.querySelector('.flip').onclick = () =>
      act('/api/helm/direction', {action: 'flip', joint: row.direction.joint}, `Reversed the model direction for ${row.label}.`);
    card.querySelector('.accept').onclick = () =>
      act('/api/helm/direction', {action: 'accept', joint: row.direction.joint}, `Direction confirmed for ${row.label}.`);
  } else {
    card.querySelector('.flip').remove();
    card.querySelector('.accept').remove();
  }
  return card;
}

function place(element, from, to, extent) {
  const at = q => 50 + Math.max(-1, Math.min(1, q / extent)) * 50;
  element.hidden = false;
  element.style.left = `${at(from)}%`;
  if (to !== undefined) element.style.width = `${Math.max(at(to) - at(from), .6)}%`;
}

function update(card, row, live) {
  const {saved, confirmed} = row;
  const untouched = !row.samples || (row.span_rad != null && row.span_rad < .02);
  const problem = untouched ? 'Not swept yet: move it to each end and hold there' : row.problem;
  const sameAsSaved = saved && Number.isFinite(row.min_rad)
    && row.min_rad >= saved.min_rad - .005 && row.max_rad <= saved.max_rad + .005;
  let status, text;
  if (confirmed && row.changed_since_confirmed) [status, text] = ['changed', 'The range grew after confirming. Confirm again.'];
  else if (confirmed) [status, text] = ['confirmed', 'Confirmed. Save to keep it.'];
  else if (saved && (untouched || sameAsSaved)) [status, text] = ['saved', `Saved ${range(saved)}`];
  else if (!row.problem) [status, text] = ['ready', saved ? 'Wider than the saved range. Confirm to replace it.' : 'Both ends held. Confirm when this joint is done.'];
  else if (saved) [status, text] = ['saved', `Saved ${range(saved)} · new sweep: ${problem}`];
  else [status, text] = ['sweeping', problem];
  if (row.jumps) text += ` · ${row.jumps} sudden jump${row.jumps > 1 ? 's' : ''} ignored`;
  if (row.discarded && !row.samples) text += ' · Reading jumped across a reconnect, so the unsaved sweep was discarded';
  else if (row.discarded) text += ' · earlier sweep discarded after a reconnect jump';
  card.dataset.status = status;
  card.querySelector('.status').textContent = text;

  card.querySelector('.v-now').textContent = signed(row.current_rad);
  card.querySelector('.v-min').textContent = signed(row.min_rad);
  card.querySelector('.v-max').textContent = signed(row.max_rad);
  card.querySelector('.v-span').textContent = row.span_rad == null ? '—' : `${row.span_rad.toFixed(3)} (${(row.span_rad * DEG).toFixed(0)}°)`;

  const values = [row.current_rad, row.min_rad, row.max_rad, saved?.min_rad, saved?.max_rad].filter(Number.isFinite);
  const extent = Math.max(.5, ...values.map(Math.abs)) * 1.1;
  const [held, savedBox, now] = ['.held', '.saved', '.now'].map(s => card.querySelector(s));
  if (Number.isFinite(row.min_rad)) place(held, row.min_rad, row.max_rad, extent); else held.hidden = true;
  if (saved) place(savedBox, saved.min_rad, saved.max_rad, extent); else savedBox.hidden = true;
  if (Number.isFinite(row.current_rad)) place(now, row.current_rad, undefined, extent); else now.hidden = true;

  const confirm = card.querySelector('.confirm');
  const undo = status === 'confirmed' || status === 'changed';
  confirm.textContent = undo ? 'Unconfirm' : 'Confirm';
  confirm.disabled = undo ? false : !(live && !row.problem);
  card.querySelector('.reset').disabled = !(row.samples || confirmed);

  const direction = row.direction;
  const chip = card.querySelector('.chip');
  if (direction) {
    chip.textContent = `${direction.sign > 0 ? '+1' : '−1'} ${direction.verified ? '✓' : '?'}`;
    chip.className = `chip ${direction.verified ? 'ok' : 'unsure'}`;
    chip.title = direction.verified ? 'Direction confirmed by the operator' : 'Direction not yet confirmed';
    for (const button of card.querySelectorAll('.flip, .accept')) button.disabled = !live;
    const accept = card.querySelector('.accept');
    if (accept) accept.hidden = direction.verified;
  } else {
    chip.hidden = true;
  }
}

// Gripper zero (helm.gripper_zero): an in-page confirmation, then the robot zeroes and saves that gripper.
let gripperZeroPending = false, gripperZeroError = null;  // A refusal shows until a newer result arrives.
function gripperZeroBlocked(state) {
  if (['starting', 'running'].includes(state?.teleop?.phase)) return 'Stop the teleop session first.';
  if (['starting', 'running'].includes(state?.tune?.phase)) return 'Finish or stop the servo sweep first.';
  if (state?.check?.running) return 'Wait for Minerva Check to finish.';
  if (state?.robot?.connection && state.robot.connection !== 'tether') return 'Choose Tether first.';
  return '';
}
function renderGripperZero(state) {
  const zero = state?.gripper_zero, busy = !!zero?.running || gripperZeroPending, blocked = gripperZeroBlocked(state);
  for (const block of document.querySelectorAll('.gripper-zero')) {
    const row = (state?.limits?.rows || []).find(r => r.name === block.dataset.motor);
    block.querySelector('.gripper-reading').textContent = Number.isFinite(row?.current_rad)
      ? `reads ${row.current_rad.toFixed(3)} rad now · closed should read 0` : 'no live reading';
    const ask = block.querySelector('.gripper-zero-ask');
    ask.disabled = busy || !!blocked;
    ask.title = blocked;
    block.querySelector('.gripper-zero-yes').disabled = busy || !!blocked;
    if (busy || blocked) block.querySelector('.gripper-zero-confirm').hidden = true;
  }
  const node = $('gripper-zero-feedback'), last = zero?.last;
  if (busy) return;
  if (blocked) { node.textContent = blocked; node.classList.remove('bad'); return; }
  if (gripperZeroError && !(last?.finished_at > gripperZeroError.at)) {
    node.textContent = gripperZeroError.message; node.classList.add('bad'); return;
  }
  if (!last) return;
  const name = last.motor === 'left_gripper' ? 'Left gripper' : 'Right gripper';
  const value = v => Number.isFinite(v) ? `${v.toFixed(4)} rad` : '—';
  node.classList.toggle('bad', !last.success);
  node.textContent = last.success
    ? `${name} zeroed: read ${value(last.before_rad)} before, ${value(last.after_save_rad ?? last.after_zero_rad)} after`
      + `${last.save_response_missing ? ' (the drive gave no reply to the save)' : ''}. Saved on the drive; power-cycle `
      + 'the arm, then check it still reads 0 when closed.'
    : `${name} not zeroed: ${last.error || 'the robot reported a failure'}`;
}
for (const block of document.querySelectorAll('.gripper-zero')) {
  const confirm = block.querySelector('.gripper-zero-confirm');
  block.querySelector('.gripper-zero-ask').onclick = () => { confirm.hidden = false; };
  block.querySelector('.gripper-zero-no').onclick = () => { confirm.hidden = true; };
  block.querySelector('.gripper-zero-yes').onclick = async () => {
    if (gripperZeroPending) return;
    gripperZeroPending = true;
    gripperZeroError = null;
    confirm.hidden = true;
    const node = $('gripper-zero-feedback');
    node.classList.remove('bad');
    node.textContent = `Zeroing the ${block.dataset.label}: the live view pauses, every drive is checked, then this one is `
      + 'zeroed and saved. Keep the jaws closed and still…';
    renderGripperZero(last);
    try {
      render(await request('/api/helm/grippers/zero', {motor: block.dataset.motor, confirm: true}, 90000));
    } catch (error) {
      gripperZeroError = {message: `Not zeroed: ${error.message}`, at: Date.now() / 1000};
    } finally {
      gripperZeroPending = false;
      renderGripperZero(last);
    }
  };
}

function render(state) {
  if (!state) return;
  last = state;
  renderGripperZero(state);
  token = state.action_token;
  const {feed, limits, robot, session} = state;
  $('robot-name').textContent = `${robot.model} · ${robot.asset_id} · ${robot.address}`;
  // The connection is fixed while a teleop session, a sweep or a check uses it (the reader steps aside).
  const owner = state.live_view?.owner;
  const inUse = !!owner || ['running', 'starting'].includes(state.teleop?.phase) || !!state.check?.running;
  for (const button of document.querySelectorAll('#via button')) {
    const key = button.dataset.connection;
    const option = (robot.connections || []).find(c => c.key === key);
    button.setAttribute('aria-pressed', String(key === robot.connection));
    button.disabled = !option || inUse;
    button.title = !option ? '' : key === 'tether'
      ? `USB Ethernet tether · ${option.address} · required for Minerva Check and live teleop`
      : `Minerva's Wi-Fi · ${option.address} · stopped reads and virtual teleop only`;
  }
  document.body.dataset.feed = feed.state;
  const live = feed.state === 'reading' && feed.fresh;
  $('feed-text').textContent = {
    reading: live ? `Live · frame ${feed.sequence} · ${Math.round(feed.age_seconds * 1000)} ms` : 'Live feed stale',
    connecting: 'Connecting…', reconnecting: 'Reconnecting…', waiting: 'Waiting for motors',
    fault: 'Connection failed',
  }[feed.state] || 'Not connected';
  if (owner && !state.link.running) $('feed-text').textContent = `Motors in use: ${owner}`;
  // The live view reads the arms whenever Minerva is online; Disconnect turns it off until Connect.
  const viewing = state.live_view ? state.live_view.enabled : state.link.running;
  const connect = $('connect');
  connect.textContent = viewing ? 'Disconnect' : 'Connect';
  connect.className = viewing ? 'danger' : 'primary';

  let banner = null, bad = false;
  if (['fault', 'connecting', 'reconnecting', 'waiting'].includes(feed.state)) [banner, bad] = [feed.message, feed.state === 'fault'];
  else if (limits.saved_error) [banner, bad] = [`Saved limits were not loaded: ${limits.saved_error}`, true];
  else if (session.stale_lock_pid && !lockNoticeShown) banner = `Took over a calibration lock left by exited process ${session.stale_lock_pid}.`;
  $('banner').hidden = !banner;
  $('banner').textContent = banner || '';
  $('banner').className = bad ? 'bad' : '';
  document.body.classList.toggle('has-banner', !!banner);

  const total = limits.rows.length;
  $('saved-summary').textContent = limits.saved_joints
    ? `Saved: ${limits.saved_joints}/${total} joints · ${new Date(limits.saved_at).toLocaleString()} · ${limits.path.split(/[\\/]/).pop()}`
    : `Nothing saved yet. Limits will be written to ${limits.path.split(/[\\/]/).pop()}.`;
  const save = $('save');
  save.disabled = !limits.pending;
  save.textContent = limits.pending ? `Save limits (${limits.pending})` : 'Save limits';

  const container = $('joints');
  if (!cards.size) {
    for (const [arm, title] of [['left', 'LEFT ARM'], ['right', 'RIGHT ARM'], ['other', 'OTHER']]) {
      const group = limits.rows.filter(row => row.arm === arm);
      if (!group.length) continue;
      const heading = document.createElement('div');
      heading.className = 'arm';
      heading.textContent = title;
      container.append(heading);
      for (const row of group) {
        const card = build(row);
        cards.set(row.name, card);
        container.append(card);
      }
    }
  }
  for (const row of limits.rows) update(cards.get(row.name), row, live);
  renderLeaders(state.leaders);
  renderTeleop(state.teleop);
  renderTune(state.tune, state);
  renderCheck(state.check);
  settingsPanel.updateContext(state);
  renderGlobalControls(state);
  renderMode(state);
  renderTask(state);
  renderTaskViewer(state);
  publishTelemetry(state);
}

// Mode is fixed at launch. The server enforces the same separation for every action.
function renderMode(state) {
  const simulation = state.mode === 'simulation';
  document.body.dataset.mode = simulation ? 'simulation' : 'robot';
  $('mode-pill').textContent = simulation ? 'Simulation only' : 'Robot mode';
  if (simulation) {
    $('feed-text').textContent = 'Physical robot disconnected';
    $('robot-name').textContent = `${state.robot.model} · simulation on this PC`;
    if (['check', 'limits', 'tune'].includes(document.body.dataset.tab)) showTab('teleop');
  }
}

const modelViewerSource = $('model-view').getAttribute('src');
let modelViewerPaused = false;

function taskOwnsViewer(state) {
  return state?.mode === 'simulation' && state.teleop?.task_configured === true
    && ['starting', 'running'].includes(state.teleop?.phase);
}

function renderTaskViewer(state) {
  const paused = taskOwnsViewer(state);
  if (paused !== modelViewerPaused) {
    // Unload the duplicate WebGL document; hiding it leaves its animation loop running.
    $('model-view').src = paused ? 'about:blank' : modelViewerSource;
    modelViewerPaused = paused;
  }
  $('model-view').hidden = paused;
  $('task-window-note').hidden = !paused;
  $('model-heading').textContent = paused ? 'LIVE TELEMETRY' : 'MODEL & TELEMETRY';
}

function renderTask(state) {
  const t = state.teleop, task = t?.task || {}, configured = !!t?.task_configured;
  const running = ['starting', 'running'].includes(t?.phase), simulation = state.mode === 'simulation';
  const recording = task.recording || {}, active = recording.active === true || recording.state === 'recording';
  const pending = task.last_command?.state === 'pending';
  $('task-mode').textContent = simulation ? 'SIMULATED CONTACT TASK' : 'BIMANUAL ROBOT CAPTURE';
  $('task-capability').textContent = simulation
    ? `${task.display_visual || 'Camera appearance is not reported yet.'} Dynamic object contacts with posed fingers; simulated arm response is not a validated coupled physics model. No synthetic grasp attachment.`
    : 'Records the current guarded receiver and existing camera streams. Both arms and grippers must be healthy; camera timing must be verified.';
  $('task-setup').disabled = running || !t;
  $('task-setup').textContent = configured ? 'Task configured' : 'Set up task';
  $('task-reset').hidden = !simulation;
  $('task-reset').disabled = !!t?.recording_fault || !task.available || !running || active || !['DISABLED', 'IDLE'].includes(t?.state);
  $('task-record').disabled = !task.available || !task.capture_ready || !running || active || pending || task.failed || t?.recording_fault || !['HOLD', 'TELEOP'].includes(t?.state);
  $('task-stop').disabled = !!t?.recording_fault || !running || !active || pending;
  const progress = task.task || {};
  $('task-state').textContent = t?.recording_fault || task.error ? 'Task needs attention.' : !configured ? 'Task not configured.'
    : !running ? 'Ready to start a session. Task controls do not enable motors.'
    : task.display_phase || progress.phase || progress.stage || (task.available ? 'Task ready.' : 'Preparing task…');
  $('task-cameras').hidden = simulation;
  $('task-cameras').textContent = task.camera_summary || 'Camera preflight starts with the bimanual session; all configured streams are required.';
  const summary = recording.state || (active ? 'recording' : configured ? 'idle' : 'not started');
  $('task-recording').textContent = `Recording: ${summary}`
    + (Number.isFinite(recording.frames) ? ` · ${recording.frames} frames` : '')
    + (recording.path || recording.directory ? ` · ${recording.path || recording.directory}` : '')
    + (simulation ? (task.cameras?.ready ? ' · simulated cameras rendered' : '')
      : task.cameras ? ` · cameras: ${Object.entries(task.cameras).map(([name, c]) => `${name} ${c.fresh ? 'fresh' : c.error || 'waiting'}`).join(', ')}` : '');
  const error = t?.recording_fault || task.error || recording.error || task.last_command?.error || '';
  $('task-error').hidden = !error;
  $('task-error').textContent = error;
}
for (const [id, action] of [['task-setup', 'setup'], ['task-reset', 'reset'],
                           ['task-record', 'start_record'], ['task-stop', 'stop_record']]) {
  $(id).onclick = () => act('/api/helm/task', {action}, action === 'setup' ? 'Task configured; start the session when ready.' : null);
}

// ---- Minerva Check -------------------------------------------------------------
const CHECK_ICON = {pass: '✓', warn: '!', fail: '✕', skip: '–', pending: '○', queued: '·', running: '…'};
const VERDICT = {go: 'GO', no_go: 'NO GO', incomplete: 'INCOMPLETE', outdated: 'OUTDATED'};
const crows = new Map();
let checkLayout = '', checkRunning = null, checkWasRunning = false;

function renderCheck(c) {
  const tab = document.querySelector('.tab[data-tab=check]');
  tab.disabled = !c;
  if (!c) {
    if (document.body.dataset.tab === 'check') showTab('limits');
    return;
  }
  const report = c.report;
  // The start note is only true while running; the verdict takes over.
  if (checkWasRunning && !c.running) $('check-status').textContent = '';
  checkWasRunning = c.running;
  const run = $('c-run');
  run.disabled = c.running;
  run.textContent = c.running ? 'Checking…' : 'Run check';

  const checks = report ? report.stages.flatMap(stage => stage.checks) : [];
  const done = checks.filter(entry => !['queued', 'running'].includes(entry.status)).length;
  const verdict = $('c-verdict');
  const [title, detail] = verdict.children;
  if (c.error) {
    verdict.dataset.verdict = 'no_go';
    [title.textContent, detail.textContent] = ['Check stopped', c.error];
  } else if (c.running || report?.state === 'running') {
    verdict.dataset.verdict = 'running';
    const current = checks.find(entry => entry.status === 'running');
    title.textContent = `Checking… ${done}/${checks.length || '—'}`;
    detail.textContent = current ? current.title : 'Starting';
  } else if (report) {
    verdict.dataset.verdict = report.verdict;
    title.textContent = VERDICT[report.verdict] || report.verdict;
    const when = new Date(report.finished_at || report.started_at).toLocaleString();
    detail.textContent = `${report.summary.replace(/^(NO GO|GO|INCOMPLETE):?\s*/, '')} · ${when}`
      + (report.seconds ? ` · ${Math.round(report.seconds)} s` : '');
  } else {
    verdict.dataset.verdict = '';
    [title.textContent, detail.textContent] = ['Not run yet', 'Press Run check.'];
  }

  const container = $('check-stages');
  const layout = report ? report.stages.map(stage => `${stage.id}:${stage.checks.map(e => e.id)}`).join('|') : '';
  if (layout !== checkLayout) {
    container.replaceChildren();
    crows.clear();
    checkLayout = layout;
    for (const stage of report ? report.stages : []) {
      const heading = document.createElement('div');
      heading.className = 'arm';
      heading.textContent = stage.title.toUpperCase();
      container.append(heading);
      for (const entry of stage.checks) {
        const row = document.createElement('div');
        row.className = 'crow';
        row.innerHTML = `<span class="cicon" aria-hidden="true"></span><div class="ctext"><span class="ctitle"></span>
          <span class="cdetail"></span></div><span class="ctime"></span>`;
        row.querySelector('.ctitle').textContent = entry.title;
        crows.set(`${stage.id}/${entry.id}`, row);
        container.append(row);
      }
    }
  }
  // The live joint sweep: attended, from here, once the stopped checks pass (the server checks too).
  const tune = last?.tune;
  const sweeping = tune && ['running', 'starting'].includes(tune.phase);
  const stoppedFailed = report ? report.stages.some(stage => stage.id !== 'sweep'
    && stage.checks.some(entry => entry.status === 'fail')) : true;
  for (const stage of report ? report.stages : []) {
    for (const entry of stage.checks) {
      const row = crows.get(`${stage.id}/${entry.id}`);
      row.dataset.status = entry.status;
      row.querySelector('.cicon').textContent = CHECK_ICON[entry.status] || '?';
      const arm = stage.id === 'sweep' && /^sweep_(left|right)$/.test(entry.id) ? entry.id.slice(6) : null;
      const live = arm && sweeping && tune.mode === 'verify' && tune.receiver_choice?.arm === arm;
      row.querySelector('.cdetail').textContent = entry.status === 'running' ? 'Checking…'
        : live ? `${tune.progress?.task || 'Starting'} · ${tune.message || ''}` : entry.detail;
      if (arm) {
        let button = row.querySelector('button.live');
        if (!button) {
          button = document.createElement('button');
          button.className = 'live danger';
          button.onclick = () => askCheck(arm);
          row.append(button);
        }
        button.textContent = live ? 'Sweeping…' : `Sweep ${arm} arm`;
        button.disabled = c.running || sweeping || stoppedFailed || report?.state !== 'finished';
        button.title = stoppedFailed ? 'The stopped checks must pass first' : '';
      }
      row.querySelector('.ctime').textContent = entry.seconds == null ? '' : `${entry.seconds.toFixed(1)} s`;
      if (entry.status === 'running' && checkRunning !== row) {
        checkRunning = row;
        row.scrollIntoView({block: 'nearest'});  // Follow the run, once per check.
      }
    }
  }
}

// ---- Leaders workspace -------------------------------------------------------
const lcards = new Map();
let leaderState = null;
const degrees = span => `${span.toFixed(2)} rad · ${(span * DEG).toFixed(0)}°`;

function buildLeader(row) {
  const card = document.createElement('div');
  card.className = 'joint';
  card.innerHTML = `<div class="joint-top"><div><span class="joint-name"></span><span class="motor"></span></div></div>
    <div class="bar" aria-hidden="true"><span class="seen" hidden></span><span class="saved" hidden></span><span class="held" hidden></span><span class="ref" hidden></span><span class="now" hidden></span></div>
    <div class="numbers"><span>position<b class="v-pos"></b></span><span>range<b class="v-span"></b></span><span>swept<b class="v-seen"></b></span><span>readings<b class="v-samples"></b></span></div>
    <div class="joint-foot"><span class="status" role="status"></span><button class="reset">Reset</button><button class="confirm primary">Confirm</button></div>`;
  const name = `${row.leader} ${row.label.toLowerCase()}`;
  card.querySelector('.joint-name').textContent = row.label;
  card.querySelector('.motor').textContent = `${row.leader} leader ch ${row.channel}`
    + (row.drives ? ` → ${row.drives_label} (${row.drives})` : ' · not routed to the robot');
  card.querySelector('.reset').onclick = () =>
    act('/api/helm/leaders/range', {action: 'reset', key: row.key}, `Cleared the sweep for the ${name}.`);
  card.querySelector('.confirm').onclick = () => {
    const undo = ['confirmed', 'changed'].includes(card.dataset.status);
    act('/api/helm/leaders/range', {action: undo ? 'unconfirm' : 'confirm', key: row.key},
      undo ? `The ${name} is no longer confirmed.` : `The ${name} is confirmed. Save when you are done.`);
  };
  return card;
}

function updateLeader(card, row, live) {
  const saved = row.saved;
  const swept = row.seen_low == null ? 0 : row.seen_high - row.seen_low;
  const untouched = !row.samples || swept < .02;
  const problem = untouched ? 'Not swept yet: move it to each end and hold there' : row.problem;
  const sameAsSaved = saved && row.low != null && row.low >= saved.low - .01 && row.high <= saved.high + .01;
  let status, text;
  if (row.confirmed && row.changed_since_confirmed) [status, text] = ['changed', 'The range grew after confirming. Confirm again.'];
  else if (row.confirmed) [status, text] = ['confirmed', 'Confirmed. Save to keep it.'];
  else if (saved && (untouched || sameAsSaved)) [status, text] = ['saved', `Saved: ${degrees(saved.span_rad)}`];
  else if (!row.problem) [status, text] = ['ready', saved ? 'Wider than the saved range. Confirm to replace it.' : 'Both ends held. Confirm when this one is done.'];
  else if (saved) [status, text] = ['saved', `Saved: ${degrees(saved.span_rad)} · new sweep: ${problem}`];
  else [status, text] = ['sweeping', problem];
  if (row.jumps) text += ` · ${row.jumps} glitch reading${row.jumps > 1 ? 's' : ''} ignored`;
  if (row.discarded && !row.samples) text += ' · The reading jumped, so the unsaved sweep was discarded';
  else if (row.discarded) text += ' · an earlier sweep was discarded after a jump';
  card.dataset.status = status;
  card.querySelector('.status').textContent = text;

  card.querySelector('.v-pos').textContent = row.fraction == null ? '—' : `${Math.round(row.fraction * 100)}%`;
  card.querySelector('.v-span').textContent = row.span_rad == null ? '—' : degrees(row.span_rad);
  card.querySelector('.v-seen').textContent = row.seen_low == null ? '—' : `${swept.toFixed(2)} rad`;
  card.querySelector('.v-samples').textContent = row.samples || '—';

  const values = [row.current, row.seen_low, row.seen_high, saved?.low, saved?.high, row.reference].filter(Number.isFinite);
  const [seen, held, savedBox, ref, now] = ['.seen', '.held', '.saved', '.ref', '.now'].map(s => card.querySelector(s));
  if (values.length) {
    const lo = Math.min(...values), hi = Math.max(...values);
    const pad = Math.max(hi - lo, .2) * .08, a = lo - pad, b = hi + pad;
    const at = q => `${Math.max(0, Math.min(100, (q - a) / (b - a) * 100))}%`;
    const span = (element, from, to) => {
      element.hidden = false;
      element.style.left = at(from);
      element.style.width = `max(calc(${at(to)} - ${at(from)}), 3px)`;
    };
    if (row.seen_low != null) span(seen, row.seen_low, row.seen_high); else seen.hidden = true;
    if (row.low != null) span(held, row.low, row.high); else held.hidden = true;
    if (saved) span(savedBox, saved.low, saved.high); else savedBox.hidden = true;
    for (const [element, value] of [[now, row.current], [ref, row.reference]]) {
      element.hidden = !Number.isFinite(value);
      if (!element.hidden) element.style.left = at(value);
    }
  } else {
    for (const element of [seen, held, savedBox, ref, now]) element.hidden = true;
  }

  const confirm = card.querySelector('.confirm');
  const undo = status === 'confirmed' || status === 'changed';
  confirm.textContent = undo ? 'Unconfirm' : 'Confirm';
  confirm.disabled = undo ? false : !(live && !row.problem);
  card.querySelector('.reset').disabled = !(row.samples || row.confirmed);
}

function renderLeaders(l) {
  const tab = document.querySelector('.tab[data-tab=leaders]');
  tab.disabled = !l;
  tab.title = l ? '' : 'Start Helm with a teleop profile (scripts/helm.ps1 does this)';
  if (!l) {
    if (document.body.dataset.tab === 'leaders') showTab('limits');
    return;
  }
  const live = l.state === 'reading' && l.fresh;
  const connected = ['reading', 'connecting'].includes(l.state);
  // The Connect note is only true while connecting; the pill and summary take over.
  if (leaderState === 'connecting' && l.state !== 'connecting') $('leaders-status').textContent = '';
  leaderState = l.state;
  $('l-pill').className = `pill ${live ? 'live' : l.state === 'fault' ? 'bad' : connected ? 'busy' : ''}`;
  $('l-text').textContent = live ? `Live · ${Math.round(l.age_seconds * 1000)} ms`
    : {reading: 'Leaders stale', connecting: 'Connecting…', fault: 'Leader fault'}[l.state] || 'Not connected';
  const button = $('l-connect');
  button.textContent = connected ? 'Disconnect leaders' : 'Connect leaders';
  button.className = connected ? 'danger' : 'primary';

  const summary = $('l-summary');
  const file = l.path.split(/[\\/]/).pop();
  summary.className = l.saved_error || l.state === 'fault' ? 'muted bad' : 'muted';
  summary.textContent = l.state === 'fault' ? l.message
    : l.saved_error ? `Saved ranges were not loaded: ${l.saved_error}`
    : l.saved_channels ? `Saved: ${l.saved_channels}/${l.total_channels} · ${new Date(l.saved_at).toLocaleString()} · ${file}`
    : `Nothing saved yet. Ranges will be written to ${file}.`;
  const zero = l.reference || {};
  if (zero.captured_at) summary.textContent += ` · Zero set ${new Date(zero.captured_at).toLocaleString()}.`;
  else if (!l.saved_error && l.state !== 'fault') summary.textContent += ' · No leader zero yet.';
  $('l-capture').disabled = !live;
  const save = $('l-save');
  save.disabled = !l.pending;
  save.textContent = l.pending ? `Save (${l.pending})` : 'Save';

  const container = $('leader-joints');
  if (!lcards.size) {
    for (const [leader, title] of Object.entries(l.titles)) {
      const heading = document.createElement('div');
      heading.className = 'arm';
      heading.textContent = title.toUpperCase();
      container.append(heading);
      for (const row of l.rows.filter(r => r.leader === leader)) {
        const card = buildLeader(row);
        lcards.set(row.key, card);
        container.append(card);
      }
    }
  }
  for (const row of l.rows) updateLeader(lcards.get(row.key), row, live);
}

// ---- Teleop workspace -------------------------------------------------------
const tcards = new Map();
const ENGAGED = ['ENGAGING', 'TELEOP'];
let teleopPhase = null;
const fixed = q => (q == null || !Number.isFinite(q)) ? '—' : signed(q, 2);

const manualFields = ['kp', 'kd', 'speed', 'follow', 'travel_rad'];
const manualCards = new Map();
let manualState = null, manualBase = null, manualDraft = new Map(), manualPending = false, manualConflict = false;
// The velocity feed-forward switch as drafted, and whether a session runs (it switches only at a session start).
let manualFeedforward = false, manualSessionRunning = false;
const manualRetiredSessions = [];
const manualNumber = value => Number.isFinite(value) ? String(Number(value.toPrecision(6))) : '—';

// Top following speed in open travel (teleop.lifecycle): a command leads the joint's position, predicted to when
// it is computed, by at most its lead, and the drive sees it about a loop later, when the joint has closed v × lag
// of it. At a steady speed v the pull Kp × (lead − v × lag) meets the damping Kd × v plus friction. J3 at 320 / 8
// (live-20261013T031504Z): 0.70–0.75 rad/s estimated, 0.69–0.71 measured. With velocity fed forward the damping
// acts about the command's speed, so Kd drops out: Kp × (lead − v × lag) meets friction alone.
function leadLimitedSpeed(following, kp, kd, feedforward) {
  const pull = kp * following.max_lead - following.friction_nm;
  return pull <= 0 ? 0 : pull / ((feedforward ? 0 : kd) + kp * following.command_lag_s);
}

function manualEstimate(row, draft) {
  const f = row.following;
  if (!f || f.gripper) return {text: '', binds: ''};
  const [kp, kd, limit] = ['kp', 'kd', 'speed'].map(field => manualValue(draft, field, row));
  if (![kp, kd, limit].every(Number.isFinite)) return {text: '', binds: ''};
  const vff = manualFeedforward, speed = leadLimitedSpeed(f, kp, kd, vff), lead = `${f.max_lead.toFixed(3)} rad`;
  const lag = `${(f.command_lag_s * 1000).toFixed(0)} ms`, values = `(${manualNumber(kp)} × ${f.max_lead.toFixed(4)} − `
    + `${f.friction_nm.toFixed(2)})`;
  const title = (vff
    ? `(Kp × lead − friction) ÷ (Kp × ${lag}) = ${values} ÷ (${manualNumber(kp)} × ${f.command_lag_s.toFixed(3)}). `
      + 'The command\'s speed is fed forward, so Kd damps the joint about it instead of dragging it.'
    : `(Kp × lead − friction) ÷ (Kd + Kp × ${lag}) = ${values} ÷ (${manualNumber(kd)} + ${manualNumber(kp)} × `
      + `${f.command_lag_s.toFixed(3)}). Position-only control sends no velocity, so Kd drags against all motion.`)
    + ' The drive sees each command about a loop late. Open travel only: commands slow near either end.';
  // As shown (two decimals): 0.998 under a 1 rad/s limit reads as the limit, not "1.00, under your 1".
  if (Number(speed.toFixed(2)) >= limit) return {binds: 'limit', title, text: `Top speed is your ${manualNumber(limit)} rad/s limit; `
    + `the ${lead} lead alone would allow ≈ ${speed.toFixed(2)} rad/s.`};
  // A drive that reports no torque leaves its sweep's friction at zero (left J1: 5e-20 Nm).
  const friction = f.friction_nm >= .01 ? '' : ' No friction was measured for this joint, so expect less.';
  return {binds: 'lead', title, text: speed > 0
    ? `Top speed ≈ ${speed.toFixed(2)} rad/s, under your ${manualNumber(limit)} rad/s limit: the ${lead} lead caps it `
      + `at these gains. Raise Kp to follow faster${vff ? '' : ' (lowering Kd does too, but damps less)'}.`
      + friction
    : `At this Kp the ${lead} lead pulls with ${(kp * f.max_lead).toFixed(2)} Nm, under the joint's `
      + `${f.friction_nm.toFixed(2)} Nm of friction: it may not follow.`};
}
// Draft text matches the Active line (9.799999999999999 shows as 9.8); text left as shown keeps the exact value.
const manualText = value => value == null ? '' : manualNumber(value);

function manualValue(draft, field, row) {
  if (field === 'follow') return draft.follow;
  const value = draft[field].trim();
  if (row && value === manualText(row[field])) return row[field];  // Untouched: the exact applied value.
  return value === '' ? (field === 'travel_rad' ? null : NaN) : Number(value);
}

function manualRowChanged(motor, row) {
  const draft = manualDraft.get(motor);
  return !draft || manualFields.some(field => manualValue(draft, field, row) !== row[field]);
}

function manualHasEdits() {
  return !!manualBase && (manualBase.rows.some(row => manualRowChanged(row.motor, row))
    || manualFeedforward !== (manualBase.velocity_feedforward === true)
    || [...manualCards.values()].some(card => [...card.querySelectorAll('input')].some(input => input.validity.badInput)));
}

function resetManualDraft(settings) {
  manualBase = settings;
  manualFeedforward = settings.velocity_feedforward === true;
  $('manual-vff').checked = manualFeedforward;
  manualDraft = new Map(settings.rows.map(row => [row.motor, Object.fromEntries(manualFields.map(field =>
    [field, field === 'follow' ? row[field] : manualText(row[field])]))]));
  manualConflict = false;
  for (const [motor, card] of manualCards) {
    const draft = manualDraft.get(motor);
    if (!draft) continue;
    for (const field of manualFields) {
      const input = card.querySelector(`[data-field="${field}"]`);
      if (field === 'follow') input.checked = draft[field]; else input.value = draft[field];
    }
  }
}

function manualFeedback(message = '', bad = false) {
  $('manual-feedback').textContent = message;
  $('manual-feedback').className = `muted${bad ? ' bad' : ''}`;
}

function manualValid() {
  return [...manualCards.values()].every(card => [...card.querySelectorAll('input')].every(input => input.checkValidity()));
}

function updateManualActions() {
  if (!manualState?.enabled) return;
  const edits = manualHasEdits(), valid = manualValid();
  $('manual-apply').disabled = manualPending || manualConflict || !edits || !valid || !manualState.can_apply;
  $('manual-revert').disabled = manualPending || (!edits && !manualConflict);
  $('manual-save').disabled = manualPending || manualConflict || edits || !manualState.can_save;
  $('manual-restore').disabled = manualPending || manualConflict || edits || !manualState.saved_revision || !manualState.can_apply;
  $('manual-vff').disabled = manualPending || manualConflict || manualSessionRunning || !manualState.can_apply;
  $('manual-vff-note').textContent = manualSessionRunning
    ? `${manualState.velocity_feedforward ? 'On' : 'Off'} for this session; it switches at the next session start.`
    : manualFeedforward
      ? "From the next session: Kd damps each joint about its command's speed instead of dragging it. None is fed "
        + 'toward an end inside its braking zone. Expect more overshoot when a fast move stops.'
      : 'Off: position only. Kd damps and also drags against motion (the safer, slower default).';
  $('manual-mirror-left').disabled = manualPending || manualConflict || !mirrorPairs('left', 'right').length;
  $('manual-mirror-right').disabled = manualPending || manualConflict || !mirrorPairs('right', 'left').length;
  $('manual-gate').textContent = manualPending ? 'Waiting for the receiver to acknowledge the settings…'
    : manualConflict ? 'The session or active settings changed. Revert edits to review the current values before applying.'
    : !valid ? 'Enter finite values within the displayed bounds. Travel may be blank.'
    : !manualState.can_apply ? manualState.reason || 'Hold with T before applying settings.'
    : edits ? 'Draft edits are not active. Apply to test them; Save writes only applied settings.'
    : !manualState.can_save && manualState.dirty ? manualState.reason || 'Saving is not available in this state.' : '';
  for (const [motor, card] of manualCards) {
    const row = manualState.rows.find(row => row.motor === motor);
    const changed = row && manualRowChanged(motor, row);
    card.dataset.edited = String(!!changed);
    card.querySelector('.manual-draft-status').textContent = changed ? 'Draft · not applied' : 'Matches active';
    const estimate = card.querySelector('.manual-estimate'), speed = row ? manualEstimate(row, manualDraft.get(motor)) : {};
    estimate.textContent = speed.text || '';
    estimate.hidden = !speed.text;
    estimate.title = speed.title || '';
    estimate.dataset.binds = speed.binds || '';
    for (const input of card.querySelectorAll('input')) input.disabled = manualPending;
  }
}

function buildManualRow(row) {
  const card = document.createElement('div');
  card.className = 'manual-row';
  card.innerHTML = `<div class="manual-row-head"><div><b class="manual-label"></b><span class="motor"></span></div>
      <label class="manual-follow"><input type="checkbox" data-field="follow"> Follow</label></div>
    <p class="manual-active"></p><div class="manual-fields"></div>
    <p class="manual-estimate"></p><span class="manual-draft-status"></span>`;
  card.querySelector('.manual-label').textContent = row.label;
  card.querySelector('.motor').textContent = row.motor;
  const fields = [['kp', 'Kp'], ['kd', 'Kd'], ['speed', 'Speed · rad/s'], ['travel_rad', 'Travel · ±rad']];
  for (const [field, title] of fields) {
    const label = document.createElement('label');
    label.textContent = title;
    const input = document.createElement('input');
    input.type = 'number';
    input.dataset.field = field;
    input.step = 'any';
    input.required = field !== 'travel_rad';
    input.setAttribute('aria-label', `${row.label}: draft ${title}`);
    if (field === 'travel_rad') input.placeholder = 'Full range';
    const bound = document.createElement('small');
    bound.dataset.bound = field;
    label.append(input, bound);
    card.querySelector('.manual-fields').append(label);
  }
  const draft = manualDraft.get(row.motor);
  for (const input of card.querySelectorAll('input')) {
    const field = input.dataset.field;
    if (field === 'follow') {
      input.checked = draft.follow;
      input.setAttribute('aria-label', `${row.label}: follow the leader (unchecked holds position)`);
    } else input.value = draft[field];
    input.addEventListener('input', () => {
      const current = manualDraft.get(row.motor);
      current[field] = field === 'follow' ? input.checked : input.value;
      manualFeedback();
      updateManualActions();
    });
  }
  return card;
}

function renderManual(t) {
  const settings = t?.tuning;
  $('manual-tuning').hidden = !settings?.enabled;
  $('settings-manual-unavailable').hidden = !!settings?.enabled;
  if (!settings?.enabled) return;
  // A late poll must not replace an acknowledged revision or a newer session.
  if (manualRetiredSessions.includes(settings.session_id)
      || (manualState?.session_id === settings.session_id && settings.revision < manualState.revision)) return;
  const changed = !manualState || manualState.session_id !== settings.session_id || manualState.revision !== settings.revision;
  if (manualState && manualState.session_id !== settings.session_id) {
    manualRetiredSessions.push(manualState.session_id);
    if (manualRetiredSessions.length > 8) manualRetiredSessions.shift();
  }
  if (changed) {
    if (manualHasEdits()) manualConflict = true;
    else resetManualDraft(settings);
  }
  manualState = settings;
  manualSessionRunning = t.phase === 'running';
  const container = $('manual-rows'), layout = settings.rows.map(row => row.motor).join(',');
  if (container.dataset.layout !== layout) {
    container.dataset.layout = layout;
    container.replaceChildren();
    manualCards.clear();
    let arm = null;
    for (const row of settings.rows) {
      if (!manualDraft.has(row.motor)) manualDraft.set(row.motor, Object.fromEntries(manualFields.map(field =>
        [field, field === 'follow' ? row[field] : manualText(row[field])])));
      const nextArm = row.motor.startsWith('left_') ? 'LEFT ARM' : row.motor.startsWith('right_') ? 'RIGHT ARM' : 'JOINTS';
      if (nextArm !== arm) {
        const heading = document.createElement('h3');
        heading.className = 'arm';
        heading.textContent = arm = nextArm;
        container.append(heading);
      }
      const card = buildManualRow(row);
      manualCards.set(row.motor, card);
      container.append(card);
    }
  }
    const applied = t.phase === 'running' ? 'Active in session' : 'Ready for next session';
    $('manual-state').textContent = `${applied} · ${settings.dirty ? 'changes not saved' : settings.saved_revision ? 'matches saved settings' : 'profile defaults'}`;
    if (t.phase === 'running' && Number.isFinite(t.observation_age_ms) && t.observation_age_ms > 500)
      $('manual-state').textContent += ` · joint display is ${(t.observation_age_ms / 1000).toFixed(1)} s old`;
  const following = settings.rows.filter(row => row.follow);
  $('manual-scope').textContent = `Applied follow selection: ${following.length ? following.map(row => row.label).join(', ') : 'none; all joints hold'}.`
    + (settings.control === 'direct-velocity-feedforward'
      ? ' Control: velocity fed forward, so Kd damps each joint about its command\'s speed (raise it to damp wobble).'
      : settings.control === 'direct-position-only' ? ' Control: position only, so Kd also drags against motion.' : '');
  for (const row of settings.rows) {
    const card = manualCards.get(row.motor);
    card.querySelector('.manual-active').textContent = `Active: Kp ${manualNumber(row.kp)} · Kd ${manualNumber(row.kd)} · `
      + `${manualNumber(row.speed)} rad/s · ${row.travel_rad == null ? 'full usable travel' : `±${manualNumber(row.travel_rad)} rad`} · ${row.follow ? 'follows' : 'holds'}`;
    for (const [field, bounds] of Object.entries(row.bounds)) {
      const input = card.querySelector(`[data-field="${field}"]`), bound = card.querySelector(`[data-bound="${field}"]`);
      if (!input || !bound) continue;
      [input.min, input.max] = bounds.map(String);
      bound.textContent = `${manualNumber(bounds[0])}–${manualNumber(bounds[1])}`;
      bound.title = 'Receiver bounds; these cannot be raised from this panel';
    }
  }
  updateManualActions();
}

async function manualAction(action) {
  if (!manualState || manualPending || busy) return;
  const edits = manualHasEdits();
  if (manualConflict || (action === 'apply' ? !edits || !manualValid() || !manualState.can_apply
    : edits || (action === 'save' ? !manualState.can_save : !manualState.can_apply))) return;
  const payload = {session_id: manualState.session_id, revision: manualState.revision};
  if (action === 'apply') {
    payload.joints = manualState.rows.map(row => ({motor: row.motor,
      ...Object.fromEntries(manualFields.map(field => [field, manualValue(manualDraft.get(row.motor), field, row)]))}));
    payload.velocity_feedforward = manualFeedforward;
  }
  manualPending = busy = true;
  manualFeedback();
  updateManualActions();
  try {
    const state = await request(`/api/helm/teleop/tuning/${action}`, payload);
    const acknowledged = state?.teleop?.tuning;
    if (!acknowledged?.enabled) throw new Error('Settings acknowledgement is missing. Refresh the session state before retrying.');
    resetManualDraft(acknowledged);
    manualState = acknowledged;
    render(state);
    const ready = state.teleop.phase === 'running' ? 'Engage with T when ready to test.'
      : 'Ready for the next session. Start, Idle, Enable, then Engage to test.';
    manualFeedback(action === 'save' ? 'Applied settings saved for future sessions.'
      : `${action === 'restore' ? 'Saved settings restored and applied.' : 'Settings applied.'} ${ready}`);
  } catch (error) {
    manualFeedback(`${error.message} Your draft edits have been kept.`, true);
  } finally {
    manualPending = busy = false;
    updateManualActions();
  }
}

$('manual-apply').onclick = () => manualAction('apply');
$('manual-vff').onchange = () => {
  manualFeedforward = $('manual-vff').checked;
  updateManualActions();
};
$('manual-save').onclick = () => manualAction('save');
$('manual-restore').onclick = () => manualAction('restore');
$('manual-revert').onclick = () => {
  if (!manualState || manualPending) return;
  resetManualDraft(manualState);
  manualFeedback('Draft edits reverted. Active motor settings are unchanged.');
  updateManualActions();
};

// Copy one arm's Kp, Kd and speed to the same joints of the other arm, as draft edits that Apply uses. Follow
// and travel stay per arm. A value outside the other joint's bounds is clamped to them, and the panel says so.
const MIRRORED_FIELDS = ['kp', 'kd', 'speed'];
function mirrorPairs(from, to) {  // [source row, same joint on the other arm], for joints both arms have.
  const rows = manualState?.rows || [];
  return rows.filter(row => row.motor.startsWith(`${from}_`))
    .map(row => [row, rows.find(other => other.motor === `${to}_${row.motor.slice(from.length + 1)}`)])
    .filter(([, target]) => target);
}

function manualMirror(from, to) {
  if (!manualState || manualPending || manualConflict) return;
  const clamped = [];
  const pairs = mirrorPairs(from, to);
  for (const [source, target] of pairs) {
    const draft = manualDraft.get(target.motor), card = manualCards.get(target.motor);
    for (const field of MIRRORED_FIELDS) {
      const value = manualValue(manualDraft.get(source.motor), field, source);
      if (!Number.isFinite(value)) continue;  // An unfinished draft entry is left for the operator.
      const [low, high] = target.bounds[field];
      const fitted = Math.min(Math.max(value, low), high);
      if (fitted !== value) clamped.push(`${target.label} ${field === 'speed' ? 'speed' : field === 'kp' ? 'Kp' : 'Kd'} ${manualNumber(fitted)}`);
      draft[field] = manualText(fitted);
      const input = card?.querySelector(`[data-field="${field}"]`);
      if (input) input.value = draft[field];
    }
  }
  manualFeedback(`Copied Kp, Kd and speed of ${pairs.length} ${from} joints to the ${to} arm as draft edits; `
    + 'follow and travel are unchanged. Apply to use them.'
    + (clamped.length ? ` Clamped to the ${to} arm's bounds: ${clamped.join(', ')}.` : ''));
  updateManualActions();
}
$('manual-mirror-left').onclick = () => manualMirror('left', 'right');
$('manual-mirror-right').onclick = () => manualMirror('right', 'left');

function buildTeleop(joint) {
  const card = document.createElement('div');
  card.className = 'tjoint';
  card.innerHTML = `<div class="joint-top"><div><span class="joint-name"></span><span class="motor"></span><span class="gains"></span></div>
      <span class="direction"><span class="chip" title="Leader-to-robot direction"></span>
        <button class="flip" title="The robot moves opposite to the leader: reverse this route">Flip</button>
        <button class="accept" title="The robot moves with the leader">Looks right</button></span></div>
    <div class="tbar" aria-hidden="true"><span class="usable"></span><span class="mark leader" hidden></span>
      <span class="mark command" hidden></span><span class="mark measured" hidden></span></div>
    <div class="tnumbers"><span class="l">leader<b class="v-leader"></b></span><span>command<b class="v-command"></b></span>
      <span class="m">robot<b class="v-measured"></b></span></div>`;
  card.querySelector('.joint-name').textContent = joint.label;
  const route = joint.route;
  card.querySelector('.motor').textContent = `${joint.motor} ← ${route.leader} leader ch ${route.channel}${route.trigger ? ' (trigger)' : ''}`;
  card.querySelector('.flip').onclick = () => act('/api/helm/teleop/route', {action: 'flip', motor: joint.motor},
    `Reversed the ${joint.label} route. Watch the virtual arm and confirm.`);
  card.querySelector('.accept').onclick = () => act('/api/helm/teleop/route', {action: 'accept', motor: joint.motor},
    `${joint.label} route confirmed.`);
  return card;
}

function updateTeleop(card, joint, editable) {
  const [lo, hi] = joint.operational;
  const at = q => `${Math.max(0, Math.min(100, (q - lo) / (hi - lo) * 100))}%`;
  const usable = card.querySelector('.usable');
  usable.style.left = at(joint.usable[0]);
  usable.style.width = `calc(${at(joint.usable[1])} - ${at(joint.usable[0])})`;
  for (const name of ['leader', 'command', 'measured']) {
    const mark = card.querySelector(`.mark.${name}`);
    mark.hidden = !Number.isFinite(joint[name]);
    if (!mark.hidden) mark.style.left = at(joint[name]);
    card.querySelector(`.v-${name}`).textContent = fixed(joint[name]);
  }
  const route = joint.route;
  card.querySelector('.gains').textContent = route.scale == null ? ''
    : `scale ×${route.scale.toFixed(2)} · ` + (route.reach > .995 ? 'full robot range' : `reaches ${Math.round(route.reach * 100)}% of the robot range`)
      + (route.overtravel > .05 ? ` · stops at its limit ${route.overtravel.toFixed(2)} rad before the leader's end` : '');
  const chip = card.querySelector('.chip');
  chip.textContent = `${route.sign > 0 ? '+1' : '−1'} ${route.verified ? '✓' : '?'}`;
  chip.className = `chip ${route.verified ? 'ok' : 'unsure'}`;
  chip.title = (route.trigger ? '+1: the trigger angle increasing opens the gripper'
    : '+1: the leader and robot joint angles increase together') + (route.verified ? ' · verified' : ' · not verified yet');
  card.querySelector('.flip').disabled = !editable;
  const accept = card.querySelector('.accept');
  accept.disabled = !editable;
  accept.hidden = route.verified;
}

// The receiver's control law (guarded_config names it): its label in the status row and what it does.
const CONTROL_LAWS = {
  'direct-position-only': ['Position only', 'No velocity is sent: Kd damps and also drags against motion.'],
  'direct-velocity-feedforward': ['Velocity feed-forward',
    "Each joint's command speed is fed forward, so Kd damps about it instead of dragging. None is fed toward "
      + 'an end inside its braking zone.'],
  'direct-feedforward': ['Ramped feed-forward',
    'Each command ramps over the loop period with its velocity and friction fed forward.'],
  'interpolated-feedforward': ['Interpolated feed-forward',
    'The receiver ramps each command over the loop period with its velocity and measured friction fed forward.'],
};

function renderTeleop(t) {
  const tab = document.querySelector('.tab[data-tab=teleop]');
  tab.disabled = !t;
  tab.title = t ? '' : 'Start Helm with a teleop profile (scripts/helm.ps1 does this)';
  if (!t) {
    renderManual(null);
    if (document.body.dataset.tab === 'teleop') showTab('limits');
    return;
  }
  const running = t.phase === 'running', starting = t.phase === 'starting';
  // The Start note is only true while starting; the panel message takes over.
  if (teleopPhase === 'starting' && !starting) $('teleop-status').textContent = '';
  teleopPhase = t.phase;
  document.body.dataset.teleopState = t.state || t.phase;
  $('t-state').textContent = running && t.state ? t.state : {starting: 'Starting…', stopped: 'Stopped', fault: 'Fault'}[t.phase] || t.phase;
  const choice = t.receiver_choice || {mode: 'virtual'};
  const live = choice.mode === 'live';
  const tether = last?.robot?.connection === 'tether';
  $('t-receiver').textContent = running && t.receiver
    ? t.receiver + (t.live ? (t.live.motors_enabled ? ' · motors on' : ' · motors off') : '') : '—';
  $('t-cycle').textContent = running && t.cycle_ms ? `${t.cycle_ms.p50.toFixed(1)} / ${t.cycle_ms.p99.toFixed(1)} ms` : '—';
  const c = t.collision || {};
  const direct = c.mode === 'disabled';
  const collision = $('t-collision');
  const enforced = c.mode === 'motion';
  // Two-arm bound: the margin left between the arms' stopping boxes, or why they hold.
  const detail = c.clear == null ? '—'
    : enforced && c.holding ? `holding · ${c.holding}`
    : enforced ? (c.clearance_m != null && c.clearance_m < .025 ? `${Math.round(c.clearance_m * 1000)} mm margin` : 'clear')
    : !c.clear ? `contact: ${(c.shapes || []).join(' · ')}`
    : c.distance_m != null ? `${Math.round(c.distance_m * 1000)} mm to keep-out` : 'clear';
  collision.textContent = direct ? 'Disabled' : running
    ? `${c.mode === 'advisory' ? 'advisory · ' : enforced ? 'two-arm · ' : ''}${detail}` : c.mode || '—';
  collision.className = enforced ? (c.holding ? 'near' : '') : c.clear === false ? 'bad' : c.distance_m != null ? 'near' : '';
  collision.title = direct ? 'No collision detection. Joint, speed, freshness and receiver limits still apply.' : enforced
    ? 'Enforced: every move is certified clear of the other arm, with each joint\'s tracking bound and stop; '
      + 'a move that could bring the arms together is slowed, else held. The body is not checked.'
    : c.mode !== 'advisory' ? ''
    : live ? 'Advisory: the target pose is checked every cycle, but nothing stops the arm on a contact.'
    : 'Advisory: the target pose is checked every cycle but the virtual receiver is not blocked.';
  // A running session's law as its receiver reports it; otherwise the law the next session starts with.
  const [lawLabel, lawTitle] = CONTROL_LAWS[t.control_law] || [], control = $('t-control');
  const ignored = !t.control_law_next && running && t.tuning?.enabled
    && (t.control_law === 'direct-velocity-feedforward') !== (t.tuning.velocity_feedforward === true);
  control.textContent = lawLabel || '—';
  if (lawLabel && t.control_law_next)
    control.append(Object.assign(document.createElement('small'), {textContent: 'next session'}));
  control.className = ignored ? 'bad' : '';
  control.title = !lawLabel ? '' : (ignored ? "This session's receiver is not running Motor tuning's velocity "
      + 'feed-forward setting. ' : '') + lawTitle
    + (t.control_law_next && t.tuning?.enabled ? ' Switch it in Motor tuning, between sessions.' : '');
  $('t-message').textContent = t.message || '';
  const where = t.target === 'jetson' ? "ON MINERVA'S JETSON" : 'ON THIS PC';
  const eyebrow = $('t-eyebrow');
  eyebrow.textContent = live
    ? `TELEOP · LIVE ${choice.arm === 'both' ? 'BOTH ARMS' : choice.arm.toUpperCase() + ' ARM'} ${where} · MOTORS MOVE · FOOT SWITCH IN REACH`
    : `TELEOP · ${last?.mode === 'simulation' ? 'SIMULATED MINERVA' : 'VIRTUAL RECEIVER'}${choice.collision === 'motion' ? ' · TWO-ARM BOUND' : ''} ${where} · NO MOTORS MOVE`;
  eyebrow.className = `eyebrow ${live ? 'live' : 'warn'}`;
  for (const button of document.querySelectorAll('#t-target button')) {
    button.setAttribute('aria-pressed', String(button.dataset.target === t.target));
    button.disabled = running || starting || (live && button.dataset.target !== 'jetson')
      || (last?.mode === 'simulation' && button.dataset.target !== 'pc');
    button.hidden = last?.mode === 'simulation' && button.dataset.target !== 'pc';
  }
  for (const button of document.querySelectorAll('#t-receiver-choice button')) {
    const liveButton = button.dataset.receiver === 'live';
    const choiceKey = liveButton ? button.dataset.arm : 'virtual';
    button.hidden = (last?.mode === 'simulation' && liveButton) || (direct && !!button.dataset.collision)
      || (Array.isArray(t.receiver_choices) && !t.receiver_choices.includes(choiceKey));
    if (!button.dataset.collision) button.title = liveButton
      ? `${button.dataset.arm === 'both' ? 'Both arms' : `The ${button.dataset.arm} arm`}; ${direct ? 'no collision detection' : button.dataset.arm === 'both' ? 'two-arm collision bound enforced' : 'collision checks advisory'}. Loop on Minerva's Jetson only.`
      : `Virtual receiver; ${direct ? 'no collision detection' : 'collision checks advisory'}. No motors move.`;
    button.setAttribute('aria-pressed', String(liveButton ? live && choice.arm === button.dataset.arm
      : !live && (choice.collision || null) === (button.dataset.collision || null)));
    button.disabled = running || starting || button.hidden
      || (liveButton && (t.target !== 'jetson' || !t.live_available || !tether
        || (t.trial && button.dataset.arm !== (t.trial.motor.startsWith('left_') ? 'left' : 'right'))));
  }
  // Say why live is unavailable, instead of leaving the operator to guess at greyed-out buttons.
  const note = last?.mode === 'simulation' ? 'Simulation only · physical robot operations are unavailable in this process.'
    : running || starting ? '' : !t.live_available ? 'Live teleop needs a live block in the teleop profile.'
    : !tether ? 'Live teleop runs over the wired tether: choose Via Tether at the top.'
    : t.target !== 'jetson' ? "Live teleop runs its loop on Minerva's Jetson." : '';
  $('t-choice-note').textContent = note;
  $('t-choice-note').hidden = !note;
  if (pendingConfirm && (pendingConfirm.kind === 'start'
      ? running || starting || !live || choice.arm !== pendingConfirm.arm : !running && !starting)) closeConfirm();
  const session = $('teleop-session');
  session.textContent = running || starting ? 'Stop session' : live ? 'Start LIVE session' : 'Start session';
  session.className = running || starting || live ? 'danger' : 'primary';
  session.disabled = starting;
  for (const button of document.querySelectorAll('.lifecycle button'))
    button.disabled = !running || (!!t.fault && button.dataset.key !== 'H')
      || (!!t.recording_fault && !['H', 'X'].includes(button.dataset.key));
  $('t-engage').innerHTML = ENGAGED.includes(t.state) ? 'Hold <kbd>T</kbd>' : 'Engage <kbd>T</kbd>';
  renderManual(t);
  $('t-tuning-summary').textContent=t.tuning?.enabled
    ? `${t.tuning.rows.filter(row=>row.follow).length} joints following · revision ${t.tuning.revision} · edit in Hold, Apply, then Engage.`
    : 'Inspect session speeds and receiver limits in Settings.';

  const container = $('teleop-joints');
  if (!t.joints.length) {
    if (tcards.size) { container.replaceChildren(); tcards.clear(); }
    return;
  }
  if (!tcards.size) {
    for (const [arm, title] of [['left', 'LEFT ARM'], ['right', 'RIGHT ARM']]) {
      if (!t.joints.some(j => j.motor.startsWith(arm))) continue;  // A live session drives one arm.
      const heading = document.createElement('div');
      heading.className = 'arm';
      heading.textContent = title;
      container.append(heading);
      for (const joint of t.joints.filter(j => j.motor.startsWith(arm))) {
        const card = buildTeleop(joint);
        tcards.set(joint.motor, card);
        container.append(card);
      }
    }
  }
  const editable = running && !ENGAGED.includes(t.state);
  for (const joint of t.joints) {
    const card = tcards.get(joint.motor);
    if (card) updateTeleop(card, joint, editable);
  }
}

async function sendKey(key) {
  // Never dropped behind another pending action: H and X must always get through.
  try {
    render(await request('/api/helm/teleop/key', {key}));
  } catch (error) {
    say(error.message, 'bad');
  }
}

function onKey(event) {
  if (event.ctrlKey || event.metaKey || event.altKey || event.repeat) return;
  const stopKey = ['H', 'X'].includes(event.key.toUpperCase());
  const numericInput = event.target?.tagName === 'INPUT' && ['number','checkbox'].includes(event.target.type)
    && event.target.closest('#panel-settings');
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target?.tagName) && !(numericInput && stopKey)) return;
  // X and H reach a running sweep from the Check tab too, where Minerva Check's live sweep is started.
  const sweeping = ['running', 'starting'].includes(last?.tune?.phase);
  if (document.body.dataset.tab === 'tune' || (sweeping && (stopKey || document.body.dataset.tab === 'check'))) {
    const key = event.key.length === 1 ? event.key.toUpperCase() : '';
    if (key === 'X' || key === 'H') {
      event.preventDefault();
      tuneKey(key);
    }
    return;
  }
  if (!['teleop','settings'].includes(document.body.dataset.tab) && !stopKey) return;
  if (!['running','starting'].includes(last?.teleop?.phase)) return;
  const key = event.key.length === 1 ? event.key.toUpperCase() : '';
  if ('IETHX'.includes(key) && key) {
    event.preventDefault();
    sendKey(key);
  } else if (key === 'Z') {
    event.preventDefault();
    setZero();
  }
}
document.addEventListener('keydown', onKey);
// The 3D view is same-origin: keys pressed while it has focus still count.
$('model-view').addEventListener('load', () => {
  try { $('model-view').contentWindow.document.addEventListener('keydown', onKey); } catch { /* not ready */ }
});
for (const button of document.querySelectorAll('.lifecycle button[data-key]')) button.onclick = () => sendKey(button.dataset.key);
async function setZero() {
  // Like H and X, never dropped behind another pending action.
  try {
    render(await request('/api/helm/teleop/zero', {}));
    say('Leader zero set and saved.', 'good');
  } catch (error) {
    say(error.message, 'bad');
  }
}
$('t-zero').onclick = setZero;
for (const button of document.querySelectorAll('#t-target button')) {
  button.onclick = () => act('/api/helm/teleop/target', {target: button.dataset.target},
    `The loop will run on ${button.textContent}.`);
}
for (const button of document.querySelectorAll('#via button')) {
  button.onclick = () => act('/api/helm/connection', {connection: button.dataset.connection},
    button.dataset.connection === 'tether' ? 'Helm reaches Minerva over the USB Ethernet tether.'
      : "Helm reaches Minerva over Wi-Fi: stopped reads and virtual teleop only.");
}
for (const button of document.querySelectorAll('#t-receiver-choice button')) {
  const arm = button.dataset.arm, collision = button.dataset.collision;
  button.onclick = () => act('/api/helm/teleop/receiver',
    arm ? {mode: 'live', arm} : collision ? {mode: 'virtual', collision} : {mode: 'virtual'},
    arm === 'both' ? 'Live on both arms: their motors are powered once the session reaches Idle.'
      : arm ? `Live on the ${arm} arm: its motors are powered once the session reaches Idle.`
      : collision ? 'Virtual receiver with the two-arm bound enforced: no motors move.' : 'Virtual receiver: no motors move.');
}
// Asked in the page, not with window.confirm: embedded browsers (such as an IDE or
// assistant preview pane) suppress native dialogs, and confirm() then returns false.
function askConfirm(kind, arm, title, items, action, onConfirm) {
  pendingConfirm = {kind, arm};
  $('t-confirm-title').textContent = title;
  $('t-confirm-list').replaceChildren(...items.map(text => Object.assign(document.createElement('li'), {textContent: text})));
  const go = $('t-confirm-go');
  go.textContent = action;
  go.onclick = () => { closeConfirm(); onConfirm(); };
  $('t-confirm').hidden = false;
  $('t-confirm-cancel').focus();
}
function closeConfirm() {
  pendingConfirm = null;
  $('t-confirm').hidden = true;
}
$('t-confirm-cancel').onclick = closeConfirm;
$('teleop-session').onclick = () => {
  const t = last?.teleop;
  if (['running', 'starting'].includes(t?.phase)) {
    const stop = () => act('/api/helm/teleop/stop', {}, 'Teleop stopped.');
    const powered = t.live && ['HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN'].includes(t.state);
    if (!powered) {
      closeConfirm();
      stop();
      return;
    }
    const arms = t.live.arm === 'both' ? 'Both arms are' : `The ${t.live.arm} arm is`;
    askConfirm('stop', t.live.arm, `${arms} powered (${t.state}). Stop the session?`,
      ['Lowered slowly to the reference first (as Shutdown X does), then disabled.',
       'Disable (H) instead removes torque at once: the arms go limp where they are.'], 'Stop', stop);
    return;
  }
  const choice = t?.receiver_choice || {mode: 'virtual'};
  if (choice.mode !== 'live') {
    act('/api/helm/teleop/start', {}, 'Starting: loading the collision model and opening the leaders.');
    return;
  }
  const arm = choice.arm, other = arm === 'left' ? 'right' : 'left';
  const direct = t?.collision?.mode === 'disabled';
  if (arm === 'both') {
    askConfirm('start', arm, 'Start LIVE teleop on both arms?', [
      'Both arms\' motors are powered when you press Idle (I).',
      'The foot switch (e-stop) is in reach.',
      'Both arms hang straight down in their reference pose, grippers closed, untouched until Idle.',
      direct ? 'Collision detection is disabled. Nothing checks the arms against each other or the body.'
      : 'Collision checks of one arm against the other are enforced: a move that could bring them together is '
        + 'slowed, else held. The hands stop about 30 cm apart. Nothing checks the arms against the body.',
    ], 'Start LIVE on both arms', () => act('/api/helm/teleop/start', {confirm: true},
      'Starting LIVE on both arms: checking that they are still and in their reference pose, then the receiver.'));
    return;
  }
  askConfirm('start', arm, `Start LIVE teleop on the ${arm} arm?`, [
    'Its motors are powered when you press Idle (I).',
    'The foot switch (e-stop) is in reach.',
    `The ${arm} arm hangs straight down in its reference pose, gripper closed, untouched until Idle.`,
    `The ${other} arm stays disabled.`,
    direct ? 'Collision detection is disabled. Nothing checks the arm against the body or the other arm.'
      : 'Collision checks are advisory: nothing stops the arm from reaching the body or the other arm.',
  ], `Start LIVE on the ${arm} arm`, () => act('/api/helm/teleop/start', {confirm: true},
    `Starting LIVE on the ${arm} arm: checking that it is still and in its reference pose, then the receiver.`));
};

// -- Tune: the servo sweep ------------------------------------------------------------------

let tuneJoints = null;  // The joints chosen to sweep; null = every joint of the arm.
let tuneSaving = new Set();  // Result cards checked for saving.
let tuneConfirm = null;
const ucards = new Map();
const fmt = (value, digits = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(digits);

function tuneMotors(state, arm) {
  // Grippers are not swept (they squeeze what they hold): Minerva Check's live sweep opens and closes them.
  return (state?.limits?.rows || []).filter(row => row.arm === arm && !row.name.endsWith('_gripper'));
}

function closeTuneConfirm() {
  tuneConfirm = null;
  $('u-confirm').hidden = true;
}

function askTune(title, items, action, onConfirm) {
  tuneConfirm = true;
  $('u-confirm-title').textContent = title;
  $('u-confirm-list').replaceChildren(...items.map(text => Object.assign(document.createElement('li'), {textContent: text})));
  const go = $('u-confirm-go');
  go.textContent = action;
  go.onclick = () => { closeTuneConfirm(); onConfirm(); };
  $('u-confirm').hidden = false;
  $('u-confirm-cancel').focus();
}
$('u-confirm-cancel').onclick = closeTuneConfirm;

async function tuneKey(key) {
  try {
    render(await request('/api/helm/tune/key', {key}));
    say(key === 'H' ? 'Disabled.' : 'Finishing: back to the reference, then disabled.', 'good');
  } catch (error) {
    say(error.message, 'bad');
  }
}

function tuneCard(motor, result) {
  const card = document.createElement('div');
  card.className = 'ucard';
  card.innerHTML = `<div class="ucard-top"><b></b><span class="state"></span></div>
    <div class="grid"></div><div class="measured"></div>
    <details><summary>How it was chosen</summary><ul class="notes"></ul></details>
    <label hidden><input type="checkbox" class="pick"> Save this joint</label>`;
  card.querySelector('.pick').onchange = event => {
    if (event.target.checked) tuneSaving.add(motor); else tuneSaving.delete(motor);
    $('u-save').disabled = !tuneSaving.size;
  };
  return card;
}

function updateTuneCard(card, motor, result, current) {
  card.dataset.state = result.status;
  card.querySelector('b').textContent = result.label || motor;
  const r = result.recommendation, v = result.verification, t = result.torque;
  card.querySelector('.state').textContent = current ? 'measuring now'
    : {pending: 'waiting', running: 'measuring', done: v?.passed ? 'done · verified' : 'done · not verified',
       stopped: 'stopped'}[result.status] || result.status;
  const now = result.current || {};
  const cells = [
    ['Stiffness', r ? `${fmt(r.hold_kp, 1)}` : '—', now.hold_kp],
    ['Damping', r ? `${fmt(r.hold_kd, 2)}` : '—', now.hold_kd],
    ['Speed', r?.max_velocity ? `${fmt(r.max_velocity)} rad/s` : '—', now.max_velocity],
    ['Braking', r?.stop_acceleration ? `${fmt(r.stop_acceleration, 0)} rad/s²` : '—', now.stop_acceleration],
    ['Tracking', r?.max_tracking_error ? `${fmt(r.max_tracking_error, 3)} rad` : '—', now.max_tracking_error],
  ];
  card.querySelector('.grid').innerHTML = cells.map(([name, value, before]) =>
    `<span>${name}<b>${value}</b>${before != null ? `<small>now ${fmt(before, before < 1 ? 3 : 1)}</small>` : ''}</span>`).join('');
  card.querySelector('.measured').textContent = t
    ? `Gravity up to ${fmt(t.gravity_max)} Nm, friction ${fmt(t.friction)} Nm · verified ${fmt(result.verified.down)} down, `
      + `${fmt(result.verified.up)} up rad` + (v?.max_lag != null ? ` · worst lag ${fmt(v.max_lag, 3)} rad` : '')
      + (result.usable ? ` · usable ${fmt(result.usable.recommended)} of ${fmt(result.usable.swept)} rad swept `
        + `at these settings (now ${fmt(result.usable.today)})` : '')
    : 'Not measured yet.';
  const notes = [...(r?.notes || []), ...(result.events || []).map(e => `Guard: ${e.text}`)];
  card.querySelector('.notes').replaceChildren(...notes.map(text => Object.assign(document.createElement('li'), {textContent: text})));
  const savable = r?.gains_met && v?.passed;
  const pick = card.querySelector('label');
  pick.hidden = !savable;
  card.querySelector('.pick').checked = tuneSaving.has(motor);
}

function renderTune(u, state) {
  const tab = document.querySelector('.tab[data-tab=tune]');
  tab.disabled = !u;
  if (!u) {
    if (document.body.dataset.tab === 'tune') showTab('check');
    return;
  }
  const running = ['running', 'starting'].includes(u.phase);
  const choice = u.receiver_choice || {mode: 'live', arm: 'right'};
  const live = choice.mode === 'live';
  const what = u.mode === 'verify' ? 'MINERVA CHECK LIVE JOINT SWEEP' : u.mode === 'return' ? 'RETURN TO REFERENCE'
    : 'SERVO SWEEP';
  $('u-eyebrow').textContent = live
    ? `${what} · LIVE ${choice.arm.toUpperCase()} ARM · MOTORS MOVE ON THEIR OWN · FOOT SWITCH IN REACH`
    : `${what} · VIRTUAL RECEIVER · NO MOTORS MOVE`;
  $('u-eyebrow').className = `eyebrow ${live ? 'live' : 'warn'}`;
  for (const button of document.querySelectorAll('#u-arm button')) {
    button.setAttribute('aria-pressed', String(button.dataset.arm === choice.arm));
    button.disabled = running;
  }
  const motors = tuneMotors(state, choice.arm);
  const chosen = new Set(u.joints_choice || motors.map(row => row.name));
  const joints = $('u-joints');
  const layout = motors.map(row => row.name).join(',');
  if (joints.dataset.layout !== layout) {
    joints.dataset.layout = layout;
    joints.querySelectorAll('button').forEach(b => b.remove());
    const everyJoint = document.createElement('button');
    everyJoint.dataset.all = '1';
    everyJoint.textContent = 'All';
    everyJoint.title = 'Sweep every joint of the arm';
    everyJoint.onclick = () => act('/api/helm/tune/configure', {arm: choice.arm, joints: null});
    joints.append(everyJoint);
    for (const row of motors) {
      const button = document.createElement('button');
      button.dataset.motor = row.name;
      button.textContent = row.label.replace(/^(Left|Right) /, '');
      button.title = row.label;
      button.onclick = () => {
        const names = motors.map(r => r.name);
        const now = last?.tune?.joints_choice;
        let list;
        if (!now) {
          list = [row.name];  // From every joint, a click picks just this one.
        } else {
          const current = new Set(now);
          if (current.has(row.name)) current.delete(row.name); else current.add(row.name);
          list = names.filter(name => current.has(name));
          if (!list.length) return;  // At least one joint; All brings the rest back.
        }
        act('/api/helm/tune/configure', {arm: choice.arm, joints: list.length === names.length ? null : list});
      };
      joints.append(button);
    }
  }
  const every = !u.joints_choice;
  for (const button of joints.querySelectorAll('button')) {
    const on = button.dataset.all ? every : !every && chosen.has(button.dataset.motor);
    button.classList.toggle('off', !on && !every);
    button.setAttribute('aria-pressed', String(on));
    button.disabled = running;
  }
  const progress = u.progress;
  $('u-phase').textContent = running ? (u.stage || u.phase) : {stopped: 'Stopped', fault: 'Fault'}[u.phase] || u.phase;
  $('u-done').textContent = progress ? `${progress.joints_done}/${progress.joints}` : '—';
  $('u-cycle').textContent = u.cycle_ms ? `${u.cycle_ms.p50.toFixed(1)} / ${u.cycle_ms.p99.toFixed(1)} ms` : '—';
  // From the receiver's own status, running or not: after a fault it reads off once the receiver has disabled.
  $('u-motors').textContent = u.motors_enabled == null ? '—' : u.motors_enabled ? 'on' : 'off';
  $('u-message').textContent = (u.fault ? `${u.fault} · ` : '') + (u.message || '');
  $('u-events').replaceChildren(...(progress?.events || []).map(e =>
    Object.assign(document.createElement('li'), {textContent: `${e.task}: ${e.text}`})));
  $('u-finish').disabled = !running;
  $('u-disable').disabled = !running;
  const session = $('u-session');
  session.textContent = running ? 'Stop session' : live ? 'Start LIVE sweep' : 'Start sweep';
  session.disabled = u.phase === 'starting';
  $('u-return').disabled = running || !live;
  if (tuneConfirm && running) closeTuneConfirm();

  const container = $('tune-results');
  const results = u.results || {};
  const order = Object.keys(results);
  if (container.dataset.layout !== order.join(',')) {
    container.dataset.layout = order.join(',');
    container.replaceChildren();
    ucards.clear();
    for (const motor of order) {
      const card = tuneCard(motor, results[motor]);
      ucards.set(motor, card);
      container.append(card);
    }
  }
  for (const motor of order) updateTuneCard(ucards.get(motor), motor, results[motor], progress?.motor === motor && running);
  for (const motor of [...tuneSaving]) {
    if (!results[motor]?.recommendation?.gains_met || !results[motor]?.verification?.passed) tuneSaving.delete(motor);
  }
  $('u-save').disabled = running || !tuneSaving.size;
}

for (const button of document.querySelectorAll('#u-arm button')) {
  button.onclick = () => act('/api/helm/tune/configure', {arm: button.dataset.arm}, `The sweep will run on the ${button.dataset.arm} arm.`);
}
$('u-finish').onclick = () => tuneKey('X');
$('u-disable').onclick = () => tuneKey('H');
$('u-save').onclick = () => {
  const motors = [...tuneSaving];
  act('/api/helm/tune/save', {motors}, `Saved ${motors.length} joint${motors.length > 1 ? 's' : ''} to the servo calibration; live teleop uses it from its next session. Deploy before the next Minerva Check: the release carries config/devices.`);
};
$('u-return').onclick = () => {
  const arm = (last?.tune?.receiver_choice || {arm: 'right'}).arm;
  act('/api/helm/tune/return', {arm}, () => {
    const offer = last?.tune?.return_offer;
    const moving = (offer?.joints || []).filter(joint => joint.moves);
    if (!moving.length) return `The ${arm} arm is already at its reference.`;
    const other = arm === 'left' ? 'right' : 'left';
    const offsets = moving.map(joint => `${joint.label} ${joint.offset >= 0 ? '+' : ''}${joint.offset.toFixed(2)} rad`);
    askTune(`Return the ${arm} arm to its reference?`, [
      'The 3D view shows the arm as its drives read it now. Compare it with the real arm: they must match.',
      `Off its reference: ${offsets.join(', ')}.`,
      `The whole arm returns together along one line, the joint farthest out at ${offer.speed} rad/s: about ` +
        `${Math.ceil(offer.seconds)} s. Then the arm is disabled, hanging.`,
      "The foot switch (e-stop) is in your hand throughout, and the arm's path is clear.",
      `The ${other} arm stays disabled. Helm disconnects from the motors to start.`,
    ], `Return the ${arm} arm`, () => act('/api/helm/tune/return', {arm, confirm: true},
      `Returning the ${arm} arm: checking it is still where you confirmed it.`));
    return 'Compare the 3D view with the arm, then confirm.';
  });
};
$('u-session').onclick = () => {
  const u = last?.tune;
  if (['running', 'starting'].includes(u?.phase)) {
    askTune('Stop the sweep session?', [
      'The arm is lowered slowly to the reference first, then disabled; the joint under test is not finished.',
      'Disable (H) instead removes torque at once: the arm goes limp where it is.'], 'Stop',
      () => act('/api/helm/tune/stop', {}, 'Sweep stopped: the arm was lowered and disabled.'));
    return;
  }
  const choice = u?.receiver_choice || {mode: 'live', arm: 'right'};
  if (choice.mode !== 'live') {
    act('/api/helm/tune/start', {}, 'Starting the sweep on the virtual receiver.');
    return;
  }
  const other = choice.arm === 'left' ? 'right' : 'left';
  const motors = tuneMotors(last, choice.arm);
  const picked = u?.joints_choice ? motors.filter(row => u.joints_choice.includes(row.name)) : motors;
  const names = picked.map(row => row.label.replace(/^(Left|Right) /, '')).join(', ');
  askTune(`Start the LIVE servo sweep on the ${choice.arm} arm?`, [
    u?.joints_choice ? `Joints: ${names} only.` : `Joints: all ${motors.length}: ${names}.`,
    `The motors move on their own, joint by joint, about 5 to 7 minutes per joint (${picked.length} joint${picked.length === 1 ? '' : 's'}).`,
    'The foot switch (e-stop) is in your hand throughout.',
    `The ${choice.arm} arm hangs straight down in its reference pose, gripper closed, and its path is clear.`,
    `The ${other} arm stays disabled.`,
    'Each move is checked against the collision model first; guards stop a task on lag, effort or heat.',
  ], `Start LIVE sweep on the ${choice.arm} arm`, () => act('/api/helm/tune/start', {confirm: true},
    `Starting the sweep on the ${choice.arm} arm: checking it is still and in its reference pose.`));
};

function closeCheckConfirm() {
  $('c-confirm').hidden = true;
}
$('c-confirm-cancel').onclick = closeCheckConfirm;

function askCheck(arm) {
  const other = arm === 'left' ? 'right' : 'left';
  $('c-confirm-title').textContent = `Run the live joint sweep on the ${arm} arm?`;
  $('c-confirm-list').replaceChildren(...[
    'The motors move on their own: each joint out and back, then across its range at its settings; the gripper opens fully and closes.',
    'The foot switch (e-stop) is in your hand throughout.',
    `The ${arm} arm hangs straight down in its reference pose, gripper closed, and its path is clear.`,
    `The ${other} arm stays disabled.`,
  ].map(text => Object.assign(document.createElement('li'), {textContent: text})));
  const go = $('c-confirm-go');
  go.textContent = `Sweep the ${arm} arm`;
  go.onclick = () => {
    closeCheckConfirm();
    act('/api/helm/check/live', {arm, confirm: true}, () => {
      showTab('tune');  // Its progress, Finish (X) and Disable (H) are there.
      return `Live joint sweep of the ${arm} arm starting: checking it is still and in its reference pose.`;
    });
  };
  $('c-confirm').hidden = false;
  $('c-confirm-cancel').focus();
}

function showTab(name) {
  const chosen = document.querySelector(`.tab[data-tab=${name}]`);
  if (!chosen || chosen.disabled) name = 'check';
  document.body.dataset.tab = name;
  for (const tab of document.querySelectorAll('.tab[data-tab]')) {
    if (tab.dataset.tab === name) tab.setAttribute('aria-current', 'page'); else tab.removeAttribute('aria-current');
  }
  for (const panel of ['check', 'limits', 'leaders', 'teleop', 'tune', 'settings']) $(`panel-${panel}`).hidden = name !== panel;
  if(name==='settings' && typeof settingsPanel !== 'undefined') settingsPanel.refresh();
  try { localStorage.setItem('helm-tab', name); } catch { /* storage unavailable */ }
}
const settingsPanel = new SettingsPanel({request,navigate:showTab,onDisplay:publishTelemetry});
$('open-motor-settings').onclick=()=>{showTab('settings');settingsPanel.show('motors');};
$('open-display').onclick=()=>{showTab('settings');settingsPanel.show('display');};
function publishTelemetry() {
  const frame=$('model-view').contentWindow;
  const telemetry=last?.telemetry;
  if(frame && !modelViewerPaused) frame.postMessage({type:'helm-viewer-overlay',version:1,settings:settingsPanel.display,telemetry},location.origin);
  const reading=(telemetry?.rows||[]).filter(row=>Number.isFinite(row.position));
  $('model-source').textContent=reading.length
    ? `${reading[0].source==='simulated'?'Virtual feedback':'Measured feedback'} · ${telemetry.stale?'stale':Number.isFinite(telemetry.age_seconds)?Math.round(telemetry.age_seconds*1000)+' ms old':'age unavailable'}`
    : 'No joint readings · reference pose';
}
$('model-view').addEventListener('load',()=>publishTelemetry());
window.addEventListener('message',event=>{
  if(event.origin!==location.origin||event.source!==$('model-view').contentWindow) return;
  if(event.data?.type==='helm-viewer-selection'&&event.data.version===1
    && (event.data.motor===''||(last?.limits?.rows||[]).some(row=>row.name===event.data.motor)))
    settingsPanel.selectMotor(event.data.motor);  // '' is no joint selected.
});
function renderGlobalControls(state) {
  const sweeping=['running','starting'].includes(state.tune?.phase);
  const running=['running','starting'].includes(state.teleop?.phase);
  const visible=sweeping||running;
  $('global-controls').hidden=!visible;document.body.classList.toggle('has-controls',visible);
  $('global-session-state').textContent=sweeping?`Servo task · ${state.tune.phase}`:`Teleop · ${state.teleop?.state||state.teleop?.phase||'stopped'}`;
  $('global-hold').hidden=sweeping;
  $('global-hold').disabled=!running||!!state.teleop?.fault||!!state.teleop?.recording_fault||!['HOLD','ENGAGING','TELEOP'].includes(state.teleop?.state);
  $('global-hold').innerHTML=ENGAGED.includes(state.teleop?.state)?'Hold <kbd>T</kbd>':'Engage <kbd>T</kbd>';
  $('global-shutdown').disabled=!sweeping&&(!running||!!state.teleop?.fault);
  $('global-disable').disabled=!visible;
}
$('global-hold').onclick=()=>sendKey('T');
$('global-disable').onclick=()=>['running','starting'].includes(last?.tune?.phase)?tuneKey('H'):sendKey('H');
$('global-shutdown').onclick=()=>['running','starting'].includes(last?.tune?.phase)?tuneKey('X'):sendKey('X');
document.addEventListener('keydown',event=>{if((event.ctrlKey||event.metaKey)&&event.key===','){event.preventDefault();showTab('settings');}});
for (const tab of document.querySelectorAll('.tab[data-tab]')) tab.onclick = () => showTab(tab.dataset.tab);
try { showTab(localStorage.getItem('helm-tab') || 'check'); } catch { showTab('check'); }

$('connect').onclick = () => {
  const viewing = last?.live_view ? last.live_view.enabled : last?.link?.running;
  act(viewing ? '/api/helm/disconnect' : '/api/helm/connect', {},
    viewing ? 'Disconnected. CAN links are down; the live view stays off until Connect.'
      : 'Connecting. Motors stay disabled.');
};
$('save').onclick = () => act('/api/helm/limits/save', {}, () => `Saved to ${last.limits.path}.`);
$('c-run').onclick = () => act('/api/helm/check/run', {}, 'Minerva Check started. Motors stay disabled.');
$('l-connect').onclick = () => {
  const connected = ['reading', 'connecting'].includes(last?.leaders?.state);
  act(connected ? '/api/helm/leaders/disconnect' : '/api/helm/leaders/connect', {},
    connected ? 'Leaders disconnected.' : 'Connecting the leaders (read-only).');
};
$('l-capture').onclick = () => act('/api/helm/leaders/reference', {}, 'Leader zero set and saved.');
$('l-save').onclick = () => act('/api/helm/leaders/save', {}, () => `Saved to ${last.leaders.path}.`);
setTimeout(() => { lockNoticeShown = true; }, 15000);

async function poll() {
  try {
    if (!busy) render(await request('/api/helm/state'));
  } catch (error) {
    document.body.dataset.feed = 'fault';
    $('feed-text').textContent = 'Helm server unreachable';
  }
  setTimeout(poll, 250);
}
poll();
