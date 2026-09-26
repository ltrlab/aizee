// Helm operator page. Reads /api/helm/state; every action needs the session token.
const $ = id => document.getElementById(id);
const DEG = 180 / Math.PI;
const cards = new Map();
let token = null, last = null, busy = false, lockNoticeShown = false;

const signed = (q, digits = 3) => (q == null || !Number.isFinite(q)) ? '—' : `${q < 0 ? '−' : '+'}${Math.abs(q).toFixed(digits)}`;
const range = entry => `${signed(entry.min_rad, 2)} … ${signed(entry.max_rad, 2)} rad`;

async function request(path, payload) {
  const options = payload === undefined
    ? {cache: 'no-store', signal: AbortSignal.timeout(3000)}
    : {method: 'POST', cache: 'no-store', signal: AbortSignal.timeout(15000), body: JSON.stringify(payload),
       headers: {'Content-Type': 'application/json', 'X-Helm-Token': token || ''}};
  const response = await fetch(path, options);
  let data = null;
  try { data = await response.json(); } catch { data = null; }
  if (!response.ok) throw new Error(data?.error || `Helm returned HTTP ${response.status}`);
  return data;
}

const STATUS = {check: 'check-status', limits: 'action-status', leaders: 'leaders-status', teleop: 'teleop-status'};

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

function render(state) {
  if (!state) return;
  last = state;
  token = state.action_token;
  const {feed, limits, robot, session} = state;
  $('robot-name').textContent = `${robot.model} · ${robot.asset_id} · ${robot.address}`;
  document.body.dataset.feed = feed.state;
  const live = feed.state === 'reading' && feed.fresh;
  $('feed-text').textContent = {
    reading: live ? `Live · frame ${feed.sequence} · ${Math.round(feed.age_seconds * 1000)} ms` : 'Live feed stale',
    connecting: 'Connecting…', reconnecting: 'Reconnecting…', waiting: 'Waiting for motors',
    fault: 'Connection failed',
  }[feed.state] || 'Not connected';
  const connect = $('connect');
  connect.textContent = state.link.running ? 'Disconnect' : 'Connect';
  connect.className = state.link.running ? 'danger' : 'primary';

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
  renderCheck(state.check);
}

// ---- Minerva Check -------------------------------------------------------------
const CHECK_ICON = {pass: '✓', warn: '!', fail: '✕', skip: '–', pending: '·', running: '…'};
const VERDICT = {go: 'GO', no_go: 'NO GO', incomplete: 'INCOMPLETE'};
const crows = new Map();
let checkLayout = '';

function renderCheck(c) {
  const tab = document.querySelector('.tab[data-tab=check]');
  tab.disabled = !c;
  if (!c) {
    if (document.body.dataset.tab === 'check') showTab('limits');
    return;
  }
  const report = c.report;
  const run = $('c-run');
  run.disabled = c.running;
  run.textContent = c.running ? 'Checking…' : 'Run check';

  const checks = report ? report.stages.flatMap(stage => stage.checks) : [];
  const done = checks.filter(entry => !['pending', 'running'].includes(entry.status)).length;
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
  for (const stage of report ? report.stages : []) {
    for (const entry of stage.checks) {
      const row = crows.get(`${stage.id}/${entry.id}`);
      row.dataset.status = entry.status;
      row.querySelector('.cicon').textContent = CHECK_ICON[entry.status] || '?';
      row.querySelector('.cdetail').textContent = entry.status === 'running' ? 'Checking…' : entry.detail;
      row.querySelector('.ctime').textContent = entry.seconds == null ? '' : `${entry.seconds.toFixed(1)} s`;
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

function renderTeleop(t) {
  const tab = document.querySelector('.tab[data-tab=teleop]');
  tab.disabled = !t;
  tab.title = t ? '' : 'Start Helm with a teleop profile (scripts/helm.ps1 does this)';
  if (!t) {
    if (document.body.dataset.tab === 'teleop') showTab('limits');
    return;
  }
  const running = t.phase === 'running', starting = t.phase === 'starting';
  // The Start note is only true while starting; the panel message takes over.
  if (teleopPhase === 'starting' && !starting) $('teleop-status').textContent = '';
  teleopPhase = t.phase;
  document.body.dataset.teleopState = t.state || t.phase;
  $('t-state').textContent = running && t.state ? t.state : {starting: 'Starting…', stopped: 'Stopped', fault: 'Fault'}[t.phase] || t.phase;
  $('t-receiver').textContent = running && t.receiver ? t.receiver : '—';
  $('t-cycle').textContent = running && t.cycle_ms ? `${t.cycle_ms.p50.toFixed(1)} / ${t.cycle_ms.p99.toFixed(1)} ms` : '—';
  const c = t.collision || {};
  const collision = $('t-collision');
  const detail = c.clear == null ? '—'
    : !c.clear ? `contact: ${(c.shapes || []).join(' · ')}`
    : c.distance_m != null ? `${Math.round(c.distance_m * 1000)} mm to keep-out` : 'clear';
  collision.textContent = running ? `${c.mode === 'advisory' ? 'advisory · ' : ''}${detail}` : c.mode || '—';
  collision.className = c.clear === false ? 'bad' : c.distance_m != null ? 'near' : '';
  collision.title = c.mode === 'advisory'
    ? 'Advisory: the target pose is checked every cycle but the virtual receiver is not blocked.' : '';
  $('t-message').textContent = t.message || '';
  const session = $('teleop-session');
  session.textContent = running || starting ? 'Stop session' : 'Start session';
  session.className = running || starting ? 'danger' : 'primary';
  session.disabled = starting;
  for (const button of document.querySelectorAll('.lifecycle button')) button.disabled = !running || !!t.fault;
  $('t-engage').innerHTML = ENGAGED.includes(t.state) ? 'Hold <kbd>T</kbd>' : 'Engage <kbd>T</kbd>';

  const container = $('teleop-joints');
  if (!t.joints.length) {
    if (tcards.size) { container.replaceChildren(); tcards.clear(); }
    return;
  }
  if (!tcards.size) {
    for (const [arm, title] of [['left', 'LEFT ARM'], ['right', 'RIGHT ARM']]) {
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
  if (document.body.dataset.tab !== 'teleop' || event.ctrlKey || event.metaKey || event.altKey || event.repeat) return;
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target?.tagName)) return;
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
$('teleop-session').onclick = () => {
  const running = ['running', 'starting'].includes(last?.teleop?.phase);
  act(running ? '/api/helm/teleop/stop' : '/api/helm/teleop/start', {},
    running ? 'Teleop stopped.' : 'Starting: loading the collision model and opening the leaders (about 10 s).');
};

function showTab(name) {
  const chosen = document.querySelector(`.tab[data-tab=${name}]`);
  if (!chosen || chosen.disabled) name = 'check';
  document.body.dataset.tab = name;
  for (const tab of document.querySelectorAll('.tab[data-tab]')) {
    if (tab.dataset.tab === name) tab.setAttribute('aria-current', 'page'); else tab.removeAttribute('aria-current');
  }
  for (const panel of ['check', 'limits', 'leaders', 'teleop']) $(`panel-${panel}`).hidden = name !== panel;
  try { localStorage.setItem('helm-tab', name); } catch { /* storage unavailable */ }
}
for (const tab of document.querySelectorAll('.tab[data-tab]')) tab.onclick = () => showTab(tab.dataset.tab);
try { showTab(localStorage.getItem('helm-tab') || 'check'); } catch { showTab('check'); }

$('connect').onclick = () => {
  const running = last?.link?.running;
  act(running ? '/api/helm/disconnect' : '/api/helm/connect', {},
    running ? 'Disconnected. CAN links are down.' : 'Connecting. Motors stay disabled.');
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
