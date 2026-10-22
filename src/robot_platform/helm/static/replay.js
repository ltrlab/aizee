// Replay: recorded episodes played back by their own clock.
//
// ReplayModel is the playhead, with no DOM: frames are shown at their recorded times, so uneven gaps between frames
// replay in real time at any speed. ReplayView renders it (episode list, cameras, scrubber, joint timeline) and
// tells subscribers about every frame it shows, the hook for later layers (3D pose, overlays, annotations).

export const ROLE_ORDER = ['left_wrist', 'head', 'right_wrist'];
export const ROLE_LABEL = {left_wrist: 'Left wrist', head: 'Head', right_wrist: 'Right wrist'};
export const SPEEDS = [0.25, 0.5, 1, 2, 4];
const PREFETCH_S = 1.2;
const JOINT_COLORS = ['#7cc4ff', '#7fdca4', '#f2c380', '#ff8b98', '#c39bff', '#7fe0dc'];

export class ReplayModel {
  constructor() {
    this.speed = 1;
    this.load([]);
  }

  load(times) {
    this.times = Array.from(times ?? [], Number);
    this.index = 0;
    this.playing = false;
    this.anchor = null;
  }

  get length() { return this.times.length; }
  get last() { return Math.max(0, this.length - 1); }
  get duration() { return this.length > 1 ? this.times[this.last] - this.times[0] : 0; }
  get time() { return this.length ? this.times[this.index] - this.times[0] : 0; }
  get atEnd() { return this.index >= this.last; }

  // The last frame recorded at or before `t` seconds from the first.
  indexAt(t) {
    if (this.length < 2) return 0;
    const target = this.times[0] + t;
    if (!(target > this.times[0])) return 0;
    if (target >= this.times[this.last]) return this.last;
    let lo = 0, hi = this.last;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.times[mid] <= target) lo = mid; else hi = mid - 1;
    }
    return lo;
  }

  seek(index, now) {
    const target = Math.round(Number(index));
    this.index = Math.min(this.last, Math.max(0, Number.isFinite(target) ? target : 0));
    if (this.playing) this.anchor = {wall: now, t: this.time};
    return this.index;
  }

  seekTime(t, now) { return this.seek(this.indexAt(t), now); }

  step(frames) {
    this.pause();
    return this.seek(this.index + frames);
  }

  play(now) {
    if (this.length < 2) return false;
    if (this.atEnd) this.index = 0;
    this.playing = true;
    this.anchor = {wall: now, t: this.time};
    return true;
  }

  pause() {
    this.playing = false;
    this.anchor = null;
  }

  toggle(now) {
    if (this.playing) {
      this.pause();
      return false;
    }
    return this.play(now);
  }

  setSpeed(speed, now) {
    if (!(speed > 0) || !Number.isFinite(speed)) throw new RangeError('Playback speed must be a positive number');
    if (this.playing) this.anchor = {wall: now, t: this.time};
    this.speed = speed;
  }

  // Advance by the wall clock; true when something to show changed (a new frame, or playback ended).
  tick(now) {
    if (!this.playing || !this.anchor) return false;
    const t = this.anchor.t + (now - this.anchor.wall) / 1000 * this.speed;
    const previous = this.index;
    if (t >= this.duration) {
      this.index = this.last;
      this.pause();
      return true;
    }
    this.index = this.indexAt(t);
    return this.index !== previous;
  }
}

export function formatSeconds(seconds) {
  if (!Number.isFinite(seconds)) return '—';
  if (seconds < 60) return `${seconds.toFixed(1)} s`;
  const minutes = Math.floor(seconds / 60);
  return `${minutes}:${(seconds - minutes * 60).toFixed(1).padStart(4, '0')}`;
}

// 'live-20261021T004507Z' -> a Date, or null.
export function sessionDate(session) {
  const match = /^live-(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})Z$/.exec(session || '');
  if (!match) return null;
  const [, y, mo, d, h, mi, s] = match.map(Number);
  return new Date(Date.UTC(y, mo - 1, d, h, mi, s));
}

// Episodes grouped by session, newest session first; within one, numbered in recording order.
export function groupEpisodes(episodes) {
  const sessions = new Map();
  for (const episode of episodes || []) {
    if (!sessions.has(episode.session)) sessions.set(episode.session, []);
    sessions.get(episode.session).push(episode);
  }
  return [...sessions.entries()]
    .sort(([a], [b]) => (a < b ? 1 : a > b ? -1 : 0))
    .map(([session, list]) => {
      const ordered = [...list].sort((a, b) => a.modified - b.modified).map((episode, i) => ({...episode, number: i + 1}));
      return {session, date: sessionDate(session), episodes: ordered.reverse()};
    });
}

// An episode's training verdict: 'keep', 'discard', or null (not reviewed, or cleared).
export function verdictOf(episode) {
  const verdict = episode?.review?.verdict;
  return verdict === 'keep' || verdict === 'discard' ? verdict : null;
}

// How many episodes are kept, discarded and still to review.
export function reviewCounts(episodes) {
  const counts = {keep: 0, discard: 0, unreviewed: 0};
  for (const episode of episodes || []) counts[verdictOf(episode) ?? 'unreviewed'] += 1;
  return counts;
}

// Whether `episode` shows under the list filter 'all', 'unreviewed', 'keep' or 'discard'.
export function shownBy(filter, episode) {
  return !filter || filter === 'all' || (verdictOf(episode) ?? 'unreviewed') === filter;
}

// The next animation frame, or a timer when the browser is not drawing the page (covered or minimized windows send
// no animation frames): playback keeps time either way, since the model plays by the wall clock.
export function nextFrame(callback, {raf = globalThis.requestAnimationFrame, timer = globalThis.setTimeout} = {}) {
  let done = false;
  const run = () => {
    if (done) return;
    done = true;
    callback();
  };
  raf?.(run);
  timer(run, 40);
}

function frameUrl(episodeId, index, role) {
  const [session, name] = episodeId.split('/');
  return `/api/helm/episodes/${encodeURIComponent(session)}/${encodeURIComponent(name)}/frame/${index}/${encodeURIComponent(role)}`;
}

// One camera: shows the latest wanted frame, never queuing more than one load behind the one in flight.
class CameraPanel {
  constructor(role, doc) {
    this.role = role;
    this.figure = doc.createElement('figure');
    this.figure.className = 'replay-camera';
    this.img = doc.createElement('img');
    this.img.alt = `${ROLE_LABEL[role] || role} camera`;
    this.caption = doc.createElement('figcaption');
    this.caption.textContent = ROLE_LABEL[role] || role;
    this.note = doc.createElement('span');
    this.note.className = 'replay-camera-note';
    this.figure.append(this.img, this.caption, this.note);
    this.current = this.wanted = null;
    this.loading = false;
    this.img.onload = () => this.settle(true);
    this.img.onerror = () => this.settle(false);
  }

  settle(ok) {
    this.loading = false;
    this.figure.dataset.state = ok ? 'shown' : 'missing';
    this.note.textContent = ok ? '' : 'Frame unavailable';
    if (this.wanted && this.wanted !== this.current) this.show(this.wanted);
  }

  show(url) {
    this.wanted = url;
    if (this.loading || url === this.current) return;
    this.loading = true;
    this.current = url;
    this.img.src = url;
  }
}

// Joint positions over the whole episode, one lane per arm and one for the grippers; click or drag to seek.
class JointTimeline {
  constructor(canvas, onSeek) {
    this.canvas = canvas;
    this.onSeek = onSeek;
    this.series = null;
    this.background = null;
    this.dragging = false;
    canvas.addEventListener('pointerdown', event => {
      if (!this.series) return;
      this.dragging = true;
      canvas.setPointerCapture?.(event.pointerId);
      this.seekTo(event, 'start');
    });
    canvas.addEventListener('pointermove', event => { if (this.dragging) this.seekTo(event, 'move'); });
    const end = event => {
      if (!this.dragging) return;
      this.dragging = false;
      this.seekTo(event, 'end');
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
  }

  seekTo(event, phase) {
    const rect = this.canvas.getBoundingClientRect();
    const fraction = Math.min(1, Math.max(0, (event.clientX - rect.left) / Math.max(1, rect.width)));
    this.onSeek(fraction * this.duration, phase);
  }

  get duration() {
    const t = this.series?.t || [];
    return t.length > 1 ? t[t.length - 1] - t[0] : 0;
  }

  setSeries(series, joints) {
    this.series = series;
    this.lanes = [];
    const lane = (label, test) => {
      const columns = joints.map((name, column) => ({name, column})).filter(({name}) => test(name));
      if (columns.length) this.lanes.push({label, columns});
    };
    lane('Left arm', name => name.startsWith('left_arm'));
    lane('Right arm', name => name.startsWith('right_arm'));
    lane('Grippers', name => name.endsWith('_gripper'));
    this.background = null;
  }

  resize() {
    const ratio = globalThis.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(this.canvas.clientWidth * ratio));
    const height = Math.max(1, Math.round(this.canvas.clientHeight * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
      this.background = null;
    }
  }

  paintBackground() {
    const {width, height} = this.canvas;
    const canvas = this.canvas.ownerDocument.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const g = canvas.getContext('2d');
    const ratio = globalThis.devicePixelRatio || 1;
    g.fillStyle = '#0e1013';
    g.fillRect(0, 0, width, height);
    const t = this.series?.t || [];
    const qpos = this.series?.qpos || [];
    if (t.length < 2 || !this.lanes.length) return canvas;
    const duration = t[t.length - 1] - t[0];
    const laneHeight = height / this.lanes.length;
    this.lanes.forEach(({label, columns}, laneIndex) => {
      const top = laneIndex * laneHeight;
      let low = Infinity, high = -Infinity;
      for (const row of qpos) for (const {column} of columns) {
        low = Math.min(low, row[column]);
        high = Math.max(high, row[column]);
      }
      const middle = (low + high) / 2;
      const span = Math.max(high - low, .1) * 1.15;  // A still joint draws flat, not magnified noise.
      const y = value => top + laneHeight * (.5 - (value - middle) / span);
      g.strokeStyle = '#2c3036';
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(0, top + .5);
      g.lineTo(width, top + .5);
      g.stroke();
      columns.forEach(({column}, i) => {
        g.strokeStyle = JOINT_COLORS[i % JOINT_COLORS.length];
        g.lineWidth = 1.25 * ratio;
        g.beginPath();
        const step = Math.max(1, Math.floor(qpos.length / width));
        for (let k = 0; k < qpos.length; k += step) {
          const x = (t[k] - t[0]) / duration * width;
          if (k === 0) g.moveTo(x, y(qpos[k][column])); else g.lineTo(x, y(qpos[k][column]));
        }
        g.stroke();
      });
      g.fillStyle = '#9ba1a8';
      g.font = `${11 * ratio}px system-ui, sans-serif`;
      g.fillText(label, 6 * ratio, top + 14 * ratio);
    });
    return canvas;
  }

  draw(time) {
    this.resize();
    const g = this.canvas.getContext('2d');
    if (!this.background) this.background = this.paintBackground();
    g.drawImage(this.background, 0, 0);
    const duration = this.duration;
    if (!(duration > 0)) return;
    const x = Math.round(time / duration * this.canvas.width) + .5;
    g.strokeStyle = '#e9ebee';
    g.lineWidth = Math.max(1, globalThis.devicePixelRatio || 1);
    g.beginPath();
    g.moveTo(x, 0);
    g.lineTo(x, this.canvas.height);
    g.stroke();
  }
}

export class ReplayView {
  // `elements` holds the page's nodes by role; `request(path)` fetches JSON or throws with the server's message.
  constructor({elements, request, now = () => performance.now(), frame = nextFrame, doc = document}) {
    this.e = elements;
    this.request = request;
    this.now = now;
    this.frame = frame;
    this.doc = doc;
    this.model = new ReplayModel();
    this.listing = {episodes: [], busy: null, error: null};
    this.episode = null;
    this.panels = new Map();
    this.prefetched = new Map();
    this.listeners = new Set();
    this.active = false;
    this.ticking = false;
    this.listedAt = -Infinity;
    this.timeline = new JointTimeline(elements.timeline, (t, phase) => this.scrubTime(t, phase));
    for (const speed of SPEEDS) {
      const option = doc.createElement('option');
      option.value = String(speed);
      option.textContent = `${speed}×`;
      elements.speed.append(option);
    }
    elements.speed.value = '1';
    elements.play.onclick = () => this.toggle();
    elements.back.onclick = () => this.step(-1);
    elements.forward.onclick = () => this.step(1);
    elements.refresh.onclick = () => this.refreshList(true);
    elements.speed.onchange = () => {
      this.model.setSpeed(Number(elements.speed.value), this.now());
      this.render();
    };
    elements.scrub.oninput = () => {
      this.model.seek(Number(elements.scrub.value), this.now());
      this.render();
    };
    // Training review: Keep or Discard each episode; the list shows and filters by it.
    this.filter = 'all';
    this.reviewing = false;
    if (elements.filter) elements.filter.onchange = () => {
      this.filter = elements.filter.value;
      this.renderList();
    };
    if (elements.keep) {
      elements.keep.onclick = () => this.review('keep');
      elements.discard.onclick = () => this.review('discard');
      elements.clear.onclick = () => this.review(null);
    }
    doc.addEventListener('keydown', event => this.onKey(event));
    globalThis.addEventListener?.('resize', () => this.render());
  }

  // Called with {episode, summary, series, index, time} for every frame shown.
  subscribe(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  setActive(active) {
    this.active = active;
    if (!active) {
      this.model.pause();
      this.render();
      return;
    }
    if (this.now() - this.listedAt > 10000) this.refreshList();
    this.render();
  }

  async refreshList(explicit = false) {
    this.listedAt = this.now();
    this.e.listStatus.textContent = 'Reading the episode list…';
    try {
      this.listing = await this.request('/api/helm/episodes');
    } catch (error) {
      this.listing = {...this.listing, error: error.message};
    }
    this.renderList();
    if (explicit && !this.listing.error && !this.listing.busy) this.e.listStatus.textContent = 'Episode list updated.';
  }

  renderList() {
    const {episodes, busy, error} = this.listing;
    const counts = reviewCounts(episodes);
    this.e.listStatus.textContent = busy || error || (episodes.length
      ? `${episodes.length} episodes · ${counts.keep} kept · ${counts.discard} discarded · ${counts.unreviewed} to review`
      : 'No recorded episodes yet.');
    this.e.listStatus.dataset.tone = error ? 'bad' : busy ? 'warn' : '';
    this.e.list.replaceChildren();
    // Numbered within the whole session first, so an episode keeps its number under any filter.
    for (const group of groupEpisodes(episodes)) {
      const shown = group.episodes.filter(episode => shownBy(this.filter, episode));
      if (!shown.length) continue;
      const heading = this.doc.createElement('div');
      heading.className = 'replay-session';
      heading.textContent = group.date ? group.date.toLocaleString([], {dateStyle: 'medium', timeStyle: 'short'}) : group.session;
      this.e.list.append(heading);
      for (const episode of shown) {
        const button = this.doc.createElement('button');
        button.type = 'button';
        button.className = 'replay-episode';
        button.dataset.episode = episode.id;
        button.setAttribute('aria-current', String(this.episode?.id === episode.id));
        const title = this.doc.createElement('b');
        title.textContent = `Episode ${episode.number}`;
        const badge = this.doc.createElement('span');
        badge.className = 'replay-badge';
        if (episode.error) {
          badge.textContent = 'unreadable';
          badge.dataset.tone = 'bad';
        } else if (episode.partial || !episode.complete) {
          badge.textContent = 'partial';
          badge.dataset.tone = 'warn';
        } else {
          badge.textContent = 'complete';
        }
        const meta = this.doc.createElement('span');
        meta.className = 'replay-meta';
        meta.textContent = [formatSeconds(episode.duration ?? NaN), `${episode.frames} frames`,
          `${(episode.size / 1e6).toFixed(0)} MB`].join(' · ');
        button.append(title, badge, meta);
        const verdict = verdictOf(episode);
        if (verdict) {
          const mark = this.doc.createElement('span');
          mark.className = 'replay-verdict';
          mark.dataset.verdict = verdict;
          mark.textContent = verdict === 'keep' ? 'kept' : 'discarded';
          if (episode.review?.note) mark.title = episode.review.note;
          button.append(mark);
        }
        button.onclick = () => this.load(episode.id);
        this.e.list.append(button);
      }
    }
    this.renderReview();
  }

  // The selected episode's training review: its verdict and note, and the Keep, Discard and Clear buttons.
  renderReview() {
    const {e} = this;
    if (!e.review) return;
    e.review.hidden = !this.episode;
    if (!this.episode) return;
    const entry = this.listing.episodes.find(item => item.id === this.episode.id);
    const verdict = verdictOf(entry);
    const complete = !!this.episode.summary?.complete && !entry?.partial;
    e.verdict.textContent = verdict === 'keep' ? 'Kept for training' : verdict === 'discard' ? 'Discarded' : 'Not reviewed';
    e.verdict.dataset.verdict = verdict || '';
    if (this.doc.activeElement !== e.note && !this.reviewing) e.note.value = entry?.review?.note || '';
    e.keep.disabled = this.reviewing || !complete;
    e.keep.title = complete ? 'Use this episode for training' : 'Only a complete episode can be kept for training';
    e.discard.disabled = this.reviewing;
    e.clear.disabled = this.reviewing || !verdict;
  }

  async review(verdict) {
    if (!this.episode || this.reviewing) return;
    const id = this.episode.id;
    this.reviewing = true;
    this.renderReview();
    this.e.reviewStatus.textContent = 'Saving the review…';
    this.e.reviewStatus.dataset.tone = '';
    try {
      const path = `/api/helm/episodes/${id.split('/').map(encodeURIComponent).join('/')}/review`;
      const {review} = await this.request(path, {verdict, note: this.e.note.value.trim()});
      const entry = this.listing.episodes.find(item => item.id === id);
      if (entry) entry.review = review;
      this.e.reviewStatus.textContent = verdict === 'keep' ? 'Kept for training.'
        : verdict === 'discard' ? 'Discarded: not used for training.' : 'Review cleared.';
    } catch (error) {
      this.e.reviewStatus.textContent = error.message;
      this.e.reviewStatus.dataset.tone = 'bad';
    } finally {
      this.reviewing = false;
      this.renderList();
    }
  }

  async load(id) {
    this.model.pause();
    this.e.status.textContent = 'Loading the episode…';
    this.e.status.dataset.tone = '';
    let data;
    try {
      data = await this.request(`/api/helm/episodes/${id.split('/').map(encodeURIComponent).join('/')}`);
    } catch (error) {
      this.e.status.textContent = error.message;
      this.e.status.dataset.tone = 'bad';
      return;
    }
    this.episode = {id, ...data};
    if (this.e.reviewStatus) this.e.reviewStatus.textContent = '';
    this.model.load(data.series.t);
    this.prefetched.clear();
    const roles = ROLE_ORDER.filter(role => data.summary.roles.includes(role))
      .concat(data.summary.roles.filter(role => !ROLE_ORDER.includes(role)));
    this.panels = new Map(roles.map(role => [role, new CameraPanel(role, this.doc)]));
    this.e.cameras.replaceChildren(...[...this.panels.values()].map(panel => panel.figure));
    this.e.scrub.max = String(this.model.last);
    this.timeline.setSeries(data.series, data.summary.joints || []);
    this.e.status.textContent = '';
    this.renderList();
    this.renderDetails();
    this.render();
  }

  renderDetails() {
    const summary = this.episode?.summary;
    if (!summary) {
      this.e.details.replaceChildren();
      return;
    }
    const rows = [
      ['Duration', formatSeconds(summary.duration)],
      ['Frames', `${summary.frames} at ${summary.rate_hz ? summary.rate_hz.toFixed(1) : '—'} Hz`],
      ['Cameras', summary.roles.map(role => ROLE_LABEL[role] || role).join(', ')],
      ['File', summary.complete ? 'complete' : 'partial: recording did not finish'],
      ['Recorded', summary.collected_at ? new Date(summary.collected_at).toLocaleString() : '—'],
      ['Task', summary.language_instruction || 'not described'],
    ];
    const list = this.doc.createElement('dl');
    for (const [label, value] of rows) {
      const term = this.doc.createElement('dt');
      term.textContent = label;
      const detail = this.doc.createElement('dd');
      detail.textContent = value;
      list.append(term, detail);
    }
    this.e.details.replaceChildren(list);
  }

  toggle() {
    if (!this.episode) return;
    this.model.toggle(this.now());
    this.render();
  }

  step(frames) {
    if (!this.episode) return;
    this.model.step(frames);
    this.render();
  }

  scrubTime(t, phase) {
    if (!this.episode) return;
    if (phase === 'start') {
      this.resumeAfterScrub = this.model.playing;
      this.model.pause();
    }
    this.model.seekTime(t, this.now());
    if (phase === 'end' && this.resumeAfterScrub) this.model.play(this.now());
    this.render();
  }

  onKey(event) {
    if (!this.active || !this.episode || event.ctrlKey || event.metaKey || event.altKey) return;
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target?.tagName)) return;
    if (event.key === ' ' && event.target?.tagName === 'BUTTON') return;  // The focused button's own click.
    const second = Math.max(1, Math.round(this.model.length / Math.max(this.model.duration, 1e-9)));
    const actions = {
      ' ': () => this.toggle(),
      ArrowLeft: () => this.step(event.shiftKey ? -second : -1),
      ArrowRight: () => this.step(event.shiftKey ? second : 1),
      Home: () => { this.model.seek(0, this.now()); this.render(); },
      End: () => { this.model.seek(this.model.last, this.now()); this.render(); },
    };
    const action = actions[event.key];
    if (!action) return;
    event.preventDefault();
    action();
  }

  render() {
    const {model, e} = this;
    const loaded = !!this.episode;
    for (const control of [e.play, e.back, e.forward, e.scrub, e.speed]) control.disabled = !loaded;
    e.play.textContent = model.playing ? 'Pause' : model.atEnd && loaded ? 'Replay' : 'Play';
    e.play.setAttribute('aria-pressed', String(model.playing));
    e.title.textContent = loaded ? `${this.episodeLabel()} · ${this.episode.id.split('/')[1]}` : 'No episode selected';
    if (!loaded) {
      e.time.textContent = '—';
      return;
    }
    e.scrub.value = String(model.index);
    e.time.textContent = `${formatSeconds(model.time)} / ${formatSeconds(model.duration)} · frame ${model.index + 1} of ${model.length}`;
    for (const panel of this.panels.values()) panel.show(frameUrl(this.episode.id, model.index, panel.role));
    if (this.active) this.timeline.draw(model.time);
    if (model.playing) this.prefetch();
    const view = {episode: this.episode.id, summary: this.episode.summary, series: this.episode.series,
      index: model.index, time: model.time};
    for (const listener of this.listeners) listener(view);
    if (model.playing && !this.ticking) {
      this.ticking = true;
      this.frame(() => this.advance());
    }
  }

  advance() {
    this.ticking = false;
    if (!this.active) return;
    if (this.model.tick(this.now())) this.render();
    else if (this.model.playing) {
      this.ticking = true;
      this.frame(() => this.advance());
    }
  }

  // Ask for the next stretch of frames before the playhead needs them (complete episodes are browser-cached).
  prefetch() {
    if (!this.episode?.summary?.complete) return;
    const {model} = this;
    const until = model.indexAt(model.time + PREFETCH_S * model.speed);
    const keep = new Set();
    for (let index = model.index + 1; index <= until; index++) {
      for (const role of this.panels.keys()) {
        const url = frameUrl(this.episode.id, index, role);
        keep.add(url);
        if (!this.prefetched.has(url)) {
          const image = new Image();
          image.src = url;
          this.prefetched.set(url, image);
        }
      }
    }
    for (const url of this.prefetched.keys()) if (!keep.has(url)) this.prefetched.delete(url);
  }

  episodeLabel() {
    for (const group of groupEpisodes(this.listing.episodes)) {
      const match = group.episodes.find(episode => episode.id === this.episode?.id);
      if (match) return `Episode ${match.number}`;
    }
    return 'Episode';
  }
}
