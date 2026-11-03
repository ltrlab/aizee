// The headset's screens drawn on 2D canvases: the head camera picture's placeholder, its ring of controls
// (xr_ring.js) with the card over it, and the alert under the view; the headset textures them in its scene
// (xr_scene.js). The status line and panel are Settings → Headset's flat copy on the operator PC (headset_mirror.js):
// the headset itself shows its status on the ring's card (the user, 2026-11-03: the floating panels got in the way).
// No three.js here.
export const TONE = {ok: '#3fbf7f', warn: '#e0a83a', bad: '#e05a5a', '': '#e9ecf1', muted: '#9aa3b2'};
export const SIZES = {picture: [1024, 768], strip: [1536, 104], panel: [1024, 768], ring: [1024, 1024],
  card: [1024, 704], alert: [1536, 480]};  // Pixels: the alert's tall enough for ALERT_LINES lines.
const BACKGROUND = 'rgba(16, 19, 24, 0.92)';
const FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif';

export function drawPlaceholder(context, text) {
  const {canvas} = context;
  context.fillStyle = '#0f1218';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = TONE.muted;
  context.font = `48px ${FONT}`;
  context.textAlign = 'center';
  context.fillText(text, canvas.width / 2, canvas.height / 2);
  context.textAlign = 'left';
}

// The status line: [[text, tone], [text, tone]], left and right (xr_status.stripText).
export function drawStrip(context, [[left, leftTone], [right, rightTone]]) {
  const {canvas} = context;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = BACKGROUND;
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.font = `500 42px ${FONT}`;
  context.fillStyle = TONE[leftTone] || TONE[''];
  context.fillText(left, 28, 68, canvas.width * .44);
  context.textAlign = 'right';
  context.fillStyle = TONE[rightTone] || TONE[''];
  context.fillText(right, canvas.width - 28, 68, canvas.width * .5);
  context.textAlign = 'left';
}

// The status panel: its rows ([label, value, tone], xr_status.panelRows) and the controls' hints.
export function drawPanel(context, rows, hints) {
  const {canvas} = context;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = BACKGROUND;
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = TONE[''];
  context.font = `700 54px ${FONT}`;
  context.fillText('Minerva head tracking', 48, 92);
  rows.forEach(([label, value, tone], index) => {
    const y = 176 + index * 66;
    context.font = `42px ${FONT}`;
    context.fillStyle = TONE.muted;
    context.fillText(label, 48, y);
    context.fillStyle = TONE[tone] || TONE[''];
    context.font = `600 42px ${FONT}`;
    context.fillText(value, 360, y, canvas.width - 400);
  });
  context.font = `32px ${FONT}`;
  context.fillStyle = TONE.muted;
  hints.forEach((hint, index) => context.fillText(hint, 48, 640 + index * 42, canvas.width - 96));
}

// `text` broken into lines no wider than `width` pixels in the context's font.
function wrap(context, text, width) {
  const lines = [];
  let line = '';
  for (const word of String(text).split(/\s+/)) {
    const next = line ? `${line} ${word}` : word;
    if (line && context.measureText(next).width > width) {
      lines.push(line);
      line = word;
    } else {
      line = next;
    }
  }
  if (line) lines.push(line);
  return lines;
}

// Lines of text centred at x from y down, each `step` apart; returns the y after them.
function centred(context, lines, x, y, step, width) {
  context.textAlign = 'center';
  for (const line of lines) {
    context.fillText(line, x, y, width);
    y += step;
  }
  context.textAlign = 'left';
  return y;
}

export const RING = {inner: 230, outer: 505};  // Its segments' radii, in its canvas pixels.

// Text sizes, in canvas pixels. The ring's 1024 pixels span about 36 degrees of view, 28 pixels a degree (the Quest
// Pro shows about 22): text of 21 to 38 pixels (0.75 to 1.35 degrees) was too small to read in the headset (the
// user, 2026-11-03). All of it is now 44 pixels (1.55 degrees) or more; long text sits on the card over the ring.
export const TEXT = {label: 52, detail: 44, title: 64, card: 48, least: 44, hint: 44, value: 120, alert: 72,
  alertLeast: 62};
export const ALERT_LINES = 5;  // An alert longer than this many lines at its least size ends with '…'.

// `lines` cut to `count`, the last one ending with '…' when any were cut.
function cut(lines, count) {
  if (lines.length <= count) return lines;
  const kept = lines.slice(0, count);
  kept[count - 1] = `${kept[count - 1]} …`;
  return kept;
}

// How wide a label may be in a segment around `angle` (radians from the right, clockwise): the segment's chord where
// it runs across (top and bottom), its band's depth where it runs down (the sides).
function labelRoom(angle, share) {
  const middle = (RING.inner + RING.outer) / 2;
  const chord = 2 * middle * Math.sin(share / 2), band = RING.outer - RING.inner;
  return Math.min(chord, Math.abs(Math.sin(angle)) * chord + Math.abs(Math.cos(angle)) * band) - 28;
}

// The ring (xr_ring.Ring.drawing): its items around it, clockwise from the top, the picked one lit, one out of reach
// dimmed, a label too wide for its segment on two lines; in its middle the level's title and the picked item, a
// slider's value, or a hold's progress around it.
export function drawRing(context, drawing) {
  const {canvas} = context;
  const [cx, cy] = [canvas.width / 2, canvas.height / 2];
  context.clearRect(0, 0, canvas.width, canvas.height);
  const count = drawing.items.length;
  const share = 2 * Math.PI / Math.max(1, count);
  const middleRadius = (RING.inner + RING.outer) / 2;
  drawing.items.forEach((item, index) => {
    const middle = -Math.PI / 2 + index * share;
    const picked = index === drawing.aim;
    context.beginPath();
    context.arc(cx, cy, RING.outer, middle - share / 2 + .015, middle + share / 2 - .015);
    context.arc(cx, cy, RING.inner, middle + share / 2 - .015, middle - share / 2 + .015, true);
    context.closePath();
    context.fillStyle = picked ? (item.disabled ? '#4a4f5a' : '#2f5fbf') : 'rgba(16, 19, 24, 0.92)';
    context.fill();
    const room = labelRoom(middle, share);
    context.font = `600 ${TEXT.label}px ${FONT}`;
    const label = wrap(context, item.label + (item.submenu ? ' ›' : ''), room).slice(0, 2);
    const step = Math.round(TEXT.label * 1.1);
    const tall = label.length * step + (item.detail ? TEXT.detail * 1.1 : 0);
    const [x, y] = [cx + Math.cos(middle) * middleRadius, cy + Math.sin(middle) * middleRadius - tall / 2];
    context.fillStyle = item.disabled ? TONE.muted : TONE[item.tone] || TONE[''];
    let below = centred(context, label, x, y + TEXT.label * .85, step, room);
    if (item.detail) {
      context.font = `${TEXT.detail}px ${FONT}`;
      context.fillStyle = picked ? TONE[''] : TONE.muted;
      centred(context, [item.detail], x, below - step + TEXT.label * .15 + TEXT.detail, 0, room);
    }
  });
  context.beginPath();
  context.arc(cx, cy, RING.inner - 10, 0, 2 * Math.PI);
  context.fillStyle = BACKGROUND;
  context.fill();
  const width = 2 * (RING.inner - 30);
  if (drawing.progress > 0) {
    context.beginPath();
    context.arc(cx, cy, RING.inner - 22, -Math.PI / 2, -Math.PI / 2 + drawing.progress * 2 * Math.PI);
    context.strokeStyle = TONE.warn;
    context.lineWidth = 20;
    context.stroke();
  }
  const item = drawing.adjust ? drawing.items.find(other => other.id === drawing.adjust.id)
    : drawing.aim === null ? null : drawing.items[drawing.aim];
  if (drawing.adjust) {
    context.fillStyle = TONE[''];
    context.font = `600 ${TEXT.label}px ${FONT}`;
    centred(context, [item?.label || ''], cx, cy - 50, 0, width);
    context.font = `700 ${TEXT.value}px ${FONT}`;
    centred(context, [`${Math.round(drawing.adjust.value * 100)} %`], cx, cy + 80, 0, width);
    return;
  }
  context.font = `700 ${TEXT.title}px ${FONT}`;
  context.fillStyle = TONE.muted;
  centred(context, [drawing.title], cx, item ? cy - 62 : cy + TEXT.title * .35, 0, width);
  if (!item) return;
  context.font = `600 ${TEXT.label}px ${FONT}`;
  context.fillStyle = item.disabled ? TONE.muted : TONE[item.tone] || TONE[''];
  centred(context, [item.label], cx, cy + 18, 0, width);
  const under = drawing.confirm ? 'Hold the stick' : item.detail;
  if (under) {
    context.font = `600 ${TEXT.detail}px ${FONT}`;
    context.fillStyle = drawing.confirm ? TONE.warn : TONE.muted;
    centred(context, [under], cx, cy + 82, 0, width - 40);
  }
}

// The card over the ring while it is open (xr_ring.Ring.drawing, and `status`: [text, tone] lines,
// xr_session.statusLines): a picked item's confirmation, a notice, or the status; and how the ring is worked now, at
// its foot.
export function drawCard(context, drawing, status = []) {
  const {canvas} = context;
  const [width, height] = [canvas.width, canvas.height];
  context.clearRect(0, 0, width, height);
  context.fillStyle = BACKGROUND;
  context.beginPath();
  context.roundRect(0, 0, width, height, 36);
  context.fill();
  const text = width - 80;
  const [hints, hintTone] = drawing.adjust ? [['Tilt left or right to set it', 'Press the stick: done'], '']
    : drawing.confirm && drawing.notice ? [[drawing.notice], 'warn']  // A hold let go too soon.
      : drawing.confirm ? [['Hold the stick down to confirm', 'A/X: back'], 'warn']
        : [['Tilt to pick · press the stick to choose', `A/X: ${drawing.depth ? 'back' : 'close'}`], ''];
  context.font = `600 ${TEXT.hint}px ${FONT}`;
  context.fillStyle = hintTone ? TONE[hintTone] : TONE.muted;
  const hintStep = Math.round(TEXT.hint * 1.2);
  centred(context, hints, width / 2, height - 40 - (hints.length - 1) * hintStep, hintStep, text);
  const room = height - 40 - hints.length * hintStep - 60;  // From the top to over the hints.
  // What it says, each [text, tone], in the largest size its lines fit (a fault's whole reason, the user, 2026-11-03:
  // a session's start fault was cut short), cut with '…' only past the least.
  const texts = drawing.confirm ? drawing.confirm.map(line => [line, ''])
    : drawing.notice ? [[drawing.notice, 'warn']] : status;
  const weight = drawing.notice && !drawing.confirm ? '600 ' : '';
  let lines = [], size = TEXT.card;
  for (size of [TEXT.card, 46, TEXT.least]) {
    context.font = `${weight}${size}px ${FONT}`;
    lines = texts.flatMap(([line, tone]) => wrap(context, line, text).map(part => [part, tone]));
    if (lines.length * Math.round(size * 1.22) <= room) break;
  }
  const step = Math.round(size * 1.22);
  const shown = Math.floor(room / step);
  if (lines.length > shown) lines = [...lines.slice(0, shown - 1), [`${lines[shown - 1][0]} …`, lines[shown - 1][1]]];
  let y = 36 + (room - lines.length * step) / 2 + size;
  context.font = `${weight}${size}px ${FONT}`;
  for (const [line, tone] of lines) {
    context.fillStyle = TONE[tone] || TONE[''];
    centred(context, [line], width / 2, y, 0, text);
    y += step;
  }
}

// The alert under the view with the ring closed: [text, tone], whole (a fault's reason runs long: the user,
// 2026-11-03, of a start fault cut short at two lines): two lines at its full size, else smaller, to ALERT_LINES
// lines; past that it ends with '…' (the ring's card, opened, says all of it).
export function drawAlert(context, [text, tone]) {
  const {canvas} = context;
  context.clearRect(0, 0, canvas.width, canvas.height);
  const width = canvas.width - 100;
  let lines = [], size = TEXT.alert;
  for (size of [TEXT.alert, 66, TEXT.alertLeast]) {
    context.font = `600 ${size}px ${FONT}`;
    lines = wrap(context, text, width);
    if (lines.length <= (size === TEXT.alert ? 2 : ALERT_LINES)) break;
  }
  lines = cut(lines, ALERT_LINES);
  const step = Math.round(size * 1.2), tall = lines.length * step + 48;
  const top = (canvas.height - tall) / 2;
  context.fillStyle = BACKGROUND;
  context.beginPath();
  context.roundRect(0, top, canvas.width, tall, Math.min(tall / 2, 60));
  context.fill();
  context.fillStyle = TONE[tone] || TONE[''];
  centred(context, lines, canvas.width / 2, top + 24 + size * .95, step, width);
}
