// The headset's screens drawn on 2D canvases: the head camera picture's placeholder, its ring of controls
// (xr_ring.js) and the one-line alert under the view; the headset textures them in its scene (xr_scene.js). The status
// line and panel are Settings → Headset's flat copy on the operator PC (headset_mirror.js): the headset itself shows
// its status in the ring (the user, 2026-11-03: the floating panels got in the way). No three.js here.
export const TONE = {ok: '#3fbf7f', warn: '#e0a83a', bad: '#e05a5a', '': '#e9ecf1', muted: '#9aa3b2'};
export const SIZES = {picture: [1024, 768], strip: [1536, 104], panel: [1024, 768], ring: [1024, 1024],
  alert: [1536, 112]};  // Pixels.
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

// `texts` wrapped `width` wide in the largest of `sizes` (pixels) whose lines fit `height`, else the smallest:
// [lines, size]. A live start's confirmation runs to five sentences, as the Teleop tab words it.
function fitted(context, texts, width, height, sizes = [30, 27, 24, 21]) {
  let lines = [];
  for (const size of sizes) {
    context.font = `${size}px ${FONT}`;
    lines = texts.flatMap(text => wrap(context, text, width));
    if (lines.length * size * 1.2 <= height) return [lines, size];
  }
  return [lines, sizes.at(-1)];
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

export const RING = {inner: 300, outer: 498};  // Its segments' radii, in its canvas pixels.

// The ring (xr_ring.Ring.drawing): its items around it, clockwise from the top, the aimed one lit, one out of reach
// dimmed; in its middle a slider being set, an aimed item's confirmation and its hold's progress, a notice, or the
// level's title over the status (`status`: [text, tone] lines, xr_session.statusLines).
export function drawRing(context, drawing, status = []) {
  const {canvas} = context;
  const [cx, cy] = [canvas.width / 2, canvas.height / 2];
  context.clearRect(0, 0, canvas.width, canvas.height);
  const count = drawing.items.length;
  const share = 2 * Math.PI / Math.max(1, count);
  drawing.items.forEach((item, index) => {
    const middle = -Math.PI / 2 + index * share;
    const aimed = index === drawing.aim;
    context.beginPath();
    context.arc(cx, cy, RING.outer, middle - share / 2 + .02, middle + share / 2 - .02);
    context.arc(cx, cy, RING.inner, middle + share / 2 - .02, middle - share / 2 + .02, true);
    context.closePath();
    context.fillStyle = aimed ? (item.disabled ? '#4a4f5a' : '#2f5fbf') : 'rgba(16, 19, 24, 0.9)';
    context.fill();
    const [x, y] = [cx + Math.cos(middle) * (RING.inner + RING.outer) / 2, cy + Math.sin(middle) * (RING.inner + RING.outer) / 2];
    context.fillStyle = item.disabled ? TONE.muted : TONE[item.tone] || TONE[''];
    context.font = `600 ${aimed ? 42 : 38}px ${FONT}`;
    context.textAlign = 'center';
    context.fillText(item.label + (item.submenu ? ' ›' : ''), x, y + (item.detail ? -4 : 14), 190);
    if (item.detail) {
      context.font = `30px ${FONT}`;
      context.fillStyle = TONE.muted;
      context.fillText(item.detail, x, y + 34, 190);
    }
    context.textAlign = 'left';
  });
  context.beginPath();
  context.arc(cx, cy, RING.inner - 12, 0, 2 * Math.PI);
  context.fillStyle = BACKGROUND;
  context.fill();
  const width = 2 * (RING.inner - 60);
  if (drawing.progress > 0) {
    context.beginPath();
    context.arc(cx, cy, RING.inner - 22, -Math.PI / 2, -Math.PI / 2 + drawing.progress * 2 * Math.PI);
    context.strokeStyle = TONE.warn;
    context.lineWidth = 14;
    context.stroke();
  }
  if (drawing.adjust) {
    const item = drawing.items.find(other => other.id === drawing.adjust.id);
    context.fillStyle = TONE[''];
    context.font = `600 44px ${FONT}`;
    centred(context, [item?.label || ''], cx, cy - 60, 50, width);
    context.font = `700 96px ${FONT}`;
    centred(context, [`${Math.round(drawing.adjust.value * 100)} %`], cx, cy + 40, 90, width);
    context.font = `30px ${FONT}`;
    context.fillStyle = TONE.muted;
    centred(context, ['Stick left or right · A/X: done'], cx, cy + 110, 36, width);
    return;
  }
  if (drawing.confirm) {
    // Narrower than the middle's widest, so the top and bottom lines stay inside its circle.
    const [lines, size] = fitted(context, drawing.confirm, width - 60, 2 * (RING.inner - 90));
    const step = Math.round(size * 1.2);
    context.fillStyle = TONE[''];
    const y = centred(context, lines, cx, cy - (lines.length * step + 40) / 2 + size, step, width);
    context.font = `600 34px ${FONT}`;
    context.fillStyle = TONE.warn;
    centred(context, ['Hold A/X to confirm'], cx, y + 14, 40, width);
    return;
  }
  if (drawing.notice) {
    context.font = `600 34px ${FONT}`;
    context.fillStyle = TONE.warn;
    const lines = wrap(context, drawing.notice, width).slice(0, 6);
    centred(context, lines, cx, cy - lines.length * 20 + 10, 40, width);
    return;
  }
  context.font = `700 48px ${FONT}`;
  context.fillStyle = TONE[''];
  let y = centred(context, [drawing.title], cx, cy - 150, 60, width);
  context.font = `32px ${FONT}`;
  for (const [text, tone] of status.slice(0, 6)) {
    context.fillStyle = TONE[tone] || TONE[''];
    y = centred(context, wrap(context, text, width).slice(0, 2), cx, y, 38, width) + 4;
  }
  context.font = `26px ${FONT}`;
  context.fillStyle = TONE.muted;
  centred(context, [drawing.depth ? 'Stick: aim · A/X: choose · centred: back' : 'Stick: aim · A/X: choose · centred: close'],
    cx, cy + RING.inner - 70, 30, width);
}

// The alert under the view with the ring closed: [text, tone].
export function drawAlert(context, [text, tone]) {
  const {canvas} = context;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = BACKGROUND;
  context.beginPath();
  context.roundRect(0, 0, canvas.width, canvas.height, canvas.height / 2);
  context.fill();
  context.font = `600 46px ${FONT}`;
  context.fillStyle = TONE[tone] || TONE[''];
  centred(context, [text], canvas.width / 2, 72, 0, canvas.width - 80);
}
