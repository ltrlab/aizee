// The headset's screens drawn on 2D canvases: the head camera picture's placeholder, the status line under it, the bar
// of the picture's controls under that, and the status panel. The headset textures them in its scene (xr_scene.js);
// Settings → Headset shows the picture, its line and the panel flat (headset_mirror.js). No three.js here.
export const TONE = {ok: '#3fbf7f', warn: '#e0a83a', bad: '#e05a5a', '': '#e9ecf1', muted: '#9aa3b2'};
export const SIZES = {picture: [1024, 768], strip: [1536, 104], controls: [1536, 128], panel: [1024, 768]};  // Pixels.
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

// The picture's controls, in the bar's canvas pixels: the Follow head and 3D switches, and the opacity slider's
// track (the value is where along it a controller points; the slider takes points a little past either end).
export const CONTROLS = {follow: {x: 20, y: 14, width: 400, height: 100}, depth: {x: 440, y: 14, width: 210, height: 100},
  track: {x: 880, y: 64, width: 460}, slider: {x: 840, width: 540}};

// The opacity a point `x` along the bar (canvas pixels) sets: 0 to 1 along the track, held at its ends past them.
export function opacityAt(x) {
  const {track} = CONTROLS;
  return Math.min(1, Math.max(0, (x - track.x) / track.width));
}

// The control a point on the bar is on ([x, y] canvas pixels): {control: 'follow'}, {control: 'depth'},
// {control: 'opacity', value} with value 0 to 1 along the track, or null between them.
export function controlAt([x, y]) {
  const {slider} = CONTROLS;
  for (const name of ['follow', 'depth']) {
    const button = CONTROLS[name];
    if (x >= button.x && x <= button.x + button.width && y >= button.y && y <= button.y + button.height) {
      return {control: name};
    }
  }
  if (x >= slider.x && x <= slider.x + slider.width) return {control: 'opacity', value: opacityAt(x)};
  return null;
}

// A switch on the bar: green while on, lighter where a controller points.
function drawSwitch(context, button, text, on, hovered) {
  context.fillStyle = on ? (hovered ? '#4fd08f' : TONE.ok) : (hovered ? '#3a4150' : '#2a303b');
  context.beginPath();
  context.roundRect(button.x, button.y, button.width, button.height, button.height / 2);
  context.fill();
  context.fillStyle = on ? '#0f1218' : TONE[''];
  context.font = `600 40px ${FONT}`;
  context.textAlign = 'center';
  context.fillText(text, button.x + button.width / 2, button.y + 64, button.width - 36);
  context.textAlign = 'left';
}

// The bar: the Follow head switch (on: the picture stays in front of the eyes), the 3D switch (the head camera's
// depth around the eyes instead of the flat picture) and the picture's opacity; `hover`: the control a controller
// points at ('follow', 'depth' or 'opacity'), drawn lighter.
export function drawControls(context, {follow, depth = false, opacity, hover = null}) {
  const {canvas} = context;
  const {track} = CONTROLS;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = BACKGROUND;
  context.fillRect(0, 0, canvas.width, canvas.height);
  drawSwitch(context, CONTROLS.follow, `Follow head · ${follow ? 'on' : 'off'}`, follow, hover === 'follow');
  drawSwitch(context, CONTROLS.depth, `3D · ${depth ? 'on' : 'off'}`, depth, hover === 'depth');
  context.fillStyle = TONE.muted;
  context.font = `40px ${FONT}`;
  context.fillText('Opacity', 676, 78);
  context.fillStyle = hover === 'opacity' ? '#5a6272' : '#3a4150';
  context.beginPath();
  context.roundRect(track.x, track.y - 8, track.width, 16, 8);
  context.fill();
  const knob = track.x + opacity * track.width;
  context.fillStyle = '#4f8cff';
  context.beginPath();
  context.roundRect(track.x, track.y - 8, knob - track.x, 16, 8);
  context.fill();
  context.beginPath();
  context.arc(knob, track.y, hover === 'opacity' ? 30 : 26, 0, 2 * Math.PI);
  context.fillStyle = TONE[''];
  context.fill();
  context.font = `600 42px ${FONT}`;
  context.textAlign = 'right';
  context.fillText(`${Math.round(opacity * 100)} %`, canvas.width - 28, 78);
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
    const y = 190 + index * 84;
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
