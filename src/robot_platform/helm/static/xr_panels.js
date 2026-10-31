// The headset's screens drawn on 2D canvases: the head camera picture's placeholder, the status line under it, the bar
// of the picture's controls under that, and the status panel. The headset textures them in its scene (xr_scene.js);
// Settings → Headset shows the picture, its line and the panel flat (headset_mirror.js). No three.js here.
export const TONE = {ok: '#3fbf7f', warn: '#e0a83a', bad: '#e05a5a', '': '#e9ecf1', muted: '#9aa3b2'};
export const SIZES = {picture: [1024, 768], strip: [1536, 104], controls: [1536, 240], panel: [1024, 768]};  // Pixels.
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

// The picture's controls, in the bar's canvas pixels: the Follow head, 3D, Model, Hands and Map switches on its first row;
// on its second, two sliders, the picture's opacity (flat or 3D, and the model with it) and the wrist cameras' (the
// user, 2026-10-31: theirs set apart). A slider's value is where along its track a controller points; it takes points
// anywhere on its own half of the row, held at its track's ends past them.
export const CONTROLS = {follow: {x: 16, y: 14, width: 330, height: 100}, depth: {x: 358, y: 14, width: 170, height: 100},
  model: {x: 540, y: 14, width: 220, height: 100}, hands: {x: 772, y: 14, width: 220, height: 100},
  map: {x: 1004, y: 14, width: 200, height: 100},
  opacity: {x: 16, y: 126, width: 744, height: 100, label: 'Picture', track: {x: 220, y: 176, width: 360}},
  wristOpacity: {x: 776, y: 126, width: 744, height: 100, label: 'Hands', track: {x: 980, y: 176, width: 360}}};
export const SWITCHES = ['follow', 'depth', 'model', 'hands', 'map'];
export const SLIDERS = ['opacity', 'wristOpacity'];

// The value a point `x` along the bar (canvas pixels) sets on a slider: 0 to 1 along its track, held at its ends.
export function sliderAt(name, x) {
  const {track} = CONTROLS[name];
  return Math.min(1, Math.max(0, (x - track.x) / track.width));
}

// The control a point on the bar is on ([x, y] canvas pixels): {control} for a switch (SWITCHES), {control, value}
// for a slider (SLIDERS, value 0 to 1 along its track), or null between them.
export function controlAt([x, y]) {
  for (const name of [...SWITCHES, ...SLIDERS]) {
    const area = CONTROLS[name];
    if (x >= area.x && x <= area.x + area.width && y >= area.y && y <= area.y + area.height) {
      return SLIDERS.includes(name) ? {control: name, value: sliderAt(name, x)} : {control: name};
    }
  }
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

// A slider on the bar: its label, its track filled to its value with the knob there, and the value in per cent.
function drawSlider(context, area, value, hovered) {
  const {track} = area;
  context.fillStyle = TONE.muted;
  context.font = `40px ${FONT}`;
  context.fillText(area.label, area.x + 24, track.y + 14);
  context.fillStyle = hovered ? '#5a6272' : '#3a4150';
  context.beginPath();
  context.roundRect(track.x, track.y - 8, track.width, 16, 8);
  context.fill();
  const knob = track.x + value * track.width;
  context.fillStyle = '#4f8cff';
  context.beginPath();
  context.roundRect(track.x, track.y - 8, knob - track.x, 16, 8);
  context.fill();
  context.beginPath();
  context.arc(knob, track.y, hovered ? 30 : 26, 0, 2 * Math.PI);
  context.fillStyle = TONE[''];
  context.fill();
  context.font = `600 42px ${FONT}`;
  context.textAlign = 'right';
  context.fillText(`${Math.round(value * 100)} %`, area.x + area.width - 24, track.y + 14);
  context.textAlign = 'left';
}

// The bar: the Follow head switch (on: the picture stays in front of the eyes), the 3D switch (the head camera's
// depth instead of the flat picture), the Model switch (Minerva's model with the 3D), the Hands switch (the wrist
// cameras above the hands), the Map switch (her splat map around the 3D), and the picture's and the wrist cameras'
// opacities; `hover`: the control a controller points at (a switch's or a slider's name), drawn lighter.
export function drawControls(context, {follow, depth = false, model = false, hands = false, map = false, opacity,
  wristOpacity = 1, hover = null}) {
  const {canvas} = context;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = BACKGROUND;
  context.fillRect(0, 0, canvas.width, canvas.height);
  drawSwitch(context, CONTROLS.follow, `Follow head · ${follow ? 'on' : 'off'}`, follow, hover === 'follow');
  drawSwitch(context, CONTROLS.depth, `3D · ${depth ? 'on' : 'off'}`, depth, hover === 'depth');
  drawSwitch(context, CONTROLS.model, `Model · ${model ? 'on' : 'off'}`, model, hover === 'model');
  drawSwitch(context, CONTROLS.hands, `Hands · ${hands ? 'on' : 'off'}`, hands, hover === 'hands');
  drawSwitch(context, CONTROLS.map, `Map · ${map ? 'on' : 'off'}`, map, hover === 'map');
  drawSlider(context, CONTROLS.opacity, opacity, hover === 'opacity');
  drawSlider(context, CONTROLS.wristOpacity, wristOpacity, hover === 'wristOpacity');
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
