// How the headset shows the head camera's picture (xr_scene.js): whether it follows the head, its opacity, and where
// it sits in front of the eyes while it follows. The operator changes them on the bar under the picture; the headset's
// browser remembers them. No DOM and no three.js, so node tests it.

export const STORAGE_KEY = 'helm.xr.view';
export const OPACITY_STEP = .05;
// Following, the picture sits this far in front of the eyes (metres: x right, y up, z back), as it starts in front
// of where the session began.
export const FOLLOW_OFFSET = [0, -.08, -1.45];
export const DEFAULT_VIEW = {follow: true, opacity: 1, offset: FOLLOW_OFFSET};

export function opacityStep(value) {
  const stepped = Math.round(Number(value) / OPACITY_STEP) * OPACITY_STEP;
  return Number.isFinite(stepped) ? Math.min(1, Math.max(0, Number(stepped.toFixed(2)))) : DEFAULT_VIEW.opacity;
}

// A picture moved closer than this, or farther, or beside or behind the eyes, while it follows is put back at its
// default place.
const NEAREST_M = .3, FARTHEST_M = 6;

export function offsetOf(value) {
  if (!Array.isArray(value) || value.length !== 3 || !value.every(Number.isFinite)) return FOLLOW_OFFSET;
  const distance = Math.hypot(...value);
  return distance >= NEAREST_M && distance <= FARTHEST_M && value[2] < 0 ? value.map(Number) : FOLLOW_OFFSET;
}

// The remembered view, or the default for anything missing or unreadable.
export function loadView(storage) {
  let saved = {};
  try {
    saved = JSON.parse(storage?.getItem(STORAGE_KEY) || '{}') || {};
  } catch {
    saved = {};
  }
  return {follow: typeof saved.follow === 'boolean' ? saved.follow : DEFAULT_VIEW.follow,
    opacity: 'opacity' in saved ? opacityStep(saved.opacity) : DEFAULT_VIEW.opacity, offset: offsetOf(saved.offset)};
}

export function saveView(storage, view) {
  try {
    storage?.setItem(STORAGE_KEY, JSON.stringify({follow: !!view.follow, opacity: opacityStep(view.opacity),
      offset: offsetOf(view.offset)}));
  } catch {
    // A private window or a full store: the view still works, unremembered.
  }
}

// Where the headset looks, as the picture follows it: [yaw, pitch] in radians (yaw left positive, pitch up
// positive) of its forward axis (-Z) from the headset's orientation [x, y, z, w]. Its roll is left out, so the
// picture stays level, as the robot's head (pan and tilt) keeps its camera.
export function lookAngles([x, y, z, w]) {
  const forward = [-2 * (x * z + w * y), -2 * (y * z - w * x), -(1 - 2 * (x * x + y * y))];
  return [Math.atan2(-forward[0], -forward[2]), Math.atan2(forward[1], Math.hypot(forward[0], forward[2]))];
}
