// The head camera's depth, inflated and filled for the headset's 3D picture (xr_depth.js draws it). Each pixel keeps
// where its colour comes from (its source pixel), so the picture never paints a far surface with a near object's
// colours:
// - the sensor's small dropouts take the farthest reading within SMALL_HOLE_PX (a dropout beside a near object's edge
//   is mostly background the projector's shadow hid);
// - a pixel the colour camera cannot see (beside the depth sensor, it sees past a near object's edge from a little
//   to the side) takes its colour from the nearest pixel along its row on the same surface that it can;
// - behind each near object's edge, a background layer: the far surface carried up to BEHIND_PX under the object,
//   as the plane it lies in (straight in inverse depth). From the camera's viewpoint the object hides it; from the
//   operator's, as they lean or the head turns, it fills what the object hid instead of a hole.
// Larger holes (out of range) are left to the colour backdrop. Once per depth frame (about 8 a second), in a worker
// (xr_fill_worker.js), off the headset's drawing. No DOM and no three.js, so node tests it.

export const EDGE_SHARE = .04;  // Neighbouring readings further apart than this share of the depth: an edge.
export const SMALL_HOLE_PX = 2;
export const BEHIND_PX = 40;  // At 320 pixels across, an eighth of the view: what a lean of ~20 cm shows at 1 m.
export const RECOLOUR_PX = 8;  // How far along its row a hidden pixel looks for its colour.
// A reading this share behind the nearest in its cell of the colour camera's view is hidden from it (a grazing table
// spreads ~5 % across a cell; a hidden one is a near object's whole depth behind).
const HIDDEN_SHARE = .1;
const SLOPE_PX = 4;  // The far surface's slope, from up to this many pixels of it either side of the edge.
const MAX_SLOPE = .02;  // At most this share of the inverse depth a pixel: noise never flings the plane away.

// The depth readings (raw units, a Uint16Array) from the bytes after a head_depth packet's JPEG
// (camera_video_worker.head_depth), inflated with the browser's own DecompressionStream.
export async function inflateDepth(packed, meta) {
  if (meta.encoding !== 'zlib-u16le') throw new Error(`Unknown depth encoding ${meta.encoding}`);
  const stream = new Blob([packed]).stream().pipeThrough(new DecompressionStream('deflate'));
  const values = new Uint16Array(await new Response(stream).arrayBuffer());  // Little-endian, as the headset is.
  if (values.length !== meta.width * meta.height) throw new Error('Depth size does not match its metadata');
  return values;
}

// The fill's registration from a frame's metadata (camera_video_worker.head_depth), or null without one.
export function registrationOf(meta) {
  const {intrinsics, color_intrinsics: colorK, to_color: toColor, scale} = meta;
  return intrinsics && colorK && toColor ? {depthK: intrinsics, colorK, toColor, scale} : null;
}

// The two layers of a frame's readings (`values`, raw units, 0 for none), each a Float32Array of two numbers a pixel:
// its reading (0 for none) and its source pixel's index (its own, for a reading of its own). `registration`
// ({depthK, colorK, toColor, scale}, registrationOf) lets the colour camera's hidden pixels take a seen one's colour.
// `filled`, `recoloured` and `behind` count the pixels each step changed.
export function fillDepth(values, width, height, {edgeShare = EDGE_SHARE, smallHole = SMALL_HOLE_PX,
  behind = BEHIND_PX, registration = null, recolourPx = RECOLOUR_PX} = {}) {
  const count = width * height;
  const foreground = new Float32Array(count * 2), background = new Float32Array(count * 2);
  for (let i = 0; i < count; i++) {
    foreground[2 * i] = values[i];
    foreground[2 * i + 1] = i;
  }
  let filled = 0;
  for (let v = 0; v < height; v++) {
    for (let u = 0; u < width; u++) {
      const i = v * width + u;
      if (values[i]) continue;
      let best = 0, source = -1;
      for (let y = Math.max(0, v - smallHole); y <= Math.min(height - 1, v + smallHole); y++) {
        for (let x = Math.max(0, u - smallHole); x <= Math.min(width - 1, u + smallHole); x++) {
          const z = values[y * width + x];
          if (z > best) {
            best = z;
            source = y * width + x;
          }
        }
      }
      if (source >= 0) {
        foreground[2 * i] = best;
        foreground[2 * i + 1] = source;
        filled++;
      }
    }
  }
  const recoloured = registration ? recolour(foreground, width, height, registration, edgeShare, recolourPx) : 0;
  return {foreground, background, filled, recoloured,
    behind: backgroundLayer(foreground, background, width, height, edgeShare, behind)};
}

// Which pixels the colour camera sees: each reading projected into its view, on a quarter-size grid keeping the
// nearest over the reading's footprint there (half a depth pixel each way: its centre alone would leave a near
// object's edge between samples); a reading well behind that nearest is hidden. A hidden one takes its colour from
// the nearest seen pixel along its row on its own surface, within `reach`. Returns how many did.
function recolour(foreground, width, height, {depthK, colorK, toColor, scale}, edgeShare, reach) {
  const [r, t] = [toColor.rotation, toColor.translation];
  const gw = Math.max(1, colorK.width >> 2), gh = Math.max(1, colorK.height >> 2);
  const nearest = new Float32Array(gw * gh).fill(Infinity);
  const cells = new Int32Array(width * height).fill(-1), along = new Float32Array(width * height);
  const halfU = colorK.fx / depthK.fx / 2, halfV = colorK.fy / depthK.fy / 2;  // Colour pixels.
  // A pixel's ray turned into the colour camera's axes, R (a, b, 1) for a = (u - cx) / fx and b = (v - cy) / fy: a
  // column's share and a row's, so each reading costs a few multiplications.
  const columns = new Float32Array(width * 3);
  for (let u = 0; u < width; u++) {
    const a = (u - depthK.cx) / depthK.fx;
    for (let k = 0; k < 3; k++) columns[3 * u + k] = r[k][0] * a;
  }
  for (let v = 0, i = 0; v < height; v++) {
    const b = (v - depthK.cy) / depthK.fy;
    const row0 = r[0][1] * b + r[0][2], row1 = r[1][1] * b + r[1][2], row2 = r[2][1] * b + r[2][2];
    for (let u = 0; u < width; u++, i++) {
      const z = foreground[2 * i] * scale;
      if (!z) continue;
      const cz = z * (columns[3 * u + 2] + row2) + t[2];
      if (cz <= 0) continue;
      const cu = colorK.fx * (z * (columns[3 * u] + row0) + t[0]) / cz + colorK.cx;
      const cv = colorK.fy * (z * (columns[3 * u + 1] + row1) + t[1]) / cz + colorK.cy;
      if (cu + halfU < 0 || cv + halfV < 0 || cu - halfU >= 4 * gw || cv - halfV >= 4 * gh) continue;
      const u0 = cu > halfU ? ((cu - halfU) / 4) | 0 : 0, v0 = cv > halfV ? ((cv - halfV) / 4) | 0 : 0;
      const u1 = Math.min(gw - 1, ((cu + halfU) / 4) | 0), v1 = Math.min(gh - 1, ((cv + halfV) / 4) | 0);
      for (let gv = v0; gv <= v1; gv++) {
        for (let cell = gv * gw + u0, last = gv * gw + u1; cell <= last; cell++) {
          if (cz < nearest[cell]) nearest[cell] = cz;
        }
      }
      if (cu >= 0 && cv >= 0 && cu < 4 * gw && cv < 4 * gh) {
        cells[i] = ((cv / 4) | 0) * gw + ((cu / 4) | 0);
        along[i] = cz;
      }
    }
  }
  const hidden = new Uint8Array(width * height);
  for (let i = 0; i < width * height; i++) {
    if (cells[i] >= 0 && along[i] > nearest[cells[i]] * (1 + HIDDEN_SHARE)) hidden[i] = 1;
  }
  let changed = 0;
  for (let v = 0, i = 0; v < height; v++) {
    for (let u = 0; u < width; u++, i++) {
      if (!hidden[i]) continue;
      const z = foreground[2 * i], limit = edgeShare * z;
      for (let d = 1, found = -1; d <= reach && found < 0; d++) {
        for (const k of [u + d < width ? i + d : -1, u >= d ? i - d : -1]) {
          if (k < 0 || hidden[k] || !foreground[2 * k] || Math.abs(foreground[2 * k] - z) > limit) continue;
          foreground[2 * i + 1] = foreground[2 * k + 1];
          changed++;
          found = k;
          break;
        }
      }
    }
  }
  return changed;
}

// The far surface's slope in inverse depth a pixel at `i` along `step` (1 along a row, the width down a column),
// from up to SLOPE_PX of its own surface either side; 0 with none.
function slope(foreground, i, step, limitBefore, limitAfter, edgeShare) {
  const z = foreground[2 * i];
  const same = k => foreground[2 * k] && Math.abs(foreground[2 * k] - z) <= edgeShare * z;
  let before = 0, after = 0;
  while (before < Math.min(SLOPE_PX, limitBefore) && same(i - (before + 1) * step)) before++;
  while (after < Math.min(SLOPE_PX, limitAfter) && same(i + (after + 1) * step)) after++;
  if (!before && !after) return 0;
  const g = (1 / foreground[2 * (i + after * step)] - 1 / foreground[2 * (i - before * step)]) / (before + after);
  const most = MAX_SLOPE / z;
  return Math.max(-most, Math.min(most, g));
}

// The background layer, grown from each far-side edge pixel (a neighbour much nearer) a pixel at a time into pixels
// much nearer than the far surface's plane there. Returns how many pixels it reached beyond the edges.
function backgroundLayer(foreground, background, width, height, edgeShare, behind) {
  const count = width * height;
  const steps = new Uint8Array(count), queue = new Int32Array(count), seedOf = new Int32Array(count).fill(-1);
  const plane = new Float32Array(count * 3);  // A seed's inverse depth and its slopes along a row and down a column.
  const offsets = [-1, 1, -width, width];
  let head = 0, tail = 0;
  for (let v = 0, i = 0; v < height; v++) {
    for (let u = 0; u < width; u++, i++) {
      const z = foreground[2 * i];
      if (!z) continue;
      const edge = z * (1 - edgeShare);  // A neighbour nearer than this is across an edge.
      const left = u > 0 ? foreground[2 * i - 2] : 0, right = u < width - 1 ? foreground[2 * i + 2] : 0;
      const up = v > 0 ? foreground[2 * (i - width)] : 0, down = v < height - 1 ? foreground[2 * (i + width)] : 0;
      if ((left && left < edge) || (right && right < edge) || (up && up < edge) || (down && down < edge)) {
        background[2 * i] = z;
        background[2 * i + 1] = foreground[2 * i + 1];
        seedOf[i] = i;
        plane[3 * i] = 1 / z;
        plane[3 * i + 1] = slope(foreground, i, 1, u, width - 1 - u, edgeShare);
        plane[3 * i + 2] = slope(foreground, i, width, v, height - 1 - v, edgeShare);
        queue[tail++] = i;
      }
    }
  }
  const seeds = tail;
  while (head < tail) {
    const i = queue[head++];
    if (steps[i] >= behind) continue;
    const seed = seedOf[i], u = i % width, su = seed % width, sv = (seed - su) / width;
    for (let k = 0; k < 4; k++) {
      if ((k === 0 && u === 0) || (k === 1 && u === width - 1)) continue;
      const j = i + offsets[k];
      if (j < 0 || j >= count || seedOf[j] >= 0) continue;
      const near = foreground[2 * j];
      if (!near) continue;
      const ju = j % width;
      const w = plane[3 * seed] + plane[3 * seed + 1] * (ju - su) + plane[3 * seed + 2] * ((j - ju) / width - sv);
      if (w <= 0) continue;  // The plane runs off past the horizon.
      const z = 1 / w;
      if (z - near <= edgeShare * z) continue;  // Not behind the near object here: the layer ends.
      background[2 * j] = z;
      background[2 * j + 1] = background[2 * seed + 1];
      seedOf[j] = seed;
      steps[j] = steps[i] + 1;
      queue[tail++] = j;
    }
  }
  return tail - seeds;
}

// A frame's depth, inflated and filled in a worker (xr_fill_worker.js) where the browser has them, else here.
export class DepthFiller {
  constructor({createWorker = () => new Worker(new URL('./xr_fill_worker.js', import.meta.url), {type: 'module'})}
    = {}) {
    this.pending = new Map();
    this.next = 0;
    this.worker = null;
    try {
      this.worker = typeof Worker === 'undefined' ? null : createWorker();
    } catch {
      this.worker = null;
    }
    if (!this.worker) return;
    this.worker.onmessage = ({data}) => {
      const waiting = this.pending.get(data.id);
      this.pending.delete(data.id);
      if (data.error) waiting?.reject(new Error(data.error));
      else waiting?.resolve(data.layers);
    };
    this.worker.onerror = event => {  // A worker that cannot start or run: this frame fails, the next fill here.
      event.preventDefault?.();
      this.worker = null;
      for (const waiting of this.pending.values()) waiting.reject(new Error('the depth worker stopped'));
      this.pending.clear();
    };
  }

  // {foreground, background, filled, recoloured, behind} (fillDepth) of a frame's packed depth and its metadata.
  async fill(packed, meta) {
    if (!this.worker) {
      return fillDepth(await inflateDepth(packed, meta), meta.width, meta.height,
        {registration: registrationOf(meta)});
    }
    return new Promise((resolve, reject) => {
      const id = ++this.next;
      this.pending.set(id, {resolve, reject});
      this.worker.postMessage({id, packed, meta});
    });
  }
}
