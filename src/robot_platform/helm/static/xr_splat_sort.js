// A splat map's Gaussians for the headset (xr_splat.js draws them): read from the trainer's compact .splat
// (scripts/splat/train_head_scene.py, splat_bytes) and ordered back to front for each view. No DOM and no three.js,
// so the sorting worker (xr_splat_worker.js) and node use it too.

export const SPLAT_BYTES = 32; // Centre and scales (float32 x 3 each), RGBA (uint8 x 4), rotation (uint8 x 4, w x y z).
export const ROW = 1024; // Gaussians a row of the drawing's data texture, four texels each.
export const TEXELS = 4;
const BUCKETS = 65536;

// A .splat's Gaussians: their count, centres (3 a Gaussian) and the data texture's floats (16 a Gaussian, ROW to a
// row of 4 x ROW texels): [centre, alpha], [covariance xx xy xz, red], [covariance yy yz zz, green], [blue, 0, 0, 0].
// Each covariance is R S S^T R^T, its rotation R from the quaternion and S its scales.
export function parseSplat(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength % SPLAT_BYTES) {
    throw new Error('Not a .splat: its size is not a whole number of Gaussians');
  }
  const count = buffer.byteLength / SPLAT_BYTES;
  const floats = new Float32Array(buffer),
    bytes = new Uint8Array(buffer);
  const rows = Math.max(1, Math.ceil(count / ROW));
  const data = new Float32Array(rows * ROW * TEXELS * 4);
  const centres = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    const f = i * 8,
      b = i * SPLAT_BYTES,
      out = i * 16;
    const [x, y, z, sx, sy, sz] = [
      floats[f],
      floats[f + 1],
      floats[f + 2],
      floats[f + 3],
      floats[f + 4],
      floats[f + 5],
    ];
    let w = (bytes[b + 28] - 128) / 128,
      qx = (bytes[b + 29] - 128) / 128;
    let qy = (bytes[b + 30] - 128) / 128,
      qz = (bytes[b + 31] - 128) / 128;
    const norm = Math.hypot(w, qx, qy, qz) || 1;
    w /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
    // R's rows, each column scaled by its axis's scale: M = R S, covariance M M^T.
    const m = [
      (1 - 2 * (qy * qy + qz * qz)) * sx,
      2 * (qx * qy - w * qz) * sy,
      2 * (qx * qz + w * qy) * sz,
      2 * (qx * qy + w * qz) * sx,
      (1 - 2 * (qx * qx + qz * qz)) * sy,
      2 * (qy * qz - w * qx) * sz,
      2 * (qx * qz - w * qy) * sx,
      2 * (qy * qz + w * qx) * sy,
      (1 - 2 * (qx * qx + qy * qy)) * sz,
    ];
    const dot = (r, s) => m[r * 3] * m[s * 3] + m[r * 3 + 1] * m[s * 3 + 1] + m[r * 3 + 2] * m[s * 3 + 2];
    data.set(
      [
        x,
        y,
        z,
        bytes[b + 27] / 255,
        dot(0, 0),
        dot(0, 1),
        dot(0, 2),
        bytes[b + 24] / 255,
        dot(1, 1),
        dot(1, 2),
        dot(2, 2),
        bytes[b + 25] / 255,
        bytes[b + 26] / 255,
        0,
        0,
        0,
      ],
      out,
    );
    centres.set([x, y, z], i * 3);
  }
  return { count, centres, data, rows };
}

// The Gaussians back to front for a view: `view` is the model-view matrix's third row ([m2, m6, m10, m14] of its
// column-major elements), so a centre lies -(m2 x + m6 y + m10 z + m14) ahead of the eyes. A counting sort on that
// depth in BUCKETS steps over the map's span: the farthest first, as the drawing blends them.
export function sortOrder(centres, count, view, out = new Uint32Array(count)) {
  const [a, b, c, d] = view;
  const depths = new Float32Array(count);
  let low = Infinity,
    high = -Infinity;
  for (let i = 0; i < count; i++) {
    const depth = -(a * centres[3 * i] + b * centres[3 * i + 1] + c * centres[3 * i + 2] + d);
    depths[i] = depth;
    if (depth < low) low = depth;
    if (depth > high) high = depth;
  }
  const scale = high > low ? (BUCKETS - 1) / (high - low) : 0;
  const keys = new Uint32Array(count),
    counts = new Uint32Array(BUCKETS);
  for (let i = 0; i < count; i++) {
    keys[i] = BUCKETS - 1 - Math.floor((depths[i] - low) * scale); // The farthest gets the smallest key.
    counts[keys[i]]++;
  }
  for (let k = 1; k < BUCKETS; k++) counts[k] += counts[k - 1];
  for (let i = count - 1; i >= 0; i--) out[--counts[keys[i]]] = i;
  return out;
}
