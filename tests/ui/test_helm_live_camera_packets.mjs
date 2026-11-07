import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
const source = await fs.readFile(
  new URL('../../src/robot_platform/helm/static/camera_live.js', import.meta.url),
  'utf8',
);
const { LiveCameraView, parseCameraPacket } = await import(
  `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`
);
const metadata = { role: 'head', sequence: 1, age_s: 0.02, source_timestamp: 1234, viewer_heartbeat: 3 };
// SOF admission fixture only: actual JPEG decoding is covered by the real-browser test.
const jpeg = new Uint8Array([255, 216, 255, 192, 0, 8, 8, 1, 224, 2, 128, 3]);
function packet(meta = metadata, pixels = jpeg) {
  const header = new TextEncoder().encode(JSON.stringify(meta));
  const result = new Uint8Array(4 + header.length + pixels.length);
  new DataView(result.buffer).setUint32(0, header.length);
  result.set(header, 4);
  result.set(pixels, 4 + header.length);
  return result.buffer;
}
assert.equal(parseCameraPacket(packet()).metadata.role, 'head');
for (const replacement of [
  { role: '__proto__' },
  { sequence: -1 },
  { sequence: 1.5 },
  { age_s: -1 },
  { age_s: 1 },
  { source_timestamp: null },
  { viewer_heartbeat: null },
  { viewer_heartbeat: -1 },
])
  assert.throws(() => parseCameraPacket(packet({ ...metadata, ...replacement })), /metadata/);
assert.throws(() => parseCameraPacket(new ArrayBuffer(3)), /size/);
assert.throws(() => parseCameraPacket(new ArrayBuffer(2 * 1024 * 1024 + 4101)), /size/);
const badLength = packet();
new DataView(badLength).setUint32(0, 4097);
assert.throws(() => parseCameraPacket(badLength), /header/);
const huge = jpeg.slice();
huge[7] = 0x20;
huge[8] = 0;
assert.throws(() => parseCameraPacket(packet(metadata, huge)), /dimensions/);
const hugeArea = jpeg.slice();
hugeArea[7] = 12;
hugeArea[8] = 0;
hugeArea[9] = 12;
hugeArea[10] = 0;
assert.throws(() => parseCameraPacket(packet(metadata, hugeArea)), /memory bound/);
assert.throws(() => parseCameraPacket(packet(metadata, new Uint8Array([255, 216, 255, 217]))), /dimensions/);
// A stream the page shows no card for (the headset's head_depth, 2026-10-31: the cameras read "Invalid camera
// stream" while a headset showed 3D) passes it by, never failing the page's stream.
assert.equal(
  LiveCameraView.prototype.frame.call({ roles: new Map() }, { metadata: { ...metadata, role: 'head_depth' }, jpeg }, 0),
  undefined,
);
console.log('Live camera binary bounds, metadata validation, and pre-decode pixel limit passed.');
