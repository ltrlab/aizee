// The head camera's depth, inflated and filled off the headset's drawing (xr_fill.js): a frame at a time from
// xr_camera.js, its two layers sent back without a copy.
import { fillDepth, inflateDepth, registrationOf } from './xr_fill.js';

self.onmessage = async ({ data: { id, packed, meta } }) => {
  try {
    const layers = fillDepth(await inflateDepth(packed, meta), meta.width, meta.height, {
      registration: registrationOf(meta),
    });
    self.postMessage({ id, layers }, [layers.foreground.buffer, layers.background.buffer]);
  } catch (error) {
    self.postMessage({ id, error: error.message });
  }
};
