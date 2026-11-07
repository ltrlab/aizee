// A splat map's order, back to front for the latest view, sorted off the headset's drawing (xr_splat.js): the
// centres once, then a view at a time, each order sent back without a copy.
import { sortOrder } from './xr_splat_sort.js';

let centres = null,
  count = 0;

self.onmessage = ({ data }) => {
  if (data.centres) {
    centres = data.centres;
    count = data.count;
    return;
  }
  if (!centres) return;
  const order = sortOrder(centres, count, data.view);
  self.postMessage({ id: data.id, order }, [order.buffer]);
};
