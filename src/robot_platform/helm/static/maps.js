// Splat maps in Helm without the DOM: what the Teleop tab's map card shows (helm.maps): a head scene capture's
// readiness, its last result, and the captures and trained maps the headset can draw.

const fixed = (value, digits = 1) => (Number.isFinite(value) ? value.toFixed(digits) : '—');
const megabytes = (bytes) => (Number.isFinite(bytes) ? `${(bytes / 1e6).toFixed(1)} MB` : '—');
export const WORLD_LABELS = { board: "the table's frame", robot: "Minerva's frame" };

// Why a capture cannot start now, or null: it needs the live head, engaged, as the operator looks around.
export function mapCaptureBlock(maps, teleop) {
  if (!maps) return 'Maps run with Helm on Minerva.';
  if (maps.busy) return 'Capturing: look around slowly in the headset…';
  if (teleop?.phase !== 'running') return 'Start a live session with the head to capture.';
  if (teleop?.receiver_choice?.mode !== 'live') return 'A capture needs the real head: start a live session.';
  if (!['HOLD', 'TELEOP'].includes(teleop?.head?.state)) return 'Enable the head (E), then engage it (T).';
  const recording = teleop?.task?.recording;
  if (recording?.active || ['starting', 'recording', 'stopping', 'finishing'].includes(recording?.state)) {
    return 'Stop the recording to capture.';
  }
  return null;
}

// The card's status line: the world a capture records now, and the last capture's result.
export function mapStatusText(maps) {
  if (!maps) return '';
  const world =
    maps.world === 'board'
      ? "Captures are placed in the table's frame (the head camera is aligned)"
      : "Captures are placed in Minerva's frame: align the head camera to place them on the table";
  const last = maps.last;
  if (!last) return world;
  const skipped = last.counts
    ? Object.entries(last.counts)
        .filter(([key, value]) => key !== 'seen' && value)
        .map(([key, value]) => `${value} ${key.replace('_', ' ')}`)
        .join(', ')
    : '';
  return (
    `${world} · last: ${last.name}, ${last.frames} frame${last.frames === 1 ? '' : 's'} kept of ` +
    `${last.counts?.seen ?? '—'} (${megabytes(last.bytes)})${skipped ? `; skipped ${skipped}` : ''}`
  );
}

// The maps (the headset draws the newest it can place) and the captures, newest first.
export function mapLines(maps) {
  if (!maps) return [];
  const lines = (maps.maps || []).map((map, index) => ({
    title: `Map ${map.name}${index === 0 ? ' (newest)' : ''}`,
    bad: false,
    text:
      `${Math.round((map.gaussians || 0) / 1000)} k splats, ${megabytes(map.bytes)} · ${fixed(map.psnr)} dB held out` +
      ` · in ${WORLD_LABELS[map.world] || map.world}`,
  }));
  for (const capture of maps.captures || []) {
    lines.push({
      title: `Capture ${capture.name}`,
      bad: false,
      text:
        `${capture.frames} frames in ${WORLD_LABELS[capture.world] || capture.world}` +
        (capture.trained ? ' · trained' : ' · train it on the operator station (scripts/splat/train_on_station.py)'),
    });
  }
  return lines;
}
