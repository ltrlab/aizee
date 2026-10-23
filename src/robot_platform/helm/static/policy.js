// Policy mode on the page: the choices (hold_still and the policies exported to Minerva), and what a running
// policy session shows. The session itself is teleop's: helm.js starts it with the live confirmation.

export const HOLD_STILL = 'hold_still';

// The select's choices from /api/helm/policies: hold_still first, then each exported policy (newest first).
export function policyOptions(listing) {
  const options = [{value: HOLD_STILL, label: 'hold_still · holds the reference pose (try policy mode)', disabled: false}];
  for (const row of listing?.policies || []) {
    const cameras = Array.isArray(row.camera_roles) ? `${row.camera_roles.length} cameras` : 'cameras unknown';
    options.push({value: `lerobot:${row.path}`, disabled: !!row.error, title: row.error || row.path,
      label: `${row.name} · ${row.policy_type || 'policy'} ${row.fps ? `${row.fps} fps` : ''} · ${cameras}`
        + (row.error ? ` · ${row.error}` : '')});
  }
  return options;
}

// A spec's short name: hold_still, or the exported policy's directory.
export function policyLabel(spec) {
  if (!spec) return '—';
  if (spec === HOLD_STILL) return HOLD_STILL;
  const path = spec.startsWith('lerobot:') ? spec.slice('lerobot:'.length) : spec;
  return path.split(/[\\/]/).filter(Boolean).pop() || spec;
}

// The card's numbers: chunks answered, the newest one's age and the inference time.
export function policyStats(policy) {
  if (!policy) return '';
  if (policy.loading) return 'loading the policy…';
  const parts = [`${policy.chunks ?? 0} chunks`];
  if (Number.isFinite(policy.chunk_age_s)) parts.push(`${policy.chunk_age_s.toFixed(2)} s old`);
  if (Number.isFinite(policy.inference_ms)) parts.push(`${Math.round(policy.inference_ms)} ms`);
  if (policy.stale_cycles) parts.push(`${policy.stale_cycles} stale cycles`);
  return parts.join(' · ');
}

// What the operator does next, by the lifecycle state.
export function policyNote(state, policy) {
  if (!policy) return '';
  if (policy.loading) return 'Loading the policy onto the GPU: its first calls take a few seconds.';
  if (state === 'TELEOP') return 'The policy drives the arms. Hold (T) pauses; Shutdown (X) lowers; the foot switch cuts power.';
  if (state === 'ENGAGING') return "Aligning to the policy's first action at the alignment speed.";
  if (state === 'HOLD') return policy.aligned
    ? "Engage (T) moves the arms to the policy's first action at the alignment speed, then the policy drives them."
    : "Computing the policy's first action from where the arms hold…";
  if (state === 'IDLE') return "Enable (E) holds the arms and asks the policy for its first action.";
  return 'Idle (I), then Enable (E): the policy plans from where the arms are; Engage (T) hands it the arms.';
}
