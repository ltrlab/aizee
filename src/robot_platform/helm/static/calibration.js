// Camera calibration in Helm without the DOM: what the Teleop tab's calibration card shows, for the wrist cameras
// (helm.calibration) and the head camera's alignment (helm.head_alignment).

export const ROLE_LABELS = { left_wrist: 'Left wrist', right_wrist: 'Right wrist', head: 'Head' };
const MODEL_LABELS = { plain: 'URDF as is', joint_offsets: 'with joint offsets' };

const fixed = (value, digits = 1) => (Number.isFinite(value) ? value.toFixed(digits) : '—');

// Why a still cannot be captured now, or null: it needs a live session holding or following.
export function captureBlock(calibration, teleop) {
  if (!calibration) return 'Calibration runs with Helm on Minerva.';
  if (calibration.busy) return 'Capturing a still…';
  if (calibration.solving) return 'Solving…';
  if (teleop?.phase !== 'running') return 'Start a live session to capture stills.';
  if (!['HOLD', 'TELEOP'].includes(teleop?.state)) return 'Enable the arms (E), then hold (T) for each still.';
  const recording = teleop?.task?.recording;
  if (recording?.active || ['starting', 'recording', 'stopping', 'finishing'].includes(recording?.state)) {
    return 'Stop the recording to capture stills.';
  }
  return null;
}

export function countsText(calibration) {
  if (!calibration) return '';
  const parts = (calibration.cameras || []).map((role) => {
    const count = calibration.counts?.[role] || { stills: 0, tagged: 0 };
    return (
      `${ROLE_LABELS[role] || role} ${count.stills} still${count.stills === 1 ? '' : 's'}` +
      (count.tagged !== count.stills ? ` (${count.tagged} see tags)` : '')
    );
  });
  return `${calibration.run ? `Run ${calibration.run}` : 'No run yet'} · ${parts.join(' · ')}`;
}

// The last still in one line: its tags, how still the arm was, and the light on the tags (white and black label
// levels, the white's color balance, the noise left in one frame).
export function lastText(last) {
  if (!last) return '';
  const head = `${last.name}: `;
  if (!last.tags?.length) {
    return (
      head +
      'no board tags seen' +
      (last.others?.length ? ` (only ${last.others.join(', ')})` : '') +
      '. Undo it, or keep it: stills without tags are left out of the solve.'
    );
  }
  const light = Number.isFinite(last.white)
    ? ` · white ${fixed(last.white, 0)}, black ${fixed(last.black, 0)}` +
      ` · R/G ${fixed(last.r_over_g, 2)}, B/G ${fixed(last.b_over_g, 2)}`
    : '';
  return (
    `${head}tag${last.tags.length === 1 ? '' : 's'} ${last.tags.join(', ')} · moved ${fixed(last.moved_mrad, 2)} mrad` +
    `${light} · noise ${fixed(last.noise, 1)}`
  );
}

// A solve's lines: per model (recommended first) each camera's error and where its mount came out.
export function resultLines(result) {
  if (!result?.models) return [];
  const lines = [];
  const keys = Object.keys(result.models).sort((a, b) => (b === result.recommended) - (a === result.recommended));
  for (const key of keys) {
    const model = result.models[key];
    const title = `${MODEL_LABELS[key] || key}${key === result.recommended ? ' (recommended)' : ''}`;
    if (model.error) {
      lines.push({ title, text: model.error, bad: true });
      continue;
    }
    for (const [role, camera] of Object.entries(model.cameras || {})) {
      const mount = camera.mount_mm?.map((v) => fixed(v)).join(', ');
      const sigma = camera.mount_sigma_mm ? ` ± ${camera.mount_sigma_mm.map((v) => fixed(v)).join('/')}` : '';
      lines.push({
        title: `${title} · ${ROLE_LABELS[role] || role}`,
        bad: false,
        text:
          `${fixed(camera.rms_px, 2)} px (held out ${fixed(camera.holdout_px, 2)}) over ${camera.stills} stills` +
          ` · f ${fixed(camera.focal?.[0])} · mount ${mount} mm${sigma}` +
          (camera.outliers?.length ? ` · outliers ${camera.outliers.join(', ')}` : ''),
      });
    }
    const agree = model.left_vs_right
      ? ` · the arms agree within ${fixed(model.left_vs_right.translation_mm)} mm, ` +
        `${fixed(model.left_vs_right.rotation_deg, 2)}°`
      : '';
    const robot = model.robot_in_board;
    const shoulders = Object.entries(robot?.j1_mm || {}).map(
      ([joint, [x, y, z]]) =>
        `${joint.startsWith('left') ? 'left' : 'right'} J1 x ${fixed(x, 0)}, y ${fixed(y, 0)}, ${fixed(z, 0)} above`,
    );
    lines.push({
      title: `${title} · placement`,
      bad: false,
      text:
        (shoulders.length
          ? `from tag 0's outer corner (mm): ${shoulders.join('; ')} · turned ` + `${fixed(robot.yaw_deg, 2)}°`
          : `tag 0's corner at ${model.board_mm?.map((v) => fixed(v)).join(', ')} mm`) + agree,
    });
  }
  for (const note of result.notes || []) lines.push({ title: 'Check', text: note, bad: true });
  return lines;
}

// The head camera: why a still cannot be captured now, or null. It needs the live head, held where its camera sees
// the tags (the robot reads the neck's angles over the frames from the head's pose feed).
export function headCaptureBlock(alignment, teleop) {
  if (!alignment) return 'Head alignment runs with Helm on Minerva.';
  if (alignment.busy) return 'Capturing a still…';
  if (alignment.solving) return 'Solving…';
  if (teleop?.phase !== 'running') return 'Start a live session with the head to capture its stills.';
  if (teleop?.receiver_choice?.mode !== 'live') return 'Head stills need the real head: start a live session.';
  if (!['HOLD', 'TELEOP'].includes(teleop?.head?.state)) {
    return 'Enable the head (E), then hold (T) where its camera sees the tags.';
  }
  const recording = teleop?.task?.recording;
  if (recording?.active || ['starting', 'recording', 'stopping', 'finishing'].includes(recording?.state)) {
    return 'Stop the recording to capture stills.';
  }
  return null;
}

// The head's run, and which alignment the next session's head uses.
export function headCountsText(alignment) {
  if (!alignment) return '';
  const count =
    `Head ${alignment.stills} still${alignment.stills === 1 ? '' : 's'}` +
    (alignment.tagged !== alignment.stills ? ` (${alignment.tagged} see tags)` : '');
  const saved = alignment.saved;
  const use = !saved
    ? 'no alignment saved: the head uses its CAD mount'
    : saved.error
      ? saved.error
      : saved.current
        ? `in use: run ${saved.run} (${fixed(saved.fit?.rms_px, 2)} px)`
        : `run ${saved.run} was saved for another head reference: the head uses its CAD mount`;
  return `${alignment.run ? `Run ${alignment.run}` : 'No run yet'} · ${count} · ${use}`;
}

// The last head still: as a wrist still's, with the neck's angles.
export function headLastText(last) {
  if (!last) return '';
  const angles = last.joints_deg
    ? ` · pan ${fixed(last.joints_deg.neck_z)}°, tilt ${fixed(last.joints_deg.neck_y)}°`
    : '';
  return lastText(last) + (last.tags?.length ? angles : '');
}

// A head solve's lines: the fit, the mount (against the CAD one), where the board lies, and against the arms.
export function headResultLines(result) {
  if (!result) return [];
  if (result.error) return [{ title: 'Solve', text: result.error, bad: true }];
  const lines = [];
  lines.push({
    title: 'Fit',
    bad: false,
    text:
      `${fixed(result.rms_px, 2)} px (held out ${fixed(result.holdout_px, 2)}), ${fixed(result.rms_mm, 1)} mm at the` +
      ` board ${fixed(result.distance_m?.median, 2)} m away, over ${result.stills} stills` +
      (result.outliers?.length ? ` · outliers ${result.outliers.join(', ')}` : ''),
  });
  const sigma = result.mount_sigma_mm ? ` ± ${result.mount_sigma_mm.map((v) => fixed(v)).join('/')}` : '';
  lines.push({
    title: 'Mount',
    bad: false,
    text:
      `${result.mount_mm.map((v) => fixed(v)).join(', ')} mm${sigma} in the head link · ` +
      `${fixed(result.cad?.translation_mm)} mm, ${fixed(result.cad?.rotation_deg, 2)}° from the CAD guess`,
  });
  const axis = result.pan_axis_in_board_mm;
  lines.push({
    title: 'Board',
    bad: false,
    text:
      `the neck's pan axis at x ${fixed(axis[0], 0)}, y ${fixed(axis[1], 0)} mm from tag 0's outer corner, ` +
      `${fixed(axis[2], 0)} mm above the table · turned ${fixed(result.spread_deg?.neck_z, 0)}°, tilted ` +
      `${fixed(result.spread_deg?.neck_y, 0)}° over the stills`,
  });
  if (result.arms) {
    lines.push({
      title: 'Against the arms',
      bad: false,
      text:
        `the board's middle ${fixed(result.arms.board_mm)} mm apart (run ${result.arms.run}); a pan zero of ` +
        `${fixed(result.arms.pan_deg, 2)}° explains all but ${fixed(result.arms.other_mm)} mm,` +
        ` ${fixed(result.arms.other_deg, 2)}°`,
    });
  }
  for (const note of result.notes || []) lines.push({ title: 'Check', text: note, bad: true });
  return lines;
}
