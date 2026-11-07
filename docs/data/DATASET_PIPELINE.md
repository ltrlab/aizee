# Dataset pipeline: steady capture, Replay review, LeRobot export

How recordings become a training dataset, and why each stage behaves as it does. For the end-to-end procedure see
[TRAINING.md](TRAINING.md); for the file layout see [EPISODE_FORMAT.md](EPISODE_FORMAT.md).

Open-source policies (ACT in LeRobot, then pi0 and pi0.5 through openpi) train from LeRobot datasets. openpi pins LeRobot
revision `0cf8648…`, dataset format v2.1, so the export targets that. The in-house trainer
(`python/training/train_minerva.py`) reads episode files directly and needs no conversion.

## Capture

`helm/task_capture.py` with `config/devices/minerva-task-capture.json` records at 15 Hz, the head D435's rate.

- A frame is recorded only when every camera has a new acquisition, so a repeated frame is never relabelled as new.
- After a skipped tick the loop looks again on its next poll (at most 10 ms, sooner when a camera publishes) instead of
  waiting a whole period. At the old 20 Hz setting the head camera, publishing about 15 fps, paced the recording into
  gaps of one and two periods; a 15 Hz grid with prompt re-polling records at the head camera's steady rate, and every
  head frame is recorded once.
- Admission bounds are unchanged: a new acquisition from every camera, camera skew at most 20 ms, receiver age at most
  30 ms, camera age at most 0.2 s.
- Format 9 also needs a fresh head pose sample (at most 0.1 s old at the frame's cutoff); without one capture is not
  ready.

A rehearsal on the Jetson with live cameras and no motors recorded 30 s episodes at 15.0 Hz. Gaps clustered near 67 ms
with a maximum of 77 ms, where the 20 Hz setting had a median of 56 ms and a maximum of 125 ms. Receiver age stayed
under its 30 ms bound.

## Review

Replay lets an operator mark each episode **Keep** or **Discard**, with an optional note, and filters by verdict
(`helm/episode_worker.py`, `helm/episodes.py`, `static/replay.js`).

- Each review is one line appended to `~/minerva/evidence/<session>/reviews.jsonl`, written by the robot-side reader.
  The latest line per episode wins.
- Episode files are never changed, and `training_promoted` stays the capture's own field.
- Reviews are refused while a recording runs, like every Replay read.

## Export

`robot_platform/data/lerobot_export.py`. The selection and resampling need only numpy and h5py and run in the offline
gate; LeRobot's `LeRobotDataset.create / add_frame / save_episode` is imported only to write, in a training
environment.

- **Selection.** Only complete episodes reviewed Keep, unless `--include-unreviewed`. Partial files, incomplete
  episodes and mixed joint rosters in one dataset are refused.
- **Span.** From the first to the last row a human drove with a valid action. A policy session's episode (rows labelled
  `policy` or `correction`, [POLICY_MODE.md](../operator/POLICY_MODE.md)) exports as its stretches instead, one LeRobot
  episode per run of rows the policy or a correction drove while the arms followed, so none spans a hold or an
  alignment between drivers. A policy stretch the operator took over from loses its last `--pre-takeover-s` (1 s),
  where it went wrong; stretches under `--min-stretch-s` (0.5 s) are left out.
- **Resampling.** Onto a fixed grid (15 fps) by the robot-clock timestamps: each grid time takes the latest recorded
  frame at or before it, so nothing comes from the future. A grid time more than `MAX_GAP_PERIODS` = 2.5 periods
  after its frame (two dropped frames in a row) refuses the episode; one dropped frame passes.
- **Features.** `observation.state` = `qpos` and `action` = `actions` (the next commanded position), both over all of the
  episode's axes: 14 for format 8, 16 for format 9. Each camera is `observation.images.<role>`, stored as video.
  Format 9 adds `observation.effort`, `observation.velocity` and `observation.head_camera_pose` when every exported row
  has them (`complete_extras`).
- **Never mixed.** Episodes with different joints, cameras, image sizes or extras go into separate datasets. Format 8
  and format 9 data are therefore never combined by the exporter; whether old data is worth training alongside new is
  an open question.
- **Provenance.** `meta/minerva_provenance.json` records each exported episode's source file, hash, review and kept
  span, who drove it (`human`, `policy` or `correction`), and the seconds trimmed before a take-over.

## What was measured

On the first real demonstration (right arm only, 64 s, recorded at the older 20 Hz setting):

- It exported in 29 s to 8.4 MB from a 435 MB episode: 966 frames at 15 fps, three AV1 videos. The old capture repeats
  90 rows on the grid, with frames up to 120 ms stale.
- LeRobot loaded it (v2.1, 15 fps). On 42 sampled frames, timestamps sat on the grid, state and action equalled the
  source rows, and decoded video was 32.6 to 35.0 dB PSNR against the stored images.
- A 200-step ACT run (52 M parameters, 0.07 s a step at batch 8 on an RTX 3090) lowered the loss from 21 to 3.0 and
  saved a checkpoint. One episode teaches nothing; the path works.

These numbers are for format 8. The format 9 export has run only against synthetic and offline test episodes.

## The training environment

`scripts/training_server/setup_lerobot.sh` builds it, with the pins listed in [TRAINING.md](TRAINING.md#4-the-lerobot-environment).
`scripts/training_server/` holds the rest: `check_lerobot_dataset.py` loads an export with LeRobot itself and compares
sampled frames with the source, and `train_act.sh` trains ACT. Newer `av` (19), `datasets` (5) and `huggingface-hub`
(1.x) break this LeRobot revision. openpi itself locks `av` 17 and converts to image rather than video features.

## Open questions

- Whether 15 Hz is enough for the handoff's fast phases (DROID trains at 15 Hz, ALOHA at 50 Hz).
- Training does not yet weight corrections above a policy's own stretches; the provenance sidecar says which LeRobot
  episode is which.
