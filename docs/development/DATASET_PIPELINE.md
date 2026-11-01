# Feature task: real-data pipeline, step 1 (steady capture, Replay review, LeRobot export)

- **Branch and worktree:** `claude/dataset-pipeline` in `C:\Users\ltr\Workspace\aizee-helm-pedals`.
- **Base commit:** `e998f64` (`claude/ups-monitor`: the deployed `3e7f18b` plus its records).
- **Owner:** Claude (the 2026-10-22 session).
- **The user's goal and decisions** (2026-10-22): "get a model trained finetuning on the real data".
  - ACT first, then π0/π0.5.
  - The handoff protocol: the left hand picks up the cube, hands it to the right hand, and the right hand puts it in the
    box.
  - The left wrist camera's cable is fixed. The user tunes the right wrist exposure at the next teleop.
  - A 2-DoF head (pan/tilt) for the D435 is being built.
- **State at the start:**
  - The Jetson holds six episodes. One real demonstration exists (right arm only, 64 s); the rest are two motionless
    acceptance recordings and three debugging partials.
  - None has a language instruction, and none is promoted for training.
  - The in-house trainer (`python/training/train_minerva.py`) reads format 8 but is not an open-source model. Nothing
    converts to LeRobot.
  - No policy runs on the real robot.
- **Problems this step solves:**
  1. **Uneven capture.** Episodes record at 14.97 Hz on average against a requested 20 Hz. Their gaps cluster at ~50 ms and
     ~100–120 ms, and 21 % of gaps in demo 1 exceed 90 ms.
     - The head D435 publishes ~15 fps, and every head frame is recorded once: the head frame number advances by
       exactly one between 951 of 952 recorded frames.
     - The capture loop records only when every camera has a new acquisition, which is right: a repeated frame is never
       relabelled. But after such a skip it waits a whole period (50 ms at 20 Hz) instead of looking again on the
       loop's next poll.
     - The wrists publish at ~50 fps and never limit it.
  2. **No review.** Nothing marks a demo to keep or discard. The raw episode files must not change; `training_promoted`
     stays the capture's own field.
  3. **No open-source training format.** openpi pins LeRobot `0cf8648…` (dataset format v2.1) and trains from LeRobot
     datasets, as does LeRobot's ACT.
- **Design:**
  - **Capture** (`helm/task_capture.py`, `config/devices/minerva-task-capture.json`):
    - After a skipped tick the loop looks again on its next poll (≤ 10 ms, sooner when a camera publishes) instead of a
      whole period later.
    - `hz` is 15, the head camera's rate.
    - Every admission rule stays: a new acquisition from every camera, camera skew ≤ 20 ms, receiver age ≤ 30 ms,
      camera age ≤ 0.2 s.
  - **Review** (Replay; `helm/episode_worker.py`, `helm/episodes.py`, server, `static/replay.js`):
    - Keep or Discard, with an optional note, per episode.
    - Each review is one line appended to `~/minerva/evidence/<session>/reviews.jsonl` (the latest line per episode wins),
      written by the robot-side reader. Episode files are untouched.
    - Replay shows each episode's verdict and filters by it. Reviews are refused while a recording runs, like every
      Replay read.
  - **Export** (`robot_platform/data/lerobot_export.py`):
    - **Core (no LeRobot needed, tested in the gate):**
      - Reads a format-8 episode and refuses incomplete ones, and mixed joint rosters within one dataset.
      - Keeps only the span from the first to the last human-authority row with a valid action.
      - Resamples onto a fixed grid (15 fps) by the robot-clock timestamps: each grid time takes the latest recorded
        frame at or before it, and a grid time more than 1.5 periods after its frame is refused.
      - Maps `observation.state` = `qpos`, `action` = `actions` (the next commanded position), the three cameras as
        `observation.images.<role>`, and the task string.
    - **Writer:** LeRobot's `LeRobotDataset.create / add_frame / save_episode` at openpi's pinned revision, imported
      only when writing, in a training environment.
    - **Selection:** only episodes reviewed Keep, unless asked otherwise. A sidecar `minerva_provenance.json` records
      each exported episode's source file, hash, review and kept span.
- **Shared interfaces held fixed:** the episode format and writer, the receiver, the motion contract, and the capture
  admission bounds.
- **Acceptance:**
  - `scripts/dev.py check --offline`, zero skips.
  - Capture rehearsal on the Jetson (cameras only, no motion): recorded gaps cluster at ~67 ms with none over 90 ms.
  - Demo 1 exports to a LeRobot v2.1 dataset on coolserver that `LeRobotDataset` loads, and a short ACT training run starts on it.
  - The user records a first batch and marks it in Replay.
- **Results:**
  - **Capture:**
    - The reproducer (`test_frames_follow_the_slowest_camera_at_a_steady_rate`; head 15 fps, wrists 50 fps) failed at
      20 Hz before the fix, with gaps of 52–60 and 104–112 ms, the same pattern as demo 1.
    - After the fix it passes at 20 and 15 Hz, requiring every head frame to be recorded once.
    - Ticks follow a grid from the first frame: scheduling each tick a period after the last admission first gave 14.1 Hz,
      with the noticing delay adding up into skipped head frames.
    - **Rehearsed on the Jetson** with the live cameras and no motors (capture rehearsal, staged release
      `20261022T140850Z-0b0318a`, never activated): two 30 s recordings of 451 frames each, 15.0 Hz.
      | | Before | After |
      |---|---|---|
      | Gap median | 55.6 ms | 67.0 ms |
      | Gap p90 | 108 ms | 73.2 ms |
      | Gap max | 124.9 ms | 77.4 ms |
      | Camera age at admission, median | 55.7 ms | 39.5 ms |
      - Receiver age stayed at most 17.1 ms against the 30 ms bound.
  - **Review:** tests for the log (latest wins, a torn line, the episode file untouched), the checks (partials, notes,
    names) and the API (token, the busy refusal).
    - In a local Helm on demo 1's copy, Discard with a note wrote its line, and the list, badge and counts followed.
  - **Export:**
    - Six tests cover the span, resampling (demo 1's jitter, one dropped frame kept, a 267 ms gap refused), selection,
      a mixed roster refused, frames and the sidecar, and a dry run without LeRobot.
    - **Demo 1 on coolserver** (`~/aizee-sim`):
      - It exported in 29 s to 8.4 MB, from a 435 MB episode: 966 frames at 15 fps, three AV1 videos.
      - Its old 20 Hz capture repeats 90 rows on the grid; frames are up to 120 ms stale.
      - LeRobot loads it (v2.1, 15 fps). On 42 sampled frames the timestamps sit on the grid and state and action equal
        the source rows; decoded video is 32.6–35.0 dB PSNR from the stored images.
      - **ACT:** 200 steps ran (52M parameters, 0.07 s a step at batch 8 on an RTX 3090), the loss fell from 21 to 3.0,
        and a checkpoint was saved. One episode teaches nothing; the path works.
    - **Training environment:** `~/aizee-sim/.venv-lerobot`, built by `~/aizee-sim/bin/setup_lerobot.sh`.
      - Pins: lerobot `0cf8648…` (openpi's), torch 2.6.0+cu124, torchvision 0.21.0, torchcodec 0.2.1, av 14.2.0,
        datasets 3.6.0, huggingface-hub 0.36.2, diffusers 0.35.2.
      - The newest av (19), datasets (5) and huggingface-hub (1.x) break this LeRobot revision: av returns an
        AVRational where it expects a fraction, and datasets a Column where it expects tensors.
      - openpi itself locks av 17 but converts to image (not video) features.
  - **Getting episodes off the robot:** the Jetson is not on Tailscale. Episodes reach a training machine through the
    terminal hop, or directly once Tailscale is on the Jetson.
- **Remaining uncertainty:**
  - The 2-DoF head will add two joints and move the head camera. Data from before and after it should not be mixed
    unless the head is held at the same pose.
  - Whether 15 Hz is enough for the handoff's fast phases (DROID trains at 15 Hz; ALOHA at 50 Hz).

## Addendum, 2026-11-01: format 9

- Episodes now record the whole robot ([EPISODE_FORMAT.md](../EPISODE_FORMAT.md)): 16 axes with the neck, the motors'
  effort and velocity, and the head camera's pose per frame.
- The exporter reads formats 8 and 9. Format 9 adds `observation.effort`, `observation.velocity` and
  `observation.head_camera_pose` when every exported row has them.
- A 14-axis (format 8) and a 16-axis episode are never mixed in one dataset: the exporter refuses the second roster. This
  settles the head's uncertainty above for export; whether old data is worth training alongside new is still open.
