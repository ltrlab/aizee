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
- **Results:** (to fill)
- **Remaining uncertainty:**
  - The 2-DoF head will add two joints and move the head camera. Data from before and after it should not be mixed
    unless the head is held at the same pose.
  - Whether 15 Hz is enough for the handoff's fast phases (DROID trains at 15 Hz; ALOHA at 50 Hz).
