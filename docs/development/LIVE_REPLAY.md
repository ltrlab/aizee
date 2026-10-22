# Feature task: live replay (an episode drives the arms) and the policy-mode groundwork

- **Branch and worktree:** `claude/live-replay` in `C:\Users\ltr\Workspace\aizee-helm-pedals`.
- **Base commit:** `290ed96` (`claude/dataset-pipeline`, deployed 2026-10-22 as Jetson `20261022T204934Z-290ed96` and
  terminal `helm-recording-290ed96413`).
- **Owner:** Claude (the 2026-10-22 session), at the user's request: "start on the policy mode. also add in the ability
  for a live replay, where an episode can be selected, the live camera feeds are shown, and the robot replays the
  episode positions."
- **Problem:**
  - Recorded demonstrations can be watched in Replay, but nothing can drive the arms from one.
  - Live replay shows whether a demonstration is reproducible on the arms (repeatability, calibration drift) before it
    trains anything.
  - It is also the first source of targets other than the leaders: policy mode is the same thing with a model in
    place of the episode.
- **Design:**
  - **The source** (`teleop/episode_source.py`, `EpisodeSource`): the episode's recorded commands (`qcmd`, receiver
    motor radians) are the loop's target, in the place of the mapped leaders.
    - It reads the episode's joints by name in the receiver's order and refuses an episode missing any of them.
    - Each target is one row per actuator (name, position, sample time, a sequence that advances every cycle), the
      shape the mapper gives the lifecycle.
    - **Playback clock:** it runs only while the lifecycle follows (TELEOP), at 0.25, 0.5 or 1×. The position between
      recorded frames is interpolated linearly, never extrapolated.
    - **Engage** first aligns the arms to the current frame at the lifecycle's alignment speed: the first frame, or
      where a pause left off. A pause (HOLD) stops the clock.
    - **The end:** the last frame holds and the loop disengages to HOLD at that pose. Engage again rewinds to the first
      frame, aligning back to it first.
  - **The loop** (`helm/teleop.TeleopCore`), for a bundle with `source: {kind: 'episode', episode, speed}`:
    - No leaders, leader ranges, mapper or grip feedback; the target comes from the source.
    - The receiver, lifecycle, collision mode, alignment and speed limits, H/X, pedals, the foot switch and the
      evidence recorder are unchanged.
    - Task recording is refused: a replayed demonstration is not a human demonstration.
    - The loop's view carries `replay`: time, frame, duration, speed, finished.
  - **The robot side** (`helm/edge.serve`): a source bundle binds no leader link and reports port 0, as the servo
    sweep does. The episode is read from `~/minerva/evidence` by Replay's own name check (`resolve_episode`).
  - **Helm:**
    - Teleop start takes `replay: {episode, speed}` and refuses unless the episode is complete and its joints cover the
      receiver's. The live confirmation is the same as teleop's.
    - A loop on this PC reads the episode from Helm's `--episodes` directory.
  - **The page** (Replay tab):
    - A **Replay on Minerva** box for the selected episode: speed, and Start, which starts the session.
    - The session's own controls (Idle, Enable, Engage, Hold, Return, Disable, and the pedals) drive it as for teleop.
    - While it runs, the live camera feeds show beside the recorded frames, and the recorded frames follow the arms'
      playback position.
- **Safety:**
  - Nothing new reaches the command path: every target goes through the same lifecycle (alignment speed first, joint
    speed caps), the receiver's limits and the profile's collision mode, as a leader target does.
  - Speeds above 1× are refused.
  - The source never invents a pose: it interpolates between recorded commands and holds the last one.
  - Rehearse in `helm --sim` (the real Rust receiver and a simulated Minerva) before the first live run, which the user
    attends.
- **Policy mode** (next; `PolicySource` behind the same interface):
  - A policy worker process off the control CPUs takes observations (the three cameras decoded as the dataset stores
    them, 320×240, and `qpos`) at 15 Hz.
    - **Where it runs:** ACT on the Jetson's GPU, or pi0/pi0.5 on a GPU server on the robot's network (the user's
      RTX 3080), served openpi-style.
    - It returns action chunks with the observation time they answer.
  - The source interpolates the latest chunk onto the loop's cycles and never uses a chunk older than a freshness bound
    (it holds instead).
  - The loop sees an ordinary target; Engage aligns to the policy's first action. The operator's Hold, Return, Disable
    and the foot switch stay in charge.
  - Policy rollouts are recorded with `policy` authority, never `human`.
- **Shared interfaces held fixed:** the receiver, the guarded and direct lifecycles, the collision envelopes, the
  leader path, the episode format, and the recording format.
- **Acceptance:**
  - `scripts/dev.py check --offline`, zero skips.
  - The source, the loop with a synthetic episode on the virtual receiver, the robot side and Helm's API under test.
  - A rehearsal in `helm --sim` replaying demo 1: the simulated joints track the recorded commands.
  - On the robot, with the user: replay a kept demonstration at 0.5×, then 1×.
- **Results:** (to fill)
