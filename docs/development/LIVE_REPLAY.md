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
- **Results (2026-10-22, offline and in `helm --sim`):**
  - **Tests:**
    - `tests/unit/test_episode_source.py`.
    - `tests/unit/test_helm_live_replay.py`: the robot side binds no leader link and reads its own recordings; the
      session asks for no leaders and starts no capture; the route refuses partial and unreadable episodes.
    - `tests/contract/test_live_replay.py` on the actual Rust receiver, with the guarded profile and with the direct
      profile with Motor tuning that the terminal runs.
    - The page checks in `tests/ui/test_helm_replay.mjs`.
  - **Rehearsal** (`helm --sim`, the real receiver on a simulated Minerva; demo 1, `live-20261021T004507Z`: 64.5 s,
    right arm only, the left arm held out at J1 0.48, J5 −0.29, J6 0.77 rad):
    - **Guarded profile at 1×:** Engage aligned to frame 0; the replay ran to its end and held. The receiver's
      commands matched the recorded `qcmd` within 2.2 mrad on every arm joint for the whole episode (right arm spans
      0.9–2.0 rad). The simulated arm tracked them within 0.049 rad. The simulated right gripper lagged up to 2 rad:
      the frozen sim's known gripper artifact.
    - **0.5×:** the playback rate measured 0.503. Hold froze the clock (8.083 s for 3 s); Engage aligned back and
      continued. A page reload mid-replay kept the card.
  - **Mirrored right J3 and J4 in the first rehearsal (the user's catch):**
    - **Cause:** that rehearsal's Helm ran from a fresh runs folder seeded with the 2026-09-24 direction file, which
      confirms only the J1s. Every other joint took the default +1, where the robot's confirmed mapping has right J3
      and J4 at −1. The MuJoCo window (and that Helm's model view and collision model) drew them mirrored.
    - **The robot is unaffected:**
      - The terminal's direction file has all 12 joints confirmed (right J1, J3 and J4 at −1).
      - The demo's receiver identity (calibration `a17c8873…`) is the joint-map hash of exactly that mapping, not of
        the seed-only one (`4a96b05b…`).
      - The simulated datasets use the documented directions (right J1, J3, J4 reversed): consistent.
      - The LeRobot export is in receiver motor radians and uses no mapping.
    - **Fix:** rehearsals use copies of the terminal's direction file and Motor tuning preset
      (`build/replay-sim/runs`).
    - **Open:** teleop, replay and `helm --sim` start on unconfirmed directions without a word; Minerva Check fails
      them. A start refusal (live) or warning (virtual) is offered, not built.
  - **Found and fixed by rehearsing on the terminal's profile:**
    - **Target order:** with Motor tuning the build's `names` is a set. The replay took its joints from it, so the
      first target was out of the lifecycle's order and faulted the session at once. It now takes the lifecycle's
      order. The guarded-only contract test had missed it: it now runs both profiles.
    - **Simulated first cycle:** the simulated arms tick in real time from when their receiver opens, so the first
      exchange simulated all of the build (108 ms) and faulted the 35 ms direct budget. The build now catches them up
      before the loop starts.
  - **Gate and deployment (approved by the user 2026-10-22):**
    - The gate on `f486fb3` passed: 1261 tests, zero skips, a clean tree before and after.
    - **Jetson:** `20261022T235916Z-f486fb3` is active; the stage and post-activation smoke both passed.
    - **Terminal:** `helm-recording-f486fb3f27` is active (previous `helm-recording-290ed96413`), restored to Live both
      with the task set up, and its page refreshed.
    - The user will record new, safer demonstrations to replay first; demo 1 is not to be replayed.
  - **Laptop timing:** two guarded-profile runs faulted on this laptop under load: stale feedback during a 47–52 ms
    loop stall, then `collision_expired` with the cycle p99 at 37 ms. In both the lifecycle braked, lowered and
    disabled as designed. On the terminal's direct profile the cycle p99 was 5.5 ms.
