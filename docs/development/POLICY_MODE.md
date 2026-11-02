# Feature task: policy mode (a trained policy drives the arms)

- **Branch and worktree:** `claude/live-replay` in `C:\Users\ltr\Workspace\aizee-helm-pedals`, after live replay
  (`docs/development/LIVE_REPLAY.md`), whose source interface it shares.
- **Owner:** Claude (the 2026-10-22 session), at the user's request: "start on the policy mode".
- **Goal:** run a policy fine-tuned on Minerva's recorded demonstrations (ACT first, then pi0/pi0.5) on the arms
  from Helm. The operator keeps every control they have in teleop.
- **Design:**
  - **The source** (`teleop/policy_source.py`, `PolicySource`): the loop's target from the policy's action chunks.
    - **Chunk timing:** a chunk answers an observation taken at `obs_time` (the loop clock); row k is meant for
      `obs_time + k * action_dt`, the evaluation runner's contract (`evaluation/runner.py`).
    - **Latency:** inference takes real time (ACT on the Jetson 100–330 ms, 2026-10-18), so a chunk arrives late.
      The source starts it at the row for now, never replaying rows already past. Past its last row it holds that
      row.
    - **Blending:** each new chunk crossfades in over `blend_s` (0.2 s), so two chunks that disagree are never a
      step.
    - **Freshness:** a chunk is followed only within `max_age_s` (1 s) of its observation. Without a fresh one the
      source gives no target, and the lifecycle does what it does for lost leaders: it holds the last bounded
      target, then drops to Hold. A request unanswered for `max_age_s` is given up and asked again.
    - **By lifecycle state:**
      - Hold (after Enable): one preview chunk from where the arms are; its first row is the target, unmoving.
      - Engage: aligns to that row at the alignment speed, as for the leaders.
      - Teleop: the rollout. The source asks for a new chunk whenever none is pending, and holds the aligned row
        until the first one arrives.
      - Hold (T), a source loss or a fault ends the rollout; Engage aligns anew from a new preview.
  - **The worker** (`teleop/policy_worker.py`, `PolicyWorker`): the policy in its own process, so the loop never
    waits on inference.
    - It loads a policy through `evaluation.policies.load_policy`: the same policies the simulation evaluator runs.
    - It answers each observation (measured `q`, `dq`, `effort`, the receiver's `qcmd`, plus the cameras' latest
      frames) with a chunk matched to its request, and reports failures without dying.
    - It keeps off the control CPUs (`prefer_background`) and starts loading while the receiver opens.
    - A policy that names no `action_dt` gets the loop's period, as in the evaluator.
  - **The loop** (`helm/teleop.TeleopCore`), for a bundle with `source: {kind: 'policy', policy: <spec>}`:
    - Each cycle it collects the worker's answers, then hands it the cycle's observation when the source wants a
      chunk.
    - No leaders, routes, zero or grip feedback. The view carries `policy`: name, requests, chunks, the chunk's age,
      inference time, stale cycles and the last error.
    - Engage before the preview has arrived is refused with a note to press T again.
  - **Safety:**
    - Every target goes through the lifecycle (alignment speed, speed caps, settle), the receiver's limits, the
      profile's collision mode, H/X, the pedals and the foot switch.
    - On the arms a policy session runs only a trained checkpoint (`checkpoint:`, `lerobot:`) or `hold_still`. The
      evaluator's other reference policies are simulation checks: `penetrating` drives a gripper through the table
      on purpose.
    - A rollout is not recorded until the capture labels its frames `policy` (it would label them `human`).
- **Done (2026-10-22):**
  - The source, the worker and the loop's policy branch.
  - `tests/unit/test_policy_source.py` and `tests/unit/test_policy_worker.py` (a real spawned worker, the live
    allowlist).
  - `tests/contract/test_policy_mode.py` on the actual Rust receiver, with the guarded profile and the terminal's
    direct profile with Motor tuning, using a scripted policy answering four cycles late:
    - preview, align and follow;
    - Hold, a new preview and resume;
    - a silent policy leaves the arms held, without a fault;
    - Return.
- **The ACT adapter (2026-10-22, `data/lerobot_policy.py`, `lerobot:<dir>` in `evaluation.policies.load_policy`):**
  - **Export** (coolserver, the pinned LeRobot): the network with LeRobot's normalization and unnormalization traced
    into one TorchScript file, plus `minerva_policy.json` (joints, fps, cameras, image shape) and a sample.
    - The traced chunk's first row equals `select_action`'s action, and the trace differs from LeRobot by 0.15 mrad
      (bound 1 mrad).
    - Why not LeRobot on the robot: importing ACT's config at this revision pulls in datasets, pandas, pyarrow and
      av, which would upgrade the numpy (1.21.5) the robot's own code runs on.
  - **On the Jetson** (system Python 3.10, torch 2.8.0, CUDA 12.6), `act-demo1-smoke` in `~/minerva/policies`:
    - loads in 1.0 s and gives a (100, 14) chunk within 0.3 mrad of coolserver's;
    - 37 ms a chunk at the median (40 ms p90) and 173 MB of GPU memory, run on the background CPUs at nice 10;
    - the first two calls take 3.3 s and 2.8 s while TorchScript optimizes, so `ExportedPolicy` warms up three
      calls before it reports ready.
  - **Robot side** (`ExportedPolicy`, plain torch):
    - refuses a receiver whose joints differ from the training joints;
    - takes the cameras in its own order at the recorded 240×320 size, and checks every answer;
    - `tests/unit/test_lerobot_policy.py` runs it with a stand-in for the TorchScript call.
- **In Helm (2026-10-22):**
  - **Cameras** (`CaptureCameras`): the worker subscribes to the capture profile's loopback endpoints and decodes
    through the task capture's `CameraCache`, so it takes the newest coherent set within the recording's age and
    skew bounds and stores the exact pixels the recorder would (`recording_images`). Helm sends the profile as
    `policy_cameras` to a loop on the robot, and starts no capture process. Without a fresh set the observation
    fails and the arms hold.
  - **The worker is a subprocess** (`python -m robot_platform.teleop.policy_worker`) with length-prefixed pickles
    on its stdin/stdout and its other output in a log file.
    - The first `helm --sim` rehearsal found that Helm's PC loop runs in a daemonic process, which multiprocessing
      forbids children.
    - The log file also keeps the policy's printing out of an SSH stream nobody reads until the session ends.
  - **Start:** teleop start takes `policy: {policy}`, not both a replay and a policy. On the robot the source travels
    as Helm sent it; only an episode source gets the robot's own root.
  - **Teleop's Policy box:**
    - Choices: `hold_still`, and the policies exported to `~/minerva/policies`, listed by Replay's reader on the
      robot (`policies` op, reading only each `minerva_policy.json`).
    - Start goes through the live confirmation.
    - While a policy drives, a card shows its chunks, chunk age, inference time, stale cycles and errors, with the
      next step. The task workflow, force feedback and Set zero are hidden or off.
- **Rehearsal (`helm --sim`, the terminal's profile):** `hold_still` loaded in its subprocess, the preview answered
  in 0.07 ms and Hold aligned. The simulated receiver then expired a hold command on time-correct cycles: the same
  laptop sim artifact as the replay rehearsals (`helm-sim-mode` notes), not policy load. The real-time test is on
  the Jetson.
- **Deploy attempt (2026-10-23 00:44Z, approved; not finished):**
  - The gate on `ed51601` passed: 1273 tests, zero skips.
  - **Jetson:** `20261023T004425Z-ed51601` is staged and smoke-tested, not active; `current` stays
    `20261022T235916Z-f486fb3`.
  - **Terminal:** the console stage failed twice, both on the terminal's hardware.
    - First, a cached numpy wheel failed its hash.
    - After a pip cache purge, the retry's PowerShell died with an access violation, Defender crashed, and the
      terminal blue-screened (0xEF CRITICAL_PROCESS_DIED) and rebooted. There is an earlier dump from 2026-09-30.
    - Both partial folders are set aside; the terminal still runs `helm-recording-f486fb3f27`.
  - The Jetson closed the open (disabled) session on "Helm went silent for 5 s", as designed.
  - Waiting on a memory test of the terminal before the deploy is retried.
- **Policy corrections (2026-11-01, `claude/policy-takeover` on `claude/teleop-modes`):** the user asked for "policy
  mode which would have human in the loop corrections", taking over with today's leaders (the arms align to them;
  powered leaders that follow the robot would need firmware), and: "policies own successful stretches should be
  recorded and concated with the corrections".
  - **The session:** Teleop's Policy mode starts `policy: {policy, takeover: true}`. The loop (`helm.teleop`)
    opens the leaders as a teleop session does (ranges, zero, routes; on the robot the edge links them), beside the
    policy source; force feedback stays off. `driver` says who drives: `policy`, or `leaders` after a take over.
  - **Switching:** O (Helm's key, and the policy card's button) or pedal B while the arms follow (`take_over`,
    `hand_back` in `helm.pedals`). Who drives changes only through a hold: engaged, the cycle disengages (the
    lifecycle's own Hold, measured pose, no target), and Resume (B or T) aligns the arms to the new driver at the
    alignment speed. To the leaders, as Engage does; to the policy, from a new preview taken where the arms hold.
    A take over drops the rollout (`PolicySource.pause`, which now also forgets the last state, so the hand back's
    Hold does not cancel its own preview). The policy is asked nothing while the operator drives. Disabled or Idle,
    O only chooses who drives first. The lifecycle, receiver, collision mode and speed limits are unchanged: this is
    target selection above the motion contract.
  - **Recording:** pedal A records in a policy session on both live arms. The capture process now carries a label
    with each receiver sample (`helm.teleop._capture_label`): `authority` (`policy`, or `correction` while the
    leaders drive; `human` in a teleop session) and the lifecycle state, written as the frame's authority and its
    task state's `lifecycle`. One episode holds the policy's stretches and the corrections in recorded order; the
    episode writer already marks an action invalid where the authority changes. A capture that cannot label (the
    simulation's) still refuses a rollout.
  - **Export:** `lerobot_export.corrected_stretches` (see `docs/development/DATASET_PIPELINE.md`).
  - **Tests:** `tests/contract/test_policy_takeover.py` on the actual Rust receiver: take over by key (arms held,
    policy no longer asked, then aligned to and following the leaders), hand back (one preview, aligned, following
    the policy's trajectory again), the full pedal sequence through the loop worker, and a session without takeover
    opening no leaders. Unit tests for the pedal offers and confirmation, the capture label (process and frame), the
    export's stretches, the session start and the edge's leader link; page tests for the card, notes and pedals.
  - **Not done:** not deployed or run live. Rehearse on the Jetson's virtual receiver with the real leaders first
    (take over, hand back, a recording), then live with the user present. Training does not yet weight corrections
    above the policy's stretches; the provenance sidecar says which LeRobot episode is which.
- **Next:**
  1. **Deploy** after the terminal's memory test.
  2. **`hold_still` on the Jetson's virtual receiver, then live** with the user present: Engage aligns to the
     reference pose and holds.
  3. **ACT trained on the new demonstrations:**
     - export it on coolserver (`python -m robot_platform.data.lerobot_policy export ...`), copy it to
       `~/minerva/policies/<name>`;
     - run it on the virtual receiver on the Jetson with the live cameras, then live.
  4. **Recording rollouts** with `policy` authority and corrections as `correction`: built (policy corrections,
     above), not yet run live.
  5. **Simulated cameras** for policy mode in `helm --sim`.
