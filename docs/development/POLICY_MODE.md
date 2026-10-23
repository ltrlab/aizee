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
- **Next:**
  1. **Cameras for the worker on the robot:** the capture profile's endpoints, decoded to 320×240 exactly as the
     recorder stores them.
  2. **The policy runtime on the Jetson:** done; its system torch runs the exports from `~/minerva/policies`.
  3. **Helm:** teleop start with `policy: {policy}`, a Policy box listing the robot's checkpoints with the live
     confirmation, and a card with the chunk age and inference time.
  4. **Recording rollouts** with `policy` authority, and Hold/Engage interventions as `correction`.
  5. **Rehearsal** in `helm --sim` with a sim-trained checkpoint and the simulated cameras, then `hold_still` on the
     arms, then the ACT checkpoint with the user present.
