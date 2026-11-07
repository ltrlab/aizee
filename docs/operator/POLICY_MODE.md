# Policy mode

A policy fine-tuned on Minerva's demonstrations (ACT first, then pi0/pi0.5) drives the arms from Helm. The operator keeps every
control they have in teleop. It shares its source interface with [Replay on the arms](LIVE_REPLAY.md). Safety:
[Safety](SAFETY.md).

**Status.** Built and tested offline (a scripted policy against the real Rust receiver, guarded and direct profiles) and
rehearsed in `helm --sim` with `hold_still`. An ACT policy exported for the Jetson loads and runs there (about 37 ms a chunk at
the median on the Jetson's GPU). No document records a policy driving the arms, or a policy-correction session, on the robot.
**It drives 14 axes**: `data/lerobot_policy.py` refuses a policy whose joints differ from the receiver's, so a policy trained on
[format 9](../data/EPISODE_FORMAT.md)'s 16 axes cannot drive the robot yet.

## Using it

Teleop > **Mode: Policy**. Choose a policy in the **POLICY MODE** card (`hold_still`, and the policies exported to
`~/minerva/policies/`; **Refresh** reads them again) and press Start. It starts a teleop session with the leaders connected and the
live confirmation. **Enable (E)** shows the policy's first action as a preview, unmoving; **Engage (T)** aligns to it at the
alignment speed; the policy then drives. A card shows the chunks, their age, inference time, stale cycles and errors. Task
workflow, force feedback and Set zero are hidden or off. On the arms a policy session runs only a trained checkpoint
(`checkpoint:`, `lerobot:`) or `hold_still`; the evaluator's other reference policies are simulation checks (`penetrating` drives
a gripper through the table on purpose).

**Corrections.** Take over with pedal **B** (or **O**, or the card's **Take over (O)**): the arms hold where they are; line the
leaders up with them and Resume (B or T) aligns the arms to the leaders at the alignment speed. B again hands back the same way:
the arms hold, the policy plans from there, and Resume aligns them to its first action. Pedal **A** records an episode of both:
each frame is labelled with who drove it (`policy` or `correction`), and the LeRobot export keeps each stretch without the holds
and alignments between them ([dataset pipeline](../data/DATASET_PIPELINE.md)). Training does not yet weight corrections above the
policy's stretches.

## How it works

- **The source** (`teleop/policy_source.py`): a chunk answers an observation taken at `obs_time`; row k is meant for
  `obs_time + k * action_dt`. Inference takes real time (ACT on the Jetson 100-330 ms in the first measurement), so a chunk
  arrives late; the source starts it at the row for now and holds the last row past its end. Each new chunk crossfades in over
  0.2 s. A chunk is followed only within 1 s of its observation; without a fresh one the source gives no target and the lifecycle
  does what it does for lost leaders: it holds the last bounded target, then drops to Hold. An unanswered request is given up
  after 1 s and asked again.
- **By lifecycle state:** Hold (after Enable) shows one preview chunk; Engage aligns to it; Teleop is the rollout, asking for a
  new chunk whenever none is pending and holding the aligned row until the first arrives; Hold (T), a source loss or a fault ends
  the rollout, and Engage aligns anew from a new preview. Engage before the preview has arrived is refused with a note to press T
  again.
- **The worker** (`teleop/policy_worker.py`): the policy runs in its own subprocess off the control CPUs so the loop never waits
  on inference. It loads through `evaluation.policies.load_policy`, the same policies the simulation evaluator runs, answers each
  observation (measured `q`, `dq`, effort, the receiver's `qcmd`, and the cameras' latest frames) with a chunk, and reports
  failures without dying.
- **Cameras:** the worker subscribes to the capture profile's loopback endpoints and decodes through the task capture's
  `CameraCache`, taking the newest coherent set within the recording's age and skew bounds, stored at the recorded size
  (320 x 240). Without a fresh set the observation fails and the arms hold.
- **Export** (on the training server, with the pinned LeRobot): the network with LeRobot's normalization traced into one
  TorchScript file plus `minerva_policy.json` (joints, fps, cameras, image shape). The trace differs from LeRobot by 0.15 mrad
  (bound 1 mrad). The robot runs it with plain torch (no LeRobot on the robot, which would upgrade the numpy its own code
  runs on). `ExportedPolicy` warms up three calls before it reports ready. See [Training](../data/TRAINING.md).
- **Safety:** every target goes through the lifecycle (alignment speed, speed caps, settle), the receiver's limits, H and X, the
  pedals and the foot switch, as a leader target does. A policy session records with `policy` authority, never `human`.

## Next

1. `hold_still` on the Jetson's virtual receiver, then live, attended: Engage aligns to the reference pose and holds.
2. ACT trained on new demonstrations, exported and copied to `~/minerva/policies/<name>`, run on the virtual receiver with the
   live cameras, then live.
3. Corrections recorded live; simulated cameras for policy mode in `helm --sim`.
