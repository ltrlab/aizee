# Replay on the arms

A recorded demonstration can drive the arms. Live replay shows whether a demonstration is reproducible on the robot
(repeatability, calibration drift) before it trains anything, and it is the first source of targets other than the leaders;
[policy mode](POLICY_MODE.md) is the same thing with a model in place of the episode.

**Status.** Built and rehearsed in `helm --sim` on the real Rust receiver (guarded and direct profiles), and tested offline. No
document records a replay run on the physical arms; the first live run is meant to be attended, at 0.5x, with a kept demonstration
recorded safely. Safety: [Safety](SAFETY.md).

## Using it

In Helm's **Replay** tab, choose a complete episode, pick 0.25x, 0.5x (default) or 1x in **ON THE ARMS** and press **Replay on
Minerva**. The receiver is the one chosen in Teleop (Virtual, or Live: the button then reads **Replay LIVE on Minerva** and the
box names the receiver), and the press starts that session. Then the session's own controls drive it:
**Idle (I)**, **Enable (E)**, **Engage (T)** (aligns the arms to the episode's first frame, or to where a pause left off, at
the alignment speed, then plays), **Hold (T)**, **Shutdown (X)**, **Disable (H)** and the pedals. While it runs, the live camera
feeds show beside the recorded frames, and the recorded frames follow the arms' playback position. Nothing is recorded: task
recording is refused because a replayed demonstration is not a human demonstration. Speeds above 1x are refused.

Helm refuses to start unless the episode is complete and its joints cover the receiver's.

## What it does

- The source (`EpisodeSource` in `src/robot_platform/teleop/episode_source.py`) uses the episode's recorded commands (`qcmd`,
  receiver motor radians) as the loop's target in place of the mapped leaders. It reads the joints by name in the receiver's order
  and refuses an episode missing any.
- The playback clock runs only while the lifecycle follows (TELEOP). The position between recorded frames is interpolated
  linearly, never extrapolated. A pause (HOLD) stops the clock.
- At the end, the last frame holds and the loop disengages to HOLD at that pose; Engage again rewinds to the first frame, aligning
  back to it first.
- Nothing new reaches the command path. Every target passes through the lifecycle (alignment speed, joint speed caps), the
  receiver's limits and the profile's collision mode (off in production), as a leader target does. H, X, the pedals and the foot
  switch are unchanged. The source never invents a pose.
- The episode is read from `~/minerva/evidence` by Replay's own name check. A session that replays binds no leader link.

## Rehearsal result (simulation)

On a recorded 64.5 s right-arm demonstration in `helm --sim` with the real receiver: at 1x on the guarded profile, Engage aligned
to frame 0 and the replay ran to its end; the receiver's commands matched the recorded `qcmd` within 2.2 mrad on every arm joint,
and the simulated arm tracked them within 0.049 rad (the simulated gripper lagged up to 2 rad, a known simulator artifact). At
0.5x the playback rate measured 0.503. Rehearse in the simulator before a first live run.

One thing to watch: the rehearsal started on unconfirmed joint directions without a word, which draws mirrored joints in the
simulator; the robot's own confirmed directions file does not have this problem, and Check fails unconfirmed directions.
