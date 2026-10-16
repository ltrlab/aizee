# Feature task: fit a joint's response from recordings and replay it

- Branch and worktree: `claude/response-fit`, `C:/Users/ltr/Workspace/aizee-actuator-response`
- Base commit (full SHA): `34d42e96` on `claude/actuator-response` (the deployed recorder change). Offline only:
  nothing runs on the robot.
- Owner: Claude Code, at the operator's request (2026-10-15): calibrate actuator responses in the real world and
  reproduce them in the simulator.
- Problem: the simulator's joints take inertia and friction from the Tune sweep's own fit, or rough nominal values.
  Nothing checked that a model reproduces a real joint's motion, and nothing could replay a recording.
- Hypothesis: a per-joint model (mechanics plus the drive's command law, delay and torque scale) fitted on part
  of a recording reproduces the held-out rest of it, closely enough for the simulator to use.
- Owned files: `src/robot_platform/characterization/response_fit.py`, `tests/unit/test_response_fit.py`, one gate
  entry, this brief.
- Acceptance: the gate, and on synthetic recordings the declared tolerances below.

## Method

`python -m robot_platform.characterization.response_fit <recording> [--joint J] [--servo servo.json] --output r.json`

Per moving joint, with alternate 10 s blocks fitted and the others held out:

1. **Mechanics from the reported effort.** `I a = tau - (A sin q + B cos q + C) - b v - f sgn v`, by least squares.
   - Every regressor is the same zero-phase low-pass of its raw signal (8 Hz, 4th-order Butterworth, run both ways).
   - v and a are central differences of the filtered position.
   - The effort is aligned to the position by searching a ±4-tick shift.
2. **Drive from each tick's command** (recordings since `5852fe2`): the delay and scale that turn the recorded
   command, at the drive's measured state, into its reported effort.
3. **Closed-loop fit from positions only** (recordings with commands): inertia, viscous and Coulomb friction,
   command delay and a gravity scale. They are chosen so that the closed loop, simulated under the recorded
   commands from each window's measured start, reproduces the measured positions. This is the simulator's job,
   and it never reads the effort.
4. **Replay of held-out windows:**
   - open loop under the reported effort (0.25 s windows), against coasting at the starting speed;
   - closed loop under the recorded commands (1 s windows), against a joint that follows its command exactly
     (the virtual receiver).

## What went wrong on the way, and why the method is as it is

Each change followed a discriminating experiment, not a guess:

- **Savitzky–Golay derivatives biased inertia +6 to +23%, growing with the window.** Their implicit smoothing
  of acceleration differs from the plain smoothing of the effort. Matched filtering plus central differences
  brought inertia within 0.6%.
- **Friction absorbs any effort-to-position timing offset.** On an exact analytic joint, one tick moved viscous
  friction by +0.36 and Coulomb friction by +0.78. The alignment search recovers injected offsets exactly
  (+2.5 ms, −1.5 ms).
- **`tanh` of the filtered speed found 27% of an exact joint's Coulomb friction.** It put a sharp sign against
  filtered torque at each reversal. Filtering the sign of a lightly filtered speed, like every other column,
  recovers it: 0.462 for 0.46.
- **Viscous friction is identifiable only up to when the drive samples its torque.** The command is held for each
  tick while the joint moves, so a sample at a fixed point of the tick reports `kp × speed × (offset from
  mid-tick)` on top of the torque that moves the joint. Sampled at each tick's end, a synthetic joint read 0.15
  for 0.40 Nm s/rad; reported as a tick average, 0.393.
  - Where the gains vary (a sweep's stiffness ladder), a `kp × speed` regressor separates the two. It recovered
    0.40 and a sampling point of half a tick.
  - With constant or unknown gains, the report flags that viscous friction includes the term.
- **Delay convention.** The closed-loop simulator first applied commands one tick earlier than the drive fit
  reads them; the two now agree.

## Synthetic results

The synthetic recordings come from an independent simulation: 10 substeps a tick, a 5 ms command delay, stick-slip
friction, 16-bit positions, ±0.08 rad/s speed noise used by the drive's own damping, and quantized effort. The
tolerances were set before the first run.

| Check | Tolerance | Result |
|---|---|---|
| Effort fit: inertia, Coulomb friction, gravity | 5% / 10% / 2% | pass |
| Effort fit: viscous friction (constant gains) | true − kp·T/2 ± 0.1, flagged | pass |
| Tick-average effort: viscous friction | ± 0.1 | pass |
| Stiffness ladder: viscous friction, sampling point | ± 0.1, 0.5 ± 0.15 tick | pass |
| Drive fit: delay, scale (also a 0.9 scale) | ± 0.5 tick, 2% | pass |
| Closed-loop fit through a 20 ms lagged effort: inertia, viscous, friction, delay | 5%, ± 0.15, 10%, ± 0.5 tick | pass |
| Held-out closed-loop replay vs exact following | at least 3× closer | pass (3.1 vs 26.8 mrad) |
| Replay tells a 3× inertia apart | 1.5× worse | pass |

The gate passed: 727 Python tests, zero failures/skips/xfails, on `5ef3828`
(`build/dev-check/20261015T231345Z-c025563c/report.json`).

## Real recordings (58 Jetson sweeps, 2026-09-27 to 10-10, mechanics only)

These sweeps predate the command fields, so only the effort fit and the open-loop replay apply. Fits whose held-out
R² was over 0.9, without implausible or dead-sensor flags, are summarized here; per-sweep JSON lines are in the
session scratchpad (`sweep-fits.jsonl`).

| Joint | Gravity A (Nm), median [range] | Servo-sweep A | Coulomb f (Nm), median | Servo-sweep f |
|---|---|---|---|---|
| Right J2 (5 sweeps) | 18.55 [18.30–18.64] | 18.55 | 0.63 | 0.83 |
| Left J2 (5) | 17.81 [16.76–18.42] | 17.88 | 0.77 | 0.90 |
| Right J3 (2) | 7.21 | 7.24 | 0.46 | 0.46 |
| Left J3 (5) | 6.75 [6.60–7.19] | 7.24 | 0.40 | 0.46 |
| Right J4 (2) | 2.62 | 2.69 | 0.27 | 0.21 |
| Left J4 (1) | 2.73 | 2.73 | 0.20 | 0.17 |

- **Gravity is robust.** It holds within about 2% of the sweep's own fits and across sweeps, whatever effort timing
  is assumed.
- **Effort-based inertia is not trustworthy on the real drives.** On right J2 the fitted inertia ran from 0.65 to
  1.33 kg m² across plausible effort lags (pure delay up to 80 ms, or a first-order lag up to 80 ms), while the
  residual moved only 0.95 → 0.83 Nm. The alignment search mostly ends at its ±10 ms edge, and the open-loop replay
  under the reported effort tracks worse than coasting. Effort fits put J2 at 0.36–0.69 and J3 at 0.09–0.16, about
  half the servo file's 1.32–1.35 and 0.32. Both methods read the same uncertain signal. The closed-loop position
  fit decides on a recording with commands.
- **J1, J5 and J6** carry little gravity, so their effort is mostly friction and ripple (held-out R² 0.5–0.85).
  Their Coulomb friction is repeatable: right J1 0.29–0.37, J5 0.16–0.18, J6 0.13–0.21 Nm.
- **Left J1's** drive reports a constant effort and is flagged; it can only be fitted from positions.
- **Residual noise.** The ~0.9 Nm residual on J2 is about the torque ripple of the drive's damping acting on its
  noisy speed reading (kd 21 × ±0.1 rad/s, filtered).

## Remaining uncertainty and next steps

- **No closed-loop fit of a real joint yet.** Any Tune sweep, Minerva Check sweep or teleop session recorded on
  release `5852fe2` or later carries the commands it needs. A sweep is best: it varies the gains, which also
  separates the drive's effort sampling from viscous friction.
- **Model scope.** One joint at a time; others held. Multi-joint teleop needs the pitch-chain gravity
  (control.gravity) or the MuJoCo model with masses. Torque saturation, backlash and a velocity-dependent friction
  curve are not modelled.
- **Into the simulator.** Done for the sim2real fits: `config/devices/simulation/minerva-production-01-actuators.json`,
  bound to each motor's UID and firmware and read by `simulation/minerva_sim.py` (SIM2REAL.md, "Into the
  simulator").
- **Excitation protocol.** A short per-joint sequence in Helm Tune (steps and ramps at two or three stiffnesses)
  would give every joint a clean closed-loop fit.
