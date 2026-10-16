# Feature task: compare the simulator with the real arm, on real responses only

- Branch and worktree: `claude/sim2real`, `C:/Users/ltr/Workspace/aizee-actuator-response`
- Base commit (full SHA): `a52dfd682dce1bf7d3ccba771ead3fb25698ade2`, which is `claude/response-fit` merged with
  `claude/helm-sim`. Offline only: nothing runs on the robot.
- Owner: Claude Code, at the operator's request (2026-10-15). The request was to compare only real responses,
  never a simulated ground truth.
- Problem: nothing measured how closely the plant behind `robot-platform helm --sim` reproduces the real arm. The
  response fit's closed-loop stage had only been checked against a rehearsed sweep, and that check did not hold up
  (`RESPONSE_FIT.md`).
- Hypothesis: replaying each real tick's drive command through the simulator's own plant, starting each joint from
  its measured state, measures the gap between sim and arm. Fitting that plant's parameters on half of the real
  data then narrows the gap on the other half.
- Owned files: `src/robot_platform/characterization/sim2real.py`, `tests/unit/test_sim2real.py`, one gate entry,
  this brief.
- Shared interfaces held fixed: `simulation.arm_plant.ArmPlant`, `simulation.minerva_sim.joint_specs`,
  `control.gravity`, and the recorder's command columns (`5852fe2`).
- Acceptance: the gate. The replay equals `ArmPlant` stepped tick by tick to within 1e-12 rad
  (`test_the_replay_is_the_simulators_plant`).

## Method

`python -m robot_platform.characterization.sim2real <recordings...> --runs <Helm runs folder> [--joint J] --output r.json`

- **The simulator** is built from the same files as Helm's: the calibration profile, the servo file beside it, and
  the runs folder's confirmed directions. That gives each motor's ArmPlant spec (Tune-sweep values where a sweep
  measured them, otherwise the rehearsals' nominal values) and its pitch-chain gravity terms.
- **The replay.** Each 1 s window is replayed one joint at a time:
  - the joint starts from its measured state;
  - each tick it takes the command the real drive took (position, velocity, feed-forward effort, kp, kd);
  - the arm's other joints sit at their measured positions, so the gravity coupling is the real one.
- **Windows left out:** those overlapping a disabled drive or a recorder gap, and those within 1 s of a FAULT.
  - A FAULT is placed on the receiver clock through `direct-latency.jsonl`.
  - If a FAULT cannot be placed, the whole recording is dropped.
- **The baseline** is a joint that follows its command exactly, which is what Helm's virtual receiver does.
- **Calibration** fits six parameters on even 10 s blocks: inertia, Coulomb and breakaway friction, viscous
  damping, a gravity scale and a 0–8 tick command delay. Odd blocks are held out for scoring. Grippers are not
  calibrated, because the replay has no objects to grasp.
- **Score:** each window's maximum position error, as median and 95th percentile over the windows. Windows where
  the command moves faster than 0.05 rad/s are scored apart from holding windows.

## Data (2026-10-15, release `20261015T034019Z-5852fe2`)

The sessions are `live-20261015T223503Z`, `223714Z`, `223850Z` and `230538Z`, mirrored and sha256-verified in
`C:/Users/ltr/Workspace/minerva-evidence/jetson/evidence`.

- `223503Z` is dropped. Its fault (right J4 moving at an audit stop) could not be placed on the receiver clock.
- `223714Z` had its drives commanded for only 1.9% of its 26 s.
- So the results rest on two sessions, both ending in a drive overcurrent:
  - `223850Z`: 142 s; right J5 tripped.
  - `230538Z`: 333 s, with full travel on every joint; left J2 tripped.
- One gain set throughout, position-only (no velocity feed-forward):

  | Joints | kp | kd |
  |---|---|---|
  | J1 | 550 | 15 |
  | J2 | 600 | 20 |
  | J3 | 600 | 11 |
  | J4–J6 | 100 | 2–3 |
  | Grippers | 25 | 2 |

- Each joint has 226 held-out windows, 38–167 of them moving.

Source identity:
- Servo file `config/devices/minerva-production-01-servo.json`, sha256 `9b0003c5…dbed881`.
- Report sha256 `44364a2e…19746229`.
- The report, its log and the J2 inertia check are in `C:/Users/ltr/Workspace/minerva-evidence/analysis/sim2real-20261015/`.

## Results: held-out moving windows, max error per 1 s window, median / p95 (mrad)

| Joint | Follows command | Current sim | Calibrated sim |
|---|---|---|---|
| Left J1 | 4.7 / 23.3 | 1.9 / 3.8 | 1.7 / 3.4 |
| Left J2 | 8.0 / 39.9 | 3.2 / 12.6 | 2.4 / 5.6 |
| Left J3 | 6.4 / 26.3 | 2.4 / 10.6 | 1.7 / 3.8 |
| Left J4 | 5.1 / 14.9 | 3.5 / 9.8 | 3.2 / 9.7 |
| Left J5 | 16.0 / 82.4 | 2.7 / 8.8 | 2.6 / 5.1 |
| Left J6 | 15.6 / 46.3 | 3.0 / 9.2 | 2.3 / 7.5 |
| Left gripper | 12.7 / 342 | 3.6 / 28.4 | not calibrated |
| Right J1 | 10.8 / 23.7 | 1.9 / 6.0 | 1.6 / 5.6 |
| Right J2 | 7.7 / 41.3 | 3.6 / 11.2 | 2.4 / 7.2 |
| Right J3 | 5.9 / 21.6 | 1.9 / 5.1 | 1.6 / 4.6 |
| Right J4 | 11.4 / 32.8 | 6.3 / 14.7 | 5.1 / 12.2 |
| Right J5 | 24.2 / 43.6 | 2.9 / 5.9 | 2.5 / 4.7 |
| Right J6 | 13.0 / 36.2 | 2.4 / 6.7 | 2.0 / 5.0 |
| Right gripper | 18.6 / 832 | 5.6 / 54.8 | not calibrated |

Holding windows are 0.2–0.6 mrad (median) for both the current and the calibrated sim. A joint that follows its
command exactly is off by 0.4–3.2 mrad there.

Calibrated parameters, servo file → fit (fit on 150 windows each; all fits converged):

| Joint | Inertia (kg·m²) | Coulomb / breakaway (Nm) | Damping (Nm·s/rad) | Gravity scale | Delay |
|---|---|---|---|---|---|
| Left J1 | 0.300 → 0.206 | 0.33 / 0.45 → 0.13 / 0.14 | 0 → 0.81 | 1.00 | 0.6 ms |
| Left J2 | 1.346 → 0.280 | 0.90 / 1.21 → 0.49 / 0.58 | 0 → 3.29 | 1.07 | 0 |
| Left J3 | 0.322 → 0.173 | 0.46 / 0.62 → 0.37 / 0.91 | 6.48 → 1.66 | 0.99 | 0 (bound) |
| Left J4 | 0.058 → 0.041 | 0.17 / 0.25 → 0.14 / 0.25 | 0.65 → 0.48 | 0.96 | 0 (bound) |
| Left J5 | 0.006 → 0.006 | 0.13 / 0.20 → 0.14 / 0.14 | 0 → 0.11 | 1.00 | 2.3 ms |
| Left J6 | 0.004 → 0.004 | 0.10 / 0.15 → 0.10 / 0.11 | 0 → 0.21 | 1.00 | 1.0 ms |
| Right J1 | 0.300 → 0.267 | 0.33 / 0.45 → 0.38 / 0.60 | 3.05 → 2.17 | 1.00 | 0 (bound) |
| Right J2 | 1.317 → 0.470 | 0.83 / 1.11 → 0.56 / 0.59 | 0 → 3.06 | 1.00 | 0 |
| Right J3 | 0.080 → 0.096 | 0.46 / 0.62 → 0.43 / 0.76 | 0 → 0.03 | 1.06 | 1.9 ms |
| Right J4 | 0.010 → 0.008 | 0.21 / 0.28 → 0.20 / 0.21 | 0 → 0.34 | 0.94 | 1.0 ms |
| Right J5 | 0.004 → 0.004 | 0.16 / 0.21 → 0.16 / 0.31 | 0 → 0.27 | 1.00 | 0.4 ms |
| Right J6 | 0.003 → 0.003 | 0.11 / 0.15 → 0.11 / 0.11 | 0 → 0.19 | 1.00 | 1.0 ms |

Left J1's values and right J1, J3 and J4–J6's inertias were nominal in the simulator: no sweep measured them.

### How well the real responses pin J2's inertia

The check holds J2's inertia fixed at each value and refits the other five parameters on the same fit windows
(`inertia_profile.py`). Held-out moving max error, median / p95 (mrad):

| Inertia | 0.15 | 0.30 | 0.50 | 0.80 | Servo file |
|---|---|---|---|---|---|
| Left J2 | 2.5 / 5.8 | 2.4 / 5.6 | 2.7 / 6.3 | 2.5 / 7.9 | 3.7 / 11.3 (1.346) |
| Right J2 | 2.5 / 8.0 | 2.3 / 7.5 | 3.0 / 7.1 | 2.8 / 7.6 | 3.2 / 10.2 (1.317) |

- **The servo file's J2 inertia is rejected.** Even with everything else refitted around it, the held-out p95 is
  1.4–2× worse. Most of the calibrated sim's J2 gain is the inertia.
- **The true value is pinned only loosely.** Left J2 lies in 0.15–0.5 and right J2 in 0.15–0.8, because the delay
  and damping trade against the inertia.
- **An independent method agrees.** The effort fits on the 09-27..10-10 sweeps (`RESPONSE_FIT.md`) found
  J2 0.36–0.69 and J3 0.09–0.16.

## Remaining uncertainty

- **J4 is not captured** on either arm. Calibration barely helps (p95 9.7 and 12.2 mrad), and its fit residual is
  the largest (2.2–2.4 mrad rms). Something there is structural, not a parameter value.
- **No torque or current limit in the plant.** Both overcurrent trips that ended these sessions are outside what
  the simulator can show. Windows near them are excluded.
- **One gain set, no velocity feed-forward, one day.** The parameters reproduce the arm at these gains. A sweep
  that varies the gains (recorded on `5852fe2` or later) would pin inertia, damping and delay separately.
- **One joint at a time.** No inertial coupling between joints; other joints sit at their measured positions.
- **Grippers** have no contact or objects. Their p95 (28 and 55 mrad) is the largest remaining sim error.
- **The servo file's inertia also drives the real controller.** `control.guarded_config` passes it to the
  acceleration feed-forward (`control.approach.ramp_effort`, `INERTIA_FF_FRACTION` 0.7 of it).
  - That path is used where the receiver ramps commands and the joint is neither position-only nor
    velocity-fed-forward. The Tune sweep feeds its own inertia fit forward the same way.
  - If J2's real inertia is 0.3–0.5, that path feeds about 2–3× the needed torque forward.
  - Today's sessions did not exercise it. It is not changed here, because it belongs to the motion-contract owner.
- **Into the simulator.** The calibrated values belong in a sim-only per-motor file read by
  `simulation/minerva_sim.py`, bound to the recordings and the motors' UIDs, not in the servo file.
