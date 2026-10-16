# Minerva policy evaluation

Closed-loop task success, not validation loss, is the progress metric. This harness runs N trials of a policy against the contact task scene ([MinervaTaskScene](MINERVA_TASK_SCENE.md): pick up the cube with one gripper, pass it to the other, release it into the bucket) and writes one report. Hand-logged real-robot trials go through the same report code, so a simulated ablation and a robot milestone gate compare field by field.

Code: `src/robot_platform/evaluation/`. Nothing in `task_scene.py` or `task_capture.py` was changed.

## Running it

Prerequisites: this checkout's `.venv-dev` (`python scripts/dev.py setup`, MuJoCo 3.15 or newer) and, for the default plant, the native receiver harness (`cargo build -p robot_control --bin control_harness`, which `scripts/dev.py check` also builds). Nothing opens USB, CAN or SSH, and no motor is commanded.

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy hold_still
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy fixture_replay --plant ideal
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy my_package.policies:load_checkpoint --out build/eval/dit-step40k
```

`--policy` takes a reference policy name or `module:factory` (called with no arguments). Each run writes `trials.jsonl` (one record per trial, with the scene's own events), `report.json` and `report.md` to `--out` (default `build/eval/<policy>-<plant>-<UTC time>`, ignored by git), plus the simulated receiver's evidence under `receiver/`. Other options: `--trials 0,3,5`, `--timeout` (s, default 30), `--grid 4x4`, `--seed`, `--jitter`, `--yaw`, `--no-nominal`, `--directions <Helm session.json>`, `--plant-seed`, `--scene-pose measured`. Runs are deterministic for a given policy, grid seed and plant seed.

## The policy interface

A policy is a callable `policy(observation) -> chunk`, where `chunk` is an `(H, 14)` array of joint commands in receiver motor coordinates and receiver joint order (`left_arm_j1..6, left_gripper, right_arm_j1..6, right_gripper`): the same space the task capture records as `qpos`, `qcmd` and `actions`. Row `k` is meant for `k * action_dt` seconds after the observation; the runner interpolates linearly between rows and holds the last. Optional attributes: `name`, `action_dt` (default: the 10 ms control period), `execute_steps` (rows executed before the policy is asked again; default the whole chunk), `camera_roles` (rendered into the observation), `start_pose(context)`, `reset(context)` and `describe()`.

The observation has `t` (s since the scene reset), `q`, `dq`, `effort` and `qcmd` from the receiver's latest feedback row (the policy sees 16-bit, noisy feedback, as on the robot), `joint_names`, and `images` when requested. The context given to `reset` and `start_pose` also carries the cube's start pose. That is privileged simulation state: scripted baselines may use it, learned policies must not.

## Plants

| `--plant` | What a command drives | Use |
|---|---|---|
| `armplant` (default) | The `helm --sim` stack: the receiver Helm derives from the saved calibration, limits, servo sweep and the direct (collision-disabled) teleop profile, run by the actual native control harness; Helm's lifecycle and direct session; and `minerva_sim.open_simulation`'s ArmPlant joints with the Tune sweeps' and actuator file's fitted values. The policy's command replaces the leader target; everything after it is code Helm runs. | Policy comparisons |
| `ideal` | Each command becomes the joint pose exactly. No receiver, lag, sag or friction. These are the conditions the frozen contact fixture was validated under. | Checking the harness itself, never evidence about the robot |

The `armplant` clock is stepped: runs are deterministic and faster than real time, but they measure no wall-clock timing and charge the policy no inference latency. The object scene follows the simulated joints' true positions after each 2.5 ms receiver tick, fed once per 10 ms control cycle, and linearly interpolated inside the cycle by the scene. `--scene-pose measured` drives it from the receiver's feedback rows instead, as Helm's `--sim` window does. Only collision-disabled direct profiles are wired; the enforced two-arm motion envelope is not.

## One trial

1. **Stage.** The arms engage onto the policy's start pose (default: the receiver's initial pose, arms hanging) the way an operator's Engage aligns, until Helm's lifecycle reports them settled. Staging motion is not simulated in the object scene.
2. **Reset.** The scene resets at that pose (`scene.reset()`, which keeps the robot pose), so the arms must be clear of the table there. A start pose that penetrates the table or bucket ends the trial as a forbidden penetration on the first scene step: the policy chose it.
3. **Place the cube** at the trial's grid pose, 2 mm above the table as the scene's own start. If the cube would sit inside the robot at that start pose, the trial is an `invalid_start` and is not counted. This happens when a policy starts with an open gripper near a grid pose.
4. **Run** the policy in closed loop. Commands outside the receiver's joint limits are clipped, as the leader mapping clips its targets, and counted per trial. The trial stops at the scene's first terminal verdict (success, dropped cube, forbidden penetration, or the cube settled in the bucket out of sequence) or at the timeout. A receiver fault, policy exception, malformed chunk or MuJoCo numerical quarantine aborts that trial. The plant is rebuilt and the run goes on.

Outcomes are read from the scene's events (`pickup`, `handoff`, `bucket_settled`, `dropped_outside_bucket`, `forbidden_robot_environment_penetration`), never re-derived; `trials.jsonl` keeps those events verbatim.

## Cube start grid

Trial 0 is the scene's own cube start (`CUBE_START`, x 0.18 m, y −0.34 m). Then come the cells of an NX×NY grid (default 3×3) over x 0.10–0.26 m and y −0.42 to −0.26 m, in row-major order. Each cell's pose is its centre plus a seeded offset of at most `--jitter` (default 0.5) of half a cell, and the centre cell is centred on `CUBE_START`. The same seed always gives the same poses. Yaw is 0 by default: the faces stay axis-aligned.

The area is kinematically reachable. An IK sweep over the table (`evaluation.reach.GraspIK`) used the fixture's own grasp geometry: the left gripper's terminal pads centred on the cube, fingers vertical, jaws along a face, within the receiver's joint limits. The left arm reaches roughly x 0–0.42 m and y −0.46 to −0.18 m without penetrating the table or bucket. The contract test re-solves every default pose and audits the posed robot for penetration. This is reach for a top grasp only, not a check of approach paths or dynamics. The coordinates are synthetic task settings, not a measured workcell.

`python -m robot_platform.evaluation grid` prints the poses. `grid --template` prints a hand-logging sheet with the poses filled in.

## Outcomes

| Outcome | Meaning | Counted |
|---|---|---|
| `success` | Pickup, handoff and bucket placement all reached, in order; eligible | yes |
| `dropped_cube` | The cube returned to the table or floor after pickup | yes |
| `forbidden_penetration` | The robot went into the table or bucket by more than 2 mm. The trial is ineligible (`eligible: false`), and this outranks every other outcome, including a success it revokes | yes |
| `incomplete_sequence` | The cube settled in the bucket without the stages before it | yes |
| `timeout` | No terminal event before the timeout | yes |
| `source_gap_or_abort` | The run stopped: receiver fault, lifecycle left TELEOP, command source gap, policy error or bad chunk, numerical quarantine; on the robot, an operator stop | yes |
| `invalid_start` | Invalid initial condition: cube inside the robot at the start pose, staging failed, or (robot) cube disturbed before the go signal | no, listed separately |

## Metrics

- **Full-task success rate** = successes / counted trials, with a **Wilson 95% interval**. Unlike the normal interval, it stays meaningful at 0/n and n/n, where small trial counts sit. 0/10 is still compatible with up to 28% success, and 10/10 with as little as 72%.
- **Stage-completion score**: the mean over counted trials of (stages reached) / 3. A stage counts only if every earlier stage was reached, so a cube pushed into the bucket is not a placement. Per-stage reach rates and their intervals are reported too. This score separates "never grasps" from "drops in the handoff" when full success is 0.
- **Time to complete**: trial start (the scene reset; on the robot, the go signal) to the `bucket_settled` event, for successful trials only. Median and p90 use linear interpolation (Hyndman–Fan type 7, numpy's default). The bucket event fires after the cube has settled for 250 ms.
- **Outcome histogram**: one row per outcome above, zeros included.
- **Trial counts**: counted, excluded as invalid starts, and eligible.
- **Protocol**: grid specification and seed, every cube pose, timeout and plant settings. **Provenance**: plant, receiver identity, profile, calibration file hashes, direction source, actuator fit record, joints still on nominal values, scene hashes, MuJoCo version, git commit and policy description. Every report states `sim2real_validated: false` and `training_promoted: false`.
- Per-trial diagnostics in `trials.jsonl`: staging time and residual, max |pose − command| per joint, clipped cycles, policy calls and wall time.

## Comparing two policies

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation compare 7/20 15/20
.venv-dev/Scripts/python.exe -m robot_platform.evaluation power 0.5 0.8
```

`compare` runs a pooled two-sided two-proportion z-test and Fisher's exact test. The verdict uses Fisher when any expected cell count is below 5. If the rates differ but the test can't tell them apart, it also says how many trials per arm would. `power` gives the trials per arm needed to detect p1 against p2 at α = 0.05 and 80% power: the standard pooled formula without continuity correction, checked against textbook values (388 for 50% vs 60%; 82 for 20% vs 40%) and by simulation. Some values:

| p1 → p2 | 0.1 → 0.3 | 0.2 → 0.5 | 0.3 → 0.6 | 0.5 → 0.8 | 0.6 → 0.9 | 0.7 → 0.9 | 0.8 → 0.95 | 0.5 → 0.6 |
|---|---|---|---|---|---|---|---|---|
| trials per arm | 62 | 39 | 42 | 39 | 32 | 62 | 76 | 388 |

A 10-trial robot gate distinguishes only large effects (3/10 vs 7/10 gives p = 0.07). Run the many-trial ablations in simulation, and size robot gates with `power`.

## Real-robot trials

1. `python -m robot_platform.evaluation grid --template > build/eval/robot-<date>.csv` gives one row per trial with the cube position (task-scene table coordinates). Use the same `--grid/--seed` as the simulated run you will compare with.
2. Per trial: place the cube, give the go signal, and log in seconds from the go signal `pickup_s` (both jaws holding it off the table), `handoff_s` (the receiving gripper alone holds it) and `bucket_s` (released and settled in the bucket). Then log `outcome` from the table above and, optionally, `eligible`, `duration_s`, `detail` and `notes`. Leave a stage blank if it was not reached. Unlogged eligibility follows the outcome: only `forbidden_penetration` is ineligible.
3. `python -m robot_platform.evaluation report build/eval/robot-<date>.csv --policy <name> --grid 3x3 --seed 0 --meta meta.json` writes `report.json`/`.md` beside the sheet. `meta.json` holds provenance: robot release, checkpoint, operator, camera setup.

The simulated stage events have precise definitions (force-bearing opposing contacts for 50 ms above the table, and so on; see [the scene](MINERVA_TASK_SCENE.md#outcomes)). A human logger applies their visible equivalents. Expect this definition gap to be part of any sim-to-real difference.

## Reference policies and measured results

All three are harness controls, not capable policies.

- `fixture_replay`: replays `tests/fixtures/minerva_task_contact_path.json` open loop from its own prepositioned start, converted to motor coordinates through the derived joint map.
- `hold_still`: holds the hanging pose.
- `penetrating`: starts at the fixture's start and lowers `left_arm_j2` 0.3 rad over 1 s, into the table.

Measured 2026-10-16 with harness commit `4556db0` (base `755085f`), default grid (seed 0, 10 poses), default timeout and plant seed:

| Policy | Plant | Counted (excluded) | Success (Wilson 95%) | Stage score | Outcomes | Eligible |
|---|---|---|---|---|---|---|
| fixture_replay | ideal | 8 (2) | 1/8 (2–47%) | 0.167 | success 1 (nominal pose, bucket at 10.83 s); dropped_cube 1 (centre cell, picked up, dropped at 7.97 s); timeout 6 | 8 |
| fixture_replay | armplant | 8 (2) | 0/8 (0–32%) | 0 | forbidden_penetration 8 (right jaw into the bucket's +x wall at 9.32 s; no pickup before it) | 0 |
| hold_still | armplant | 10 (0) | 0/10 (0–28%) | 0 | timeout 10 | 10 |
| penetrating | armplant | 8 (2) | 0/8 (0–32%) | 0 | forbidden_penetration 8 (left jaw into the table at 0.16 s) | 0 |

The two excluded trials (4 and 6) are grid poses where the cube would spawn inside the open left jaws of the fixture's prepositioned start. Wall time on the development workstation was about 10 s per trial (an `armplant` hold-still trial with a 30 s timeout took about 25 s).

What this shows:

- **The harness detects success.** With exact tracking, the replay succeeds from its own start.
- **Grid poses other than the fixture's start fail, as an open-loop replay should.** Most time out with the jaws closing on nothing. At poses where the open jaws already sit where the cube would spawn, the start is invalid.
- **The frozen fixture does not survive the calibrated plant.** Through the receiver and ArmPlant, no trial reaches pickup. At 9.3 s the right jaw pad, carrying over the bucket, goes into the bucket's +x wall: forbidden penetration, ineligible.

  The margins explain it. Under exact tracking, the fixture squeezes the 36 mm cube by only about 0.65 mm per jaw. Its right jaw already dips 1.35 mm into that same wall at 9.25 s, which is 0.65 mm short of the scene's 2 mm ineligibility threshold. The nominal trial's diagnostics show what the plant does: arm joints trail their commands by up to about 0.03 rad, millimetres to centimetres at the gripper. Both grippers lag about 0.6 rad while closing, then rest about 0.05 rad short of their command, which is about 0.5 mm of jaw travel.

  This is the first measured failure category: open-loop position replay with sub-millimetre margins, against tracking error, sag and gripper friction. The gripper's friction is a nominal value (`NOMINAL['gripper']`), not a sweep measurement, so the size of the gripper rest error is itself unmeasured.

## Tests

```powershell
.venv-dev/Scripts/python.exe -m pytest tests/unit/test_eval_stats.py tests/unit/test_eval_report.py tests/contract/test_eval_reference_policies.py -q
```

- The statistics tests check the Wilson interval against Newcombe (1998) and scipy, both tests against scipy's chi-square and Fisher implementations, sample sizes against textbook values and a power simulation, and percentiles against numpy.
- The report tests cover outcomes from scene events, record validation, a hand-logged CSV producing exactly the simulated report's metrics, grid determinism and the CLI.
- The contract tests run the actual scene, receiver and simulated joints:
  - fixture replay succeeds with exact tracking;
  - the ArmPlant replay record equals the scene's verdict;
  - hold-still gives 0 successes and all timeouts;
  - the penetrating policy is ineligible;
  - policy errors abort one trial only;
  - every default grid pose is reachable without penetration.

These files are not yet in `scripts/dev.py`'s required list; that is the integrator's call.

## Limitations

- Simulation inherits every [scene limitation](MINERVA_TASK_SCENE.md): kinematic mocap robot, no contact reaction on the arms, unmeasured contact and cube properties, approximate finger pads, synthetic cameras. ArmPlant joints are independent and carry only measured gravity. Simulated results rank policies; they do not predict robot success rates.
- Stepped clock: no USB or leader path, no wall-clock deadlines, no inference latency. A policy that is too slow on the Jetson will look fine here.
- Directions default to the documented test directions (`REVERSED_MOTORS`), as in the receiver capture contract. Pass `--directions` with the operator's Helm `session.json` to use the confirmed mapping.
- Only collision-disabled direct profiles are supported, and staging paths are not audited for penetration.
- Stage times on the robot are hand-logged, at human resolution.
- Nothing here marks data or models `sim2real_validated` or `training_promoted`.
