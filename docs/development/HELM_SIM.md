# Feature task: the leader arms drive a simulated Minerva in Helm

- Branch and worktree: `claude/helm-sim`, `C:/Users/ltr/Workspace/aizee-helm-sim`
- Base commit (full SHA): `6fb0510c1182f8d6bddf0090bf4c6b3cb7e93211` (`claude/vff-toggle`, the line Helm runs from,
  59 commits ahead of `codex/overhaul-integration`), with `claude/minerva-physical-model` merged in for the MJCF.
  Based on the deployed line because Helm's teleop code has moved well past integration.
- Owner: Claude Code, at the operator's request (2026-10-14): GELLO arms in Helm should drive the sim for demo
  collection, training and testing; the operator watches a MuJoCo window; sim and real demos are co-trained.
- Problem / saved reproducer: Helm's virtual receiver moves joints exactly to their commands, so the leaders cannot
  rehearse or collect anything in simulation. The rehearsal tests already ran TeleopCore against the real receiver
  and a sweep-measured plant, but only as a test fixture.
- Hypothesis: TeleopCore's live path (an `exchange`, `last` and per-tick rows) works unchanged over a
  `PlantReceiver`, so a simulated Minerva needs only a plant builder, a window and a launch option.
- Owned files: `src/robot_platform/simulation/minerva_sim.py`, `tests/contract/test_helm_sim.py`, this brief; small
  hooks in `helm/teleop.py` (TeleopCore `simulation`, TeleopSession `simulation`) and `helm/cli.py` (`--sim`); the
  `PlantReceiver` clock fix in `simulation/arm_plant.py` with its reproducer in `tests/contract/test_mujoco_receiver.py`.
- Shared interfaces held fixed: the Rust receiver, receiver derivation, lifecycle, recording format, leader mapping,
  live and virtual paths, Helm's UI.
- Dependencies / integration order: after the `claude/*` stack it is based on, and after `claude/minerva-physical-model`.
- Required behavior and numerical/time bounds: the leaders engage, follow and shut down both simulated arms through
  the real receiver with no fault; every 2.5 ms tick recorded; the 100 Hz direct loop keeps its period on this PC
  with the window open.
- Acceptance command and expected result: `python scripts/dev.py check`, zero failures/skips/xfails.
- Source/model identity and seed: servo calibration `config/devices/minerva-production-01-servo.json`; MJCF
  `urdf/minerva/mujoco/minerva.xml`; plant seed 1.

## Change

`robot-platform helm --sim` (needs `--teleop`; the loop runs on this PC) makes Teleop's virtual receiver drive a
simulated Minerva:

- The receiver is the one Helm derives for the real arms. It ticks every 2.5 ms against an `ArmPlant`.
- Each joint carries its Tune sweep's inertia, friction and gravity slices. Where a sweep measured none, the joint
  takes the rehearsals' rough `NOMINAL` value, and the session's `events.jsonl` lists which ones:
  - left J1 and both grippers are fully nominal;
  - right J1 and J3–J6 have nominal inertia.
- Gearbox damping makes up a 0.6 damping ratio where the sweep's gains fall short, as the rehearsals assume.
- A MuJoCo window shows the arms through Helm's confirmed directions and the grippers' preview transmissions. The
  window draws at 30 Hz in its own thread and never steps physics.
- Sessions record every tick (`samples.f32`) and event under `<runs>/<asset>-sim/sim-<UTC>/`, as a live session
  records them.

`PlantReceiver` read the caller's clock again after ticking. On a real clock a slow tick put a request past the next
scheduled tick, which then stepped the receiver back (`clock_regression` at the first request, while the window
started). It now reads the clock once per exchange and takes a request at the next tick, as the live service does.
Tick times also round to whole milliseconds: tick 1606 is 4015 ms, not int(4014.9999999999995).

## Results

- **Reproducer.** It jumps the clock 6 ms every 3, 5 or 7 readings. Before the fix both service modes faulted
  `clock_regression`; after it, all 6 variants pass.
- **Contract tests.** Scripted leaders engage, follow and shut down both simulated arms through the real receiver,
  under both the guarded (`minerva-teleop.json`) and the direct (`minerva-teleop-direct.json`) profile.
  - The shoulders and elbows moved; the wrists, left alone, held.
  - Every tick was recorded.
  - The rehearsals and MuJoCo receiver tests still pass (41 tests).
- **Real clock on this PC**, scripted leaders, window open, Helm's loop scheduling, 10 s of TELEOP:

| Profile | Teleop cycle host time (ms) p50 / p99 / max | Over the 10 ms period |
|---|---|---|
| Direct | 4.27 / 6.28 / 11.74 | 1 |
| Direct | 4.16 / 6.20 / 7.56 | 0 |
| Direct | 4.06 / 6.49 / 7.70 | 0 |
| Direct manual tuning | 3.79 / 6.49 / 7.24 | 0 |

- Without Helm's loop scheduling (`prefer_responsive`), two of three direct runs expired a command. Windows'
  throttling, seen before on the live loop, is the likely cause. Helm's loop process always sets the scheduling.
- Gate: 741 Python tests, zero failures/skips/xfails, and 125 Rust tests, on `c9ce3b2`
  (`build/dev-check/20261015T025114Z-1f8dfe49/report.json`). ARM64 binaries were cross-built locally for the
  release test; nothing was deployed.

## Running it

Close any running Helm first: two would share the runs folder's lock and the leaders' USB ports. Use the operator
runs folder, which holds the verified directions; a fresh runs folder falls back to the seed and mirrors right
J3/J4. From this worktree:

```powershell
.venv-dev/Scripts/python.exe -m robot_platform helm --profile config/devices/minerva-joint-calibration.json --runs C:/Users/ltr/.codex/worktrees/dev-baseline/aizee/build/manual-tuning/operator-runs --seed docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json --teleop config/devices/minerva-teleop-direct-tuning.json --sim --port 8760
```

In Teleop, keep the virtual receiver selected and start. The MuJoCo window opens beside Helm. I, E, T, H and X work
as with the arms. Leader ranges and zero come from this checkout's `config/devices/gello-dual-leader-ranges.json`,
which matches the operator's current file.

## Remaining uncertainty / next steps

- **Not a physics scene.** There are no objects or contacts, and the joints do not load each other beyond their
  measured gravity. Pick-and-place needs `MujocoPlant` on the MJCF with masses, jaw contact geometry, a table and
  objects, and the three cameras. It also needs episode recording in the trainers' HDF5 format. Masses come from
  the operator's Onshape actuator masses plus weighed printed parts.
- **Nominal values.** Nominal joints and the gearbox damping are rehearsal assumptions, not measurements. The
  actuator-response track replaces them with fitted values.
- **Not run here.** Real leaders were not connected to the simulated loop: the operator's Helm held them.
