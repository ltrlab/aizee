# Simulate

You can run Helm against a simulated Minerva on a PC with no robot, no Jetson and no network. It is the safe way to
learn the interface, rehearse a session, and try changes to the teleop loop. This page covers Helm's simulation
(`helm --sim`), the model viewer, and the headless simulator, with links to the deeper simulation pages.

## Helm on a simulated Minerva

Install the development environment first (`python scripts/dev.py setup`, [DEVELOPMENT.md](../DEVELOPMENT.md)) and
build the receiver harness the simulation runs (`cargo build -p robot_control --bin control_harness`, which
`python scripts/dev.py check` also does; `ROBOT_CONTROL_HARNESS` names another binary). Then, from the repository root
on Windows:

```powershell
.venv-dev/Scripts/python.exe -m robot_platform helm --profile config/devices/minerva-joint-calibration.json --teleop config/devices/minerva-teleop-direct.json --sim
```

Open `http://127.0.0.1:8760/` (`--port` changes it). The header says **Simulation only** and the process prints
`SIMULATION ONLY; no physical robot connection`. A simulation-only Helm cannot make a robot connection: it never
opens SSH, the stopped-motor reader, Minerva Check, the sweep or drive-zero workspaces, the UPS watch or the Jetson's
episodes. `--sim` needs `--teleop` and cannot be combined with `--on-robot`. For the production profile's timing
(100 Hz, collision checking off) use `config/devices/minerva-teleop-direct-tuning.json` instead.

### What you need

- **Leaders: yes, for Teleop.** The simulated receiver is driven by the same GELLO leaders, over their USB ports on this
  PC. Teleop needs the leader ranges and zero saved (the profile's `leader_ranges`, edited in the Leaders tab), and a
  session owns the leaders' USB ports, so close the leader viewer and disconnect the Leaders tab first. Without
  leaders the pages still open and the 3D view works, but no session starts. The tests drive the same path with
  scripted leaders; there is no operator-facing switch for that.
- **A window: yes.** The first session start opens a MuJoCo window beside Helm that draws the arms. It draws at 30 Hz
  in its own thread and never steps the physics, so closing or covering it does not slow the loop.
- **No robot files.** The profile names a device and SSH identity, but a simulation does not read them.

Close any other Helm first: two would share the runs folder's lock and the leaders' USB ports.

### Using it

In **Teleop**, keep the virtual receiver (the only choice in `--sim`) and start. The keys and pedals work as on the
robot: I Idle, E Enable, T Engage, X Shutdown, H Disable ([Helm manual](../operator/HELM_MANUAL.md)). The task setup
in `--sim` builds the simulated task scene (a table, bucket and cube from a workcell file, with simulated cameras);
`--sim-workcell` picks the workcell file or `synthetic`, and `--sim-visual-mode` chooses the cameras' appearance
(`collision_proxy`, the default, or full `cad`). See [SIM_WORKCELL.md](../simulation/SIM_WORKCELL.md) and
[MINERVA_TASK_SCENE.md](../simulation/MINERVA_TASK_SCENE.md).

### What it does

Helm derives the receiver for the simulation exactly as it does for the real arms: limits, gains, feed-forward and
lifecycle are the production ones, and the real Rust receiver runs it, ticking every 2.5 ms. Only what the receiver
drives differs: an `ArmPlant` per arm instead of motors.

Every joint's plant takes its values from measurement, in this order:

1. **Fitted on real teleop.** `config/devices/simulation/minerva-production-01-actuators.json` holds inertia, friction,
   damping, the drive's command delay and a gravity scale per motor, fitted from recorded sessions (see
   [SIM2REAL.md](../simulation/SIM2REAL.md), "Into the simulator"). The file binds each entry to the motor it was fitted
   on by UID and firmware, so it applies only to those motors. The session's `simulation` event records the file (path
   and sha256) and which motors took it.
2. **The Tune sweep.** A joint the file does not cover (both grippers, or a motor that does not match) carries the
   inertia, friction and gravity slices its Tune sweep measured.
3. **Rehearsal nominals.** Where a sweep measured none, the joint takes a rough value from the rehearsal tests
   (`NOMINAL` in `simulation/minerva_sim.py`, not measured), and the session's `events.jsonl` lists which joints. With
   the actuator file only the grippers are fully nominal; without it, left J1 and both grippers are, and right J1 and
   J3-J6 have nominal inertia.

Gearbox damping makes up a 0.6 damping ratio where the sweep's gains fall short of it; the fitted damping replaces
that where the file applies. The arm joints hold at the gains and on the day they were recorded.

### How it differs from the robot

- **It is the receiver and the operator loop on measured joints, not a physics scene.** The task scene adds a table,
  bucket and a cube with contact, but contact does not load the arms or change a motor's response, the joints do not
  load each other beyond their measured gravity, and the grippers' values are nominal (calibrating them needs objects
  to grasp).
- There is no CAN, no motor faults from real hardware, no cameras from the Jetson, and no UPS.
- Timing differs: the loop runs on this PC, where Windows' 15.6 ms clock and scheduling can move the cycle. Helm's loop
  process sets its own scheduling, which a run without it showed to matter (two of three runs expired a command).
- The Teleop tab's Live receiver choices are hidden.
- The directions come from Helm's runs folder. A fresh `--runs` folder falls back to a seed and mirrors right J3 and
  J4; there the right arm's J2-J4 also lose their fitted gravity scales, and the session's `simulation` event lists
  them under `gravity_unscaled`. To use the verified directions, point `--runs` at a folder that holds them, or
  `--seed` at an earlier directions file.

### Where runs are written

`--runs` (default `calibration_runs`, git-ignored) holds Helm's state. A simulated session records every tick
(`samples.f32`) and its events under `<runs>/minerva-production-01-sim/sim-<UTC>/`, the way a live session records.
What a task scene records is described in [MINERVA_TASK_SCENE.md](../simulation/MINERVA_TASK_SCENE.md).

## Other ways to look at Minerva

**The model viewer** shows the robot's model and jogs its joints. It commands no motor.

```powershell
.venv-dev/Scripts/python.exe -m robot_platform view --robot minerva
```

`view` takes `--robot`, `--model <model.json>` or `--urdf <file>`, and `--port`. See
[MODEL_VIEWER.md](../simulation/MODEL_VIEWER.md); it is also how a new robot's model is checked.

**The headless simulator** runs a robot's receiver against a loopback simulation, with scenarios for the failures
(`feedback-loss`, `frozen-feedback`, `malformed-feedback`, `camera-loss`, `emergency-stop`):

```powershell
.venv-dev/Scripts/python.exe -m robot_platform sim run --robot minerva --output simulation_runs/minerva-001
```

`--output` must not exist yet; `--duration` is in seconds (0 runs until Ctrl-C), `--seed` fixes the noise,
`--no-cameras` leaves the camera stream out. How the run's settings, freshness checks and evidence work is in
[SIMULATION_IMPLEMENTATION.md](../simulation/SIMULATION_IMPLEMENTATION.md).

## Further reading

- [MuJoCo plant and receiver](../simulation/MUJOCO.md) and [Minerva's MJCF](../simulation/MINERVA_MJCF.md)
- [Simulation against the real robot](../simulation/SIM2REAL.md): how the fitted values were found and how far to trust
  them
- [Policy evaluation](../simulation/MINERVA_EVAL.md): N-trial closed-loop runs in simulation or on the robot
- The generation line (`docs/simulation/SIM_*.md`): scripted demonstrators, randomised layouts, sim data for training
