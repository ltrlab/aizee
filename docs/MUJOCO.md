# MuJoCo simulation

MuJoCo is the coupled-physics backend. `MujocoPlant` implements the existing
`PlantReceiver` plant boundary: named motor positions, timed integration and
feedback. `PlantReceiver` continues to run the real Rust receiver at 400 Hz.
`ArmPlant` remains useful for cheap fault injection and existing regressions.

## Run the synthetic rehearsal

```sh
python scripts/dev.py setup
python scripts/dev.py check
.venv-dev/Scripts/python.exe scripts/rehearse_mujoco.py --output simulation_runs/mujoco-first
```

On Linux, use `.venv-dev/bin/python`. The output directory must be new. The demo
uses an explicitly synthetic slider, no sockets or hardware, and records motor
feedback, receiver state, engine version and model/binding identity. It is not
the `robot-platform sim run` service or a Minerva dynamics demonstration.

The gate includes analytic gravity and coupled-inertia tests, signed transmission
power checks, contact, receiver integration, deterministic sensor noise, silence,
and generated command-expiry intervals. Physics advances in five 0.5 ms steps per
2.5 ms receiver tick. Simulation time and host execution time are separate; passing
these tests establishes neither real-time Jetson performance nor hardware safety.

## Adapter contract

- Bind each receiver motor explicitly to one MuJoCo hinge/slide joint. Ordering
  follows the receiver, never the XML. Other model joints remain passive.
- `q_model = offset + scale*q_motor`; velocity has the same scale. The generalized
  force is `torque_motor/scale`, preserving power for positive and negative scales.
  A jaw slide needs a measured m/rad transmission, not a copied angular command.
- The adapter applies the receiver's MIT law and clamps effort to each binding's
  explicit limit. MJCF actuator elements are rejected to prevent applying two
  controllers to the same joint. Motor electrical dynamics, saturation response,
  gearbox backlash, torque estimation and communication delay remain unvalidated.
- Mechanics include MuJoCo's model gravity, inertias, friction and contacts. Sensor
  quantization/noise is applied only to feedback. `quiet` withholds fresh readings;
  `dropped` additionally makes the motor limp, as in the existing fault plant.
- URDFs must pass the physical-input audit. URDF mimic joints are rejected here
  until translated into explicit tested MJCF constraints/tendons. Existing CAD
  collision policy is independent of MuJoCo contact policy; neither silently
  replaces the receiver's collision envelope.
- Root XML hashes identify the synthetic fixture. For a robot asset bundle,
  retain the existing release manifest covering all includes, meshes, profiles,
  mappings and calibration files; a root XML hash alone is insufficient.

## Audit Minerva now

```sh
.venv-dev/Scripts/python.exe -m robot_platform.models.physics_audit urdf/minerva/minerva.urdf --output build/minerva-physics-audit.json
```

This currently exits 1 because all 17 masses are exporter placeholders of
`1e-9 kg`. It reports per-link mass and principal inertia, graph/asset errors and
joint-limit omissions. A successful audit means structurally usable input only;
it always reports physical validation as `unvalidated`.

The [measurement checklist](MINERVA_PHYSICS_MEASUREMENTS.md) is the next physical
input task. Start with one arm, verify its joint frames independently, then compare
static torque and motion traces before enabling dual-arm or contact scenarios.
Do not invent mass properties to obtain a green Minerva demo.

## Minerva's MJCF (kinematic)

`urdf/minerva/mujoco/minerva.xml` is generated from the active URDF and committed with its
`minerva.manifest.json`, which lists the source URDF, every mesh and their hashes. The gate fails if
the export no longer matches the URDF. After a URDF or mesh change, delete the export and regenerate:

```sh
.venv-dev/Scripts/python.exe -m robot_platform.models.mjcf urdf/minerva/model.json --output urdf/minerva/mujoco/minerva.xml
```

- **Frames.** Every link is a named body at its URDF link frame. Tests compare the link frames with the
  project's URDF kinematics at 202 poses, and every mesh's placement with its source STL.
- **Jaws.** Each mimic is a rigid joint equality with time constant 2 × timestep. Backlash and compliance
  are not modeled.
- **Inertials.** These are the URDF's 17 placeholders, so the manifest says `dynamics_usable: false` and
  `MujocoPlant` refuses the model. Replacing them is the measurement task.
- **Not physics.** Contacts are off, and ranges are the URDF limits, not hard stops. There is no damping,
  friction, armature or actuator, and motor-to-joint bindings are not part of the model.

View it, kinematics only:

```sh
.venv-dev/Scripts/python.exe scripts/view_mujoco.py
.venv-dev/Scripts/python.exe scripts/view_mujoco.py --render minerva.png --pose '{"left_arm_j2": 0.5}'
```

`--collision` shows the source collision meshes instead of the visuals. The viewer's joint sliders
recompute kinematics only. See [the task brief](development/MINERVA_MJCF.md) for results and limits.

## Import decisions

Keep CAD/URDF as the source of geometry, frames and inertial evidence, and keep
MuJoCo-specific contacts, coupling and drive assumptions in a versioned MJCF
simulation model. MuJoCo loads both formats, but its richer physics configuration
uses MJCF. [MuJoCo modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)

During validation, declare compiler settings explicitly: radians, inertia from
measured inertials, no automatic inertia balancing or mass floor. Preserve named
frames until kinematics comparisons pass. Review visual-discarding/static-fusion
settings when importing URDF; their defaults differ from MJCF. Automatic inertia
balancing can conceal a bad tensor by replacing its principal moments.
[Compiler reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler)

Use native `MjModel`/`MjData` and explicit `mj_step` calls in the adapter. The viewer
is optional and is not part of the deterministic gate.
[MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)
