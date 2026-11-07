# Driven grippers in the task scene

**Status: done and in use** by the evaluation harness and by generation (`--grippers driven`). Live `helm --sim` still uses kinematic grippers. Simulation only; the squeeze force is not calibrated against the real gripper.

In the task scene ([MINERVA_TASK_SCENE.md](MINERVA_TASK_SCENE.md)) every robot body is posed kinematically, the jaws included. Squeezing the cube therefore comes from how far the posed jaws sink into it, and the gripper reading ignores the cube. The real gripper is different: it stops at the cube's width and squeezes with its drive's torque, so its reading stays open while it holds the cube. A policy trained on kinematic jaws would see "fully closed" while holding, and on the robot "stopped at the cube".

Driven grippers close that gap. The arms stay kinematic. Each gripper becomes a motor hinge, and its two jaws slide on the wrist through the model's transmission, so contact pushes back on the motor.

## The model

- **Motor hinge** (`task/<side>_gripper/drive`), a child of the wrist. Its angle is the receiver's gripper coordinate.
  - Armature: the gripper's ArmPlant inertia (0.002 kg m², nominal).
  - Frictionloss: its friction (0.1 Nm, nominal).
  - Damping: its viscous term.
- **Jaws** (`carriage_lwl7b*`) keep their CAD placement on the wrist and their slide joints and ranges.
  - Jaw a follows the motor through the joint map's transmission. On this robot that is 9.2 mm per motor radian, 48.2 mm of travel per jaw.
  - Jaw b follows jaw a through the model's mimic equality.
  - Both couplings are held near-rigid (impedance 0.9999).
- **Torque.** Each 0.5 ms physics step applies the drive's MIT law to the hinge: kp (position - q) + kd (velocity - v) + effort. The command comes from the receiver.
- **Torque limit.** The torque is clipped to the receiver configuration's `max_effort` (14 Nm). The drive's own current limit is not measured.
- **Contact.** The pads and the cube use a 1 ms contact time constant (impedance 0.99-0.999), so the pads stay within a fraction of a millimetre of the cube at hundreds of newtons. These are numerical choices: the real pad and wood contact is not measured.
- **Arm motion.** A jaw's surface velocity carries only its wrist's motion; MuJoCo already integrates the slide.
- **Masses.** The rotor and jaw bodies carry 1 g each. They exist only so the bodies can move, and are tiny next to the reflected rotor inertia (about 24 kg at a jaw).

The jaws' combined squeeze is the drive torque divided by the transmission (each jaw takes half). Through the receiver's direct profile (gripper kp 2, kd 1), commanding 0.5 rad on the 36 mm cube stops the left gripper at 1.92 rad. It reports -2.85 Nm, about 310 N of combined squeeze. A wood cube and a hard pad hold that easily; whether the real gripper squeezes that hard is not measured. A squeeze on a kitchen scale would tell.

## Use

```powershell
.venv-dev/Scripts/python.exe -m robot_platform.evaluation run --policy <policy> --grippers driven [--workcell <file>]
```

`--grippers driven` needs the `armplant` plant: the ideal plant sends no drive commands. The runner builds the scene with the plant's gripper values (`ArmPlant.gripper_drives`).

1. It stages the arms as before, then resets the scene with the drives at the plant's gripper positions.
2. It couples the plant to the scene. From then on, each 2.5 ms receiver tick steps the scene: the arms move from the tick's start pose to its end pose, and the grippers move under the commands the drives hold.
3. The plant's gripper position, speed and torque are replaced by the scene's. The receiver reads them back with the usual 16-bit steps and noise.
4. After the trial it uncouples.

`MinervaTaskScene(..., grippers={motor: {inertia, friction, viscous, max_effort, source}})` builds a driven scene. `step_driven(start, end, commands, dt)` advances it, and `update` still poses everything, drives included, for staging jumps. Without `grippers` the scene is unchanged: the same model and the same frozen-fixture results.

Not driven yet:
- **Live `helm --sim`.** The scene runs in its own process, a step behind the control loop. Coupling it tick by tick would put physics in the loop's process. Human demonstrations recorded there still have kinematic grippers.
- **The arms.** They do not feel contact: the table, the bucket or the other gripper cannot push them back.

## Tests

- `tests/unit/test_task_scene_grippers.py`:
  - a gripper commanded past the cube stops on it, and its torque is kp times the shortfall;
  - the pads' force adds up to torque divided by the transmission, with under 0.5 mm of penetration;
  - a gripper closing on nothing reaches its command;
  - the torque is clipped at 14 Nm;
  - the grip lifts the cube, and opening drops it, since nothing else holds it;
  - posing places the drives.
- `tests/contract/test_driven_grippers.py`: through the actual receiver, the left gripper's reading stays at least 0.8 rad short of its command, pressing with negative effort within the limit. The empty right gripper reaches its command, and the reading matches the scene's drive.

The frozen fixture still fails through `armplant` with driven grippers, as it did with kinematic ones: no pickup, then the right jaw goes into the bucket wall at 9.3 s. Open-loop replay against arm tracking error is the cause (MINERVA_EVAL.md), not the grippers.
