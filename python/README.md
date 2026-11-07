# `python/`

The older application code that has not been moved into the installable package (`src/robot_platform/`). Two things keep
it apart: it is large GUI, hardware or training code that needs a checkout (assets, calibration files, URDFs), and parts
of it still run on the robot as services. New work goes into `src/robot_platform/`; the map of both is in
[the architecture overview](../docs/architecture/OVERVIEW.md). `robot-platform run collect|train` launches the collectors
and trainers here through `apps/launcher.py`.

| Folder | What it holds | Active for |
|---|---|---|
| `nodes/` | Camera and power services: `gripper_camera_node.py` (wrist UVC) and `camera_node.py` (RealSense, the head D435) run on Minerva as `minerva-*-wrist` and `aizee-scene-cam`; `display_node.py`, `ups_node.py`, `act_policy_node.py`, `find_motors.py`. | Cameras: **Minerva and AIZEE**. UPS and display: AIZEE (Minerva uses `minerva-ups`, `src/robot_platform/ops/ups.py`). |
| `scripts/` | `collect_demo.py` and `collect_demo_app/` (AIZEE collector), `collect_minerva.py` and `collect_minerva_app/` (the older Minerva Qt collector, format 6/7), calibration and probe tools (`minerva_calibrate.py`, `minerva_motor_probe.py`, `openrb_calibrate.py`, `gravity_calibrate.py`), standalone teleop and replay, `actuator_char/` and `characterize_actuator.py` bench tools. | AIZEE collectors and tools; for Minerva the older collector and bench tools. Minerva recording is Helm's task capture. |
| `training/` | ACT, JEPA and flow-matching models, datasets, training, inference. `train_minerva.py`, `minerva_dataset.py`, `minerva_model.py` are the older Minerva trainer. | AIZEE trains here. Minerva policies come from the LeRobot export and [TRAINING.md](../docs/data/TRAINING.md). |
| `teleop/` | Leader drivers for standalone teleop: OpenRB, SO-101, Quest. | AIZEE and the older Minerva scripts. Minerva's live leader path is `src/robot_platform/teleop/`. |
| `ik/`, `control/` | Inverse kinematics, collision sweeps and workspace bounds; gravity models. | AIZEE; `control/minerva_gravity.py` for the older collector. |
| `web/` | The Quest WebXR teleop server and its static page. | AIZEE. Minerva's headset page is Helm's `/xr`. |
| `tools/` | The heartbeat dashboard and the `/setup` validation wizard engine. | AIZEE. |
| `common/` | Constants and a `wire.py` compatibility shim that re-exports `robot_platform.transport.wire`. | Both, for the scripts above. |
| `tests/ml/` | Qt and recording regressions for the Minerva collector; run by name ([tests/README.md](../tests/README.md)). | Minerva older collector. |
| `rerun_bridge.py` | Rerun viewer bridge. | AIZEE. |

Scripts import the package, so install it first (`python scripts/dev.py setup`, or `pip install -e ".[operator,dev]"`).
Hardware-facing tools here run only when invoked and may open serial or CAN ports; read each header first.
