# Scripts

`scripts/` is a flat folder of build, deploy, bench and maintenance scripts. It is not organised by robot, and many names
say `aizee-*` for things Minerva uses (the CAN helpers, the head camera unit); `docs/robots/aizee/README.md` sorts those
names. Run scripts from the repository root. Nothing here is run by the package; each script's header says what it
does and whether it touches hardware. Anything that reaches a robot needs the owner's approval.

Minerva is normally built and deployed with `robot-platform deploy` and the `ops` modules (see
[the architecture overview](../docs/architecture/OVERVIEW.md)); the scripts below are the building blocks, the bench
tools and the older AIZEE path.

## Development and CI (both robots)

| Script | What it does |
|---|---|
| `dev.py` | `setup` makes `.venv-dev`; `check` is the full gate; `quick` runs the tests that touch what changed. See [DEVELOPMENT.md](../docs/DEVELOPMENT.md). |
| `gate_time_study.py` | Where a gate run's time went (stages, files, slowest tests) from `build/dev-check`. |
| `ci_windows_gl.py` | Mesa software renderer for hosted Windows CI, checkout-local. |
| `build_release.py` | Builds the ARM64 release prerequisites on a Windows or Linux x86_64 host. No robot contact. |
| `build_jetson.ps1`, `build_jetson_verify.py` | Cross-compile the Rust services for the Jetson from Windows x64; inspect the ELF files. See [JETSON_CROSS_BUILD.md](../rust/JETSON_CROSS_BUILD.md). |
| `build_collision.ps1` | Build the native collision library (`-Test` also runs its tests). |
| `test_control_core.ps1` | Build and test the portable Rust control core on Windows. |
| `validate_simulation.py` | Loopback simulator scenario matrix, both robots. |

## Minerva

### Build, install and deploy

| Script | What it does |
|---|---|
| `deploy_minerva_arms.sh` | Deploy the guarded arm runtime to the Jetson as a verified release (the `robot-platform deploy` path wraps the same release). |
| `setup_minerva_system.sh` | One-time system setup on the Jetson (needs sudo, reads `scripts/.jetson_password`, which is git-ignored): CAN helpers, udev, USB autosuspend off. |
| `setup_operator_station.ps1` | One-time setup of the operator station as Administrator: OpenSSH server, sleep and USB suspend off, tether adapter power saving off. |
| `install_gello_bridge.ps1` | Install or update the GELLO bridge on the operator station. See [HELM_ON_JETSON.md](../docs/architecture/HELM_ON_JETSON.md). |
| `install_scene_cam.sh`, `deploy_scene_cam.sh` | Install or deploy the head D435 publisher (unit `aizee-scene-cam`). |
| `aizee-can-up`, `aizee-reset-usb-can`, `aizee-can-reset` | CAN interface helpers, installed to `/usr/local/bin` by `setup_minerva_system.sh` (`canL`, `canR`, 1 Mbit). |
| `helm.ps1` | Starts Helm on a Windows PC against the robot over the tether, with the guarded `minerva-teleop.json`. This is the retired operator-PC deployment; Minerva's Helm runs on the Jetson. `robot-platform helm --sim` is the simulation entry. |
| `calibrate_minerva.ps1` | The stopped motor-to-CAD calibration wizard. See [JOINT_CALIBRATION](../docs/operator/JOINT_CALIBRATION.md). |
| `view_leaders.ps1` | The read-only dual-GELLO viewer ([GELLO_TELEOP.md](../docs/architecture/GELLO_TELEOP.md)). |

### Leader boards

| Script | What it does |
|---|---|
| `openrb_leader_setup.py` | OpenRB leader boards (firmware 2+): read info, or set the servos' Return Delay Time. |
| `probe_leader_timing.py`, `probe_leader_reopen.py` | Read-only timing probe of the leaders; USB re-open behaviour check. |

### Benchmarks and qualification (offline, no physical CAN)

| Script | What it does |
|---|---|
| `benchmark_direct_loop.py` | The full teleop loop with synthetic input and virtual motors. |
| `benchmark_direct_teleop.py` | Direct updates on the native harness or Linux vcan. |
| `benchmark_guarded_teleop.py`, `benchmark_guarded_teleop_vcan.py` | The guarded loop with the collision supervisor, on the harness and on vcan. |
| `benchmark_leader_teleop.py` | A 25 Hz real-leader shadow test against a virtual receiver. |
| `benchmark_collision.py`, `benchmark_keepout.py` | Whole-robot proximity and keep-out latency. |
| `guarded_vcan_bench.py` | The actual Linux guarded service on verified virtual CAN with synthetic motors. |
| `qualify_direct_motion.py`, `qualify_manual_tuning_loop.py` | Offline command qualification of the direct profile; the tuning loop on private vcan. |
| `rehearse_mujoco.py`, `view_mujoco.py` | Record a MuJoCo fixture driven by the offline receiver; pose the generated MJCF. |
| `compose_collision_model.py` | Combine detailed visuals with a collision-only URDF. |
| `camera_noise_stack.py`, `webrtc_link_probe.py` | Camera noise from frame stacks (on the Jetson); measure a headset link from the headset's browser. |

### Subfolders

| Folder | Contents |
|---|---|
| `sim/` | Task-scene probes: workcell reach, demonstrator reach map, layout trials and refusals, fiducial and head-depth checks and renders. For the generation line, [SIM_GENERATION.md](../docs/simulation/SIM_GENERATION.md). `grab_head_depth.py` runs on the Jetson, read-only. |
| `fiducials/` | Table tag labels: render, print (Brother QL-800, 300 dpi) and a synthetic decode check. See [TABLE_FIDUCIALS](../docs/operator/TABLE_FIDUCIALS.md). |
| `splat/` | Gaussian-splat training for the headset (WSL2 on the operator station). Its headers name the station by hostname; see [HEADSET_3D_PLAN.md](../docs/architecture/HEADSET_3D_PLAN.md). |
| `training_server/` | Scripts for the GPU training server, never run on the robot. See its [README](training_server/README.md). |
| `maintenance/gs_usb/` | A candidate gs_usb driver backport for the USB-CAN adapters, one unit's investigation. See its README. |

## AIZEE (the earlier robot)

| Script | What it does |
|---|---|
| `bootstrap_jetson.sh`, `setup_jetson.sh` | Fresh-Jetson bootstrap from the dev machine, and on the Jetson. |
| `deploy_jetson_rover.sh`, `deploy_python_package.sh`, `deploy_lidar_control.sh`, `deploy_gripper_camera.sh`, `deploy_heartbeat.sh`, `deploy_tufty2040.sh` | Deploy the rover module, the Python package, LiDAR, gripper camera, heartbeat dashboard and display. |
| `deploy_common.sh` | Target and key resolution sourced by the AIZEE deploy scripts. Its defaults (`192.168.0.27`, a workspace key path) are the author's; set `AIZEE_TARGET` and the key yourself. |
| `install_lidar_service.sh`, `install_lidar_udev.sh` | LiDAR service and udev. |
| `launch_motor_control.sh`, `restart_motor_service.sh`, `reset_display.sh`, `shutdown_all.sh`, `check_rover_status.sh` | Run, restart and check the rover's services. The restart and reset scripts hard-code the author's host and key. |
| `set_gantry_end_torque_limit.sh` | Write a torque limit on the gantry-end motor (stop the service first). |
| `run_policy.sh` | ACT inference with a chosen AIZEE checkpoint. |
| `start_teleop_record.sh`, `start_so101_teleop.sh` | tmux sessions for teleop with recording, and for SO-101 leader teleop. |
| `test_realsense.py`, `jetson_camera_monitor.sh` | RealSense self-test; tegrastats and camera memory logger. |

## Retired but still in the tree

| Script | Note |
|---|---|
| `setup_ssh_keys.sh` | Pushes keys to four Raspberry Pi camera hosts of the early AIZEE setup. |
| `install_itmux.sh`, `build_librealsense_rsusb.sh` | Windows tmux and a Raspberry Pi librealsense build; unrelated to either current robot. |
