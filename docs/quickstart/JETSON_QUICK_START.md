# Jetson Quick Start

Daily-driver reference for connecting to the AIZEE robot and running teleop or
data collection. Run all dev-machine commands from the repo root.

The on-robot brain is a **Jetson Orin Nano** (JetPack 6.x). All robot
subsystems run as systemd services that auto-start on boot, so after power-up
you usually just connect, glance at the dashboard, and run.

## Modes

| Mode   | What runs              | Notes                                   |
|--------|------------------------|-----------------------------------------|
| ROVER  | Wheels + arm           | Full mobile manipulator                 |
| STATIC | Arm only (+ scene cam) | Wheels idle; optional subsystems auto-detect |

Optional subsystems (gripper cam, scene cam, LiDAR) auto-detect — you don't
select a mode explicitly.

## 1. Connect

Three ways to reach the Jetson. The operator tools auto-resolve in this
priority order, so usually you don't pick one manually:

| Priority | Path                       | IP             |
|----------|----------------------------|----------------|
| 1        | LAN / WiFi client          | `192.168.0.27` |
| 2        | USB-C ethernet (tether)    | `10.42.0.1`    |
| 3        | WiFi AP `aizee` (robot's own network) | `192.168.50.1` |

SSH in with:

```bash
ssh -i ssh-keys/aizee_rover_id ltr@<ip>
```

(user is `ltr`).

## 2. First check — the heartbeat dashboard

Before anything else, open the dashboard in a browser:

```
http://<jetson-ip>:8088
```

- Over USB-C: <http://10.42.0.1:8088>
- Over the AP: <http://192.168.50.1:8088>
- On the LAN:  <http://192.168.0.27:8088>

It shows service status, recent journald logs, host metrics (CPU/mem/disk/
thermal), and live robot telemetry (motors, batteries, cameras). This replaces
ad-hoc status checks — if everything is green here, you're ready to run.

## 3. Run teleop

From the dev machine, repo root:

```bash
python python/teleop/teleop.py
```

Endpoints come from `config/teleop.yaml` (default). Useful flags:

| Flag                       | Effect                          |
|----------------------------|---------------------------------|
| `--keyboard-only`          | Disable gamepad                 |
| `--endpoint tcp://IP:5555` | Override command endpoint       |
| `--config path/to.yaml`    | Use a different config          |
| `--log-level DEBUG`        | Verbose logging (to `teleop.log`) |

### Controls

| Key      | Action                                 |
|----------|----------------------------------------|
| WASD     | Drive (forward / back / turn)          |
| E        | Enable all motors                      |
| Q        | Disable all motors                     |
| Back btn | Emergency stop (gamepad)               |
| Start btn| Clear E-stop + faults (gamepad)        |
| ESC      | Exit                                   |

Gamepad: A = enable, B = disable, Back = E-stop, Start = clear E-stop/faults,
left stick = drive.

## 4. Collect demonstration data

A leader arm (SO-101, OpenRB-150, or Quest VR) drives the arm while episodes
are recorded. From the dev machine, repo root:

```bash
python python/scripts/collect_demo.py --gui
```

The rover IP, leader device, and scene camera are all **auto-detected**:

- `--gui` opens the Qt viewer (omit for the terminal renderer).
- `--leader openrb|so101|quest` forces a specific leader kind (default `auto`).

Common recording controls:

| Key  | Action                                        |
|------|-----------------------------------------------|
| E    | Enable arm motors (align to leader)           |
| R    | Toggle recording                              |
| H    | Hold target at current position               |
| Z    | Capture current leader pose as zero reference |
| X    | Soft shutdown (return to zero, disable)       |
| Q    | Quit                                          |
| WASD | Drive wheels (enable with arm)                |

## Telemetry quick test

Confirm the Jetson is publishing motor telemetry:

```bash
python -c "import zmq,json;c=zmq.Context();s=c.socket(zmq.SUB);s.connect('tcp://10.42.0.1:5556');s.setsockopt(zmq.SUBSCRIBE,b'');print(s.recv())"
```

(Swap the IP for whichever path you connected over.)

## Ports & motors at a glance

ZMQ on the Jetson: `:5555` commands, `:5556` telemetry, `:5562` UPS,
`:5563` gripper cam, `:5564` scene cam, `:8088` heartbeat dashboard.

9× ROBSTRIDE motors on `can1` @ 1 Mbps:

| Motor       | CAN ID | Motor        | CAN ID |
|-------------|--------|--------------|--------|
| left_wheel  | 0x02   | gantry_mid   | 0x06   |
| right_wheel | 0x04   | gantry_end   | 0x07   |
| swivel      | 0x03   | wrist_pitch  | 0x08   |
| gantry_base | 0x05   | wrist_roll   | 0x09   |
|             |        | gripper      | 0x0A   |

For post-reboot verification, service management, and troubleshooting, see
[QUICK_START_AFTER_REBOOT.md](QUICK_START_AFTER_REBOOT.md).
