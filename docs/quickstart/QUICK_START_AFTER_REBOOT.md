# Quick Start After Jetson Reboot

Verify the robot came up cleanly after a reboot, manage its services, and
troubleshoot. For everyday connect-and-run, see
[JETSON_QUICK_START.md](JETSON_QUICK_START.md).

## What happens on boot

On-robot subsystems run as systemd services and **auto-start on boot**, so a
reboot normally needs no manual setup:

| Service                       | Role                                    |
|-------------------------------|-----------------------------------------|
| `aizee-motor-control-rover`   | Configures `can1` + runs control loops  |
| `aizee-gripper-cam`           | ELP UVC gripper camera (`:5563`)        |
| `aizee-scene-cam`             | RealSense scene camera (`:5564`)        |
| `aizee-ups-monitor`           | UPS battery telemetry (`:5562`)         |
| `aizee-heartbeat`             | Dashboard on `:8088`                    |
| `aizee-display`               | Tufty2040 status display                |
| `aizee-estop-bridge`          | Hardware E-stop bridge                  |

`aizee-lidar-control` is often disabled. Optional camera subsystems auto-detect
their hardware, so missing devices degrade gracefully.

The motor control service runs CAN init + control loops: arm @ 400 Hz, base @
100 Hz, telemetry @ 50 Hz.

## 1. First check — the dashboard

Open the heartbeat dashboard:

```
http://<jetson-ip>:8088
```

(e.g. <http://10.42.0.1:8088> over USB-C, <http://192.168.50.1:8088> over the
AP, <http://192.168.0.27:8088> on the LAN). It shows service status, recent
journald logs, host metrics, and live telemetry. If everything is green, you're
done — start teleop or data collection per
[JETSON_QUICK_START.md](JETSON_QUICK_START.md).

Alternatively, from the dev machine repo root:

```bash
./scripts/check_rover_status.sh
```

This checks network reachability, the CAN interface, the motor control service,
and live telemetry.

## 2. Confirm telemetry

```bash
python -c "import zmq,json;c=zmq.Context();s=c.socket(zmq.SUB);s.connect('tcp://10.42.0.1:5556');s.setsockopt(zmq.SUBSCRIBE,b'');print(s.recv())"
```

Swap the IP for whichever path you connected over. No data within a few seconds
means the motor control service isn't publishing — see troubleshooting below.

## Service management

Run on the Jetson over SSH (`ssh -i ssh-keys/aizee_rover_id ltr@<ip>`), or
prefix each with the SSH command from the dev machine. Substitute any service
name from the table above.

```bash
sudo systemctl status  aizee-motor-control-rover   # current state
sudo systemctl restart aizee-motor-control-rover   # restart
sudo systemctl stop    aizee-motor-control-rover   # stop
sudo systemctl start   aizee-motor-control-rover   # start
sudo systemctl enable  aizee-motor-control-rover   # auto-start on boot
sudo systemctl disable aizee-motor-control-rover   # cancel auto-start

sudo journalctl -u aizee-motor-control-rover -f    # live logs
sudo journalctl -u aizee-motor-control-rover -n 100  # recent logs
```

## Troubleshooting

### CAN interface down

The service configures CAN on startup. To check the unit's startup steps:

```bash
sudo journalctl -u aizee-motor-control-rover | grep ExecStartPre
```

Bring `can1` up manually:

```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

### Motor control service not running

```bash
sudo systemctl status aizee-motor-control-rover
sudo journalctl -u aizee-motor-control-rover -n 50
sudo systemctl start aizee-motor-control-rover
```

Common causes:

- CAN failed to come up (check `ExecStartPre` logs above).
- **A wheel motor is absent from the bus.** Both wheels must be physically
  wired or motor_control wedges during CAN init. For arm-only (STATIC) use,
  the wheels still need to be present on `can1`.
- Binary not built — re-run the deploy (below).

### No telemetry / motors not responding

1. Confirm the service is active (dashboard or `check_rover_status.sh`).
2. Verify the CAN interface and bitrate:
   ```bash
   ip link show can1
   ```
3. Watch raw CAN traffic:
   ```bash
   candump can1
   ```
4. In teleop, press **E** (or gamepad **A**) to enable motors; press
   **Start** to clear E-stop and faults.

## Redeploying after code changes

From the dev machine, repo root. Each deploy script syncs the relevant code,
(re)installs its service, and restarts it.

```bash
./scripts/deploy_jetson_rover.sh      # motor control (rust) + config
./scripts/deploy_gripper_camera.sh    # aizee-gripper-cam
./scripts/deploy_scene_cam.sh         # aizee-scene-cam
./scripts/deploy_heartbeat.sh         # aizee-heartbeat dashboard
```

Each accepts an optional `ltr@<ip>` target argument (default `192.168.0.27`,
heartbeat defaults to the USB-C tether `10.42.0.1`). After deploying, re-check
the dashboard or run `./scripts/check_rover_status.sh`.

## Manual motor control (no service)

If you need to run motor control by hand:

```bash
ssh -i ssh-keys/aizee_rover_id ltr@<ip>
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
cd ~/aizee
AIZEE_CONFIG=config/hardware_jetson_rover.yaml RUST_LOG=info \
    ./rust/target/release/motor_control
```
