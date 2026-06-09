# Tufty2040 Status Display

**Hardware:** Pimoroni Tufty2040 (RP2040, 320×240 IPS LCD)
**Communication:** USB CDC serial at 115200 baud
**Firmware:** MicroPython (`tufty2040/main.py`)
**Bridge:** `python/nodes/display_node.py` (systemd service on Jetson)

---

## Overview

The Tufty2040 provides a live robot-health dashboard. It connects to the Jetson via USB and is driven by `display_node.py`, which subscribes to motor telemetry (ZMQ :5556) and UPS telemetry (ZMQ :5562), assembles a compact JSON packet, and writes it to the device over serial at 2 Hz.

### Screen Layout (320×240)

```
┌──────────────────────────────────────┐  y=0
│ AIZEE STATUS   192.168.0.27       OK │  title bar + IP
├──────────────────────────────────────┤  y=22
│ JETSON BATTERY    │  MOTOR BATTERY   │
│  11.4V   73%      │  24.1V    81%    │  both halves get voltage+%
│  ████████████░░░  │  █████████░░░░   │  both halves get bar
├──────────────────────────────────────┤  y=80
│ MOTORS:  [ ENABLED ]                 │  enable pill
├──────────────────────────────────────┤  y=100
│ BASE: [lw:R][rw:R][sw:R]            │  motor state boxes
│ ARM:  [gb:R][gm:R][ge:R]            │
│       [wp:R][wr:R][gr:R]            │
├──────────────────────────────────────┤  y=154
│ SERVICES:                            │
│ [MOTOR  OK][LIDAR  OK][UPS    OK]   │  service status
│ [RELAY  OK][DISP   OK]              │
├──────────────────────────────────────┤  y=200
│ PIES: [P1 UP][P2 UP][P3 UP][P4 UP] │  RPi camera node reachability
└──────────────────────────────────────┘  y=240
```

When motor battery is disconnected (`mv` absent): right half shows "DC" in gray, bar is empty.

**Motor state colours:** green=running, yellow=enabling, dark=disabled, red=error

**Service state colours:** green=active, red=failed, yellow=inactive/activating, grey=unknown

---

## Hardware Setup

1. Plug the Tufty2040 into any USB port on the Jetson.
2. The udev rule (`config/udev/99-aizee-display.rules`) creates the symlink `/dev/tufty_display` and automatically starts `aizee-display.service`.

### Install udev rule (one-time)

```bash
sudo cp config/udev/99-aizee-display.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout ltr   # allow ltr to access serial devices
```

Verify the symlink appears when the device is plugged in:

```bash
ls -l /dev/tufty_display
```

---

## Deploying Firmware

The MicroPython firmware lives at `tufty2040/main.py` on the dev machine. Use the deploy script to flash it remotely via the Jetson.

### Prerequisites (one-time on Jetson)

```bash
pip install mpremote   # installs to ~/.local/bin/mpremote
```

### Deploy

From the dev machine:

```bash
./scripts/deploy_tufty2040.sh
```

The script:
1. SCPs `tufty2040/main.py` to `/tmp/tufty_main.py` on the Jetson
2. Stops `aizee-display` to release the serial port
3. Runs `mpremote connect /dev/tufty_display cp /tmp/tufty_main.py :main.py + reset`
4. Waits 3 s for the board to re-enumerate
5. Restarts `aizee-display` and prints its status

### Deploy display_node.py alongside firmware

`display_node.py` runs on the Jetson (not on the Tufty2040) and must be deployed separately when changed:

```bash
scp -i /p/Workspace/ssh-keys/aizee_rover_id \
    python/nodes/display_node.py \
    ltr@192.168.0.27:/home/ltr/aizee/python/nodes/display_node.py

./scripts/reset_display.sh   # restart service to pick up new code
```

Or, if you are also reflashing the firmware, just run `deploy_tufty2040.sh` after the SCP — it restarts the service as its final step.

---

## Service Management

The display bridge runs as a systemd service that starts automatically when the Tufty2040 is plugged in.

```bash
# Check status
sudo systemctl status aizee-display

# Follow logs
sudo journalctl -u aizee-display -f

# Restart (e.g. after updating display_node.py)
./scripts/reset_display.sh
```

---

## JSON Packet Format

`display_node.py` sends one JSON line per update (2 Hz) over the serial port:

```json
{
  "mv": 24.1,
  "mp": 81,
  "up": 11.4,
  "ub": 73,
  "me": true,
  "ms": {"lw":"r","rw":"r","sw":"r","gb":"r","gm":"r","ge":"r","wp":"r","wr":"r","gr":"r"},
  "sv": {"motors":"a","lidar":"a","ups":"a","relay":"a","disp":"a"},
  "ip": "192.168.0.27",
  "pi": {"pi1":"u","pi2":"u","pi3":"u","pi4":"u"},
  "t": 1740000000.0
}
```

| Field | Description |
|-------|-------------|
| `mv`  | Motor bus voltage (V), or absent when actuator power is off |
| `mp`  | Motor battery percentage (0–100), or absent when `mv` is absent |
| `up`  | UPS voltage (V) |
| `ub`  | UPS battery percentage |
| `me`  | Motors enabled (all 3 base motors in running state) |
| `ms`  | Per-motor state: `r`=running `e`=enabling `d`=disabled `x`=error `?`=unknown |
| `sv`  | Per-service state: `a`=active `f`=failed `i`=inactive `e`=activating `?`=unknown |
| `ip`  | Jetson WiFi IP address string (e.g. `"192.168.0.27"`), or `""` |
| `pi`  | Per-Pi reachability: `{"pi1":"u","pi2":"d",...}` — `u`=up, `d`=down, `?`=unknown |
| `t`   | Unix timestamp |

Service status is polled every 5 seconds via `systemctl is-active`. Monitored services: `aizee-motor-control-rover`, `aizee-lidar-control`, `aizee-ups-monitor`, `aizee-display`, `aizee-gripper-cam`, `aizee-scene-cam`.

---

## Troubleshooting

### Display shows "WAITING..." indefinitely

The display has not received a packet from `display_node.py`. Check:

```bash
sudo systemctl status aizee-display
sudo journalctl -u aizee-display -n 30
```

Common causes: service not running, `/dev/tufty_display` symlink missing (udev rule not installed), or `ltr` not in the `dialout` group.

### "NO SIGNAL" after data was showing

No packet received for >5 seconds. `display_node.py` is probably still running but lost the serial connection (USB unplug/replug):

```bash
./scripts/reset_display.sh
```

### `mpremote: command not found` during deploy

Install mpremote on the Jetson:

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    "pip install mpremote"
```

### Service status boxes all show "???"

`systemctl is-active` failed in `display_node.py`. Check the display node logs:

```bash
sudo journalctl -u aizee-display -n 50 | grep "Service status"
```

---

## Related Files

| File | Purpose |
|------|---------|
| `tufty2040/main.py` | MicroPython firmware — renders the dashboard |
| `python/nodes/display_node.py` | Jetson bridge — ZMQ → serial |
| `config/systemd/aizee-display.service` | Systemd service definition |
| `config/udev/99-aizee-display.rules` | udev rule for `/dev/tufty_display` symlink |
| `scripts/deploy_tufty2040.sh` | Flash firmware remotely via Jetson |
| `scripts/reset_display.sh` | Restart `aizee-display` service |
