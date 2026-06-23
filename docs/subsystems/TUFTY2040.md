# Tufty2040 Status Display

**Hardware:** Pimoroni Tufty2040 (RP2040, 320×240 IPS LCD)
**Communication:** USB CDC serial at 115200 baud, `/dev/tufty_display` (udev symlink)
**Firmware:** MicroPython — `firmware/tufty2040/main.py`
**Bridge:** `python/nodes/display_node.py` (systemd service `aizee-display` on the Jetson)

## Overview

The Tufty2040 shows a live robot-health dashboard. `display_node.py` runs on the
Jetson, subscribes to motor telemetry (ZMQ :5556) and UPS telemetry (ZMQ :5562),
assembles a compact JSON packet, and writes it to the board over serial at 2 Hz.
The board renders the packet; if no packet arrives it shows a waiting / no-signal
state.

### Screen layout (320×240)

```
┌──────────────────────────────────────┐
│ AIZEE STATUS   192.168.0.27          │  title bar + Jetson IP
├──────────────────────────────────────┤
│ JETSON BATTERY     │  MOTOR BATTERY   │  UPS (up/ub) | motor bus (mv/mp)
│  11.4V   73%       │  24.1V    81%    │
├──────────────────────────────────────┤
│ MOTORS:  [ ENABLED ]                  │  me (all base motors running)
├──────────────────────────────────────┤
│ per-motor state boxes (ms)            │  sw/gb/gm/ge/wp/wr/gr (+ wheels lw/rw)
├──────────────────────────────────────┤
│ SERVICES: motors lidar ups disp …    │  sv (systemctl is-active)
└──────────────────────────────────────┘
```

When the motor bus is unpowered, `mv`/`mp` are absent and the motor-battery half
shows no reading.

**Motor state colours:** green=running, yellow=enabling, dark=disabled, red=error
**Service state colours:** green=active, red=failed, yellow=inactive/activating, grey=unknown

## Hardware setup

1. Plug the Tufty2040 into any USB port on the Jetson.
2. The udev rule (`config/udev/99-aizee-display.rules`) creates `/dev/tufty_display`
   and the systemd device unit that starts `aizee-display.service` on plug-in
   (and stops it on unplug).

The rule matches the Pimoroni MicroPython USB ID (`idVendor=2e8a`,
`idProduct=1002`) and assigns group `dialout`.

```bash
sudo cp config/udev/99-aizee-display.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout ltr   # allow ltr to access the serial device

ls -l /dev/tufty_display       # verify the symlink appears when plugged in
```

## Deploying firmware

The MicroPython firmware is `firmware/tufty2040/main.py`. Flash it remotely from
the dev machine with the deploy script (requires `mpremote` on the Jetson:
`pip install mpremote`).

```bash
./scripts/deploy_tufty2040.sh
```

The script:
1. SCPs `firmware/tufty2040/main.py` to `/tmp/tufty_main.py` on the Jetson.
2. Stops `aizee-display` to free the serial port.
3. Runs `mpremote connect /dev/tufty_display cp /tmp/tufty_main.py :main.py + reset`.
4. Waits ~3 s for the board to re-enumerate.
5. Restarts `aizee-display` and prints its status.

`display_node.py` runs on the Jetson, not on the board, so deploy it separately
when changed:

```bash
scp -i ssh-keys/aizee_rover_id \
    python/nodes/display_node.py \
    ltr@192.168.0.27:/home/ltr/aizee/python/nodes/display_node.py

./scripts/reset_display.sh   # restart service to pick up new code
```

## Service management

`aizee-display` is bound to the USB device (`BindsTo=dev-tufty_display.device`),
so it starts when the board is plugged in and stops on unplug.

```bash
sudo systemctl status aizee-display
sudo journalctl -u aizee-display -f
./scripts/reset_display.sh        # restart (e.g. after updating display_node.py)
```

## JSON packet format

`display_node.py` sends one JSON line per update (2 Hz) over serial:

```json
{
  "mv": 24.1,
  "mp": 81,
  "up": 11.4,
  "ub": 73,
  "me": true,
  "ms": {"sw":"r","gb":"r","gm":"r","ge":"r","wp":"r","wr":"r","gr":"r"},
  "mpos": {"sw":0.0,"gb":0.0},
  "sv": {"motors":"a","lidar":"a","ups":"a","disp":"a","grip":"a","scene":"a"},
  "ip": "192.168.0.27",
  "t": 1740000000.0
}
```

| Field | Description |
|---|---|
| `mv` | Motor bus voltage (V); absent when actuator power is off |
| `mp` | Motor battery percentage (0–100, 6S range 19.8–25.2 V); absent when `mv` absent |
| `up` | UPS voltage (V); absent when stale |
| `ub` | UPS battery percentage; absent when stale |
| `me` | Motors enabled — all base motors (`lw`,`rw`,`sw`) in `running` state |
| `ms` | Per-motor state char: `r`=running `e`=enabling `d`=disabled `x`=error `?`=unknown |
| `mpos` | Per-motor position (rad), present only for motors reporting a position |
| `sv` | Per-service state: `a`=active `f`=failed `i`=inactive `e`=activating `?`=unknown |
| `ip` | Jetson IP string (first `192.168.*`), or `""` |
| `t` | Unix timestamp (rounded) |

Motor abbreviations: `lw`/`rw` wheels, `sw` swivel, `gb`/`gm`/`ge` gantry
base/mid/end, `wp` wrist pitch, `wr` wrist roll, `gr` gripper.

Service status is polled every 5 s via `systemctl is-active`. Monitored units:
`aizee-motor-control-rover` (`motors`), `aizee-lidar-control` (`lidar`),
`aizee-ups-monitor` (`ups`), `aizee-display` (`disp`), `aizee-gripper-cam`
(`grip`), `aizee-scene-cam` (`scene`).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Display stuck waiting for data | Service not running, `/dev/tufty_display` missing (udev rule not installed), or `ltr` not in `dialout`. Check `systemctl status aizee-display` and `journalctl -u aizee-display -n 30`. |
| Data stops after working | Lost serial link (USB unplug/replug). Run `./scripts/reset_display.sh`. |
| `mpremote: command not found` during deploy | `ssh … "pip install mpremote"` on the Jetson. |
| Service boxes all `???` | `systemctl is-active` failed in the node. Check `journalctl -u aizee-display -n 50`. |

## Related files

| File | Purpose |
|---|---|
| `firmware/tufty2040/main.py` | MicroPython firmware — renders the dashboard |
| `python/nodes/display_node.py` | Jetson bridge — ZMQ → serial |
| `config/hardware_jetson_rover.yaml` | `display:` config + ZMQ endpoints |
| `config/systemd/aizee-display.service` | systemd unit (device-bound) |
| `config/udev/99-aizee-display.rules` | udev rule for `/dev/tufty_display` |
| `scripts/deploy_tufty2040.sh` | Flash firmware remotely via the Jetson |
| `scripts/reset_display.sh` | Restart `aizee-display` |
