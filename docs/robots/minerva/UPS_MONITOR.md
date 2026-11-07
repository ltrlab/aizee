# Minerva UPS monitor and low-battery shutdown

Minerva's Jetson runs from a UPS. The system service `minerva-ups` reads the battery once a second, shows it in Helm,
and shuts the Jetson down cleanly before the battery runs out, ending any session and lowering the arms first.
Installing it is step 7 of [INSTALL_JETSON.md](../../getting-started/INSTALL_JETSON.md#7-ups-monitor).

It replaced AIZEE's `aizee-ups-monitor`, which only published readings over ZMQ (`tcp://*:5562`) for a status display
and never shut anything down. The install stops and disables that service; port 5562 is closed on Minerva
([AIZEE's UPS page](../aizee/UPS.md) describes the old one).

## Hardware

- A Waveshare UPS Power Module (C), a 3S lithium pack, read through an INA219 on I2C bus 7 at `0x41` (header pins 3
  and 5). The shunt is 0.01 ohm. The robot's user must be in the `i2c` group.
- **A positive shunt voltage means charging** on this module. This was measured read-only: with the charger running
  the Jetson and charging the pack, the pack read 10.43 V with +13.8 mV across the shunt (+1.38 A), then 10.49 V five
  minutes later with the same current. A rising voltage means charging. AIZEE's driver documentation says the
  opposite. On battery the current reverses.
- Combined write-then-read I2C transactions (`I2C_RDWR`) work on this controller. They keep each register read atomic
  even when another reader shares the chip.
- The pack's capacity (28 Wh) and resistance (0.2 ohm) in `config/devices/minerva-ups.json` are **assumed** for three
  18650 cells. The time left is shown as "about" until a logged discharge measures them.

## How the service works

`robot_platform/ops/ups.py` is standard library only, so the installed copy runs without a release.

- **Reading:** once a second it averages five quick readings of the bus and shunt registers. It never writes the
  chip's configuration.
- **Charge estimate:** the pack's resting voltage is the measured voltage minus current times pack resistance, per
  cell, read through a lithium-ion open-circuit table and averaged over 20 s. The window holds only the current
  source's readings and restarts when the power source switches. With a wrong resistance the estimate errs low while
  charging and high on battery, so a window spanning an unplug could call a healthy pack critical.
- **On battery** means the pack discharging by at least 0.1 A for 3 s. Power counts as back after 5 s of charging or
  idle current.
- **Low:** 25 % or less, on battery. The warning starts here.
- **Critical:** 10 % or less, or a loaded cell voltage at or under 3.35 V averaged over 5 s, on battery. A shutdown
  starts here. The charge estimate needs 10 s of on-battery readings first (`settle_s`), and the cell floor needs its
  full 5 s window of them.
- **Shutdown, in order:**
  1. It announces the shutdown in the state file. Helm stops any recording, then ends the session, lowering the arms
     first.
  2. It waits at least 30 s, and until no Helm session's loop (`robot_platform.helm.edge`) runs on the robot.
  3. If a loop still runs at 60 s, it sends that process alone SIGTERM: the loop lowers the arm and closes as if Helm
     had gone.
  4. At 90 s at the latest, or as soon as the loops have ended after the first 30 s, it syncs and runs
     `systemctl poweroff`.
  5. Charging for 5 s cancels the shutdown at any point before power-off.
- **State file** `/run/minerva-ups/state.json` (tmpfs): one JSON object, rewritten atomically each second.
- **Event log** `/var/log/minerva-ups/events.jsonl` (persistent, so it survives the power cut the journal does not):
  starts, battery and power-back transitions with the energy used on battery, level changes, the shutdown steps,
  sensor errors, and a sample a minute on battery. It is a record of what happened and the data for measuring the
  pack's real capacity.
- **Scheduling:** CPUs 0-3 at nice 10, clear of the control CPUs. It looks for Helm session loops in `/proc` only when
  a shutdown is near or under way; one scan costs about 14 ms of CPU, which every second would be most of the
  service's load.
- **Install:** run with sudo from the active release (`ops.ups install`). It copies the module and the profile to
  `/usr/local/lib/minerva-ups/` as root-owned files, because a root service must not run files the robot's user can
  change, writes the unit, stops and disables `aizee-ups-monitor`, starts `minerva-ups` and waits for a fresh reading.
  Editing the profile needs another install. `remove` undoes it; `status` and `read` (one reading) need no sudo, and
  `run --dry-run` rehearses the service with its own state and event files, never signalling a process or powering
  off.

## On the robot's loop and in Helm

- **The loop** (`helm/edge.py`): SIGTERM ends the session like a closed channel, so an enabled arm is lowered before
  the receiver closes. The handler only writes a byte to a pipe; a thread ends the inbox, so no lock is taken in signal
  context. A start is refused while a UPS shutdown is announced, which also guards against an older Helm.
- **Helm** (`helm/ups.py`, `static/ups.js`):
  - A watcher keeps one pinned SSH command open on the robot that prints the state file and its age every 2 s. The age
    is measured by the robot's own clock, never compared with the viewer's.
  - A battery pill sits in the header on every tab: green when charging or full, amber on battery, red when low or
    shutting down, grey with no reading.
  - A banner shows on battery and during a shutdown, with the charge, the estimated time left and what to do.
  - While critical or shutting down Helm refuses to start teleop, recording, the servo sweep or return, Minerva Check's
    live sweep and the gripper zero. When a shutdown is announced it stops the recording, then the teleop session and
    any sweep, through their normal stop paths.

## What has been seen on the robot

- The service was installed on 2026-10-21 and read the pack (11.1 V, +1.39 A, charging); Helm showed the battery pill.
- A dry run on the Jetson cost 0.44 s of CPU over 20 s, start-up included.
- The Jetson's own rails read 6.26 W at 5 V; with the carrier, cameras, CAN adapters and NVMe the UPS should see
  roughly 11-16 W on battery (an estimate, not a measurement). That is the check on the shunt.
- A real low-battery shutdown has not been run, and neither has an unplug test on a pack above the shutdown level.
  Unplugging while the estimate still read a charging value would likely have started a shutdown at once, which the
  per-source window and `settle_s` were added to prevent.

## Not known

- The pack's real capacity and resistance.
- Whether the Jetson powers itself back on when the charger returns after a shutdown. With the UPS still holding it up,
  the dev kit stays off until its power is cycled.

## Tests

`tests/unit/test_ups.py` (register decoding, the charging sign, the open-circuit curve, profile bounds, the debounces,
low hysteresis, the cell floor, every shutdown path, sensor errors, install and remove, dry runs, the installed copy
running alone), `tests/unit/test_helm_ups.py` (the watcher, refusals, one action per announced shutdown),
`tests/unit/test_helm_edge.py` (SIGTERM and the start refusal) and `tests/ui/test_helm_ups.mjs` (the pill and banner
wording). All run in the gate.
