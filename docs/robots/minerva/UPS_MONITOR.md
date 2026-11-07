# Feature task: Minerva UPS monitor and low-battery shutdown
- **Base commit:** `447c051` (`claude/gripper-force-feedback`: the deployed release `0f96a61` plus its record).
  displays easily for the operator and also shutdowns the jetson gracefully when its low."
- **Problem:**
  - On 2026-10-21 at about 02:30Z the Jetson lost power mid-evening: its UPS ran flat while unplugged. Nothing warned
    the operator, and the Jetson was cut off rather than shut down.
  - The journal is not persistent, so the last minutes before the cut are gone.
  - AIZEE's old `aizee-ups-monitor` service still runs from the old `~/aizee` checkout. It only publishes readings over
    ZMQ (`tcp://*:5562`) for a status display that is no longer fitted, and it never shuts anything down.
- **Hardware** (the UPS AIZEE used, per the user: a Waveshare Jetson UPS; `docs/robots/aizee/UPS.md`):
  - Waveshare UPS Power Module (C), a 3S lithium pack, read through an INA219 on I2C bus 7 at `0x41` (header pins 3/5).
  - Shunt 0.01 Ω (AIZEE's calibration constants assume it). The robot's user is in the `i2c` group.
  - **Measured 2026-10-21, read-only:**
    - About 5 minutes after the power came back, the pack read 10.43 V with +13.8 mV across the shunt, so +1.38 A.
    - Five minutes later it read 10.49 V with the same current. A rising voltage means the pack was charging, so a
      **positive shunt voltage means charging** on this module. AIZEE's driver documentation said the opposite.
    - The charger ran the Jetson and charged the pack at the same time. On battery the current reverses, about −1.4 A at ~15 W.
  - Combined write-then-read I2C transactions (`I2C_RDWR`) work on this controller. They keep each register read
    atomic even when another reader shares the chip.
- **Design:**
  - **Robot service** (`robot_platform/ops/ups.py`, run as the system service `minerva-ups`):
    - **Self-contained:** standard library only, so the installed copy runs without a release.
    - **Reading:** once a second it averages five quick readings of the bus and shunt registers. It never writes the
      chip's configuration.
    - **Charge estimate:** the pack's resting voltage is the measured voltage minus current × pack resistance
      (0.2 Ω), per cell, read through a lithium-ion open-circuit table. It is averaged over 20 s and shown as "about".
      The window holds only the current source's readings and restarts when the power source switches. With a wrong
      resistance, the estimate errs low while charging and high on battery, so a window spanning an unplug could call
      a healthy pack critical.
    - **On battery** means the pack discharging by at least 0.1 A for 3 s. Power counts as back after 5 s of charging
      or idle current.
    - **Low:** 25 % or less, on battery. The warning starts here.
    - **Critical:** 10 % or less, or a loaded cell voltage at or under 3.35 V averaged over 5 s, on battery. A
      shutdown starts here. The charge estimate needs 10 s of on-battery readings first (`settle_s`), and the cell
      floor needs its full 5 s window of them.
    - **Shutdown, in order:**
      1. It announces the shutdown in the state file. Helm stops any recording, then ends the session, lowering the arms first.
      2. It waits at least 30 s, and until no Helm session's loop (`robot_platform.helm.edge`) runs on the robot.
      3. If a loop still runs at 60 s, it sends that process alone SIGTERM: the loop lowers the arm and closes as if
         Helm had gone.
      4. At 90 s at the latest, or as soon as the loops have ended after the first 30 s, it syncs and runs
         `systemctl poweroff`.
      5. Charging for 5 s cancels the shutdown at any point before power-off.
    - **State file** `/run/minerva-ups/state.json` (tmpfs): one JSON object, rewritten atomically each second.
    - **Event log** `/var/log/minerva-ups/events.jsonl` (persistent): starts, battery and power-back transitions with the
      energy used on battery, level changes, the shutdown steps, sensor errors, and a sample a minute on battery.
      That gives a record of what happened, and data to measure the pack's real capacity later.
    - **Scheduling:** CPUs 0–3 at nice 10, clear of the control CPUs 4–5 (`jetson-cpu-split`).
    - **Session scan:** it looks for Helm session loops in /proc only when a shutdown is near or under way. One scan of
      about 300 processes costs ~14 ms of CPU; every second, that was 80 % of the service's 1.7 % of a core.
    - **Install** (the user runs it with sudo, once, from the active release, like `ops.cpu_partition`):
      - It copies the module and `config/devices/minerva-ups.json` to `/usr/local/lib/minerva-ups/` as root-owned files,
        because a root service must not run files the robot's user can change.
      - It writes the unit, stops and disables `aizee-ups-monitor`, starts `minerva-ups`, and waits for a fresh reading.
      - `remove` undoes it; `status` needs no sudo.
  - **Robot loop** (`helm/edge.py`):
    - SIGTERM ends the session like a closed channel, so an enabled arm is lowered before the receiver closes.
    - The handler only writes a byte to a pipe; a thread ends the inbox. No lock is taken in signal context.
    - A start is refused while a UPS shutdown is announced. This guards against an older Helm too.
  - **Helm** (`helm/ups.py`, `static/ups.js`):
    - **Reading:** a watcher keeps one pinned SSH command open on the robot, which prints the state file and its age
      every 2 s. The age is measured by the robot's own clock, never compared with this PC's.
    - **Pill:** a battery pill in the header on every tab. Green when charging or full, amber on battery, red when low
      or shutting down, grey when there is no reading.
    - **Banner:** shown on battery and during a shutdown, with the charge, the estimated time left and what to do.
    - **Starts refused:** teleop, recording, the servo sweep or return, Minerva Check's live sweep and the gripper
      zero, while critical or shutting down.
    - **Automatic stop:** when a shutdown is announced, Helm stops the recording, then the teleop session and any sweep,
      through their normal stop paths.
- **Shared interfaces held fixed:** the motion contract (the stop paths are the existing ones), the receiver, the
  recording format, and the deploy process. `minerva-ups` is a new system service.
- **Acceptance:**
  - `python scripts/dev.py check --offline`, zero skips.
  - On the robot, with the user:
    1. A dry-run service reads the live pack, and Helm shows it.
    2. Unplugging the charger shows "On battery" within about 5 s, and plugging it back in clears it.
    3. A dry-run with a raised threshold walks the whole shutdown without powering off.
    4. After the user's sudo install, the service runs and `aizee-ups-monitor` is off.
    5. Optional, with the user's say-so: a real low-battery shutdown on a raised threshold.
- **Results:**
  - **Tests:**
    - `tests/unit/test_ups.py` (19): register decoding against the live reading, the charging sign, the lithium-ion curve,
      profile bounds, the 3 s and 5 s debounce, low hysteresis, the cell floor, every shutdown path, sensor errors, the
      service writing and logging, dry runs, the start refusal, install, remove, and the installed copy running alone.
    - `tests/unit/test_helm_ups.py` (6): the robot-side watch script, the watcher's states and reconnects, refusals, one
      action per announced shutdown, and the server refusing starts and stopping a recording, the session and a sweep.
    - `tests/unit/test_helm_edge.py` (+2): SIGTERM ends the session like a closed channel; no start during a shutdown.
    - `tests/ui/test_helm_ups.mjs`: the pill and banner wording for every state.
  - **Gate:** `scripts/dev.py check --offline` passed on `2e303c6`: 1218 tests, none skipped
    (`build/dev-check/20261021T031123Z-c7067092/report.json`).
  - **Rehearsed on the Jetson** before any release, from `/dev/shm`, exactly as installed (`python3 -I -B`):
    - `read` gave 10.832 V, +1.386 A (charging), 3.611 V a cell, about 4 %. The pack had charged for about 40 minutes
      after running flat.
    - A 20 s dry run on CPUs 0–3 at nice 10 used 0.44 s of CPU, start-up included. It wrote the state file and logged
      `started` and `on_power`.
    - The Jetson's own rails read VDD_IN 6.26 W at 5 V. With the carrier, cameras, CAN adapters and NVMe, the UPS
      should see roughly 11–16 W, about 1.0–1.5 A, on battery. That is the check on the 0.01 Ω shunt.
  - **Local Helm** (`--no-live`, 1280 × 720 like the terminal):
    - The first pill, "UPS offline" at 110 px, left the header no room, and longer wording clipped the Connect button.
    - The pill is now a battery glyph with a few characters, 60–77 px, and the header no longer overflows.
    - Injected states rendered charging (green, with a bolt), on battery (amber, with the banner) and shutting down (red,
      with a countdown and banner).
  - **Deployed 2026-10-21 at about 03:25Z** with the user's approval ("yes, deploy it once the gate passes"), by the process
    in `build/terminal-deploy-20261017/`:
    - Jetson release `20261021T032340Z-2e303c6` (previous `20261021T023352Z-0f96a61`).
    - Terminal console `helm-recording-2e303c6be5` (previous `helm-recording-0f96a6131d`); receiver and task restored.
    - Helm read the robot over SSH and reported `missing` (no service yet) until the user's install.
  - **Installed by the user** (03:28Z, through the terminal hop):
    - `minerva-ups` became active and enabled, and `aizee-ups-monitor` was stopped and disabled. Its port 5562 is closed.
    - The service runs as root at nice 10 on CPUs 0–3, with root-owned files; its first reading was 11.104 V, +1.386 A.
    - Helm on the terminal showed a green battery with a charging bolt at "5%", and the tooltip gave the reading.
  - **Found before the unplug test, fixed on this branch:**
    - The 20 s charge window spanned the switch to battery. Unplugging at "5%" (the charging estimate) would very likely
      have started a shutdown at once, even with the pack above the shutdown level on battery. The window now restarts
      per source and a charge-based shutdown waits `settle_s`, 10 s.
    - The once-a-second /proc scan cost 1.7 % of a core; it now runs only near a shutdown.
    - Needs a Jetson release and the user's install again; Helm is unchanged.
    - Deployed with the force-feedback fix (`3e7f18b`, Jetson `20261021T035909Z-3e7f18b`, terminal `helm-recording-3e7f18b365`).
      The installed service keeps root's older copy until the user runs the install again.
    - **Terminal bit flips:** the first console stage of `3e7f18b` stopped on "Copied model differs".
      - Two files in the running release `helm-recording-2e303c6be5` had changed since their staging check that morning:
        `RB03_BODY.gltf` (one bit, `j` to `k` at byte 3,920,891) and `rb03_output__3.stl`. Size and timestamp were
        unchanged, the SSD reports healthy, and there were no disk events.
      - Both were restored from the verified `helm-recording-0f96a6131d` copies; all 602 model files then matched.
      - The partial stage was renamed `helm-recording-3e7f18b365-failed-stage-20261021T114442Z`.
      - Suspect the terminal's RAM: run Windows Memory Diagnostic.
- **Remaining uncertainty:**
  - The pack's capacity (28 Wh assumed, 3 × 18650) and resistance (0.2 Ω assumed). The time left is shown as "about"
    until a logged discharge measures them.
  - Whether the Jetson powers itself back on when the charger returns after a shutdown. With the UPS still holding it
    up, the dev kit stays off until its power is cycled.
