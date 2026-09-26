# Minerva Arms Bring-Up (Path A — two instances, dual GELLO)

**2026-09-24 bring-up:** the existing production computer is now reachable as
`ltr@192.168.8.180` (`aizee-rover`). Serial-based CAN naming is installed and both
adapters are verified as `canL`/`canR` after reconnecting the left adapter.
Both legacy motor services are stopped and disabled for boot. The ESP-NOW/serial
stop bridge has been retired at the user's request. The test stand tolerates
support loss; no programmatic power cutoff is available. Use the
[guarded commissioning procedure](POWERED_COMMISSIONING.md) for current tests.
The legacy procedures below are historical and do not use the new guarded service.
See the [production device record](devices/MINERVA_PRODUCTION_01.md).

Bring up the two 6+1-DoF Minerva arms for **calibration + dual-GELLO teleop**, with
no head/lift/URDF/cameras. Each arm is a self-contained motor_control instance on
its own CAN bus and ZMQ port pair; the two arms are identical (motor ids 4..10) and
are told apart only by their bus. The laptop-side tools **auto-detect the Jetson** the
same way collect_demo does — LAN `192.168.0.27` → USB-C direct `10.42.0.1` → WiFi AP
`192.168.50.1` — so `--host` is optional (it just sets which address to try first).

| Arm   | CAN bus | cmd port | telem port | motor_control config / service                |
|-------|---------|----------|------------|-----------------------------------------------|
| left  | `canL`  | 5555     | 5556       | `hardware_minerva_left.yaml`  / `aizee-minerva-left`  |
| right | `canR`  | 5575     | 5576       | `hardware_minerva_right.yaml` / `aizee-minerva-right` |

- **CAN names are udev-pinned by adapter serial** (`config/udev/80-minerva-can.rules`):
  `canL`=`004D002C4730511520303650`, `canR`=`0021002F4D53530920303937`.
  These assignments were confirmed by unplugging the left adapter. Names follow
  the adapters across USB ports; the internal Tegra mttcan is unused. Reconfirm
  the mapping if adapters are replaced or switched between arms.
- **Right ports are 5575/5576** (not 5557–5560 — those are held by `aizee-camera-relay`).
- Joint order per arm (arm-group = swivel idx0 + arm[]):
  `j1(id4) j2(id5) j3(id6) j4(id7) j5(id8) j6(id9) gripper(id10)`. Shoulder id-4 = **RS03**
  (confirmed); ids 5–10 mirror the AIZEE arm.

---

## 0. Deploy (from the dev machine)

```bash
./scripts/deploy_minerva_arms.sh          # builds motor_control, installs both
                                          # services + udev rule + aizee-can-up +
                                          # sudoers; retires the rover service; does
                                          # NOT start (start only with the bus powered)
```
The script installs legacy services **disabled for boot** and does not start them.
It no longer installs the retired stop bridge. Follow the guarded commissioning
procedure above before considering a manual legacy-service start.

## 1. Start the follower instances (Jetson, motors POWERED)

```bash
sudo systemctl start aizee-minerva-left aizee-minerva-right
```
Each runs `aizee-can-up` (brings its udev-named bus up at 1 Mbps — **no** USB unbind/
rebind) then motor_control. A healthy bus shows balanced RX≈TX and `state ERROR-ACTIVE`
with 0 bus-off (`ip -s -d link show canL`).

## 2. Establish the mechanical reference

Use the [stopped mechanical-reference procedure](MECHANICAL_REFERENCE.md),
including pinned identities and an actual power-cycle readback. A save response
alone does not prove persistence. Minerva's 2026-09-24 reference is already
recorded; do not repeat zero writes as part of routine calibration or startup.

## 3. Calibrate motor-to-CAD directions and alignment

```powershell
.\scripts\calibrate_minerva.ps1
```

Move the disabled arms by hand and compare the live CAD viewer at port 8766.
Use terminal `flip`, `align`, and `accept` commands to save candidate corrections.
See [live calibration](JOINT_CALIBRATION.md) for the guided capture alternative,
resume and evidence. The old enable/free-move min/max script is retired.
Physical limits and loaded stopping require separate commissioning; this preview
does not qualify the legacy teleoperation paths documented below.

## 4. Calibrate the two GELLO leaders

Each OpenRB-150 leader needs its own file (servo min/max/zero differ per unit):
```bash
python python/scripts/openrb_calibrate.py --port <left-port>  --output config/openrb_left.json
python python/scripts/openrb_calibrate.py --port <right-port> --output config/openrb_right.json
```

## 5. Teleop — terminal tool OR the GUI collector

Both front-ends share the same controls + both zero functions. The GUI drives the
two instances directly (`collect_minerva_app/follower.py`) and adds 3 cameras + v6
recording.

Terminal (no cameras/recording):
```bash
python python/scripts/minerva_teleop.py --host <jetson-ip> \
    --left-calib config/openrb_left.json --right-calib config/openrb_right.json
```
GUI collector:
```bash
python python/scripts/collect_minerva.py --host <jetson-ip> \
    --left-calib config/openrb_left.json --right-calib config/openrb_right.json
```
First registration: press **Z** (leader-zero) or **M** (mirror) so the leaders map to
the arms, then **E** to track (or **I** to backdrive and read positions). Both default
to `--kp-scale 0.3`; raise once tracking looks right.

---

## Controls (mirrors collect_demo) — IDLE-FIRST

**Sequence, every session: Idle → zero → Enable.** Idle (zero torque) is always the
first action — it reports live actuator state safely. These RobStrides can **lose their
zeros**, so you read state and re-zero (RobStride mech `K` + GELLO `Z`/`M`) **before**
applying any gains. **Enable is gated: it only works from Idle.**

| Key | Action |
|-----|--------|
| `I` | **idle — enable at ZERO torque. ALWAYS FIRST** (read positions, limp/backdrivable) |
| `E` | enable gains → HOLD. **Only from Idle** (after zeroing); blocked from DISABLED |
| `H` | toggle HOLD ↔ TRACKING |
| `Z` | **leader zero** — snapshot each leader pose as its zero; save to calib |
| `M` | **mirror** — set each leader zero so it maps to the arm's current pose; save |
| `K` | **RobStride mechanical zero** + SaveConfig, both arms (disable first) |
| `X` | soft shutdown — ramp both arms to zero, then disable |
| `Q` | quit |

Absolute leader mapping: `target = direction · (leader_rad − zero)` per joint; a
per-step velocity guard bounds every command so engaging never snaps.

## Files

- `config/hardware_minerva_{left,right}.yaml` — follower instance configs
- `config/minerva_calibration.json` — follower min/max (step 3)
- `config/openrb_{left,right}.json` — per-leader calibration (step 4)
- `python/scripts/minerva_calibrate.py` — compatibility entry point for shared stopped live calibration
- `python/scripts/minerva_teleop.py` — terminal dual-GELLO teleop + zero functions
- `python/scripts/collect_minerva.py` (+ `collect_minerva_gui.py`) — GUI collector
- `collect_minerva_app/follower.py` — `DualArmTransport` (two instances → one 17-vec)
- `collect_minerva_app/teleop.py` — `MinervaTeleop` absolute mapping + Z/M zeros

## Integrated into the GUI

The controls + both zeros are wired into the `collect_minerva` GUI (buttons +
E/I/H/T/R/Z/M/K/P/X/Q shortcuts). It keeps the 17-DoF data model with head/lift
(indices 14..16) held at zero until that hardware exists, and drives the two
instances via `DualArmTransport`.

## Idle / telemetry probe (non-interactive)

```bash
python python/scripts/minerva_motor_probe.py --host 10.42.0.1 --arm both   # enable + zero-torque
python python/scripts/minerva_motor_probe.py --host 10.42.0.1 --no-enable  # telemetry only
```
This runs the same sequence as collect_demo's **Idle** (`enable`, then `arm_joints`
with kp=kd=torque=0 — motors energized but limp). RobStride MIT motors are **silent
until enabled**, so `0/7` while passive is normal; the enabled read is the real check.
Both arms validated **7/7 motors running** this way (left clean; right after the wiring
fix below).

## Hardware gotchas (learned on the bench)

- **Two gs_usb (candleLight 1d50:606f) adapters.** Kernel enumeration is order-dependent,
  so we pin names by their unique serials via udev (above). Do **not** unbind/rebind them to recover
  (the old `aizee-reset-usb-can` path) — with two adapters it renames the wrong one and
  can drop an adapter off USB. `aizee-can-up` only does `ip link` up, never a USB reset.
- **Adapter drops off the bus under stress** (gone from `lsusb`, `ip link set … → "No such
  device"`). Software can't revive it — **power-cycle the adapter's physical switch**, or
  unplug/replug both. It re-enumerates on its port and udev re-names it. A rapid service
  restart-loop is what stresses it, so:
  - motor_control now **degrades instead of exiting** on persistent CAN failure (was:
    exit → systemd restart "with USB reset"). A dead/absent bus keeps the process alive
    (slow link-retry) so one arm's trouble can't knock the other's adapter offline.
  - the arm services have `StartLimitBurst=5/60s` so they can't flap.
- **CANH/CANL swapped** on an arm ⇒ **no ACK**: motor_control TX fails (`errno 105
  ENOBUFS`), `RX: 0 packets`, bus-off climbing — while a healthy arm shows balanced
  RX≈TX and 0 errors. That's how we found the right arm was miswired.
- **Latched fault after a CAN fault** (e.g., the miswiring): motors report `state=running`
  but carry a `MotorError` with *all* flags false. **Avoid `clear_fault`** — in testing it
  silenced the arm. Clear it with a **disable→re-enable** cycle or a motor power-cycle.
- **RobStride positions are multi-turn/absolute** (j5/j6 read ~6 rad near power-on); the
  min/max calibration (step 3) captures the usable range.

## Not yet in scope

Head/lift + waist actuators, a Minerva URDF, and the single-endpoint 17-DoF policy
path (`minerva_policy_node` still assumes one endpoint).
