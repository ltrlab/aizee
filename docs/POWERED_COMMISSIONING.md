# Guarded powered commissioning

## Current result — 2026-09-24

**All 14 Minerva motors communicate; full arm motion remains blocked.** Physical
identity, stopped feedback and parameter reads are recorded. Command-loss tests
have passed on all 14 motors with their currently installed firmware. After the
user updated left j6/gripper to RS00 0.0.3.32 and left j4/j5 to RS02 0.2.3.34,
all four passed their tests. The previously observed j4/j6 failures are resolved
under the tested nominal-zero-gain command-loss condition. No failed or deferred
watchdog results remain for the current firmware. These were individual trials,
not a simultaneous loaded test of both arms.

The guarded CAN adapter now requires fresh watchdog verification before physical
enable. On the production Jetson, a wrong UID was rejected before opening the
command socket; the correct right-gripper UID reached ready and remained disarmed
through periodic watchdog audits. This hardware check did not request network
enable. Adapter enable/refusal paths are covered by deterministic tests.

The rack-and-pinion right gripper now has measured motor-space motion after two
earlier ineffective low-gain jogs. A bounded feedforward ramp detected 0.003068 rad
travel at +0.200375 Nm and -0.206 Nm. The positive-direction trial passed. The
negative-direction trial **failed its post-stop speed bound**: 0.229114 rad/s
triggered the failure, and the final cleanup reply reached 0.254292 rad/s against
a 0.2 rad/s limit. Stop was acknowledged, but Reset does not prove zero velocity.
No bound was relaxed. That immediate-stop failure remains in the historical record.

The subsequent **damped-stop follow-up passed all four trials**, two in each
direction at the same starting region. Peak reported speed was 0.0932-0.1123 rad/s;
maximum travel was 0.003068-0.003452 motor rad and peak reported effort stayed
below 0.215 Nm. Each trial passed the watchdog behavior check, a continuous
settling check with zero gains, post-disable observation and final stop reply.
See the [four-trial evidence](devices/MINERVA_DAMPED_GRIPPER_20260924.json).

The encoder reversed briefly after drive removal, consistent with recoil or
backlash. A subsequent 100-sample stopped capture spanned only 0.000307 rad in
position, while reported velocity varied up to 0.0971 rad/s. This does not
separate mechanical dynamics from velocity-estimate noise. The roughly 0.20 Nm
values describe reaching a small encoder-displacement threshold, not calibrated
jaw force or exact static friction. The four follow-up trials repeat that small
motor-space test; independent jaw travel and controlled return remain unverified.

Final diagnostics: 182/182 replies, all 14 motors in Reset with no fault bits.
Both CAN interfaces were returned down; legacy motor services remain disabled.
See the [device log](devices/MINERVA_PRODUCTION_01.md) and
[motor evidence](devices/MINERVA_MOTOR_COMMISSIONING_20260924.json) for identities,
firmware, artifact hashes, measurements and raw-report locations. The
[RS00 follow-up](devices/MINERVA_RS00_FIRMWARE_RETEST_20260924.json) and latest
[RS02 follow-up](devices/MINERVA_RS02_FIRMWARE_RETEST_20260924.json) preserve both
retests. The [startup/gripper evidence](devices/MINERVA_WATCHDOG_GATE_GRIPPER_20260924.json)
and latest [damped-stop evidence](devices/MINERVA_DAMPED_GRIPPER_20260924.json)
contain raw trials, stopped telemetry and artifact hashes. At final inspection,
left j4/j5 and the right gripper read watchdog 600; the other eleven read zero.
Historical passes do not mean protection is currently configured.

Target: `ltr@192.168.8.180`, asset `minerva-production-01`, module serial
`1421623029971`. Check the pinned SSH key, module serial, machine ID and adapter
serials. The user retired the ESP-NOW/serial stop design. No programmatic power
cutoff exists. The stand tolerates arms falling on torque loss, as confirmed by
the user. Software stop/fault handling remains; powered trials must be attended.

## Tools and boundaries

The single Rust workspace contains `robot_control`; the shared Python package
provides its client and commissioning tools. Robot roster, protocol ranges,
transmission mapping and interface identities are configuration data.

| Tool / mode | Allowed physical effect |
|---|---|
| `guarded_controller`: `virtual_can` | Synthetic motion on kernel-verified vcan only |
| `observe` | Receive only; all CAN writes, including disable, blocked |
| `parameter_probe` | Explicit floating-point position, velocity and voltage reads |
| `motor_diagnostics` | MCU UID, firmware, stop feedback and whitelisted typed parameter reads; no enable |
| `zero_torque_trial` service mode | Stopped watchdog setup before socket readiness; explicit single-actuator nominal zero feedforward/P/D enable, at most 250 ms; no move commands |
| `motor_watchdog_trial` executable | Pinned single-actuator watchdog write/readback and command-loss experiment; optional separately authorized bounded jog or breakaway ramp |

Physical service trials require `--allow-powered-trial` and a finite duration
no longer than 30 seconds. The service rejects physical `move` requests; its
`bounded_jog_trial` and `breakaway_trial` profiles cannot arm through the network
service. The finite standalone executable is the only physical movement path. The GUI
and legacy publishers are not connected to it.

The receiver binds to loopback; use SSH forwarding for remote clients. Exclusive
interface locks prevent simultaneous guarded owners. Legacy services must stay
stopped because they do not share that lock. No tool auto-arms, clears faults,
resets motor zero, flashes firmware or saves configuration to flash.

## Mandatory watchdog startup gate

Physical `CanHardware::enable()` requires a fresh, single-use proof. Kernel-verified
vcan has an explicit bypass; the network cannot choose that bypass for physical CAN.
The `zero_torque_trial` service profile must contain one `watchdog_identities`
entry per motor (MCU UID and qualified firmware version).

Startup checks UID, firmware, stopped fault-free feedback, existing run mode 0,
volatile `0x7028 = 600` write/readback, and a neutral frame that must leave the
motor in Reset. Only then does it open the command socket. Read/stop/run-mode/
neutral audits refresh readiness while disarmed, without rewriting the watchdog.
Replies have a 40 ms budget and proof must be younger than 40 ms at enable.
Wrong identity, changed readback, missing replies, CAN errors or shutdown revoke
readiness. Failed startup attempts best-effort stop only identified motors.

An enable consumes proof. Disable/fault requires a new service process and fresh
startup verification; network reset alone cannot authorize another enable.
Each finite experiment separately repeats the behavior trial and reads back the
watchdog immediately before each enable. No startup procedure can eliminate the
race with a power reset occurring after the final reply. Enabled-mode feedback
loss/unexpected Reset still faults the receiver.

Firmware behavior qualification remains a separate commissioning requirement:
matching UID/version and register readback cannot prove autonomous stopping.
All 14 installed motors have that nominal-zero-gain qualification; loaded stops
and full production movement remain unqualified. Old zero-torque profiles without
startup pins are rejected and must be regenerated.

## Watchdog experiment

1. Match MCU UID and firmware to diagnostics. Require stopped, fault-free feedback,
   acceptable temperature/speed and existing motion-control mode 0.
2. Read `0x7028`, write **600 ticks in RAM**, and verify readback.
3. Prove a zero-gain neutral control frame does not enable a stopped motor.
4. Enable with nominal zero feedforward/P/D and confirm Run.
5. Send nothing for 150 ms, then use the previously checked neutral frame to
   query status. Require Reset **before** transmitting the final stop.
6. Attempt stop on every exit after identity verification, record the response,
   terminate the finite process and return the interface down.

Passing means Reset was observed after the tested silence interval. It does not
measure exact watchdog latency or prove host-crash, disconnected-CAN, loaded
braking or electrical isolation behavior. Readback alone is insufficient: the
initial failures accepted 600 but remained in Run. At the latest inspection,
left j4, left j5 and right gripper read 600; the other eleven read 0.
No setting was saved to flash by the trial tools.
Configure and verify protection again before enabling a motor after reset/power
cycling. Signed wire quantization makes requested zero torque nominal.

The optional jog first repeats this watchdog test successfully. It uses motor
coordinates, not uncalibrated CAD coordinates: 0.6 s outward ramp, 0.2 s dwell,
0.6 s return, 0.2 s dwell, at most 0.03 rad requested travel. Current fixed gains
are Kp 2.0 Nm/rad, Kd 0.05 Nm·s/rad and zero feedforward. Stops trigger on reported
effort above 0.08 Nm, measured speed above 0.2 rad/s, displacement above 0.04 rad,
excessive command gaps, lost feedback or motor faults. Predicted PD effort is
also checked before sending. These are software checks on motor telemetry,
**not a hardware torque clamp or independently measured contact force**. Stored
motor torque/current limits were not changed.

## Bounded gripper breakaway experiment

`--breakaway-direction -1|1` prepares a profile tied to the freshly captured
motor position. Execution requires `--allow-breakaway`; it cannot be combined
with a jog. Starting feedback must be within 0.02 rad of that prepared position,
stationary under the test threshold, fault-free and low effort.

After a successful command-loss test, feedforward ramps at 0.125 Nm/s, Kp 0 and
Kd 0.05 Nm·s/rad, for at most two seconds / 0.25 Nm. Drive stops as soon as
directed displacement reaches 0.003 rad. Bounds are 0.02 rad displacement,
0.2 rad/s reported speed, 0.35 Nm reported effort and 20 ms command gaps.
Opposite motion, lost feedback, no movement by the cap or any bound violation
fails the trial. Stopped feedback is sampled briefly to capture recoil; the
final stop reply also contributes to maxima and pass criteria. No automatic
return, stronger retry or parameter/flash limit change is performed.

The opt-in `--damped-stop` profile setting replaces immediate disable after
movement with four bounded phases:

| Phase | Duration | Command |
|---|---|---|
| Release drive | 100 ms | Smoothly taper feedforward to zero; Kp 0, Kd 0.2 Nm·s/rad |
| Damping | 200 ms | Zero feedforward, Kp 0, Kd 0.2 |
| Fade damping | 200 ms | Zero feedforward and Kp; smoothly taper Kd to zero |
| Zero gains | 150 ms | Zero feedforward/P/D; verify settling before disable |

Damping and zero-gain phases each require at least 100 continuous ms / 10 samples
within a 0.0008 rad position span and 0.1 rad/s reported speed. A failed settling
check ends the trial; time budgets are never extended to obtain a pass. Predicted
feedforward-plus-damping effort must stay within 0.25 Nm. The existing measured
0.35 Nm / 0.2 rad/s / 0.02 rad envelopes remain active in every phase and after
disable. Fault exits bypass damping and attempt immediate stop. After successful
settling, twenty stopped samples monitor recoil, followed by the final stop reply.
The total experiment remains limited to four seconds and 2,048 raw CAN events.

Profiles without a `breakaway.stop` field retain historical `immediate` behavior;
new damped trials explicitly serialize `"stop": "damped"`. The four hardware
trials completed the active stop phases in 656-662 ms. They do not qualify this
stop for gravity-loaded joints or other mechanisms.

These bounds monitor telemetry; they are not hardware torque/speed clamps.
Position and effort are motor-side quantities until transmission calibration.

## Build and verify

```powershell
./scripts/test_control_core.ps1
./scripts/build_jetson.ps1 -Offline -Package robot_control
./.venv-platform/Scripts/python.exe -m pytest tests/unit tests/contract -q
```

The cross-build produces `control_harness`, `guarded_controller` and
`motor_watchdog_trial` for AArch64. It does not deploy or operate hardware.
For synthetic Linux tests, create and bring up `vcanGuardL` and `vcanGuardR`, then:

```sh
PYTHONPATH=src python3 scripts/guarded_vcan_bench.py \
  --binary rust/target/release/guarded_controller --cycles 1000 \
  --output simulation_runs/guarded-vcan-new
```

The 25 Hz benchmark measures renewal and acknowledged dispatch with a separate
400 Hz receiver timer. It excludes collision work, cameras, physical CAN
arbitration and actuator dynamics; it is not full-loop hardware acceptance.

## Repeatable stopped diagnostics

Generate a fresh profile; this creates a file without connecting:

```powershell
./.venv-platform/Scripts/python.exe -m robot_platform.control.commissioning `
  --device config/devices/minerva-production-01.json --root . `
  --mode motor_diagnostics --output simulation_runs/minerva-diagnostics.json
```

After host/adapter identity, service conflicts, artifact hashes and attended stand
conditions are checked, the established remote sequence is:

```sh
sudo /usr/local/bin/aizee-can-up canL
sudo /usr/local/bin/aizee-can-up canR
./guarded_controller motor_diagnostics.json --allow-powered-trial --seconds 30
# In another terminal while the bounded service runs:
PYTHONPATH=src python3 -m robot_platform.control.motor_diagnostics \
  --profile motor_diagnostics.json --output diagnostics-new.json
# After the service exits, including on an unsuccessful trial:
sudo ip link set canL down
sudo ip link set canR down
```

The diagnostic client checks receiver identity, epoch and roster, matches replies
by bus/motor/host/request/timestamp, and preserves raw bytes. `complete` describes
communication completeness; inspect fault bits and stopped status separately.

The watchdog-profile generator takes the diagnostic client's JSON output:

```powershell
./.venv-platform/Scripts/python.exe -m robot_platform.control.watchdog_trial `
  --device config/devices/minerva-production-01.json --diagnostics diagnostics.json `
  --motor right_gripper --output simulation_runs/right-gripper-watchdog-new.json
```

It prepares a UID/firmware-pinned profile only. Execution requires the standalone
binary and `--allow-zero-torque-trial`. A separately prepared `--jog-radians`
profile requires `--allow-bounded-jog` and physical clearance. Do not repeatedly
rerun a known failed motor to obtain a passing report.

For the separate damped profile, add `--breakaway-direction 1` (or `-1`),
`--damped-stop`, and
use a new output filename. Run it with `motor_watchdog_trial PROFILE.json
--allow-breakaway` only under the established attended commissioning conditions.
For network zero-torque startup, extract the generated profile's `service`
object into its own JSON file; the standalone tool expects the full wrapper.

## Remaining gates

The [offline transmission calibration workflow](GRIPPER_TRANSMISSION_CALIBRATION.md)
pairs stopped diagnostics with manually measured jaw gaps and checks both
approach directions. It produces an interpolation candidate without installing
live settings or enabling motors.

1. Use the [manual encoder/caliper measurements](devices/calibration/minerva-right-gripper-20260924/README.md)
   as an approximate unloaded transmission estimate (18.4104 mm/rad total
   aperture). The first 1 mm fit failed; a separate coarse candidate uses a
   provisional 2 mm criterion after the user confirmed finger flex and caliper
   variation. Powered jaw accuracy and motion-versus-compliance still need
   confirmation. Current motor position is outside the candidate's shared
   interpolation interval; do not extrapolate the absolute gap estimate.
2. Establish useful jaw motion and controlled return in both directions, then map
   travel and force. Current repeatability evidence is two small encoder tests
   per direction at one pose, not a calibrated open/close cycle.
3. Measure motor-to-CAD sign/zero/transmission and physical limits; validate
   torque interpretation, gains, speeds and stopping bounds under supported load.
4. Collision verdict expiry and joint-box enforcement are implemented in the
   receiver and passed 1,000 production-host vcan cycles including real geometry.
   Qualify physical stopping bounds, real leader transport and same-arm/body
   policy. Preserve finger exclusions. See [guarded teleop](GUARDED_TELEOP.md).
5. Qualify host/CAN loss response and full 25 Hz operation under collision/camera
   load before loaded holding, arm motion, application migration or dual-arm use.

## Protocol references and codec correction

### Firmware update results

The manufacturer's [V26.05.19 release](https://github.com/RobStride/Product_Information/releases/tag/V26.05.19)
provides RS00 `0.0.3.32` and RS02 `0.2.3.34`, matching the corresponding right-arm
versions that passed this command-loss trial. Published asset hashes:

| Manufacturer asset | Published SHA-256 |
|---|---|
| `rs00-0.0.3.32.bin` | `a889f1b027b53fd7b18bb6db5e7df1f97d4edd62d6ff9c8b5665293c1f9ffa80` |
| `rs02-0.2.3.34.bin` | `767e427cd1f952fd4e0a30427fb7d9bf8af2d4f2a0d183c03fc0aa04b8be97bd` |

The user installed RS00 0.0.3.32 on left j6/gripper and RS02 0.2.3.34 on left
j4/j5. Their original UIDs/CAN IDs and new versions were read back; all four
passed the bounded test. The source assets used by the user were not independently
identified. A version match alone is not proof of a repair; the behavior was
retested. Left j6 position changed from 6.166641 rad
in the earlier capture to near zero after maintenance; motor-to-CAD calibration
remains unverified, and old pose targets must not be replayed.
The manufacturer has changed parameter semantics across releases: the
[V25.02.27 notes](https://github.com/RobStride/Product_Information/releases/tag/V25.02.27)
describe an RS02 torque-feedback correction and RS03/RS04 mechanical-zero
recalibration after that update. A blanket fleet update is not part of this trial.

### Wire ranges

User-confirmed labels on both arms, IDs 4..10:
**RS03, RS04, RS03, RS02, RS02, RS00, RS00**. Guarded commissioning uses current
manufacturer wire ranges, including RS00 ±14 Nm / ±33 rad/s / Kp 500 and RS03
±20 rad/s. Encoding ranges are not approved operating limits. The disabled legacy
motor controller still has older ranges; do not enable it.

Primary references: RobStride's
[product manuals](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature),
[RS00 manual](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS00/RS00User%20Manual260713.pdf),
[reference implementation](https://github.com/RobStride/RobStrideSPC), and Linux
[SocketCAN](https://docs.kernel.org/networking/can.html) /
[timestamping](https://docs.kernel.org/networking/timestamping.html) documentation.
Kernel arrival timestamps preserve reception age, not motor sampling time.
