# Live motor-to-CAD calibration

Move the disabled robot by hand and compare it with the live model. A shared
calibration session displays raw motor angles and mapped CAD angles, records
direction/offset corrections, and preserves progress between runs. Robot layout,
motor identity and CAD source come from profiles; the implementation contains no
Minerva joint-name or CAN-ID assumptions.

## Minerva: launch from Windows

From the repository root:

```powershell
.\scripts\calibrate_minerva.ps1
```

For the current Minerva setup, connect both USB-CAN adapters directly to the
Jetson. The external hub path repeatedly lost replies; the direct right-arm
connection passed a [30-minute stopped-read test](devices/MINERVA_DIRECT_USB_20260924.json).
The patched host driver subsequently passed a clean
[60-second dual-arm comparison](devices/MINERVA_CAN_DRIVER_SHORT_PASS_20260926.json).

Open **http://127.0.0.1:8766/**. The terminal stays open for commands; the model
follows hand movement automatically. Motors remain disabled. The two previously
checked shoulder swivel signs are imported: left J1 +1, right J1 -1. Other signs
start at provisional +1 and show `?`; visually checked signs show a checkmark.
All arm angle scales currently assume 1:1. Existing sessions preserve their
operator-confirmed signs; the current Minerva session has all twelve directions
confirmed, including negative right J1, J3 and J4.

The device profile's optional `preview_transmissions` map rotary motor readings
to independent prismatic CAD joints. Minerva's sliders use **0 m = fully closed**;
the second jaw follows the URDF mimic relation automatically. The live table
shows motor angles in radians and slider positions in millimetres. Slider
position is one jaw's travel, not total jaw gap.

Minerva currently uses an **approximate display scale** of 9.205 mm per jaw per
motor radian, derived from the historical right-gripper gap measurements. The
left magnitude is borrowed from the right and has not been measured separately.
Positive motor rotation was observed to open the left gripper; the right uses
the earlier positive-opening observation, pending fresh visual confirmation.
The current fully closed mechanical reference anchors both mappings. The old
fit's absolute intercept and interpolation interval remain invalid after
hardware zeroing and are not reused. These preview mappings do not establish
aperture accuracy, contact force, mechanical limits, or powered-control approval.

Move one joint at a time. Use these terminal commands, with the joint name shown
in the viewer:

```text
flip left_arm_j2
align left_arm_j2 0
accept left_arm_j2
status
q
```

- `flip`: reverse that joint's displayed direction, retaining the same raw zero.
  This marks its direction unchecked again.
- `align JOINT CAD_RADIANS`: declare that the **current physical pose** corresponds
  to the entered CAD angle. This adjusts a software offset only. Use it after
  visually matching a known pose; it is not a motor mechanical-zero command.
- `accept`: record the operator's confirmation that physical movement and the
  displayed direction agree. It does not certify angle scale or physical limits.
- `status`: print all candidate mappings.
- `q`: save and close, receiving confirmation that physical CAN links are down.

Each edit autosaves to
`calibration_runs/minerva-production-01-directions/session.json`, under
`preview_mapping`, using `cad_rad = sign * motor_rad + offset_rad`. Edit history
includes the complete raw pose and sample sequence. Restart the same command to
resume. Wrong CAD/reference/device revisions reject resume. A stale or missing
motor feed freezes the last displayed pose with a prominent warning over the
model and rejects edits. The LIVE badge includes the current frame number and
conservative sample age. A browser watchdog also marks the pose stale if the
HTTP request hangs; a reader exit publishes an explicit FEED STOPPED state.
Do not correct an unexpected encoder-zero change by guessing an offset: use the
[mechanical reference check](MECHANICAL_REFERENCE.md) first.

The session is a **calibration candidate**, with `runtime_approved: false`.
No live-controller config, firmware parameter, hardware zero, or old min/max
calibration is overwritten. These mappings still need angle-scale/pose review,
physical bounds and loaded-stop qualification before teleoperation.

## Other modes

```powershell
# Identity, six stationary samples per motor, and CAN-down cleanup; no movement.
.\scripts\calibrate_minerva.ps1 -Mode Check

# Guided Enter-to-capture positive-direction checks, including return to reference.
.\scripts\calibrate_minerva.ps1 -Mode Directions

# Validate profiles and list steps without opening hardware.
.\scripts\calibrate_minerva.ps1 -Mode Plan
```

The guided mode resumes completed steps and rejects too-small movements,
substantial movement of another joint, non-stationary captures and wrong motor
identity. It records signs only; the demonstration angle is not used to infer a
transmission ratio. Its measured results and the live operator edits have separate
provenance in the session. An offset edit does not redefine the hardware reference
used by guided checks.

The old `python/scripts/minerva_calibrate.py` entry point now forwards to this
shared live workflow. Its former legacy enable/zero-impedance min/max behavior is
retired. It no longer writes `config/minerva_calibration.json`; that legacy file
still has runtime readers and is not an approved input to guarded teleoperation.

## Shared entry point and device behavior

```powershell
python -m pip install -e ".[calibration]"
python -m robot_platform calibrate --profile path/to/device-calibration.json --live --resume
```

The profile supplies a device, immutable mechanical reference, model profile,
SSH fingerprint/key **path**, and motor-to-CAD joint steps. Optional display-only
`preview_transmissions` specify a motor, independent prismatic joint, matched
motor reference, CAD reference in metres, and signed metres per motor radian.
Their scale and runtime approval stay false. They are separate from the rotary
`flip`/`align`/`accept` terminal commands. Use
`config/devices/minerva-joint-calibration.json` as the format example. Never put
private-key contents in a profile. The existing host key, machine/module IDs,
USB adapter serials and complete motor UID/firmware roster are checked. Competing
services/receiver and already-up CAN interfaces prevent startup.

Profile changes invalidate automatic session resume. Preserve the old profile
and session, verify unchanged device/reference/CAD and rotary-step bindings, and
record any deliberate migration before carrying the operator's existing arm
edits into a revised display profile. Never silently discard a binding mismatch.

The client stages two content-hashed Python files on the CAN host. The Linux
worker transmits only identity, stop, firmware and position/run-mode/velocity
reads. Its exchange wrapper rejects enable, control, zero, save and parameter
writes. Live scans use stop + raw position reads for every motor; quiet-window
captures are reserved for the guided workflow. Requests are spaced by 10 ms for
the installed adapters; Minerva's complete scan takes about 280 ms (roughly
3.4 display updates/second including transport). A five-second client heartbeat,
500 ms maximum scan duration and 15-minute remote-reader cap bound the preview.
At a healthy time limit, the local session verifies complete CAN-down cleanup
and starts a fresh reader with the full identity checks. The model briefly shows
RECONNECTING while holding the last pose, then resumes. This scheduled renewal
does not consume or reset the fault-retry budget, and never renews an unknown
cleanup result. Use `q` to end the local session.
Incomplete scans are discarded; three consecutive failed scans close the remote
reader and bring both CAN links down. This stopped-only preview can reconnect
up to three times, **only** after confirmed CAN-down cleanup and previously
verified identities. Each attempt rechecks host/module, adapters, motor UIDs,
firmware and Reset before publishing a fresh pose. The viewer shows RECONNECTING
during the gap and retains the reconnection count afterward. Unknown cleanup,
network disconnects, identity/protocol failures, or exhausted attempts stop the
preview. Every attempt keeps its own cleanup report and raw evidence. This
recovery is not used by the powered controller and does not qualify the faulty
transport for motion.
Each displayed frame answers a unique client request. The full measured request
round trip counts toward its age, so buffered network replies cannot appear
fresh. The viewer marks data stale above 750 ms total age, including HTTP round
trip and elapsed browser time. The longer heartbeat tolerates brief Wi-Fi/desktop
scheduling stalls without accepting their delayed poses. This is a
calibration display, not the 25 Hz motion controller or a collision interlock.

Raw live frames and CAN events are journaled on the device under
`device-maintenance/joint-calibration-<source hash>/captures/`. Exit/failure
restores CAN down and writes a cleanup report; successful client cleanup reports
are also saved in the local session. A missing cleanup acknowledgement is an
error, not a successful exit. After a hard PC crash, verify no calibration process
is running before removing a leftover `session.lock` and resuming.

The calibration viewer is loopback-only and reads a fixed local pose feed. It
tails complete JSONL frames, avoiding Windows file-replacement conflicts and
ignoring an unfinished frame while a write is in progress. It
exposes no endpoint for motor commands or calibration writes. Physical measured
angles are rendered without clamping to provisional CAD sweep ranges, so a wrong
mapping remains visible. The ordinary viewer keeps its existing range checks.
