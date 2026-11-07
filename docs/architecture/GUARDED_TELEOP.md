# Guarded teleoperation: current implementation

Guarded teleoperation is how Helm drives Minerva's arms from the GELLO leaders: the leaders' latest sample goes through
a mapping and a supervisor, and the Rust receiver (`rust/control_core`) decides what the motors may do.

- **Status.** First powered teleop of one arm ran on 2026-09-27 and both arms on 2026-10-09; the head joined later. The
  production profile is the direct one (`config/devices/minerva-teleop-direct-tuning.json`: 100 Hz, collision checking
  off). The collision-gated path described below is the guarded profile (`minerva-teleop.json`, 25/50 Hz), used in
  rehearsal and the viewer; see [COLLISION_RUNTIME_IMPLEMENTATION.md](COLLISION_RUNTIME_IMPLEMENTATION.md).
- The loop runs on Minerva's Jetson (`helm/edge.py`), with leader samples over the authenticated UDP link from the
  operator station's GELLO bridge ([GELLO_TELEOP.md](GELLO_TELEOP.md)).
- Loaded stopping and host and CAN loss response are still attended-test matters; the arms hold after a joint-speed
  trip (`_trip_hold_due` in `helm/teleop.py`; merged, not yet on the robot).

## Components and ownership

| Component | Responsibility |
|---|---|
| `teleop/openrb.py` | Read-only input, USB serial identity, full seven-channel source validity and age |
| `teleop/leader_viewer.py` | Configurable leader schematics, separate display calibration, and same-owner raw input snapshots |
| `teleop/leader_bridge.py` | Latest-source cache and loopback input server, nonce/epoch validation over a pinned SSH tunnel |
| `teleop/leader_mapping.py` | Profile-provided routes, explicit relative registration, speed/interval constraints |
| `teleop/supervisor.py` | Whole-roster 25 Hz proposals, collision certification, lease and error stop |
| `control/collision_envelope.py` | Converts a motor target and receiver reserve into one conservative CAD joint box |
| `rust/control_core/src/collision.rs` | Receiver-local envelope validation and expiry independent of producer traffic |
| `rust/control_core/src/reference.rs` | Single-use raw encoder reference check before physical adapter enable |
| `control/mechanical_zero.py` | Explicit, stopped-only mechanical-zero and optional parameter-save maintenance |
| `teleop/leader_link.py` | Authenticated UDP leader samples over the tether; the PC end in its own responsive process |
| `helm/edge.py` | Helm's teleop loop on the robot computer, driven over one pinned SSH channel |
| `helm/teleop.py` | The session lifecycle (Idle, Enable, Engage/Align, Hold, Shutdown, Disable) and the control cycle |
| `teleop/direct_lifecycle.py`, `teleop/guarded_lifecycle.py` | The direct (collision-free) and the collision-gated lifecycles |
| `control/live_receiver.py` | A live session's stopped pre-scan, CAN link and `teleop` service for one arm or both |

Paths above Python modules are relative to `src/robot_platform/`. Robot joint
names, leader roles, USB identity, routes and CAD mapping belong in profiles;
the input driver has no Minerva-specific motor mapping. Current leader identities
are in `config/devices/minerva-leaders-01.json`. COM names may change.

## Collision command contract

The receiver issues an offer **before** geometry work begins. A verdict binds
that single-use offer, model/calibration/policy identity, world revision, scope,
and ordered lower/upper joint bounds. It expires from offer issuance, not message
arrival. Renewing a lease cannot extend a previous target or collision verdict.
The independent receiver initiates its stop if either expires. Bounds remain
active during braking and stopped hold. This is a trusted-local-producer contract,
not authentication of an arbitrary network collision service.

Per-joint reserved travel is tracking allowance + maximum speed times
`(verdict lifetime + feedback timeout + reaction time)` + `speed²/(2*stop acceleration)`.
The compiler adds possible progress while computing. Its single midpoint query
with conservative per-pair travel bounds covers the entire coordinate box,
including asynchronous joint motion. Every returned candidate pair is checked;
truncated results are rejected. A distant moving link no longer enlarges the
bound of an unrelated stationary pair. Targets use fixed holding gains and zero
velocity/effort feedforward. Physical speed, tracking and stopping bounds remain
unqualified assumptions until measured.

Minerva uses the requested coarse inter-arm keep-out boundaries. All pairs
involving the four sliding finger links remain excluded. Same-arm/body and
environment/payload protection are not qualified by these results. CAD viewer
diagnostics are a separate geometry/pair policy.

## Input and reference behavior

OpenRB freshness poll `0xA6` adds a complete-read validity flag, sample counter,
sample time and reply time to the signed encoder ticks. Cached or failed bus
reads cannot acquire a new timestamp. Host input measures conservative age using
board elapsed time plus full round-trip time. The bridge adds network round-trip
time without comparing clocks on different computers. Repeated, restarted,
corrupt or invalid input latches and closes the input; it never automatically
reconnects. A complete, CRC-checked sample that is late, skewed or reports a
partial servo read is skipped instead (`LeaderSampleSkipped`): the USB stream
stays synchronized, nothing from it is published, and the previous sample ages
out for consumers. An engaged session holds its last bounded target for at most
two missed cycles, then drops to Hold at the measured pose; only an explicit
Engage resumes following. Registration pairs current leader and robot positions
to avoid a jump when engaging. Source age is checked again after collision computation.

Both installed boards report that no joystick is present, so engagement is never inferred from joystick defaults.
The lifecycle is Idle, Enable, Engage/Align, and Shutdown with a slow return to the reference, driven from Helm's
keys and pedals. In the guarded profile alignment and shutdown motion also pass the collision and freshness gate.
Input loss or collision rejection stops motion; neither is an instruction to drive blindly to zero. See the
[GUI integration map](../history/plans/TELEOP_GUI_INTEGRATION.md) for the earlier design.

Physical adapter enable now also requires configured raw reference positions,
three or more fresh Reset-state samples spanning at least 100 ms, and the
independently confirmed physical reference pose. Configured tolerances cannot
exceed 0.1 rad; no modulo-turn normalization hides a reference change. A failed
check, read error or disable revokes the proof. The physical zero-torque service
requires `--confirm-reference-pose` as well as its finite trial flags. Old profiles
without reference positions fail validation. Identity/firmware/watchdog checks
remain mandatory. Neutral/current-position initialization is retained.

Encoders alone cannot establish physical pose following an arbitrary zero change.
Until independent reference sensing exists, verify the known pose at every new
physical enable session and after a power cycle, firmware update, reference
change or lost feedback. Never auto-zero during startup or replay old targets.
Dedicated finite watchdog experiments are separate commissioning entry points;
the legacy controller has not been migrated and remains disabled.

## Measured evidence

These numbers are from 2026-09-24 bench runs, before the direct profile, and measured the guarded 25 Hz path:

- On Minerva's Jetson, 1,000 cycles at 25 Hz with the real CAD keep-out compiler and the 400 Hz Rust receiver over
  **vcan with synthetic motors** accepted every cycle: median 14.595 ms, p99 14.733 ms, maximum 17.227 ms, no scheduled
  miss. Stopping after a silent collision producer was verified. Receiver: 16,075 ticks, no overruns, maximum work
  453 microseconds, lateness 386 microseconds.
- They excluded real leader transport, cameras, motor dynamics and loaded stopping.
- An SSH-tunnelled leader input missed its 25 ms reply timeout, which led to the UDP leader link. Hop budgets for the
  current path are in [TELEOP_LATENCY.md](TELEOP_LATENCY.md).

Raw results are kept in the [device log](../devices/MINERVA_PRODUCTION_01.md) and
[compact evidence](../devices/MINERVA_TELEOP_20260924.json).

## Rehearse in Helm

[Helm's Teleop tab](../operator/HELM_MANUAL.md#teleop-virtual-receiver) drives this lifecycle from
the real leaders into the native receiver harness, with receiver limits derived
from the hand-swept joint limits and each leader channel's
[hand-swept range](../operator/HELM_MANUAL.md#sweep-leader-ranges) mapped onto its robot joint's
range. In the guarded profile collision checks there are advisory; the production direct profile has none. Live
teleop on the motors is covered in the [Helm manual](../operator/HELM_MANUAL.md).

## Reproduce without moving motors

```powershell
python -m robot_platform.teleop.openrb
python -m robot_platform.teleop.leader_bridge --profile config/devices/minerva-leaders-01.json --seconds 60
python scripts/benchmark_guarded_teleop.py --model urdf/minerva/model.json --output simulation_runs/new-teleop.json --cycles 1000
```

The leader bridge binds only loopback. `scripts/benchmark_guarded_teleop_vcan.py` verifies virtual CAN interfaces and
uses an explicit synthetic motor and CAD identity map; it must not be repurposed as a physical motor profile.
`scripts/guarded_vcan_bench.py` runs the Rust receiver on Linux vcan.

## Before live teleoperation of a new unit

1. Mechanical zero: stopped zero and a power-cycle readback of every motor
   ([MECHANICAL_REFERENCE](../operator/MECHANICAL_REFERENCE.md)). Startup reference checks stay mandatory.
2. Verify CAD directions, reference uncertainty and usable intervals, including gripper geometry. A mechanical-zero
   change invalidates an older gripper fit's encoder offset.
3. Confirm loaded hold, controlled stop and command-loss behaviour for each arm in attended tests.
4. Rehearse with the real leaders against the virtual receiver, then run attended bounded arm tests.
