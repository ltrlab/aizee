# Guarded teleoperation: current implementation

Updated 2026-09-24. **Physical dual-arm teleoperation is not enabled.** The shared
input, target mapping, collision-envelope compiler and independent Rust receiver
are implemented. Hardware reference, loaded stopping, operator engagement and
the input network still need qualification.

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

Both installed boards report that no joystick is present. An operator engagement
control still needs wiring into the runner. The operator confirmed the existing
GUI workflow (2026-09-26 UTC): Idle, Enable, Engage/Align, and Shutdown with a slow
return to zero. Preserve those actions; do not replace them with a new Start/Stop
workflow or require hold-to-run. Alignment and shutdown motion must also pass
the collision/freshness gate. Input loss or collision rejection stops motion;
neither is an instruction to drive blindly to zero. No engagement is inferred
from an absent joystick's default values. See the
[GUI integration map](architecture/TELEOP_GUI_INTEGRATION.md).

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

On Minerva's production computer, a 1,000-cycle 25 Hz run using the real CAD
keep-out compiler and independently scheduled 400 Hz Rust receiver over **vcan
with synthetic motors** accepted all cycles: median 14.595 ms, p99 14.733 ms,
maximum 17.227 ms; zero scheduled misses. Stopping after a silent collision
producer was verified. Receiver: 16,075 ticks, zero overruns/misses, maximum work
453 microseconds and lateness 386 microseconds. Physical CAN links stayed down.

These results exclude real leader transport, cameras, physical motor dynamics
and loaded stopping. The Windows native-harness test correctly stopped on a
scheduler stall after 416 accepted cycles. Direct dual-USB input completed
1,500 cycles without protocol errors but had one 40 ms scheduled miss. The
read-only SSH input tests failed their 25 ms reply timeout (after one and then
twelve cycles); the network is **not qualified**. A native OpenSSH comparison
could not start because Windows rejected existing private-key file ACLs; no key
contents or permissions were changed. Do not increase deadlines to conceal
these results. Earlier low-resolution-clock timings are retained but are not
precise performance evidence.

Tests: 512 Python unit/contract tests and 47 portable Rust tests passed;
Rust Clippy passed with warnings denied. Linux ARM64 binaries cross-compiled.
See the [device log](devices/MINERVA_PRODUCTION_01.md) and
[compact evidence](devices/MINERVA_TELEOP_20260924.json).

## Rehearse in Helm

[Helm's Teleop tab](HELM.md#teleop-virtual-receiver) drives this lifecycle from
the real leaders into the native receiver harness, with receiver limits derived
from the hand-swept joint limits and each leader channel's
[hand-swept range](HELM.md#sweep-leader-ranges) mapped onto its robot joint's
range. Collision checks there are advisory, and only for the virtual receiver.

## Reproduce without moving motors

```powershell
python -m robot_platform.teleop.openrb
python -m robot_platform.teleop.leader_bridge --profile config/devices/minerva-leaders-01.json --seconds 60
python scripts/benchmark_guarded_teleop.py --model urdf/minerva/model.json --output simulation_runs/new-teleop.json --cycles 1000
```

The bridge binds only loopback. The Linux benchmark
`scripts/benchmark_guarded_teleop_vcan.py` verifies virtual CAN interfaces and
uses an explicit synthetic motor/CAD identity map. It must not be repurposed as
a physical motor profile. The isolated deployment stage is documented in the
device log; it is not an installed production service.

## Before live teleoperation

1. Completed: all 14 motors received the requested stopped mechanical zero and
   one save attempt each. All passed raw-position readback after a physical
   power cycle, including five with absent save responses. Future startup
   reference checks remain mandatory. See [reference evidence](devices/MINERVA_MECHANICAL_ZERO_20260924.json).
2. Verify CAD directions, reference uncertainty and usable intervals, including
   gripper geometry. Any mechanical-zero change invalidates the old gripper fit's
   encoder offset; retain its raw measurements but do not apply that offset live.
3. Qualify loaded hold, controlled stop and command-loss behavior for each arm.
4. Resolve input-network jitter, provide operator engagement, then test real
   leaders + collision compiler + virtual motors together under deployment load.
5. Run attended bounded arm tests, then sustained 25 Hz full-load acceptance.
   Advance to 50/100 Hz only after timing and stopping evidence support it.
