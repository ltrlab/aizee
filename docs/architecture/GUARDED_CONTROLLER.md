# Guarded controller core and protocol

Implemented 2026-09-24: a robot-independent Rust core, ideal hardware harness,
strict RobStride CAN adapter, and independently scheduled Linux commissioning
service. The service has run on Minerva's production computer using virtual CAN
and physical stopped-motor diagnostics. A separate finite executable has tested
motor-side command loss, two unsuccessful micro-jogs, and right-gripper breakaway
ramps. The initial reverse ramp triggered a post-stop speed bound. An opt-in
damped stop subsequently passed four bounded trials, two in each direction.

**The existing ROBSTRIDE/ZMQ motor service does not use this core yet.** Its
watchdog, enable and fault behavior have not changed. The new session adapter
requires synthetic, loopback-only settings. The new `guarded_controller` entry
point is separate from the legacy service. Physical modes allow observation,
explicit parameter/identity/version reads or a bounded single-actuator nominal
zero-torque trial; physical network movement commands are rejected. The separate
`motor_watchdog_trial` requires a pinned UID/firmware and explicit flags. See
[powered commissioning](../POWERED_COMMISSIONING.md).

## Code ownership

| Path | Responsibility |
|---|---|
| `rust/control_core/` (`robot_control`) | Typed protocol, controller state machine, configuration, hardware trait, stopping trajectory, ideal loopback |
| `rust/control_core/src/bin/control_harness.rs` | Bounded MessagePack frames over stdin/stdout; deterministic receiver clock and fault injection |
| `src/robot_platform/control/guarded.py` | Explicit ownership/renewal client and shared RobotSession batch adapter |
| `src/robot_platform/simulation/control_harness.py` | Native process bridge and source-aged telemetry conversion |
| `tests/contract/test_guarded_controller.py` | Actual Python-to-Rust exchanges for Aizee and Minerva, including partial delivery |
| `scripts/test_control_core.ps1` | Windows native build and Rust tests using the repository toolchain |
| `rust/control_core/src/robstride.rs`, `can_adapter.rs` | Strict frame conversion, configured transmission units, bounded reads/writes and partial-write errors |
| `rust/control_core/src/watchdog.rs` | Nonblocking stopped startup verification, fresh single-use enable proof, periodic disarmed audits and fail-closed invalidation |
| `rust/control_core/src/service.rs`, `service/linux_can.rs` | Monotonic control schedule, kernel reception age, exclusive CAN ownership, acknowledged loopback ZMQ |
| `src/robot_platform/control/guarded_transport.py`, `commissioning.py`, `parameter_probe.py`, `motor_diagnostics.py` | Explicit transport, observation/diagnostic profiles, typed parameters and raw identity/firmware/stop evidence |
| `rust/control_core/src/watchdog_trial.rs`, `src/bin/motor_watchdog_trial.rs` | Finite command-loss experiment, separately enabled micro-jog or breakaway ramp, acknowledged final stop and raw trace |
| `src/robot_platform/control/watchdog_trial.py` | Offline generator of single-motor profiles pinned to captured UID/firmware |
| `scripts/guarded_vcan_bench.py` | Actual Linux service, synthetic motors over vcan, fault/restart tests and 25 Hz command soak |

There is still one Python distribution and one Rust workspace, now containing
five crates. The portable core uses Serde, JSON and MessagePack; the optional
`linux-service` feature adds SocketCAN, ZeroMQ and libc. No
Intrinsic runtime, ROS, Kubernetes, C++ bridge or additional IPC stack is used.

## Receiver-owned freshness

1. Client requests an **offer**. The receiver assigns a single-use ID and a
   deadline in its own monotonic clock, starting immediately at issuance.
2. Client echoes that offer with a session ID, group and pinned model,
   calibration and collision-policy identities. Acquisition never enables.
3. Receiver grants exclusive ownership, retaining the offer's original deadline.
4. Client explicitly arms and sends complete joint batches with boot epoch,
   session, lease ID and increasing sequence numbers.
5. Renewal requires another unexpired offer and the current authority. It
   replaces the lease ID. Old permit IDs and old sequence numbers are rejected.

A packet arriving late cannot obtain a fresh lifetime merely by being received.
Client and receiver monotonic clocks are never compared. The client does not
automatically retry, renew, reset, arm or replay a target.

**Renewing ownership does not renew the last motion command.** A command keeps
the deadline from its original lease. The producer must submit another explicit
command under the new permit. The core checks expiry before processing a request,
and its periodic `tick()` must also run independently when no requests arrive.

Offers are stored in a bounded 16-slot ring; an overwritten offer is rejected.
The protocol is an ownership/freshness mechanism, not network authentication.
Production adapters must generate a unique boot epoch and bound transport work.

## State and hardware contract

- Acquisition starts with hardware disarmed.
- Explicit arm captures current measured positions with configured hold gains.
- Motion validates every joint, revision, dimension, finite value, limit and gain
  before changing any targets. Invalid batches do not partially update targets.
- Lease/command expiry or a normal stop begins a local acceleration-limited stop
  using current measured position and velocity, then supported hold in `stopped`.
- Stopping revokes ownership. A new claim, explicit reset, then explicit arm are
  required before further motion. Reset itself disables; it never enables.
- Emergency stop is accepted independently of ownership and latches a fault.
- Missing, stale, invalid or regressing feedback, motor faults, excessive effort,
  temperature, velocity, position or tracking error latch a fault.
- A hardware read/write failure revokes ownership. Disable is best effort and
  cannot establish that hardware actually stopped.

The `Hardware` trait reads timestamped/sequenced joint samples, enables/disables,
and applies complete joint commands. Profile joint units are radians or metres;
effort is correspondingly torque or force. Samples retain actual source age;
republishing cannot rejuvenate a frozen sample. Hardware adapters must preserve
these semantics and report partial writes as errors.

Joint names, count, units, limits and revision identities are configured, with
no Aizee/Minerva branches. Test limits, gains and braking accelerations are
synthetic fixtures, not approved physical settings.

The stop generator is independent per joint and acceleration limited. It does
not provide jerk limits, synchronized finishing times, gravity compensation or
a physically validated braking model. Motion targets are setpoints, not a new
trajectory planner. The collision gate must ultimately authorize the actual
executed path and stop response, not just the setpoint endpoints.

## Shared session behavior

`GuardedSessionClient` validates complete commands through the existing
`RobotSession`, then sends acknowledged batches to each group's receiver. It
checks any configured collision verdict again before each dispatch. It preserves
the existing collision gate's observe/enforce-simulation distinction.

If a receiver rejects, loses its acknowledgement or becomes unreachable after
another receiver accepted, the adapter attempts emergency stop on **all**
receivers, including the uncertain one. The error records accepted groups and
failed stop attempts. A disconnected receiver must independently expire its
lease; separate endpoints are not an atomic transaction. A failed acquisition
instead releases only claims obtained by this client, leaving another owner's
group alone.

The user confirmed a test stand that tolerates the arms dropping on torque loss
and retired the old ESP-NOW/serial bridge. Software stop is best-effort CAN
disable, not electrical isolation. Reset responses have been captured after stop
requests, but loaded stopping remains unqualified. A host crash cannot guarantee
delivery of a disable frame. All 14 motors have now passed the bounded nominal-
zero-gain command-loss test after the user's left RS00/RS02 firmware updates;
this does not qualify loaded stopping or full host/CAN failure response.

## Run the checks

Windows with the existing repository-local toolchain:

```powershell
./scripts/test_control_core.ps1
./.venv-platform/Scripts/python.exe -m pytest tests/contract/test_guarded_controller.py -q
./scripts/build_jetson.ps1 -Offline -Package robot_control
```

Linux with Rust and the shared Python package installed:

```sh
cargo test --locked -p robot_control
cargo build --locked -p robot_control --bin control_harness
python -m pytest tests/contract/test_guarded_controller.py -q
```

Python integration tests explicitly skip if the native harness has not been
built. A skip is not evidence of protocol validation. Use a freshly built binary.

The harness takes a MessagePack setup frame containing `config` and
`initial_positions`, then request/tick frames. Each frame has a little-endian
32-bit length and a 256 KiB limit. Request frames carry `protocol_version: 1`,
`now_ms`, optional `request`, and optional fault injection. Replies include the
controller status, actual sampled feedback and accepted/rejected decisions.
It can also take a setup JSON path as its command-line argument.

The driver advances time explicitly; this harness has no wall-clock scheduler,
CAN access or sockets. Silence tests advance independent controller ticks while
sending no further client commands. Its ideal hardware follows commands directly
and cannot validate dynamics, timing on a Jetson, or stopping distance.

## Validation and remaining integration

Verified on the Windows development host on 2026-09-24:

| Check | Result |
|---|---|
| `cargo test -p robot_control --locked --offline` | 40 passed, including watchdog startup/refusal, staged damped-stop failures and allocator instrumentation |
| Python native guarded contract tests | 15 passed with the newly built executable |
| Full offline Python unit/contract suite | 432 passed in 24.91 seconds; includes 20 affected commissioning/guarded contract tests |
| `cargo clippy -p robot_control --all-targets --locked --offline -- -D warnings` | Passed |
| Jetson cross-build | Three binaries verified as AArch64 ELF64; Linux service and finite trial tool executed on production Ubuntu 22.04.5 |

The ARM64 harness SHA-256 is
`eeb8e1b6b9c5dd564045c9dabb3fababa90826f6fc2c24cc0c7b15b4d13a8294`.
The separate Linux commissioning binary and its deployed hashes are recorded in
the device log. The linker
reported an ignored deprecated optimization setting; the build and ELF checks
succeeded. Python regression tests requiring temporary files were rerun outside
the sandbox after Windows directory permissions prevented fixture setup inside it.

Rust tests exercise expiry boundaries, stale offers, exclusive ownership, epoch
changes, replay/reordering, complete-batch rejection, non-finite values, renewal,
explicit recovery, frozen feedback, hardware faults and stop equations. A separate
allocator test verifies no allocation **or deallocation** during nominal ticks,
expiry, stopping and a fault with the ideal adapter. Parsing, status serialization
and ownership requests remain outside that audited region.

Python tests execute the actual Rust binary and route both robot definitions
through the shared session. They cover source-aged feedback, partial acceptance,
lost acknowledgements, collision-verdict expiry and refusal of physical settings.

Linux commissioning increment:

- `guarded_controller` uses a 2.5 ms timer independent of request arrival,
  bounded nonblocking CAN operations, at most one bounded ZMQ request per tick,
  and a fresh process epoch. Old `arm_joints` traffic is rejected.
- The same ID can exist on different buses; serial identities and joint order
  are checked before opening a physical deployment. The codec rejects values
  outside configured wire ranges instead of silently clamping them.
- Kernel arrival timestamps retain queue age. Clock shifts, stale/invalid data,
  receive floods and partial writes fail closed. Conversion is joint-space
  position/velocity/effort with transmission scaling. Guarded wire ranges now use
  manufacturer manuals and user-confirmed motor labels; the motor-to-CAD mapping
  and independent torque calibration remain unverified.
- 40 portable Rust tests and 432 Python unit/contract tests passed. Minerva's
  virtual-CAN fault and 1,000-cycle 25 Hz tests passed; exact results/hashes are in
  the [device record](../devices/MINERVA_PRODUCTION_01.md).
- Status serialization and transport are outside the core's allocation audit.
  This is measured Linux scheduling, not a hard real-time guarantee. The
  synthetic 25 Hz run excludes collision work and camera load.
- Requests update validated targets, which are applied at the next scheduled
  tick. Polling cannot accelerate CAN setpoint traffic. Arm initializes its hold
  immediately and software emergency/fault disable remains immediate. The
  adapter stages neutral frames before enabling. Finite trials checked that
  neutral frames do not auto-enable stopped motors. The dedicated watchdog tool
  has observed command-loss Reset on all 14 motors after the left RS00/RS02
  updates. Final readback has 600 on left j4/j5 and right gripper, zero on the
  other eleven. Prior passes do not replace reconfiguration after power cycling.
- Physical adapter enable now requires a fresh watchdog proof pinned to UID and
  qualified firmware. Explicit zero-torque service startup completes stopped
  setup before opening its socket, then audits every roughly 10 ms plus reply
  cycles without rewriting. Any failure revokes proof; each enable consumes it.
  A new service process is required after disable/fault. Kernel-verified vcan
  alone bypasses physical preflight. Physical move commands remain unavailable.
- Jetson checks rejected a wrong UID and held the correct gripper disarmed/ready
  through audits. No network enable was requested in that hardware check. The
  adapter's actual enable/refusal behavior passed deterministic tests. The same
  service binary passed 16 vcan checks and 100 cycles at 25 Hz: p99 7.620 ms,
  zero 40 ms deadline misses. This remains synthetic protocol timing.
- The final trial binary records final-stop feedback explicitly and includes it
  in breakaway maxima/acceptance. The earlier trial artifact's complete raw
  trace includes a 0.254292 rad/s reverse cleanup reading that was outside its
  per-ramp summary. Historical evidence is preserved, and no failed trial was
  rerun just to get a passing report.
- An explicit damped stop now tapers drive, verifies settling under damping,
  tapers damping, then verifies settling with zero gains before disabling. The
  four-second experiment budget and measured envelopes are unchanged. Four
  right-gripper trials passed; the mechanism's jaw travel/force remain uncalibrated.
  Historical immediate-stop profiles retain their behavior. Fault exits still
  attempt immediate disable rather than continuing active damping.

Remaining integration work:

1. Independently verify jaw travel versus elastic encoder displacement and
   establish bounded useful motion/return. Four small damped-stop trials passed
   the existing bounds; this is limited repeatability evidence at one pose.
   The mandatory watchdog gate is implemented; all 14 behavior passes remain
   individual, nominal-zero-gain qualification rather than loaded-stop evidence.
2. Migrate applications from legacy motion publishing after physical settings
   and the full movement gate are qualified. No automatic legacy fallback.
3. Receiver-owned collision bounds and expiry are implemented and tested with
   the real CAD compiler over vcan. Qualify physical stopping bounds, real input
   transport and same-arm/body policy while preserving finger exclusions.
4. The 1,000-cycle production-host vcan run including collision work passed at
   25 Hz. Run sustained tests including real leaders, cameras and physical CAN.
5. Verify motor-to-CAD mapping, firmware units and supported physical stopping
   before enabling guarded hardware motion. Continue at 50/100 Hz after measured
   results support it.

Architectural references: Intrinsic's
[command freshness validator](https://github.com/intrinsic-ai/intrinsic-core/blob/895281d62b950ca46d21885906796848692b9638/intrinsic_control/intrinsic/icon/hal/command_validator.h)
and [ICON architecture](https://github.com/intrinsic-ai/intrinsic-core/blob/895281d62b950ca46d21885906796848692b9638/developer_resources/learn/tutorials/interactive_tutorials/icon/icon_introduction/icon_introduction.md).
This implementation is original Rust/Python code; no Intrinsic source was copied.
Any later copied/adapted source must retain its applicable Apache notices.
