# Integrate guarded teleoperation into the existing GUIs

Status: implementation plan. The existing GUI workflow is the product interface;
the guarded backend and leader viewer do not replace it. Physical dual-arm
teleoperation is still blocked by the service modes and qualification work below.

Preserve **Idle → Enable → Engage/align → Teleop**, disengage to Hold, and
**Shutdown → slowly return to zero → Disable**. Keep the existing I/E/T/H/X
controls for Minerva. Do not introduce a new Start/Stop workflow or a required
hold-to-run control.

## Existing behavior and integration points

### Minerva

The authoritative collector phases are `DISABLED`, `IDLE`, `HOLD`, `ENGAGING`,
`TELEOP`, and `SHUTDOWN` in
[`collect_minerva.py`](../../python/scripts/collect_minerva.py), lines 68–74.
The GUI queues the same key actions used by the terminal; it does not command
motors itself.

| Control | Existing implementation | Behavior to preserve and improve |
| --- | --- | --- |
| I — Idle | GUI lines 1611–1619; collector lines 515–518 and 678–681 | Enable zero-torque backdrive and read measured positions. Idle never sets mechanical zero automatically. |
| E — Enable | Collector lines 443–451 and 509–514 | Accepted from Idle; capture the current measured pose and apply holding gains. Enabling must not begin following the leaders. |
| T — Teleop/engage | Collector lines 521–534 | From Hold, align gently to the mapped leader pose, then track. T again disengages to Hold. Existing Minerva also accepts T from Idle; guarded integration must perform the same explicit hold/enable preflight before any aligning motion. |
| H — Disable | Collector lines 453–465 and 519–520 | Clear pending/held targets and disable. Keep this distinct from a planned return-to-zero shutdown. |
| X — Shutdown | Collector lines 547–555 and 649–656 | Disengage, start from measured positions, ramp to zero, then disable after measured completion. Also accept X during Engaging; the current handler accidentally omits that phase. |

Source links:
[`collect_minerva_gui.py`](../../python/scripts/collect_minerva_gui.py),
[`collect_minerva.py`](../../python/scripts/collect_minerva.py).
The GUI's `_key()` at line 1816 writes `cmd_queue`; the collector drains that
queue at lines 582–588 and calls `handle_key()`. Keep this route and the existing
controls, status display, recording controls, and leader/follower difference bars.

Two details need correction during integration:

* `MinervaTeleop.target()` currently performs **absolute range-to-range mapping**
  for the two arms. It maps each leader's calibrated fraction to the follower's
  interval, including configured direction and arm swapping. See
  [`collect_minerva_app/teleop.py`](../../python/scripts/collect_minerva_app/teleop.py),
  `_arm_source()` at line 308, `_range_map()` at line 320, and `target()` at line
  448. The stored engage anchors are not used to map the arm joints. Do not
  silently replace that behavior with the shadow benchmark's relative mapper.
* Current Minerva Engaging ramps the permitted command lead over **0.3 seconds**,
  then enters Teleop solely because that timer elapsed (`ENGAGE_RAMP_S`, lines
  77–80; update at lines 630–641). That is gentle torque onset, not proof that
  the physical arms have aligned. Use measured alignment for the transition.

The existing Minerva shutdown decrements its target by 0.02 rad per 30 Hz tick
and checks the first 14 measured joints against 0.05 rad. These are legacy
constants, not validated values for every robot or the new CAD slider units.
Replace tick-dependent amounts with per-joint speed and completion tolerances
from the selected robot configuration; retain the slow zero-return behavior.

### Aizee: reuse the existing alignment method

[`collect_demo.py`](../../python/scripts/collect_demo.py) already contains a
bounded approach to the leader:

* Its `State` enum is at lines 801–808: `READY`, `IDLE`, `TRACKING`, `HOLD`,
  `ENGAGING`, `SHUTDOWN`, `ESTOP`. `READY` corresponds to Minerva's disabled
  presentation and `TRACKING` corresponds to Minerva's Teleop phase.
* Engaging starts from the measured arm pose, limits the previous command's lead
  over measured positions, and advances toward the leader by a bounded amount
  (lines 1776–1797). It promotes only when every measured joint is close to its
  target (lines 1804–1811). This is the existing method to generalize, using
  per-joint units, elapsed time, and validated limits.
* Shutdown holds for one second, approaches zero at 0.2 rad/s, and clears the
  sender before disabling (lines 1697–1747). It also has a Cancel-to-Hold action
  at lines 1604–1606. Do not copy its missing-feedback fallback or timeout-based
  assumption of completion into the guarded path.

Aizee's current keys differ: E starts Engaging directly when a leader exists,
and H toggles Hold, while Minerva E enters Hold and H disables. The Aizee GUI
buttons are in [`collect_demo_gui.py`](../../python/scripts/collect_demo_gui.py),
lines 4679–4683; `_send_key()` at line 5096 feeds its existing command queue.
Keep each GUI's established bindings when adapting it. Share semantic lifecycle
operations internally rather than reinterpreting the same letter globally.

## Shared implementation boundary

### Retain GUI actions; replace their command backend

Keep the existing key handlers as compatibility adapters. Extract their lifecycle
decisions into a testable component with no socket or USB ownership: Idle,
Enable/Hold, Engage, Disengage/Hold, Shutdown, and Disable. Its result is a phase
and a proposed target/service operation, not an independently transmitted command.

The current compatibility boundaries already provide useful attachment points:

* [`DualArmTransport`](../../python/scripts/collect_minerva_app/follower.py)
  exposes `enable()` (line 128), `disable()` (line 131), `set_target()` (line 139),
  `clear_target()`, and fresh observations. It already uses `RobotSession` and
  `ThreadedRobotClient`; legacy 17-slot arrays remain at this boundary, with
  unrealized head/lift slots inactive.
* [`AizeeSessionTransport`](../../python/scripts/collect_demo_app/session.py)
  owns the Aizee session boundary. The collector's `_post_bundle()`,
  `_post_tracking()`, and `_clear_bundle()` are at lines 683, 691, and 715 of
  `collect_demo.py`. The sender implementation is
  [`collect_demo_app/zmq_io.py`](../../python/scripts/collect_demo_app/zmq_io.py).

The guarded Minerva path must submit one complete ordered batch for both arms
to one receiver. Do not keep the old two independent endpoint sends or partial
arm continuation for this path: inter-arm collision checks require a fresh
whole-robot pose. Preserve useful legacy observation/recording adapters without
fabricating missing positions or letting a legacy re-emitter renew an old motion.

### One leader acquisition owner

Reuse [`OpenRBInputs`](../../src/robot_platform/teleop/openrb.py) and
[`LatestLeaderInputs`](../../src/robot_platform/teleop/leader_bridge.py), configured
for 100 Hz acquisition. The GUI, leader diagram, and guarded producer consume
the same age-preserving latest snapshots. A loopback client can use the existing
`/api/sample` nonce/epoch/serial/sequence protocol; an in-process GUI adapter can
consume the same snapshot contract directly.

Do not start the old `MinervaTeleop` USB reader alongside the new owner. Keep its
leader preview, calibrated mapping, routing selection and GUI-facing methods as
adaptation points, while replacing acquisition with the fresh shared source.
Display-only zeros and signs in the new leader diagram are not an approved
follower calibration.

The read-only diagram may recover its display when fresh samples resume. A
complete but late or skewed pair is skipped rather than closing USB; corrupt,
repeated or restarted input still latches. While Engaging or in Teleop, a
missing mapped target holds the last bounded target for at most two cycles,
then the lifecycle captures a measured Hold. Recovery is the existing explicit
Engage (T); a returning USB sample never re-engages the robot. Outside
engagement the leader mapping only previews and may resume by itself.

### Certify every phase that can move or hold the robot

Reuse the receiver offer, same-response feedback, complete collision envelope,
acknowledged batch, and local expiry semantics in
[`GuardedTeleopSupervisor`](../../src/robot_platform/teleop/supervisor.py) and
[`CollisionEnvelopeCompiler`](../../src/robot_platform/control/collision_envelope.py).
Refactor the orchestration to expose separate lifecycle operations:

1. **Idle:** zero-gain backdrive with fresh measured feedback and required
   physical startup checks; no automatic zeroing, leader alignment, or motion.
2. **Enable:** certify and capture the current measured pose as Hold. Leaders
   are not allowed to change the hold target merely because they move.
3. **Engage:** obtain a fresh mapped absolute leader target; approach it using
   configured speed/lead bounds and collision checks each cycle. Enter Teleop
   only after fresh measured alignment is within the configured tolerances.
4. **Teleop:** keep the existing calibrated mapping and input behavior. Apply
   slew bounds and certify the complete two-arm batch before dispatch.
5. **Disengage:** capture a fresh measured hold pose and stop following leaders.
6. **Shutdown:** discard leader targets, generate a slow trajectory to the
   configured zero/reference pose, and check each proposed segment against
   fresh whole-robot feedback and collision geometry. Disable after measured
   arrival and settling. A blocked path, stale feedback or communication fault
   invokes the guarded stop; it must not continue blindly toward zero.
7. **Disable/fault:** clear proposed motion before sending the appropriate
   receiver stop/disable operation. No automatic rearming or shutdown ramp.

The existing supervisor's `begin()` currently combines acquisition, hold-arm,
relative leader registration and entry to its `tracking` state. Its `stop()`
requests a stop; it is not a return-to-zero trajectory. Neither method alone is
a drop-in implementation of the complete GUI lifecycle.

Keep the current coarse inter-arm collision policy and explicit finger
exclusions visible. That policy does not qualify same-arm, base, payload or
environment collisions. A zero-return trajectory gets the same available
collision checks as engagement and teleoperation, never a bypass.

## Remaining powered-backend blockers

These are implementation and evidence requirements, not a replacement GUI flow.

* The receiver core now implements `Idle` (enabled, zero gains, commands follow
  the measured pose; owner or command loss disables rather than braking) and
  `Disable` (torque off immediately, ownership released, faults never cleared).
  `Arm` accepts Disarmed or Idle and never re-enables from Idle, so a physical
  enable permit is consumed once. The existing GUI lifecycle now runs end to end
  through the native receiver in `tests/contract/test_guarded_lifecycle_session.py`.
* [`rust/control_core/src/service.rs`](../../rust/control_core/src/service.rs)
  still rejects `Move` and `Idle` outside `VirtualCan`, and `Arm` outside
  `VirtualCan`/`ZeroTorqueTrial`. The latter is a bounded single-motor trial,
  not the GUI's continuous dual-arm Idle. Implement an explicit whole-robot
  physical mode with the existing identity, startup reference and watchdog
  checks; do not simply remove these request guards.
* Complete the deployed motor/CAD and leader/follower mappings, operational
  intervals, reference uncertainty, gripper calibration and loaded stopping
  bounds. Visual direction confirmations and synthetic benchmark parameters
  alone do not establish these values.
* Resolve the production input/network timing path and test it under the
  intended GUI/camera load. The local leader shadow benchmark is useful evidence
  for software timing, not a powered-arm qualification.
* The current guarded collision path requires zero velocity/torque feedforward
  and bounded position targets. Legacy gain sliders, gravity compensation,
  leader haptics and replay must not route around these constraints. Integrate
  only qualified settings through the same backend; show an explicit
  unavailable reason for unsupported options.

## Migration stages and acceptance tests

1. **Extract and test lifecycle logic offline.** Feed the existing key actions
   into the reusable phase/target component. Use named joint configurations,
   per-joint units and an injected monotonic clock. Keep robot transport outside
   this component.
2. **Connect the existing GUI to a shadow backend.** Reuse the single leader
   owner, show measured/leader/commanded positions and the existing phase, and
   run the native virtual receiver plus collision compiler. No physical service
   mode is enabled by this step.
3. **Complete and qualify the physical backend.** Validate identities and
   reference checks, continuous zero-torque Idle, loaded Hold, bounded
   alignment, stopping and command loss. Then exercise the same GUI actions in
   attended bounded tests.
4. **Demonstrate the full 25 Hz path under load.** Include input acquisition,
   transport, collision computation, receiver acknowledgement and GUI/cameras;
   preserve freshness and deadline limits. Increase frequency only after this
   path and its stopping behavior pass.

Required behavior tests:

* I produces zero gains and never writes mechanical zero. E from Disabled is
  refused; E from qualified Idle holds the measured pose without engaging.
* Moving either leader while Hold is active cannot move the follower target.
* T begins from measured pose, advances within configured limits, and remains
  Engaging while any measured joint is outside its alignment tolerance—even
  after 0.3 seconds. Stale or absent feedback cannot satisfy alignment.
* T during Engaging or Teleop freezes a fresh measured Hold target and clears
  pending leader motion. Routing/calibration changes cannot take effect during
  active engagement.
* X works from Idle, Hold, Engaging and Teleop. Its per-joint speed is independent
  of scheduler rate; it disables only after verified arrival/settling. A
  collision-blocked zero return stops and reports the obstruction.
* Losing one arm's feedback, either leader during engagement/tracking, the
  producer, a collision verdict, or an acknowledgement cannot continue stale
  motion or automatically recover the active session.
* H/Disable clears every pending target and prevents the old re-emitter from
  sending afterward. Fault recovery uses explicit existing lifecycle actions.

See [guarded teleoperation](../GUARDED_TELEOP.md) for the current backend and
benchmark evidence. This plan changes no powered command behavior by itself.
