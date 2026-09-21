# Joint limits and collision checking

Updated 2026-09-19. This design separates per-arm calibration from whole-robot
collision prevention. The initial implementation is offline: an FCL collision
checker, a torque/stall candidate classifier and synthetic signal tests. It
does not connect to motors, discover physical limits, or prevent collisions in
the running collectors, policy, IK or replay applications.

There is no powered ramp generator or automatic backoff driver in this change.
The completed `onshape-to-robot` import has 17 links, 16 joints and two mimic
relations, and passes the static audit. Its source arm angular limits are
absent and source masses are zero. Generated angular defaults are not measured
limits. This supports geometry-only investigation, not validated dynamics.

## Separate the constraints

Each arm has its own measured joint limits. Calibrating the left arm does not
copy limits, signs, zero offsets or torque thresholds to the right arm.
Inter-arm collisions are a whole-robot constraint evaluated during operation;
they must not be encoded as permanent per-joint limits from one arm pose.

| Constraint | Meaning | Evidence required |
|---|---|---|
| Mechanical endpoint | Travel stopped by a designed mechanism | Inspection plus repeatable measurement in a documented fixture/pose |
| Cable or hose limit | Travel restricted before damaging routing | Routing inspection across the intended pose envelope; do not discover by pulling a cable taut |
| Usable joint interval | Conservative operational bounds inside the accepted mechanical/routing range | Directional measurements, uncertainty, stopping allowance and review |
| Self-collision | Links on the same arm, torso, tools or base intersect | Audited geometry and the complete relevant robot configuration |
| Inter-arm collision | Either arm or its payload intersects the other | Both arms' synchronized measured states and their proposed motion |
| Environment collision | Robot or carried object intersects its surroundings | Current obstacle scene, frames and geometry coverage |

Joint bounds are necessary but insufficient: two individually valid arm poses
can collide with each other. A valid start and goal also do not establish that
the path between them is safe. A collision checker reports validity; a motion
planner searches for a path that satisfies that checker and other constraints.

MoveIt's Planning Scene separates robot state, collision geometry, bounds and
constraint checking, with explicit allowed collision pairs. Its Planning Scene
Monitor maintains state and world geometry from robot/sensor inputs. We use
that architecture as a reference; this repository does not currently integrate
MoveIt or its planning stack.
[Planning Scene](https://moveit.picknik.ai/main/doc/examples/planning_scene/planning_scene_tutorial.html),
[Planning Scene Monitor](https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html).

## What torque rise and a stall establish

A slowly advancing target, rising motor-reported effort and little measured
motion can identify a **contact/stall candidate**. The same signature can arise
from an obstacle, cable tension, static friction, gravity/load change, a
controller limit, incorrect torque decoding or stale telemetry. It does not
prove a designed hard stop. The classifier must retain those alternatives and
must not write accepted joint limits automatically.

Use a bounded position/velocity ramp with a cap on target-minus-measured-position
lead. Do not progressively increase requested torque or allow target error to
accumulate after suspected contact. A slow target alone does not bound force:
the actuator's position and velocity gains can generate substantial effort.

The eventual detector requires:

- Fresh, finite, advancing source samples for position, velocity and reported
  torque; duplicate published samples do not extend a stall observation window.
- An attempted-motion condition, a measured-progress window, and persistent
  effort change relative to an appropriate baseline. One noisy sample or a
  high gravity-support torque is insufficient.
- Independent abort limits for reported effort, speed, tracking error,
  temperature, elapsed time, travel and missing/faulted feedback. These bounds
  come from verified per-device evidence, not generic software defaults.
- The commanded target, actual accepted command when available, sample
  sequences, source ages, full robot pose and stop outcome in the trace.

The current offline [limit probe classifier](../../src/robot_platform/characterization/limit_probe.py)
requires a full observation window with elevated residual torque, stalled
measured position and velocity, an advancing requested position and tracking
error before reporting a candidate. It aborts on stale/repeated source samples,
source restart, configured travel/time/effort/velocity/temperature caps, faults,
and collision or unknown collision status. Collision status is supplied by a
caller; the classifier does not check geometry. An independent watchdog must
detect entirely absent samples because the classifier only runs when called.
These are signal checks, not a hardware adapter. Synthetic thresholds are
examples for the tests, not
recommended physical settings. No combination of passing synthetic cases
establishes a safe powered search.

## Future measurement procedure

Powered measurement requires a dedicated, supervised hardware adapter after
the controller changes and stop tests below. Preparation establishes a bounded
exploration envelope, an inspected collision-free setup, supported loads and
a verified stop response. Existing saved motor endpoints are evidence to
review, not permission to exceed an accepted operating interval.

1. Record actuator model, firmware, CAN identity, calibration revision, torque
   encoding/shaft reference, gains, payload, cable routing and all joint poses.
   Establish motor-to-CAD sign, zero and any shaft ratio at known reference
   poses. Do not reset a live motor's mechanical zero as part of the search.
2. Characterize free-motion effort and measurement noise inside the approved
   exploration envelope. Hold or support the remaining joints as documented;
   monitor the full robot even when only one joint is excited.
3. Approach one lower or upper candidate using a ramp whose velocity,
   acceleration, target lead and total actuator effort are bounded. Stop the
   ramp on candidate detection; abort immediately on any independent trip.
4. Unload or back off only using the verified supported-arm procedure and a
   checked retreat segment. Blind reversal is not a universal recovery action.
   Check that effort falls and motion resumes; otherwise retain an ambiguous
   obstruction result and stop the procedure.
5. Reapproach from the interior repeatedly at the same documented pose and
   load. Record endpoint spread and hysteresis. Investigate a changed candidate
   under other inspected poses before classifying it as a joint endpoint.
6. Repeat independently for the opposite direction and the other arm. Preserve
   raw observations and candidate classifications. Review the physical cause,
   measurement uncertainty and stop distance before creating a usable interval
   with inward margins.

Do not probe until contact with a cable or an unknown obstacle. Contact
classification is a stop signal for investigation; it is not an instruction
to push harder until something becomes immovable. The accepted result should
distinguish mechanical endpoints, routing constraints and operational margins.

Gripper motor coordinates remain `left_gripper`/`right_gripper` in radians.
The four modeled jaw slides use metres. Their closed/open offsets, coupling,
drive relation and total aperture need separate verification; actuator torque
does not directly specify fingertip force. Arm angular measurements must never
be copied into jaw translation limits.

## Current components and blockers

| Component | Reusable behavior | Gap before powered discovery/runtime prevention |
|---|---|---|
| [Shared session](../../src/robot_platform/control/session.py) | Canonical routing, finite-vector checks, torque-ready observations, source-freshness support and explicit commands | Source freshness is optional by default; observations alone do not enforce torque thresholds, faults/modes, collision checks or confirmed stopping |
| [Bench profile validation](../../src/robot_platform/characterization/audit.py) | Requires device/protocol/limit/stop evidence before CAN setup | Existing protocol-scale conflicts and physical stop response still require device evidence; an approved bounded exploration profile is required |
| [Characterization harness](../../python/scripts/actuator_char/harness.py) | Slew bounds, reported-effort/feedback checks, source traces and injectable clocks | Direct CAN ownership is separate from production control; disable submission is not stop acknowledgement; it is not a limit-search adapter |
| [Manual Minerva calibration](../../python/scripts/minerva_calibrate.py) | Operator-driven endpoint capture | Position-only cache lacks source-age/torque checks; it is unsuitable as an automatic stall detector |
| [Existing mesh world](../../python/ik/mesh_world.py) | Full-tree transforms including prismatic joints and FCL collision queries | Silently skips missing meshes, ignores mimic relations, exempts all adjacent pairs and automatically allows home collisions; those defaults cannot protect powered motion |
| [Existing sweep](../../python/ik/collision_sweep.py) | Offline configuration sampling and a reviewable report | Ancestor sampling is incomplete; unsampled postures/segments remain unknown, and zero valid postures currently fall back to URDF limits |
| [Existing IK](../../python/ik/dls_ik.py) | Damped least-squares solve with joint-bound clamping | No collision validity constraint; its Aizee kinematics helper is not a whole-Minerva model |

The production Rust controller requires specific changes before automatic
contact probing:

- In [motor.rs](../../rust/motor_control/src/motor.rs), `max_torque` clamps the
  feedforward term only. It does not cap the complete on-motor PD effort.
  `LIMIT_TORQUE` is declared in the protocol module but is not configured by
  the controller. Any motor-side current/torque limit must be supported by the
  actual model/firmware, set/read back as applicable, and bench-verified.
- The command watchdog holds the last position target with its gains. Losing
  a probe producer could therefore keep pushing into contact. Soft-limit
  rejection also preserves the preceding target and refreshes its watchdog.
  A dedicated probe mode needs explicit lease expiry, contact unloading and
  fault behavior; it cannot inherit ordinary position-hold semantics blindly.
- The feedback timeout in [main.rs](../../rust/motor_control/src/main.rs) is
  currently two seconds. There is no measured-effort or temperature trip in
  its host safety loop, and `is_within_limits()` has no caller. Probe-specific
  measured-state checks and response timing must be implemented and measured.
- Emergency stop changes arm motors to an enabled state that sends zero gains
  and effort. This removes active support rather than guaranteeing a
  load-bearing hold. The e-stop latch also filters ordinary disable commands.
  Test supported-arm unloading, gravity support and explicit disable behavior
  separately; none is interchangeable without understanding the physical load.

The known RS00 torque/gain scale and RS03 velocity-scale discrepancies in the
repository audit must be resolved per installed device. A motor-reported torque
threshold is meaningless if its conversion or shaft reference is wrong.

## Whole-robot runtime architecture: future work

One command arbiter at the shared-session boundary must accept every motion
source: teleoperation, IK, policy, replay and future calibration. These sources
submit proposals; only the arbiter may issue accepted motion to controllers.
Direct legacy sends and competing publishers must be removed or denied for
the guarded deployment. An optional GUI check would leave bypasses.

The arbiter must use a synchronized snapshot of both arms and the robot body,
including jaw coupling and carried objects. Define and measure permissible
source age, inter-arm skew, prediction horizon and update latency. A stale
stationary arm is still an unknown obstacle. Reject new motion if either arm's
relevant state or required scene data is unavailable; apply the independently
verified stop/hold response.

Check current state, proposed state and intervening motion against accepted
per-joint bounds and whole-robot collision geometry. Check both arms moving
together, not each against an obsolete pose of the other. Include clearance
for geometry/calibration error, transport delay and measured stopping distance.
Finite interpolation is an approximation; document resolution and require
continuous checking or a justified swept-motion bound where it can miss
contact. A planner may then search for collision-free alternatives; arbitrary
per-joint clipping is not a substitute for validating the resulting path.

Run time-critical enforcement on the edge host. Rust must reject expired,
replayed or unauthorized motion, bound commands locally, monitor feedback and
enter the verified response when its supervisor or communication fails. A
Windows process and network-delivered stop alone cannot establish this behavior.
The shared arbiter, controller lease enforcement and physical stop path form
separate tested layers; the current Python lease mechanism does not implement
this complete runtime gate.

## Implementation stages and acceptance evidence

1. **Offline foundation, current scope:** [collision.py](../../src/robot_platform/simulation/collision.py)
   provides strict FCL checking; [limit_probe.py](../../src/robot_platform/characterization/limit_probe.py)
   classifies recorded or synthetic torque/stall
   signals. No hardware adapter, active command gate or runtime collision
   prevention is delivered by this step. Require complete local geometry and
   explicit collision exclusions; do not auto-ignore initial overlaps.
2. **Offline model validation:** verify FK/axes/units, jaw mimic transforms,
   known colliding and clear fixtures, and the full Minerva geometry coverage.
   Test missing meshes, invalid exclusions, stale/partial configurations and
   intersections between samples. Validate explicit contact exemptions across
   travel. Lack of physical masses does not prevent geometry-only checks, but
   it prevents credible gravity/effort predictions.
3. **Detector validation:** replay clean motion, noise, gravity/load changes,
   stiction, hard-stop-like signals, obstacles, drift and reversed directions.
   Test frozen/duplicate/reordered samples, source restart, missing torque,
   fault flags and inconsistent units. Record false positives/negatives and
   distinguish ambiguity from accepted endpoint evidence.
4. **Controller and virtual-CAN validation:** exercise total-effort limits,
   command rejection, bounded target lead, lease expiry, feedback loss,
   e-stop/disable, process crash, transport recovery and no automatic resume.
   Confirm every motion source passes the same whole-robot arbiter and both
   controllers execute the agreed failure response.
5. **Supported bench measurement:** verify model-specific effort limiting and
   stop timing physically, then run bounded single-joint approaches with an
   inspected fixture and reviewed parameters. Repeated lower/upper endpoint
   candidates and independent physical inspection produce reviewed usable
   limits; deployment remains a separate step.
6. **Operational collision prevention:** demonstrate two-arm trajectories,
   simultaneous opposing commands, payload changes, a stale arm, scene updates,
   network interruptions and edge overload. Validate stopping clearance and
   recoverability before claiming prevention on the real robot.

Keep generated candidate reports separate from accepted calibration/configuration.
Every accepted limit and collision exclusion needs source/model revisions,
units, evidence, the tested pose/load envelope and review history. The existing
[simulation validation plan](SIMULATION_VALIDATION_PLAN.md) remains the gate
for physical claims; a static or synthetic pass is not real-world validation.

## Run the current offline tools

### CAD 1.1 collision-limit review

The shared `sim sweep-limits` command now produces reference-pose candidate
ranges for independently declared branches. The Minerva **3DP URDF Version 1.1**
round checked 2,994 poses using complete source meshes after the left shoulder
correction. Angular steps were 1°, contact refinement at most 0.03125°, and
inward angular margins 1°. Jaw steps/margins were 0.5 mm, with refinement at
most 0.015625 mm. All 14 independent joints have candidate viewing intervals.
Both j5 searches reached ±180° without contact, so those remain search bounds.

The [review summary](../../urdf/minerva/limits/README.md),
[request](../../urdf/minerva/limits/collision-round-1-request.json) and
[full report](../../urdf/minerva/limits/collision-round-1.json) record the scope,
source hashes, samples, contact brackets and 44 explicit provisional part-level
interface hypotheses. Those hypotheses have not been approved as runtime
exemptions. The report narrows only the shared viewer controls and provides
one-joint endpoint previews for visual review. It does not edit the source URDF
or hardware calibration and does not prove that combinations of joint values
within these intervals are collision-free. Whole-robot checks still include
the opposite arm and all assembly contacts.

### State/path checks and synthetic probe validation

Install the shared package's `collision` extra for FCL/trimesh support. The
versions were exercised on this Windows host; Jetson installation/performance
still needs testing. From the repository root:

```sh
python -m robot_platform sim validate-limit-probe
python -m robot_platform sim check-collision --urdf urdf/minerva/onshape-to-robot/minerva.urdf --scene path/to/scene.json
```

The first command replays ten fabricated signal scenarios and emits a JSON
report. It tests decision logic, not motor physics, a motion ramp or hard-stop
identification. The second consumes a complete scene request:

```json
{
  "positions": {"joint_name": 0.0},
  "end_positions": {"joint_name": 0.1},
  "max_joint_step": {"joint_name": 0.01},
  "allowed_pairs": [],
  "environment": [{"name": "table", "size": [1.0, 1.0, 0.1], "xyz": [0, 0, -0.2]}]
}
```

This schema example needs replacement with **every independent joint** in the
selected URDF: Minerva has 12 arm angles and two A-jaw translations. Mimic
B jaws are computed. Positions/steps are radians for rotations and metres for
slides; environment boxes use metres and optional `rpy` radians. Omit both
`end_positions` and `max_joint_step` for one state. Paths interpolate both arms
together; finite samples do not prove continuous clearance. Allowed pairs are
explicit link names, with `environment:<name>` for obstacles; they need review.

Both commands accept `--output <new-report.json>` and refuse to overwrite a
report. Exit 0 means the requested offline checks passed, 2 means collision or
invalid input, and 130 means interrupted. Reports explicitly mark the runtime
interlock and physical validation as inactive.

The first full-mesh Minerva check used all 14 independent coordinates at CAD
zero with **no exclusions**. It found 18 contacting link pairs. These include
joint/bearing and gripper interfaces requiring geometry review; none were
automatically accepted as safe. This is a diagnostic CAD pose, not a calibrated
motor pose or approved home. Generated evidence is in the local ignored
`simulation_runs/limits-and-collision-20260919/` directory.

The full 129-shape model took about 20.4 seconds to load and 0.172 seconds for
one state check on this Windows host. This is not a real-time benchmark or
acceptable 400 Hz control-loop timing. Review simplified conservative collision
shapes and benchmark on the edge device before runtime integration. FCL triangle
meshes check intersecting surfaces; they do not establish solid containment.
Use reviewed primitives/convex solids or a separately validated containment
method where nested geometry matters. Sparse path samples can also miss contact.
