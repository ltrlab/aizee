# 25 Hz collision supervisor and hardware validation

Updated 2026-09-24 EDT. The next milestone is a measured **25 Hz whole-robot
supervisor (40 ms period)**, followed by 50 Hz and ultimately 100 Hz. This is an
acceptance plan, not an enabled hardware interlock.

**September 24 integration:** receiver-owned collision envelope expiry and a
profile-driven dual-leader supervisor are implemented. Minerva's production
computer accepted 1,000/1,000 synthetic vcan cycles including real collision
geometry at 25 Hz, p99 14.733 ms, maximum 17.227 ms, zero misses; loss of the
producer caused the independent receiver to stop. Input-network timing, physical
reference and loaded stopping remain unqualified. See [current implementation
and evidence](../GUARDED_TELEOP.md). Earlier baselines below remain historical.

The existing Minerva compute unit remains the deployment target. The new Orin
Nano starts as a development/build and benchmark host. Measure both machines;
timing on one does not qualify the other. No further Onshape retrieval is needed.

## What 25 Hz means

Keep the collision supervisor separate from actuator control. The checked-in
Minerva profiles configure a 400 Hz arm loop and 50 Hz telemetry. Do not reduce
the motor loop to 25 Hz. Fresh feedback collection, command expiry and stop
handling must continue independently while collision work runs.

Each supervisor cycle consumes a coherent snapshot of both arms, evaluates the
proposed whole-robot motion and stopping envelope, and produces a bounded,
expiring command decision. Use monotonic scheduling and coalesce new proposals;
do not queue obsolete motion or reuse a verdict outside its validated envelope.
Source age, inter-arm skew, computation, scheduling, transport and actuator
response all contribute to the reaction-distance budget. Their bounds need
measurement; a 40 ms period alone is not a stopping-time guarantee.

Initial Minerva 1.2 baseline was **contact-only, one pose per query**, on Windows:

| Measure | Result |
|---|---:|
| Median | 23.17 ms |
| p99 | 35.21 ms |
| Maximum | 54.54 ms |
| Queries over 40 ms | 2 / 1,000 |

The motion gate performs clearance queries for measured stopping motion, the
commanded path and commanded stopping motion. Accepted commands can require
three or more queries. Those costs and transport are absent from the table.
The current gate also defaults to a 10 ms deadline; changing that default alone
does not qualify a 25 Hz deployment.

**Jetson baseline, 2026-09-20:** the new development host now builds and runs the
native collision backend. Minerva 1.2 contact queries measured 54.07 ms median,
72.62 ms p99 and 127.91 ms maximum; all 1,000 exceeded 40 ms. This makes further
optimization necessary before even the contact-only query meets 25 Hz on that
host. The existing Minerva compute unit still needs its own measurement.
See [device identity, setup and evidence](../devices/JETSON_DEV_01.md).

**Hierarchy update, 2026-09-20:** branch/link/part filtering is implemented with
separate inter-arm, within-arm, body and environment scopes. Whole-robot coverage
remains the default and the motion guard rejects partial-scope results. Jetson
whole-robot contact timing improved to 48.74 ms median / 67.18 ms p99, still over
the target. Inter-arm contacts alone measured 1.96 ms median / 6.66 ms p99, with
one 70.24 ms outlier in 1,000 poses. A 100-pose inter-arm **5 mm clearance** run
reached 85.25 ms maximum (two misses over 40 ms). Exact distance work and the
unreviewed assembly contacts remain blockers; contact-only speed cannot qualify
the stopping-envelope checks. See [measurements](COLLISION_HIERARCHY_RESULTS.json).

## Implementation order and evidence

### Conservative keep-out milestone — 2026-09-20 EDT

The user selected conservative enclosing geometry as the arm-to-arm boundary,
with every pair involving Minerva's four sliding finger links excluded. The
policy is profile-driven, revisioned and separate from CAD diagnostic modes.
No fine CAD queries run in keep-out mode. Whole-robot checking remains the
command gate default; an inter-arm replay is explicitly partial coverage.

On the development Jetson, a **1,000-cycle synthetic replay paced at 25 Hz**
completed with zero 40 ms scheduled deadline misses: median **26.15 ms**,
p99 **34.75 ms**, maximum **36.67 ms**. It accepted 851 cycles and rejected 149.
Each cycle evaluates up to three short paths (up to nine proximity queries),
with conservative between-sample travel bounds. These paths are synthetic and
do not represent calibrated physical braking. Fixed radii/ancestry are cached;
changing extension and motion are recomputed. Individual pose median/p99/max:
**1.91 / 2.34 / 3.91 ms**, at a 5 mm inspection clearance.

One hundred CAD audits found no missed nonexcluded proximities; 39 additional
envelope restrictions are expected, and 34 CAD pair occurrences involving
excluded fingers were deliberately omitted. Finger-to-arm protection is absent
under this requested policy. CAD diagnostics still include those links.
Whole-robot replay rejects all 1,000 sampled motions because unapproved
same-arm/body interfaces remain; its short rejection time is not a usable
whole-robot supervisor result. See [evidence](COLLISION_KEEPOUT_RESULTS.json).

This short run is an intermediate result. It does not meet the longer nominal
load qualification below (including the proposed p99 <= 30 ms headroom), test
the full telemetry/lease pipeline, or qualify physical stopping.

```sh
python scripts/benchmark_keepout.py --model urdf/minerva/model.json --output simulation_runs/keepout-new.json --scope inter_branch --samples 1000 --audit-samples 100 --threshold 0.005 --rate 25
```

### 1. Replay and synthetic loopback at 25 Hz

- Add a configurable supervisor runner with separate feedback ingestion and
  collision work. Keep robot identity, joint mapping and rates in profiles.
- Record cycle start/end, source age/skew, model/scene/calibration revisions,
  proposed and accepted commands, query count, deadline misses and decisions.
- Exercise both-arm trajectories, near-contact paths, stops and reversals with
  the full motion gate. Include clear commands that must be accepted; rejecting
  every command because of unresolved interfaces is not a successful test.
- Inject frozen, absent and reordered feedback; one missing arm; stalled
  collision work; stale/queued commands; process restart and transport loss.
- Proposed nominal-load acceptance: 30 minutes / 45,000 cycles on the intended
  edge host with cameras, logging and transport active; full supervisor work
  p99 <= 30 ms and no cycle exceeding its 40 ms deadline in that run. Retain
  maximum and miss counts, not only percentiles. This is measured qualification
  for the tested workload, not a hard real-time proof.
- Fault-injection acceptance is separate: late/unavailable decisions must not
  authorize new motion, the receiver must expire its lease, and recovery must
  require deliberate rearming. Stop latency must fit measured clearance.

### 2. Collision policy and coordinate calibration

- Review the 44 provisional assembly-interface hypotheses over their relevant
  travel. Implement a versioned, shape-specific allowed-contact policy with
  evidence and tests. Runtime rejects unapproved part-interface exclusions;
  the separate keep-out policy permits only explicitly configured link omissions. Viewer
  labels do not exempt those contacts from the strict command gate. Preserve
  inter-arm checking and reject blanket adjacent-link exemptions.
- Inspect simplified geometry coverage and containment behavior. Record the
  changed RB03 OUTPUT / Part 66 contact (0.631 mm new clearance), cable/tool
  coverage and the clearance allowance for geometry and calibration error.
- Verify each arm independently: CAN/joint identity, encoder sign, CAD zero,
  shaft ratio and units. Existing min/max captures are not a motor-to-CAD map.
  Verify gripper motor radians to jaw metres, coupling and aperture separately.
- Establish reviewed operational intervals inside the inspected physical and
  cable-routing envelope. CAD sweep candidates remain viewing limits until
  physical evidence supports operational limits.
- Pin the model, calibration, contact policy, scene and payload for each run.

### 3. Receiver-side command and stop behavior

- Add a guarded Rust command protocol with sequence/session identity, command
  lease and replay/staleness rejection at consumption time. Cross-host expiry
  must use a defined clock/lease protocol; do not compare unrelated monotonic
  timestamps. A Python check before ZMQ send is insufficient.
- Permit one command authority for the guarded deployment. Route teleop, IK,
  policy, replay and future calibration through it; prevent legacy publishers
  from bypassing the guard. Validate complete batches before motor mutation,
  and define the response when only one arm receives/accepts a batch.
- Implement and test controller-local effort, velocity, tracking-error, fault,
  temperature and feedback-age handling using verified device units and
  measured bounds. Resolve the recorded RS00 torque/gain and RS03 velocity
  codec conflicts against installed firmware before using those units as trips.
- Define controlled stop, supported hold and emergency disable separately.
  Current command timeout holds the **old target**, and the current emergency
  stop removes active gains/support. Neither is a measured braking controller.
  The current 500 ms command timeout and 2 s feedback timeout do not establish
  a 40 ms supervision response.
- Prove receiver expiry, crash handling, one-arm failure, e-stop/disable and no
  automatic resume using virtual CAN/loopback before powered trials. Verify
  supported physical stop behavior before collision-prevention claims.

### 4. Hardware observation and supported measurements

Observation can precede physical enforcement. Bring up the native library and
Python/FCL dependencies on the actual host, replay locally, then subscribe to
live telemetry with command sending disabled. Align the rendered robot to known
physical poses and log feedback age, skew and collision verdicts. If telemetry
requires motor enable on the installed firmware, that is a separate supervised,
supported setup; a passive subscriber must not enable anything automatically.

After controller fault tests and coordinate/protocol checks, measure small,
bounded single-joint motion in an inspected clear region with supported loads.
Record command-to-response delay, tracking error, motor-reported effort and
stopping displacement versus speed and load. Derive initial margins and speed
caps from this evidence. Use the user's calibration weights for
[paired static load trials](KNOWN_LOAD_CALIBRATION.md), adding an independent
static load reference before later dynamic torque instrumentation.
Do not begin with automatic hard-stop discovery or intentional arm contact.

Mass/inertia assignment is not required for passive geometric observation.
It remains required for credible dynamics/gravity predictions; static geometry
and motor-reported torque alone do not validate the simulator's dynamics.

### 5. Guarded motion and higher rates

Enable a dedicated hardware enforcement mode only after the previous gates.
Start with bounded single-joint motion, then one arm with the other stationary,
then both arms. Validate avoidance using positive-clearance stopping boundaries,
including stale-state and interrupted-command cases. Keep a supported,
independently verified stop procedure throughout initial testing.

Optimize measured bottlenecks using the cached model, then repeat the same
acceptance and physical-margin checks at 50 Hz and 100 Hz. The eventual target
remains 100 Hz with collision compute p99 below 5 ms; raising a loop-rate setting
is not evidence that either timing or physical stopping improved.

## Current readiness

**2026-09-24 software increment:** the [guarded controller core](GUARDED_CONTROLLER.md)
implements receiver-owned permits, sequence/epoch checks, local stopping and
explicit recovery in the Rust core and a Linux SocketCAN/ZMQ commissioning
service. Python/Rust contract tests exercise both robot definitions and partial
two-arm delivery. A 1,000-cycle 25 Hz command soak on Minerva's production host
passed on virtual CAN with independent 400 Hz receiver ticks. It excludes
collision queries and camera load, so it does not qualify the full supervisor
or change the physical-movement gates below. See
[powered commissioning](../POWERED_COMMISSIONING.md).

- Geometry import, local viewer, CAD/keep-out checks and synthetic guard tests:
  available. The scoped keep-out replay above completed 1,000 cycles at 25 Hz.
- Sustained 25 Hz full motion gate, approved contact policy, motor-to-CAD map,
  collision authorization bound to receiver leases and measured physical stopping: outstanding.
- Hardware enforcement: deliberately unavailable in `CollisionCommandGate`.
- Next implementation: resolve within-arm/body contact policy, integrate the
  paced workload with full-session telemetry/fault replay and receiver leases,
  then run the longer nominal-load and physical-margin qualification above.

See [current implementation](COLLISION_RUNTIME_IMPLEMENTATION.md),
[limit and controller gaps](COLLISION_AND_LIMIT_CALIBRATION.md), and
[CAD 1.2 validation](../../urdf/minerva/collision-v1.2/README.md).
