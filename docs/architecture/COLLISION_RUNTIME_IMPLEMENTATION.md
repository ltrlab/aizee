# Collision implementation status

Updated 2026-09-20 EDT. The viewer checks CAD contacts or explicit conservative
keep-out boundaries. The next milestone is a
[25 Hz supervisor and staged hardware validation](COLLISION_25HZ_HARDWARE_PLAN.md);
the eventual 100 Hz whole-robot target remains open. See [research](COLLISION_RUNTIME_RESEARCH.md) for
the design rationale and primary sources.

## Geometry and contact decisions

The first convex-enclosure prototype filled bearing holes, bracket openings
and spaces around grippers. Reporting its overlaps as collisions was incorrect.
The **CAD modes** use those enclosures only as broad-phase bounds:

1. Persistent Rust/Parry groups, links and shapes reject distant pairs cheaply.
2. Cached FCL objects confirm remaining candidates against the original CAD
   triangles and URDF primitives.
3. Only CAD-confirmed contacts or distances reach the viewer or command gate.

The separate **Conservative keep-out** mode treats enclosing convex shapes as
deliberately restrictive boundaries and performs no fine CAD queries. This
trades usable workspace for speed; a boundary violation is not a CAD contact.
The user-authorized `keepout_ignored_links` profile map excludes any pair with
one of the named links. Minerva lists its four sliding finger links, including
the two coupled fingers. Finger-to-arm and finger-to-finger pairs are excluded;
wrist/gripper bodies remain checked. CAD modes still include every finger.
Unknown link names and missing reasons are rejected. No same-arm assembly
interfaces have been approved for exclusion.

Each profile's `collision_model` artifact pins the URDF and every collision mesh
hash. Its `geometry_role` is `broad_phase_only`. The loader checks instance
coverage, source revisions, transforms, primitive dimensions, and that every
source vertex lies inside its convex bound. The artifact remains labeled
`broad_phase_only`: it does not claim CAD fidelity. Selecting keep-out explicitly
applies a separately hashed policy to those enclosing shapes. CAD modes display
source collision shapes; keep-out displays the actual envelopes with excluded
links omitted. Results distinguish geometry basis, policy revision, selected
scope and excluded links. `whole_robot_checked` means all scopes under that
policy; `full_geometry_coverage` is false when links are excluded.

Compact proximity results are selected **after** confirming every candidate;
the nearest enclosure is not necessarily the nearest actual part. Contact-only
queries avoid computing unused clearance distances. Errors propagate as
unavailable rather than implying a clear state.

Mesh distance and contact queries currently have the same surface semantics as
the original FCL checker. Closed-solid containment is not certified for arbitrary
CAD assemblies. These checks are not a physically validated robot interlock.

## Code map

| Area | Responsibility |
|---|---|
| `rust/collision` | Persistent Parry f64 bounds, AABB pruning, distance queries, C ABI |
| `simulation/runtime_model.py` | Revision/coverage validation and FCL CAD refinement |
| `simulation/native_collision.py` | Native bounds with CAD confirmation or conservative keep-out decisions |
| `simulation/collision_groups.py` | URDF branch membership and complete pair-scope partition |
| `simulation/collision.py` | Shared FK, persistent FCL objects, source contact/path checks |
| `control/collision_guard.py` | Whole-robot motion/stopping-envelope checks and verdict lifetime |
| `control/session.py`, `control/background.py` | Optional shared command gate and batched arm commands |
| `viewer` | CAD/envelope overlays, explicit coverage and exclusions, stale-result handling |
| `scripts/benchmark_collision.py` | Seeded latency and independent source-result comparisons |
| `scripts/benchmark_keepout.py` | Envelope containment audits and paced synthetic motion replay |

All robot identities, geometry, joints and transforms come from data. This adds
one crate to the existing Rust workspace and modules to the existing Python
package; it introduces no ROS dependency or new messaging system.

## Hierarchical coarse/fine checks

The native scene compiles pairs into **branch-pair → link-pair → part-pair**
blocks. Each pose updates conservative world AABBs. A separated branch or link
rejects its entire block before part tests; surviving parts get convex-distance
checks, then source-CAD confirmation. All three levels respect the requested
clearance threshold plus the existing native numeric margin. Overlapping bounds
only create candidates in CAD mode. In keep-out mode, envelope proximity itself
violates the chosen boundary; fingers are removed before pair compilation.

`collision_branches` is optional profile data mapping labels to root joints.
Fixed and mimic descendants stay with their branch; without configuration, the
first movable joint along each root path starts a branch. Explicit roots cannot
overlap or nest. Unassigned links are body geometry. Minerva declares the two
shoulder roots; the code contains no robot-specific joint names.

The pair partition is exhaustive: `inter_branch`, `within_branch`, `body`, and
`environment`. `proximity(..., scope="all")` remains the default. Each result
includes `query_scope`, `whole_robot_checked`, a grouping-policy hash,
`stage_ms`, native pruning counters and per-scope candidate/contact counts.
The command gate rejects partial-scope results. Offline `check_linear_motion`
can explicitly inspect one scope and labels that result as partial. The viewer can inspect each scope,
but a clear inter-arm result makes no claim about same-arm/body contacts.

Source bounds are batched with NumPy and the fine phase reuses the native
query's FK snapshot. Cached FCL objects remain protected by the scene lock.
Compact results still confirm all candidates before selecting the closest;
there is no early-exit native motion predicate. Keep-out uses its separately
filtered pair set and never invokes FCL refinement.

**Native ABI is now version 3.** Rebuild the library and restart running viewers
when updating Python code. An old library fails explicitly with an ABI mismatch.
The benchmark accepts `--scope inter_branch` and records stage timings, pruning
counts, scope coverage and 40 ms deadline misses alongside the existing audits.

## Build and use

```powershell
.venv-platform/Scripts/python.exe -m pip install -e ".[viewer,collision]"
./scripts/build_collision.ps1 -Test
.venv-platform/Scripts/python.exe -m robot_platform view --robot minerva
```

Select **Accelerated CAD** for the two-stage query, or **Source CAD** for the
independent FCL contact baseline. Without the native library, Accelerated CAD
falls back to FCL CAD. Joint changes coalesce while a query is running. The
collision geometry checkbox shows the source collision shapes in CAD modes.
**Conservative keep-out** requires the native library and initially selects
**Between branches / arms**. Its checkbox shows enclosing geometry; the optional
inspection gap defaults to 5 mm and is not a calibrated stopping margin.

Minerva now uses **URDF 1.2 No Holes** for collision geometry and retains 1.1
detailed visuals in a composed URDF. Its current bounds are
`urdf/minerva/collision/broadphase-v1.2.json`. See the
[CAD import and validation](../../urdf/minerva/collision-v1.2/README.md).

Regenerate bounds to a **new** file after changing CAD:

```powershell
.venv-platform/Scripts/python.exe -m robot_platform sim build-collision-model --model urdf/minerva/model.json --output urdf/minerva/collision/broadphase-v2.json
```

Then update the profile's `collision_model` path and restart the viewer.
This command operates entirely on local files. It never fetches Onshape data.

Run an offline contact benchmark:

```powershell
.venv-platform/Scripts/python.exe scripts/benchmark_collision.py --model urdf/minerva/model.json --output simulation_runs/minerva-benchmark.json --samples 1000 --reference-samples 100 --audit-samples 100
```

`--threshold 0` is the default and measures contact queries. A positive value
measures CAD surface clearances in metres; do not compare those timings directly
with contact-only results. Reports include seed, model revision, sampled poses,
latencies, missed contacts and extra contacts. Historical enclosure-only timings
are not timings for the corrected CAD-confirmed pipeline.

## Command gate

`RobotSession(..., command_gate=gate)` is opt-in. `CollisionCommandGate` supports
observation and **synthetic loopback simulation enforcement**. Hardware
enforcement is intentionally not implemented. No settings enable it implicitly.

It requires complete mechanical-to-actuator calibration, whole-robot position
and velocity feedback, bounded source age/skew, speed bounds and braking
accelerations. It checks measured-velocity stopping envelopes, the proposed
motion and commanded-velocity stopping envelopes. The stopping envelope covers
a joint-coordinate box because different joints may stop at different times.
Checks use displacement bounds rather than assuming sampled poses alone prove
the path clear. Query count and wall-clock limits fail closed.

Motion bounds use the actual enclosing vertices in keep-out mode. Fixed shape
radii and joint ancestry are cached; changing prismatic extension and all joint
displacements are still evaluated per path. Finger exclusions also apply when
selecting the largest relative-motion bound.

Verdicts bind model/world revision, geometry/policy identity, state sequence, the complete command batch
and expiry time. Dispatch rechecks validity before sending each arm's command.
Rejected or expired simulation motion stops the whole arm set. This is an
emergency-stop behavior in the simulator, not a calibrated physical braking
controller. Stale telemetry, incomplete targets and unsupported command forms
are covered by tests. Observation mode does not modify commands.

## Validation and remaining work

The hollow-bracket regression puts a probe inside a U-shaped opening whose
convex enclosure overlaps it. CAD confirmation reports clearance in the opening
and contact at either wall. Other tests cover threaded object reuse, source
revision changes, undersized bounds, primitive distances/containment, rotated
links, environment obstacles, motion tunneling, stopping envelopes, stale state,
expiry between arm sends and whole-robot background dispatch.

In the detailed Minerva 1.1 baseline reference pose, 140 bound overlaps reduce to 42 source CAD
assembly-interface contacts; the other 98 are not collisions. Seeded comparisons
against the independent source checker are recorded alongside latency results
in [benchmark results](COLLISION_BENCHMARK_RESULTS.json).

Desktop contact timings (1,000 seeded poses, milliseconds; loading excluded):

| Robot | Full result median / p99 | Compact result median / p99 |
|---|---|---|
| Minerva, detailed 1.1 baseline | 39.01 / 67.15 | 35.07 / 61.80 |
| Minerva, simplified 1.2 (active) | 23.17 / 35.21 | 23.90 / 35.74 |
| Aizee | 21.13 / 42.51 | 17.01 / 34.28 |

Native Jetson development-host measurement, 2026-09-20 (Orin Nano, 25 W mode,
Minerva 1.2, 1,000 poses): full median **54.07 ms**, p99 **72.62 ms**, maximum
**127.91 ms**; all 1,000 checks exceeded the 40 ms budget. Compact median/p99
were 53.77/72.48 ms. The 100 reference comparisons and 100 source audits had no
disagreements. This is an individual-pose benchmark, not the full 25 Hz guard
or a camera/controller-load qualification. See the
[identified host and setup log](../devices/JETSON_DEV_01.md) and
[native validation report](../devices/jetson-dev-01-validation.json).

For each robot, 100 poses were checked against independent FCL queries and 100
against the source contact checker: zero disagreements, missed contacts or extra
contacts in those samples. This is sampled evidence, not exhaustive geometric
or real-time qualification. The verification run passed 231 Python, 3 Rust and
5 JavaScript tests. The viewer also reported the four expected different-arm
link contacts in a deliberately intersecting Minerva pose.

Remaining acceptance work (the [25 Hz plan](COLLISION_25HZ_HARDWARE_PLAN.md)
defines the intermediate milestone and implementation order):

- Achieve the 100 Hz/p99-under-5-ms compute target with accurate geometry.
  Current desktop CAD confirmation takes tens of milliseconds. Convex
  decomposition or simplified meshes need explicit error/contact validation;
  keep-out is an explicit conservative policy, not an equivalent CAD model.
- Qualify solid containment, contact policy and collision-model tolerances.
  Assembly-interface labels are not approved allowed-collision entries.
- Move remaining expensive confirmation work into an appropriate native path
  and benchmark on the Orin Nano under camera, transport and controller load.
- Add calibrated physical braking/speed scaling and a controller-side command
  lease/watchdog. Python dispatch expiry alone does not protect a delayed ZMQ
  message after it has left the process.
- Validate CAD/actuator transforms, slider units, real response delay, braking,
  torque and clearances on hardware before enabling physical enforcement.

### Hierarchy measurements — 2026-09-20 EDT

The new runs use the identical seeded poses as the earlier Minerva 1.2 baseline.
Full results, stage timings, pruning counters and source/binary hashes are in
[the hierarchy report](COLLISION_HIERARCHY_RESULTS.json).

| Host / scope | Samples | Median | p99 | Maximum | Over 40 ms |
|---|---:|---:|---:|---:|---:|
| Desktop, whole robot | 1,000 | 16.37 ms | 26.31 ms | 54.46 ms | 3 |
| Desktop, between arms | 1,000 | 0.50 ms | 2.07 ms | 19.77 ms | 0 |
| Jetson, whole robot | 1,000 | 48.74 ms | 67.18 ms | 76.72 ms | 1,000 |
| Jetson, between arms | 1,000 | 1.96 ms | 6.66 ms | 70.24 ms | 1 |
| Jetson, between arms, 5 mm clearance | 100 | 1.99 ms | 41.76 ms | 85.25 ms | 2 |

Contact-only is the default in this table; the last row also requests surface
distances. Every run compared 100 poses with independent FCL queries with zero
disagreements. The four contact runs additionally audited 100 poses against the
original source-contact checker with zero missed/extra contacts. The clearance
run did not claim a contact audit. Both hosts passed 92 selected Python tests
and five Rust tests; six viewer JavaScript tests passed on the desktop.

Whole-robot checks still include every same-arm/body interface. At the reference
pose, 92 within-arm candidates become 39 CAD contacts, and 12 body candidates
become three contacts; no inter-arm candidate survives. These are unapproved
assembly interfaces, not grounds for silently removing checks. Across all poses,
the hierarchy reduces 3,864 eligible part pairs to about 727 part-AABB tests and
111 CAD candidates on average. Between arms alone, 1,764 eligible pairs reduce
to about 38 part-AABB tests and 2.14 CAD candidates.

On the Jetson, whole-robot median stage costs are 2.40 ms transforms, 1.21 ms
coarse filtering and 44.64 ms fine CAD. The single 70 ms inter-arm contact outlier
was measured in the transform stage; its cause has not been established. The
5 mm clearance tail is in fine CAD distance computation. Therefore neither a
25 Hz complete motion supervisor nor a 100 Hz deadline is qualified. Next work
must address the fine distance path and explicitly review assembly interfaces,
then test accepted and rejected motions under the complete supervisor workload.

### Conservative keep-out measurements — 2026-09-20 EDT

Native ABI 3 compiles an explicitly filtered pair set. Minerva's four finger
links reduce inter-arm part pairs from 1,764 to 900. No FCL queries run in this
mode. The viewer was checked with a crossing pose (two red arm-link pairs) and
the clear reference pose; excluded fingers retain their visuals without
envelope overlays. CAD modes remain available for full-finger diagnostics.

Jetson inter-arm pose latency at 5 mm: **1.91 ms median / 2.34 ms p99 / 3.91 ms
maximum** across 1,000 poses. A synthetic three-path replay paced at 25 Hz
accepted 851 and rejected 149 cycles, with **26.15 / 34.75 / 36.67 ms** cycle
latency and **zero scheduled deadline misses**. Cached immutable motion radii
and ancestry removed repeated geometry work; the earlier uncached run missed
18 scheduled deadlines and is retained in the evidence.

One hundred inter-arm CAD proximity audits had zero missed nonexcluded pairs,
39 additional conservative pair restrictions, and 34 intentionally excluded
finger-pair occurrences. Whole-robot replay still rejects every sampled cycle
because of unapproved assembly interfaces. That is not successful whole-robot
motion supervision. This short synthetic workload excludes physical braking,
cameras, transport and controller load. Full 25 Hz qualification remains open.
See [keep-out results](COLLISION_KEEPOUT_RESULTS.json) and the
[acceptance plan](COLLISION_25HZ_HARDWARE_PLAN.md).

Validation: 98 selected Python tests and six Rust tests pass on the Jetson;
seven viewer JavaScript tests pass on the desktop. Regressions exercise omitted
finger-to-arm/finger-to-finger pairs, retained nonfinger boundaries, no FCL
refinement, crossing motion, policy-bound verdicts and changing prismatic reach.

## Jetson artifact

```powershell
./scripts/build_jetson.ps1 -Offline -Package robot_collision
```

The result is `rust/target/aarch64-unknown-linux-gnu/release/librobot_collision.so`.
The build verifies an AArch64 shared ELF compatible with the JetPack 6 glibc
baseline. This library supplies the native broad phase; CAD confirmation still
requires Python/FCL dependencies on the target. Cross-compilation and ELF
inspection are not Jetson execution or timing validation.
