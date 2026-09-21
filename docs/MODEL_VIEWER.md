# Shared robot model tools

The browser viewer, static audit and collision checker use one mechanical model
profile. Robot names, arm counts, joint names and presentation groups are data.
The tools have no Aizee/Minerva branches. Repository naming is unchanged.

## Launch

From the checkout:

```sh
python -m pip install -e ".[viewer,collision]"
python -m robot_platform view --robot minerva
```

Open **http://127.0.0.1:8765**. The model selector also contains Aizee. On the
current Windows development machine, use `.venv-platform/Scripts/python.exe`
instead of `python`. `--port` changes the local port.

The viewer runs independently of collectors, ROS, Onshape, cameras and motor
services. JavaScript assets are bundled locally; no CDN or CAD API is used.
`collision` is optional: without it, rendering/jogging works and a collision
query reports its missing dependency. Freshly installed wheels include the UI
but do not contain the large CAD library; pass an external profile/URDF or set
`ROBOT_PLATFORM_REPO` to a hydrated source checkout.

The WebXR server's existing `/preview` link mounts **the same shared viewer**
at `/preview/`. Its old robot-specific page and permissive collision endpoint
have been removed. The viewer does not mirror a live Quest session, write
motor alignment, or change accepted calibration files. Existing WebXR control
and its in-headset Aizee mirror are separate migration work.

## Controls

- Select a model; groups and controls come from its profile and URDF.
- Drag to orbit, right-drag to pan, scroll to zoom. Use Fit, Front or Top to
  reframe the complete model. Coordinates are URDF-native, with Z up.
- Sliders, numeric fields and jog buttons articulate revolute, continuous and
  prismatic joints. Angular values use radians, translations use metres.
  Jog increments are 0.02 rad or 0.001 m; these are display steps only.
- Mimic joints follow their source, including offsets, negative multipliers
  and chained relations. Their bounds constrain the source slider's feasible
  viewing interval. They do not get independent controls.
- Reset returns to the nearest feasible zero configuration. Continuous joints
  use a finite viewing window rather than claiming a mechanical end stop.
- Export/Load pose stores model identity, URDF hash and complete independent
  positions. Import rejects a different URDF revision or invalid coordinates.
  Exported poses also work as `check-collision --scene` inputs.
- **Live collisions** checks the displayed pose automatically after
  joint changes (on by default), including contacts between arms. Contacting
  links turn red and the viewport banner reports the result. **Collision
  geometry** shows the source collision shapes and requests an immediate check
  when enabled; exact contacting shapes turn red in that overlay. **Check pose**
  also runs a check when live checks are off. Link axes are a visual overlay.
- **Source CAD** checks the original meshes with FCL. **Accelerated CAD** first
  rejects distant groups, links and parts with native bounds, then confirms every remaining
  contact against the same source CAD. Both modes display the actual source
  collision shapes.
  Build the native library with `./scripts/build_collision.ps1 -Test`; without
  it the second mode falls back to FCL CAD. See the
  [implementation status](architecture/COLLISION_RUNTIME_IMPLEMENTATION.md).
  Provisional
  bearing/rail interface pairs from a revision-matched limit review appear in a
  separate amber list; they remain in the strict result and are not approved
  clearances. Other contacts, including different parts on those same links,
  remain red. Models without a review classify every contact as red.
- **Conservative keep-out** skips CAD refinement and reports the enclosing
  boundary as a motion restriction. This mode requires the native library
  (ABI 3) and starts with **Between branches / arms**. **Keep-out envelopes**
  shows those boundaries. A 5 mm **Inspection gap** is the default viewing
  threshold, not a calibrated stopping margin. Boundary violations turn red,
  including interfaces that CAD modes label provisionally. The profile's
  `keepout_ignored_links` map supplies explicit link names and reasons; Minerva
  excludes all four sliding finger links from every coarse pair, including
  finger-to-arm pairs. The wrists/gripper bodies remain included. The viewer
  lists these exclusions and omits their envelopes; finger visuals remain.
  Source CAD and Accelerated CAD still check and display the fingers.
- **Check scope** defaults to **Whole robot**. **Between branches / arms** checks
  only different branches; **Within each branch / arm**, **Body contacts** and
  **Environment contacts** inspect the other scopes separately. A partial result
  is labeled **selected scope only** in the viewport. Accelerated results show
  coarse and CAD timings and the number of candidates reaching confirmation.
  Changing scope, mode or inspection gap invalidates any in-flight result.
- Joint edits are debounced by 30 ms. Only one request runs at a time and queued
  edits coalesce to the latest complete pose. Changes immediately invalidate old
  highlights; late responses cannot label a newer pose or model as checked.
  Missing dependencies or check failures display **unavailable**, never clear.
  These are sampled-pose diagnostics; they do not check swept paths or stop
  jogging/hardware motion.

## Collision-limit review

A model profile may reference a `limit_review` JSON report produced by
`robot-platform sim sweep-limits`. The viewer validates the URDF and every mesh
hash before applying candidate ranges. The imported URDF and hardware calibration
stay unchanged. Reset uses the recorded sweep reference.

Each **Lower / Upper** button restores that complete reference pose, then moves
one joint to its inward candidate boundary. Amber links identify the parts that
caused the limiting contact. The full report includes exact contact poses,
sample traces, margins, resolution and provisional part-interface exceptions.
An unresolved contact locks the affected viewing range. A search bound without
contact is labeled explicitly; it is not a discovered mechanical endpoint.

Each declared branch is swept against itself and the shared body, with other
branches excluded. Other joints stay at the reference. Arbitrary combinations
of the resulting intervals can still collide. The whole-robot **Check pose**
in CAD modes retains both arms and has no interface exclusions. Keep-out applies
its separately declared finger policy. Live checks use the selected
scope, with whole-robot coverage by default. Bearing/rail assembly contacts listed in the review are
labeled separately for display; contact results still contain them.

To rerun a request against its pinned model, choose a new output path:

```sh
robot-platform sim sweep-limits --model urdf/minerva/model.json --request urdf/minerva/limits/collision-round-1-request.json --output /path/to/new-review.json
```

Requests declare the exact URDF hash, full reference positions, branch root
joints, joint lists, angular/linear sampling steps, refinement tolerances,
inward margins and explicit part-interface hypotheses with reasons. Interfaces
are not automatically exempted from the home pose. The report is provisional
evidence for visual review; it does not enable a hardware motion gate.

## Geometry fidelity

Display meshes preserve every source triangle and are cached in memory.
Automatic simplification is disabled: it distorted joint housings and removed
thin CAD features. Collision checks use the selected collision geometry.
Minerva combines detailed 1.1 visuals with user-simplified **1.2 No Holes** CAD
collision meshes; the two roles are separate URDF elements. Its collider overlay
therefore has fewer details than its visible parts. See the
[1.2 import and validation](../urdf/minerva/collision-v1.2/README.md). Flat shading
preserves hard CAD edges and planar faces. URDF RGBA colors are preserved;
texture materials are not rendered. Loading may take tens of seconds on the first
selection. Restart the viewer after changing source models or profiles; it
does not watch files or synchronize CAD.

## Add any supported robot model

Optional `collision_branches` maps labels to subtree root joints, independently
of the `groups` used to arrange controls. For Minerva:

```json
"collision_branches": {"left": "left_arm_j1", "right": "right_arm_j1"}
```

Every descendant, including fixed attachments and coupled fingers, belongs to
that branch. Roots must exist and cannot repeat or nest. Links outside explicit
branches remain in the body scope, even if articulated. Without this field,
each first movable joint below the base starts a branch. A robot with a shared
moving torso can declare its arm roots explicitly. Grouping does not authorize
contact exclusions. Optional `keepout_ignored_links` is a map of exact existing
URDF link names to nonempty reasons, independent of robot identity. It applies
only to Conservative keep-out, not CAD modes or collision-limit sweeps. Avoid
wildcard/name-pattern exclusions; the declared list is included in each verdict.

A standalone URDF needs no platform motor definition:

```sh
robot-platform view --urdf /path/to/robot.urdf --package-root /path/to/package
```

For labeled groups, multiple mesh packages or model notes, create a profile
next to the robot assets:

```json
{
  "schema_version": 1,
  "id": "inspection-machine",
  "title": "Inspection machine",
  "urdf": "robot.urdf",
  "front_direction": [1, 0, 0],
  "package_root": ".",
  "packages": {"machine_description": "."},
  "groups": {
    "Turntable": ["rotation"],
    "Linear stage": ["carriage"]
  },
  "notes": ["Prototype geometry; travel bounds are not physically validated."],
  "provisional_limits": ["rotation"]
}
```

Paths resolve relative to the profile; package aliases explicitly map ROS mesh
package names to local directories. Referenced meshes cannot escape those
directories. Missing/unhydrated assets produce errors instead of substitute
geometry. Mesh formats: STL, OBJ, PLY, embedded GLTF and GLB. Primitive boxes,
spheres and cylinders work without mesh files. External GLTF buffers/textures,
remote meshes, floating/planar joints and closed loops are not supported by this
viewer; unsupported structures fail explicitly.

`front_direction` points from the model toward a viewer standing in front of
the robot, in URDF coordinates. It must be a nonzero horizontal vector (Z up)
and defaults to +X. Minerva's CAD front is −Y; Aizee uses +X. It sets the Front
camera direction and aligns Top with the same left/right orientation, placing
the robot's front toward the bottom of the screen. Joint coordinates and
collision geometry are unchanged.

Groups are optional. Unlisted independent joints appear automatically. Group
members must be mechanical URDF names, not actuator names. Built-in examples:
[Aizee](../urdf/aizee/model.json) and [Minerva](../urdf/minerva/model.json).
The latter distinguishes motor-radian grippers from the CAD jaw slides.

```sh
robot-platform view --model /path/to/model.json
robot-platform view --models-dir /path/to/catalog --robot inspection-machine
robot-platform sim audit-model --model /path/to/model.json
robot-platform sim check-collision --model /path/to/model.json --scene /path/to/pose.json
```

A catalog discovers `*/model.json`; adding a robot requires no Python/JavaScript
changes. Existing explicit `--urdf` audit/collision commands remain supported.

## Software boundaries and validation

| Layer | Responsibility |
|---|---|
| `robot_platform.models` | Profile loading, local asset resolution, mechanical joint tree, mimic relationships, units and presentation metadata |
| `robot_platform.viewer` | Generic local HTTP UI, rendering and pose files; no hardware command routes |
| `robot_platform.simulation` | Shared static audit and FCL collision diagnostics |
| `robot_platform.robots` / `settings` | Actuator/recording contract and application/deployment selection |

Future tools should consume these contracts and select behavior by capabilities
and device interfaces. A robot identifier belongs in a profile/catalog, not in
an algorithm's joint list. Robot-specific calibration and actuator mappings
remain explicit data; the viewer does not invent those mappings.

Tests cover an unrelated wheel/linear-stage model, negative/chained coupling
constraints, malformed profiles/graphs, complete pose validation, package and
LFS handling, local HTTP asset isolation, mounted routes and profile reuse by
audit/collision. Viewer collision tests cover two independent branches, exact
shape-level interface classification, and HTTP failures. Browser scheduling
tests cover rapid edits, stale results, model switches, disabling live checks
and backend errors:

```sh
python -m pytest tests/unit/test_model_viewer.py tests/unit/test_limit_sweep.py tests/unit/test_collision.py tests/unit/test_collision_cli.py -q
node tests/viewer/collision-checks.test.mjs
```

Both repository models have been loaded. Browser rendering and jogging are
checked separately from server/unit tests.

This tool performs kinematic inspection. It neither simulates validated dynamics
nor enforces a runtime motion gate. Minerva's viewer uses provisional collision
sweep ranges; the source URDF ranges and hardware limits remain unchanged.
FCL mesh queries test triangle intersections, and a clear sampled pose does not
prove a clear path or solid containment. See the
[collision and limit calibration plan](architecture/COLLISION_AND_LIMIT_CALIBRATION.md).
