# Minerva CAD handoff

Updated 2026-09-19. The active viewer now combines detailed 1.1 visuals with
**URDF 1.2 No Holes collision geometry**. The 1.2 import used 185 requests in one
retrieval; subsequent work ran offline. It reduces collision triangles from
2,437,466 to 916,832 with unchanged joint coordinates. See the
[1.2 import and validation](../urdf/minerva/collision-v1.2/README.md).

The native export is retained as audited reference geometry.
The intended `onshape-to-robot` export succeeded: 17 links, 12 arm revolutes,
four jaw slides and two mimic relations. Native-export failures did not
require rebuilding the source CAD joints or rigid groups.

## Source and export constraint

[Minerva Onshape document](https://cad.onshape.com/documents/359ad50fa475851ee9487588/w/4683cfa4b62fbd0f36348cf6/e/f4466d56fce7b6028aae0a06)
is the user-supplied workspace. The detailed visual snapshot is
[3DP URDF Version 1.1](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/38a2716bb2c6c1ebbe27c900/e/f4466d56fce7b6028aae0a06),
assembly `minerva_urdf`. The 1.1 version was resolved by an authenticated version
lookup and imported once after the user's left shoulder correction. The user
supplied an earlier GLTF-based export in `urdf/minerva/` and confirmed it came
from **1.0**. This historical native export is separate from
the earlier STL export request. All 131 supplied files have source hashes;
mesh resolution and the original archive checksum are unknown. See the
[native import audit and exporter comparison](../urdf/minerva/README.md) and
[source manifest](../urdf/minerva/source_manifest.json).

The user has a free Onshape account and explicitly authorized the intended
API-backed `onshape-to-robot` workflow. Detailed 1.1 and simplified collision 1.2
retrievals are complete. The latter is pinned to version
`0e3aef4a909ae5843d82a11a`; its assembly element is unchanged. Preserve both caches
and perform subsequent conversion and model work offline. Do not start background synchronization or
repeat retrieval unnecessarily. The 1.1 retrieval used 519 network requests,
plus one version lookup; account quotas have not been assumed. See the
[export workflow](../urdf/minerva/onshape-to-robot/README.md).

Current state:

- The user reports CAD preparation complete and selected the version above.
- All 129 GLTF meshes are available locally and load successfully.
- The native-exported tree has 64 movable coordinates, six missing intended arm
  names and six arm names exported as translation-plus-rotation pairs.
  Native export did not preserve the intended articulated mechanism.
- The user confirmed physical materials and mass overrides are not assigned.
  Exported masses total only 25 g and imply density approximately 1 kg/m³.
- Gripper fingers are straight sliding jaws, with 48.2 mm exported stroke per
  jaw. Coupling, aperture, drive ratio, motor zero and CAD-to-motor signs remain
  unverified. Straight travel alone does not establish a symmetric mechanism.

## Intended export: onshape-to-robot

To render and jog the local model, run `robot-platform view --robot minerva`.
The [shared model viewer](MODEL_VIEWER.md) uses the same implementation for
other robots; mechanical sliders do not command motors or apply calibration.

Use the installed `onshape-to-robot` 1.8.2 against the selected frozen version,
following the [export workflow](../urdf/minerva/onshape-to-robot/README.md).
Retrieval and offline conversion are complete. The structural audit passes
with zero errors/warnings. Source masses/inertias are all zero, and 12 source
arm limits are absent; numerical exporter defaults are not physical values.
The two jaw mimic relations are preserved. See the
[manifest](../urdf/minerva/onshape-to-robot/source_manifest.json).

The user selected independent measured limits for each arm and whole-robot
collision checking during operation. Follow the
[collision and limit calibration plan](architecture/COLLISION_AND_LIMIT_CALIBRATION.md)
instead of treating the exporter's angular bounds as approved travel.

The native URDF and `onshape-to-robot` interpret the assembly differently.
The latter recognizes the `dof_` naming convention, treats Cylindrical mates
as revolute joints, and merges rigid mate groups. Compare its output with the
intended joint contract before requiring any source mate changes. The earlier
blanket recommendation to rebuild the CAD mate graph is withdrawn.

Retain the native GLTF export and audit as a separate reference. Preserve source
checksums, export settings and tool versions for both outputs. Verify actual
joint names, limits, frame signs and jaw coupling after conversion. Missing
source materials/mass overrides remain an independent blocker for physical
dynamics regardless of exporter. A STEP or mesh-only export does not supply
the articulated joint model required for this handoff.

## Mate names

Use these names on the moving mate features. The `dof_` convention is for the
existing `onshape-to-robot` workflow: it produces robot joint names without that
prefix; Slider mates produce prismatic joints and Revolute or Cylindrical mates
produce revolute joints. Native Onshape export must be checked separately.
[Exporter naming reference](https://onshape-to-robot.readthedocs.io/en/latest/design.html).

| CAD mate | Type | Meaning |
|---|---|---|
| `dof_left_arm_j1` through `dof_left_arm_j6` | Revolute or Cylindrical | Left arm rotation, matched to existing motor IDs 4 through 9 |
| `dof_right_arm_j1` through `dof_right_arm_j6` | Revolute or Cylindrical | Right arm rotation, matched to existing motor IDs 4 through 9 |
| `dof_left_gripper_finger_a_slide`, `dof_left_gripper_finger_b_slide` | Slider | Proposed names for the left gripper's two moving jaws |
| `dof_right_gripper_finger_a_slide`, `dof_right_gripper_finger_b_slide` | Slider | Proposed names for the right gripper's two moving jaws |

Name only moving jaws this way; a stationary jaw belongs to its rigid body.
Verify the arm's actual actuator-to-mate assignment before naming; mirrored
geometry does not establish motor direction. Record what finger `a` and `b`
refer to using a diagram or identifiable CAD part names.

The [shared robot definition](../src/robot_platform/robots/definitions/minerva.json)
retains one **motor-angle input in radians** per gripper: `left_gripper` and
`right_gripper`, CAN ID 10 on their respective buses. Do not assign those names
to a distance-valued Slider mate. If the rotary drive itself is modeled, its
Revolute mate may use `dof_left_gripper` or `dof_right_gripper`. Otherwise the
future simulation adapter will map motor angle to jaw displacement. The jaw
coordinates are internal mechanics, not extra independently commanded motors.

Reserved `head_pan`, `head_tilt` and `lift` remain inactive in software. Their
presence in CAD must not activate routes or alter the 17-slot recording schema.

### Rigid link groups

Group mates are appropriate for parts that form one rigid link. Keep separate
groups on opposite sides of every moving joint; grouping across a joint would
remove its intended motion. The locally inspected `onshape-to-robot` 1.8.2
implementation merges unsuppressed root-assembly mate groups into bodies.
There is no need to rebuild working rigid groups solely because the design
guide illustrates part/subassembly instances. Verify the actual export's link
tree and aggregated mass properties, especially for nested assemblies.

## Setting limits for sliding fingers

1. Give each moving jaw a Slider mate relative to its gripper body. Align the
   mate connector's Z axis with the guide rail. Keep the body attached to the
   wrist; constrain the jaw against unwanted rotation.
2. Choose and document the fully closed reference. Ideally arrange the mate
   connectors so its slider coordinate is zero there and positive travel opens
   the jaw. If using another reference, record its closed coordinate explicitly.
3. Double-click the Slider mate, enable **Limits**, and enter the closed/open
   coordinate bounds with explicit length units, such as `mm`. A negative
   opening direction needs negative coordinates; do not substitute motor angles.
   [Onshape Slider mate instructions](https://cad.onshape.com/help/Content/Assembly/slider_mate.htm).
4. Determine stroke from the actual guide/mechanical stops and jaw geometry.
   The total aperture is the gap between opposed gripping surfaces. For two
   equally moving jaws only, each jaw's stroke is
   `(maximum gap - minimum gap) / 2`. For example, a hypothetical 0–60 mm gap
   gives 0–30 mm per jaw. These are explanatory numbers, not Minerva limits.
   One moving jaw instead covers the entire gap change.
5. If the mechanism enforces equal opposing travel, add a **Linear relation**
   between the two sliders with ratio 1:1. Use its **Reverse** option as needed
   for the selected mate-axis directions. Verify that both jaws open together.
   Do not add a duplicate relation if the modeled drive already constrains them.
   [Onshape Linear relation instructions](https://cad.onshape.com/help/Content/Assembly/linear_relation.htm).
6. Accept the mates, then right-click a driving mate and **Animate** the
   assembled mechanism. Check both endpoints and intermediate travel for
   interference and inconsistent constraints. The preview inside a mate dialog
   can ignore other mates, so it is not the final coupled-motion check.
   [Onshape mate animation reference](https://cad.onshape.com/help/Content/Assembly/mates.htm).

Existing `config/minerva_calibration.json` gripper endpoints are actuator
radians, not jaw millimetres. Some legacy UI constants also describe a normalized
0–1 opening. Neither supplies physical jaw stroke. The eventual conversion must
specify motor zero/sign, transmission, jaw displacement and total aperture;
convert CAD millimetres to model metres at that boundary. CAD travel alone does
not validate grip force or actuator timing.

## Local audit before physics integration

The native Minerva import provides plausible geometry scale, but its masses
and exported joint graph failed the [intake audit](../urdf/minerva/README.md).
This result does not diagnose the source mate graph for `onshape-to-robot`.
The saved gravity fits are empirical joint-torque models, and the fast simulator's
parameters are synthetic. Neither is an independent mechanical reference.
The legacy Aizee kinematics helper assumes revolute joints and must not be
used unchanged for Minerva's sliding jaws.

- Verify each exported moving joint's name, axis, zero and units. Check that
  rigid components stay attached and moving mechanisms remain articulated.
- Record jaw endpoints and coupling for each gripper separately. Preserve the
  relation explicitly in the simulator and check it after export; do not assume
  the selected exporter/engine preserves an Onshape Linear relation.
- Audit link masses, centres of mass and inertia, including motors and attached
  hardware. Record camera/tool frames and collision-geometry simplifications.
- Preserve the selected `3DP URDF Version 1.0` URL, assembly/configuration,
  export settings and tool version with the downloaded files. Revisions require
  a new source version; keep the provenance of each import.
  [Onshape version instructions](https://cad.onshape.com/help/Content/Document/versions_and_history.htm).
- Check mesh paths resolve locally, metre/radian units are consistent, and the
  joint graph has a single intended base with no disconnected or duplicate links.
- Map all 12 arm joints and both gripper motor inputs explicitly. Verify finite
  joint limits, zero poses, axis signs, jaw endpoints and coupling; preserve the
  distinction between motor radians and finger metres.
- Check positive physical masses, centres of mass and valid inertia tensors;
  identify missing hardware and separate visual meshes from collision shapes.
- Load the audited model, exercise joint endpoints and intermediate poses,
  then check gravity, contacts and coupled motion before physical claims.

The current synthetic simulator and upcoming Rust/virtual-CAN work can proceed
while exporter comparison and physical-property work proceed. Coupled contact
and geometry validation are later gates in the
[simulation plan](architecture/SIMULATION_VALIDATION_PLAN.md).
