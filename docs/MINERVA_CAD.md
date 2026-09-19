# Minerva CAD preparation

Updated 2026-09-19. This records preparation decisions, not an imported or
validated mechanical model.

## Source and export constraint

[Minerva Onshape document](https://cad.onshape.com/documents/359ad50fa475851ee9487588/w/4683cfa4b62fbd0f36348cf6/e/f4466d56fce7b6028aae0a06)
is the user-supplied source. Its `/w/` URL identifies a workspace; a specific
export version has not been selected. The assembly has not been inspected or
downloaded in this preparation step.

The user has a free Onshape account and requested restraint with API access.
Do not bulk-fetch the document, start background synchronization, or repeatedly
run an API-backed exporter. Prefer user-provided manual exports where practical.
Before a future API export, establish its scope and request budget; account
quotas and exporter request counts have not been verified. Cache source files
locally and perform subsequent mesh/model work offline.

Current user-reported state:

- Joint DOFs have not yet been named.
- Gripper fingers are straight sliding jaws; their limits are not set.
- Jaw stroke, coupling, drive ratio, motor zero and CAD-to-motor signs remain
  unknown. Straight travel alone does not establish a symmetric mechanism.

## Mate names

Use these names on the moving mate features. The `dof_` convention is for the
existing `onshape-to-robot` workflow: it produces robot joint names without that
prefix; Slider mates produce prismatic joints and Revolute mates produce
revolute joints. This is preparation only; no exporter is being run.
[Exporter naming reference](https://onshape-to-robot.readthedocs.io/en/latest/design.html).

| CAD mate | Type | Meaning |
|---|---|---|
| `dof_left_arm_j1` through `dof_left_arm_j6` | Revolute | Left arm, matched to existing motor IDs 4 through 9 |
| `dof_right_arm_j1` through `dof_right_arm_j6` | Revolute | Right arm, matched to existing motor IDs 4 through 9 |
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

## Before the first mechanical-model export

- Name and verify each moving mate, its axis, zero and units. Attach rigid
  components consistently; keep moving mechanisms articulated.
- Record jaw endpoints and coupling for each gripper separately. Preserve the
  relation explicitly in the simulator and check it after export; do not assume
  the selected exporter/engine preserves an Onshape Linear relation.
- Audit link masses, centres of mass and inertia, including motors and attached
  hardware. Record camera/tool frames and collision-geometry simplifications.
- Create a named Onshape version such as `minerva-sim-v1` after preparation,
  and record its version URL, assembly/configuration and export tool version.
  Versions are immutable snapshots.
  [Onshape version instructions](https://cad.onshape.com/help/Content/Document/versions_and_history.htm).
- Export once within the agreed scope, preserve source files/checksums locally,
  and audit scale, kinematic tree, axes, jaw coupling and mass before dynamics.

The current synthetic simulator and upcoming Rust/virtual-CAN work can proceed
while these CAD details are completed. Coupled contact/geometry validation is a
later gate in the [simulation plan](architecture/SIMULATION_VALIDATION_PLAN.md).
