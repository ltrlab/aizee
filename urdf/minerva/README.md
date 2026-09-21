# Minerva models and native-export audit

The active [model profile](model.json) uses [minerva.urdf](minerva.urdf), composed
from detailed 1.1 visuals and **URDF 1.2 No Holes collision geometry**. See the
[1.2 import, validation and timing results](collision-v1.2/README.md). The original
native 1.0 export below is retained as reference evidence.

Audited 2026-09-19. **This native export is reference geometry only.** Its
joint-graph failures do not establish that the source CAD must be rebuilt.
The intended `onshape-to-robot` export now has the correct 17-link, 16-joint
graph and two jaw mimic relations; see the
[export results](onshape-to-robot/README.md). The current simulator still uses
its explicitly synthetic joint model.

## Source

The user confirmed these extracted files came from
[3DP URDF Version 1.0](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/b17483f38edc0b337da34858/e/f4466d56fce7b6028aae0a06),
assembly `minerva_urdf`. The URDF header identifies Onshape exporter `1.221`.
These are **GLTF** assets, separate from the earlier STL export request. Their
resolution is unverified. No original ZIP was supplied.

- [Raw URDF](urdf/minerva_urdf.urdf): retained unchanged.
- `meshes/`: 129 self-contained GLTF files; 4,455,390 triangles in total.
- `launch/`: original ROS launch metadata, retained for provenance; it is not
  a launcher for this repository and does not establish a ROS package.
- [Source manifest](source_manifest.json): source identity, intended joints,
  and SHA-256 hashes for all 131 original files (252,204,626 bytes).
- [Automated audit](audit.json): repeatable structural checks. Passing those
  checks alone does not establish physical validity.

Resolve `package://minerva_urdf/` to this directory. The GLTF files are configured
for Git LFS; hydrate them after checkout before auditing or loading meshes.

## Native-export findings

These observations describe Onshape's native exporter `1.221` output. They
are retained as evidence for comparison, not as a verdict on the assembly's
compatibility with `onshape-to-robot`.

| Check | Observed result | Consequence |
|---|---|---|
| File completeness | Every mesh exists, embedded buffers match declared sizes, all 129 meshes load with trimesh 4.12.2 | No new export is needed solely to obtain meshes |
| Tree structure | 178 links, 177 joints, one connected tree rooted at `root` | Structurally valid tree; this native output does not represent the intended mechanism |
| Motion | 113 fixed, 42 prismatic, 22 continuous joints; 64 movable coordinates | Expected 12 arm rotations and 4 jaw slides, with each jaw pair coupled |
| Intended arm names | Six absent; six represented as a slider plus continuous rotation | Cannot map the 12 arm actuator coordinates correctly |
| Extra motion | 16 planar decompositions contribute 48 movable coordinates | Unwanted translations and rotations survive export |
| Bounds | 38 sliders have placeholder bounds of -10,000 to +10,000 metres | These are not usable mechanical limits |
| Loop closures | 10 dummy closure leaves, no corresponding exported constraints | Native output does not preserve those closures as constraints |
| Jaw travel | Four intended slides present; 48.2 mm stroke each | Useful CAD bounds; aperture and motor conversion remain unverified |
| Coupling | No mimic or transmission elements | Jaws would move independently without an explicit relation |
| Mass | All listed inertials sum to 0.0250045245 kg, including the stand | Unusable for real torque/gravity dynamics |
| Inertias | 139 finite, positive tensors satisfy triangle inequalities | Numerically valid tensors still have incorrect physical magnitude |
| Collision | No collision elements | Contact geometry must be built separately |

### Exact arm-joint problems

| Side | Missing intended names | Exported as prismatic + `_1` continuous |
|---|---|---|
| Left | `dof_left_arm_j1`, `dof_left_arm_j5`, `dof_left_arm_j6` | j2, j3, j4 |
| Right | `dof_right_arm_j1`, `dof_right_arm_j3`, `dof_right_arm_j5` | j2, j4, j6 |

Freezing translations and renaming the `_1` rotations would not repair this
native output. For example, its left j4 rotation only owns one downstream
link; left j2 owns five and reaches no jaw. Several arm paths instead run
through anonymous planar joints. The intended `onshape-to-robot` 1.8.2 exporter
handles Cylindrical mates as revolute joints and merges rigid mate groups.
Its output now confirms the intended graph for this same frozen assembly.
The earlier blanket source-rebuild recommendation is withdrawn.

### Scale, density and grippers

Geometry dimensions appear consistent with metres: a nominal 42 mm bearing
measures about 42 mm across. With all joint coordinates zero and all URDF
transforms applied, visual bounds are approximately
**0.651278 × 0.192180 × 1.132670 m**. This is an export sanity check, not an
independent measurement of the robot.

Across all 129 meshes, exported mass divided by tessellated volume is
**0.99915–1.00408 kg/m³** (median 1.00051). The export effectively uses density
1 kg/m³ throughout. The user confirmed materials and mass overrides have not
yet been assigned. Populate those source properties, then compare the next
export against Onshape's mass-properties display and actual component
specifications. Do not multiply all masses by an invented factor.

Left jaw limits are approximately `[0, 0.0482] m`; right jaw limits are
`[-0.0482, 0] m`. Different signs are compatible with different coordinate
origins/directions. Stroke alone does not establish the aperture or motor
transmission. `left_gripper` and `right_gripper` remain motor radians in the
shared robot contract. Four mechanical jaw coordinates do not add four
independent actuators.

Offline conversion to STL was checked on representative bearing, rail,
stand, motor and structural meshes. Preserve URDF visual origins and mesh
coordinates during conversion: 26 visual origins contain offsets, and many
meshes retain assembly coordinates. Recentring them would break alignment.
Use simplified collision shapes rather than the full 4.46-million-triangle
visual model for contacts.

## Exporter comparison and acceptance sequence

1. Completed the authorized [onshape-to-robot workflow](onshape-to-robot/README.md)
   against the frozen version: 12 arm rotations, four jaw slides, two mimic
   relations and correct independent arm chains. The native import is retained
   for comparison. Source angular limits are absent; dynamic calibration is
   planned. Exported +/-pi bounds are not authorized physical travel.
2. Verify source mass properties, including motors, bearings, printed parts
   and attached hardware. Assign appropriate materials or justified mass
   overrides where missing; compare the next export to the displayed values.
   Onshape's mass display excludes parts without assigned materials.
   [Mass properties](https://cad.onshape.com/help/Content/View/mass_properties_tool.htm).
3. Save a new immutable version after any physical-property or justified mate
   changes. Preserve this raw snapshot and its hashes. Retrieve the selected
   revision deliberately, retain its cache, and perform later conversion work
   offline. Audit the resulting joint tree and masses before engine integration.
   GLTF remains acceptable as reference geometry.
4. Verify exported jaw coupling and measure the motor-angle-to-jaw relation, then review
   collision shapes and verified camera/tool frames locally. Compare link
   masses and dimensions to independent references.
5. Only then load the articulated physics model and run endpoint, gravity,
   contact and coupled-motion tests. Actuator timing and torque validation
   still require real measurements and held-out runs.

## Re-run the offline structural audit

From the repository root, with the shared package and simulation dependencies:

```powershell
python -m robot_platform sim audit-model --urdf urdf/minerva/urdf/minerva_urdf.urdf --package-root urdf/minerva --expectations urdf/minerva/source_manifest.json
```

Exit code **2** is expected for this import: it fails the joint contract.
`--output <new-file.json>` writes a new report without overwriting an existing
one. The command performs local checks only. Density plausibility, source
constraints and actuator calibration still require the review above.
