# Minerva collision-limit review — collision CAD 1.2

The active profile now uses [collision-v1.2.json](collision-v1.2.json), generated
from [collision-v1.2-request.json](collision-v1.2-request.json) against the composed
URDF with **URDF 1.2 No Holes** collision meshes. All 2,994 samples were rerun;
all 14 candidate intervals are identical to the 1.1 results. Existing interface
hypotheses were remapped by source part-instance identity and remain provisional.

The `collision-round-1*` files below remain the detailed 1.1 comparison baseline.

Status: **awaiting visual review**. These are viewer-only candidates, not physical travel calibration.

CAD: [3DP URDF Version 1.1](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/38a2716bb2c6c1ebbe27c900/e/f4466d56fce7b6028aae0a06).

Full source meshes; 2,994 sampled poses. Angular steps: 1°, contact refinement: ≤0.03125°, inward margin: 1°. Jaw steps: 0.5 mm, refinement: ≤0.015625 mm, inward margin: 0.5 mm.

Each arm was evaluated independently against itself and the shared body. Other joints were held at CAD zero, with jaw inputs at +24 mm (left) and −24 mm (right). The opposite arm was excluded. There are 44 explicit provisional bearing, motor-output and rail interface exceptions. None is an approved runtime exemption.

## Candidate ranges

| Joint | Left | Right |
|---|---:|---:|
| j1 | -92.61° to 45.56° | -92.61° to 45.56° |
| j2 | -90.19° to 89.25° | -89.25° to 90.19° |
| j3 | -115.47° to 120.73° | -115.47° to 120.73° |
| j4 | -119.44° to 113.78° | -119.44° to 113.78° |
| j5 | -180.00° to 180.00° | -180.00° to 180.00° |
| j6 | -75.30° to 75.30° | -75.30° to 75.30° |

j5 reached the ±180° source search bounds without contact on both arms. No mechanical end stop was found there. The gripper input ranges are +0.508 to +47.422 mm (left) and −47.422 to −0.508 mm (right); the second jaw follows its mimic relation.

## Visual review

Open `robot-platform view --robot minerva`. Use each joint’s **Lower / Upper** buttons: they restore the exact sweep reference, move that joint to its candidate boundary, and highlight the limit-driving links in amber. The collision is just beyond the candidate range, inside the recorded bracket. **Sweep reference pose** restores the starting arrangement.

Review the 44 part-interface hypotheses and the contact-driving parts in [the report](collision-round-1.json), particularly the corrected shoulder. The source URDF, all mesh hashes, sample traces and contact brackets are recorded. The [request](collision-round-1-request.json) reproduces the sweep offline.

Combining independently found ranges does not establish a collision-free pose or path. The viewer’s **Check pose** remains a strict whole-robot diagnostic, including inter-arm checks and all bearing/rail contacts; it does not apply the provisional interface exceptions. Finite mesh samples do not establish continuous clearance or solid containment. Hardware calibration and motor settings are unchanged.
