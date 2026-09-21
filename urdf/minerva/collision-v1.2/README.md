# Minerva simplified collision CAD — URDF 1.2 No Holes

Imported once from the user's [frozen Onshape version](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/0e3aef4a909ae5843d82a11a/e/f4466d56fce7b6028aae0a06)
on 2026-09-19, using `onshape-to-robot==1.8.2`.

## Active model

The shared [model profile](../model.json) now loads [the composed URDF](../minerva.urdf):

- Detailed **1.1 visuals**, colors, joint tree and coordinate frames.
- **1.2 collision meshes**, imported directly from the simplified CAD.
- The same 12 arm rotations, four jaw slides and two mimic relations.
- New [collision-limit review](../limits/collision-v1.2.json) and
  [broad-phase bounds](../collision/broadphase-v1.2.json), pinned to this composition.

The collision checkbox displays the 1.2 shapes. The visible robot retains its
detailed appearance. Convex bounds only reject distant pairs; final contacts
still use the imported collision triangles. No collision exclusions were added.

`minerva.urdf` in **this directory** is the normalized collision-only source
export. It is separate from the composed URDF one directory above. The two
export-generated gripper-body link names changed, but the composer maps links
by their unchanged joints. No joint origins, axes, limits or mimic parameters
changed. [composition.json](composition.json) records that mapping and hashes.

## Retrieval cost and reuse

The single retrieval made **185 HTTP requests** (including redirects), versus
519 for the previous full import. `no_dynamics: true` and a constant color skip
per-part mass and color lookups. Configuration and mesh data were still fetched
from the pinned version. No version-discovery request was needed.

The trusted local snapshot, request log and raw exporter output remain under
ignored `simulation_runs/minerva-cad-v1.2/`. The exporter's existing cache was
preserved. Conversion, normalization, comparison, composition, sweeps and
benchmarks ran offline. Do not retrieve again to tune meshes or output format.
The [source manifest](source_manifest.json) records all promoted source hashes.

No mass/inertia data was requested. The collision-source [audit](audit.json)
therefore reports 17 zero-mass errors and 17 degenerate-inertia warnings. These
are expected for this geometry-only export; it must not be used as a dynamics
model. Kinematic and mesh-path checks passed. The composed model retains the
old inertials, which were already unvalidated numerical placeholders.

## Measured changes

| Measure | Detailed 1.1 collision CAD | Simplified 1.2 collision CAD |
|---|---:|---:|
| Collision instances | 129 | 91 |
| Collision triangles | 2,437,466 | 916,832 |
| Full contact query median | 39.01 ms | 23.17 ms |
| Full contact query p99 | 67.15 ms | 35.21 ms |

Both timing runs used the same 1,000 complete robot poses on the development
Windows host. Loading is excluded. The triangle count fell 62.4%; median query
time fell 40.6%. This does **not** yet meet the 100 Hz / p99-under-5-ms target.

Validation in [validation.json](validation.json):

- All 2,994 single-joint sweep samples were rerun; all 14 candidate viewer
  intervals are unchanged. The 44 provisional interface hypotheses were mapped
  by retained source instance identity and remain unapproved.
- Across 100 old/new poses, no collision between non-adjacent links disappeared.
  One individual pair, `RB03 OUTPUT <2>` / `Part 66 <2>`, changed from contact
  to **0.631 mm clearance** at sample 49. Other contacts retain the colliding
  link pair in that pose. Its full pose is recorded for geometry review.
- Against the current 1.2 source checker, the accelerated checker had zero
  missing or extra contacts and zero reference disagreements in 100 poses.
- Composer/model/runtime regressions: 45 tests passed, including rejection of
  changed joint axes and missing replacement geometry.

These are sampled comparisons, not a conservative-coverage proof or physical
clearance validation. The removed parts and changed-contact example remain
explicit evidence. No hardware collision enforcement is enabled.

## Reproduce the composition offline

`scripts/compose_collision_model.py` is robot-independent. It rejects changed
joint coordinates or topology, allows renamed links, preserves detailed visuals,
and rewrites local mesh paths. Use new output/report paths:

```sh
python scripts/compose_collision_model.py --visual-profile urdf/minerva/onshape-to-robot/model.json --collision-urdf urdf/minerva/collision-v1.2/minerva.urdf --output urdf/minerva/minerva-next.urdf --report urdf/minerva/collision-v1.2/composition-next.json
```

The [detailed 1.1 reference profile](../onshape-to-robot/model.json) also works
with the shared viewer and benchmark tools through `--model`.

Future collision optimization can use these cached meshes locally. Remaining
high-detail assets include the two RS00 motors (183,560 triangles together) and
the two RB04 bearing mounts (141,886 triangles together).
