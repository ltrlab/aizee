# Minerva Head V1 export with onshape-to-robot

This directory holds the **Head V1+urdf** version of the `minerva_urdf` assembly:
[3DP URDF 1.1](../onshape-to-robot/README.md) plus the head on a two-joint neck.
It is a source and preview model. The active model ([../model.json](../model.json))
still composes the 1.1 visuals with the 1.2 collision geometry. It gains the neck
only once its collision model covers the head; see the
[head and neck handoff](../../../docs/development/HEAD_NECK_MODEL.md).

- Source: [Head V1+urdf](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/349129ac06492b67378d6724/e/f4466d56fce7b6028aae0a06),
  pinned in [config.json](config.json); `onshape-to-robot==1.8.2`.
- [URDF](minerva.urdf): **19 links, 18 joints**. That is 12 arm revolutes, 2 neck revolutes, 4 jaw slides and
  2 jaw mimics. It passes the [structural audit](audit.json) against [expected_joints.json](expected_joints.json).
- [Manifest](source_manifest.json): source identity, the local normalization, joints, masses, the head's
  geometry, file hashes and every retrieval request.

## The neck and the head

| Joint | Parent → child link | Origin at zero (mm) | Axis |
|---|---|---|---|
| `neck_z` (pan) | `ikea_desk_leg` → `part_6` | (0.1, −1.5, 1131.2) | +z |
| `neck_y` (tilt) | `part_6` → `awg_rgb_camera_5` | (−18.0, −1.5, 1187.8) | −x |

The exporter names each link after one of its parts:

- `part_6` is the pan bracket.
- `awg_rgb_camera_5` is the whole head: body, covers, front panel and camera.

Both neck ranges are the exporter's ±π default, not the servos' travel.

The centre of the `AWG_RGB_CAMERA` part is the head camera's view origin (the user, 2026-10-25). At zero it is
at (32.8, −110.5, 1269.9) mm in the robot frame, or (−109.0, −82.1, −50.8) mm in the head link's frame. The
part is a module about 5 mm across.

## Against 1.1

- **Arm and jaw joints:** all 16 joint frames match 1.1 within 0.005 mm and 0.001°.
- **Existing parts:** their geometry is unchanged apart from re-tessellation (vertices within 0.13 mm).
  - Onshape renumbered six stand parts: Part 148–153 are now Part 282–287.
  - The Part 41/42 instance suffixes moved.
  - Because of these renames, this export cannot replace the 1.1 directory in place: the active model
    references the 1.1 file names.
- **Masses:** 12 of the 157 retrieved parts now carry material mass. Eight arm links weigh 0.31–2.3 kg; the rest
  are zero. The model is still kinematic only.

## Retrieve once, then convert locally

On 2026-10-25 the export used **631 Onshape requests**, all successful:

- **First attempt, 3 requests:** stopped by a local console error printing a Chinese part name. Those 3
  responses were cached and reused.
- **Second attempt, 628 requests.** The conversion then ran offline with the network blocked.

The user has a limited Onshape API budget. A re-export after a change to the assembly only reuses the cached part
data in `~/.cache/onshape-to-robot/`; a change to a Part Studio downloads its parts again.

The commands are the [1.1 export's](../onshape-to-robot/README.md#retrieve-once-then-convert-locally). Run them
from this directory with its own ignored `.env`, and set `PYTHONIOENCODING=utf-8` so the exporter can print
Chinese part names.

After conversion, two local normalizations were applied:

- `package://meshes\x.stl` references became `meshes/x.stl`.
- The three servo-horn parts named in Chinese became ASCII. 金属舵盘 means metal servo horn; 从动 is the driven
  side and 驱动 the drive side. Their new names are `metal_servo_horn_driven_v2`, `metal_servo_horn_drive_v2`
  and `metal_servo_horn_drive_v2__2`.

One instance was dropped. `金属舵盘（从动） v2 <2>` was not mated to the robot, so it formed a second root,
and URDF keeps only the first. The user confirmed it is a stray copy.

## Check and view

From the repository root:

```sh
python -m robot_platform sim audit-model --model urdf/minerva/onshape-to-robot-head-v1/model.json --expectations urdf/minerva/onshape-to-robot-head-v1/source_manifest.json
python -m robot_platform view --model urdf/minerva/onshape-to-robot-head-v1/model.json --port 8771
```

The viewer shows the head with **Head** sliders for `neck_z` and `neck_y`. Its source-CAD collision modes check
detailed meshes, which touch wherever parts meet, so they mark many contacts. This profile has no collision model.
