# The head and neck in the robot model

Minerva's head and neck are modelled in a second export, **Head V1**, beside the active model. The Onshape assembly is
[Head V1+urdf](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/349129ac06492b67378d6724/e/f4466d56fce7b6028aae0a06).
The neck is movable in the model. The loose servo horn is dropped, and the centre of `AWG_RGB_CAMERA` is the origin of
the head camera's view. How the neck is driven is in [NECK_FEETECH.md](NECK_FEETECH.md).

## The export

The export is in [urdf/minerva/onshape-to-robot-head-v1](../../../urdf/minerva/onshape-to-robot-head-v1/README.md),
beside the untouched 1.1 export that the active model references. It cost one of the two export runs the Onshape
account allowed (631 requests).

- **The model:** 19 links, 18 joints.
  - `neck_z` (pan) is about +z at (0.1, -1.5, 1131.2) mm, from the base to the pan bracket.
  - `neck_y` (tilt) is about -x at (-18.0, -1.5, 1187.8) mm, from the bracket to the head.
  - The structural audit passes against expectations that include the two neck joints.
- **Nothing else moved:** the 16 arm and jaw joint frames match 1.1 within 0.005 mm and 0.001 degrees, and the
  geometry of the existing parts is unchanged apart from re-tessellation.
- **Preview:** the profile `Minerva (Head V1)` shows the head with neck sliders in the 3D viewer. It has no collision
  model of its own.
- **Unchanged:** the active model (`urdf/minerva/model.json` and `minerva.urdf`), the collision models, the limit
  reviews and the MuJoCo export. The active model has no neck.

The robot definition names the neck's motors `head_pan` and `head_tilt`; they drive the CAD joints `neck_z` and
`neck_y` through the head's directions, as an arm motor drives its CAD joint.

## Where the head model is used

- **Helm's 3D view** draws it (`helm.display_model`). Helm checks at start that this model is the active one plus the
  neck: every active joint has the same links, axis, frame, bounds and coupling. Otherwise the view draws the active
  model. This is for display only:
  - every guide (the stopped reader's, a session's, a sweep's) and the joint telemetry are drawn on it by joint name;
  - the neck shows the head's measured angles (`HeadTeleop.status()['joints']`, each servo through its axis) while a
    teleop session drives it; otherwise nothing reads the neck and the view draws it at its zero;
  - its 73 meshes that the active model lacks go to Minerva's mesh store with the active model's
    (`ops.helm_robot meshes`).
- **The head's receiver** derives its limits and sweep from this model (`config/devices/minerva-head.json`,
  `model_profile`), and the release carries its URDF with its mesh digests in the asset lock.
- **The head camera's pose** is forward kinematics on this model, the camera at the `AWG_RGB_CAMERA` centre
  (`neck/head_pose.py`). At zero the centre sits at (32.8, -110.5, 1269.9) mm in the robot frame, or
  (-109.0, -82.1, -50.8) mm in the head link's frame. Its optical axes and calibration are in
  [HEAD_CAMERA_ORIENTATION](../../operator/HEAD_CAMERA_ORIENTATION.md).
- The guard, the arms' receivers, the guides and the recordings keep the active model.

## The arms can reach the head

At zero, the head and neck occupy x -90 to 90, y -117 to 60 and z 1128 to 1428 mm. That box is 177 mm from the left
arm's j2 origin and 244 mm from the right's, and each arm reaches about 945 mm from j2. **The active collision model has
no head.** The guard does not know the head is there, and nothing stops an arm from hitting it in teleop, replay or a
policy run. Helm's 3D view showing the head only draws it. The design decision is that the head needs no collision
checks; the operator keeps the arms clear of it.

The head also reaches its own shoulder: turned to a side and tilted all the way down. That is a coupled limit of the
neck's two joints, measured by hand in Helm (Limits, Head, shoulder corners) and enforced on every head target
(`neck.clearance`, see [NECK_FEETECH.md](NECK_FEETECH.md)). The head is jogged, never swept.

## Not done

- **A collision model with the neck.** Not planned. If it is ever needed: add `neck_z` and `neck_y` to a new collision
  URDF with simplified geometry for the pan bracket and the head, extend the broad phase
  (`urdf/minerva/collision/broadphase-*.json`) and the pair lists for arm against head, the limit review and the
  keep-out policy, rerun the collision sweeps and timing, and compose the active model from the two with
  `scripts/compose_collision_model.py` (it refuses mismatched joint sets, and visible links without collision
  geometry). The MuJoCo export's test pins the URDF hash.
- **Masses.** Only 12 of 157 parts carry mass, eight of them arm links. The head model has no physics use yet.
- **Link names.** `part_6` and `awg_rgb_camera_5` come from part names. `link_` mate connectors in CAD would rename
  them, at the cost of a re-export (cheap if only the assembly changes).

## Acceptance and identity

- **Audit:** `python -m robot_platform sim audit-model --model urdf/minerva/onshape-to-robot-head-v1/model.json --expectations urdf/minerva/onshape-to-robot-head-v1/source_manifest.json`
  reports 0 errors and 0 warnings, with static checks passed.
- **Viewer:** `python -m robot_platform view --model urdf/minerva/onshape-to-robot-head-v1/model.json --port 8771`. The
  head renders, `neck_z` pans and `neck_y` tilts it.
- **Identity:**
  - URDF sha256: `e78833725fce41fd344ee67bd8c54661b54e1cf9ee21bd1f4860c1159390eca2`.
  - Exporter output before normalization: `cf74e84e5d9e3dbf892c41c529c3bc70266e7f18adb924ed5af18845e9fec00e`.
  - Every mesh is hashed in the manifest.

The exporter's neck limits are its default of plus and minus pi for both joints; the real limits are the unit's swept
ones, saved in Helm.
