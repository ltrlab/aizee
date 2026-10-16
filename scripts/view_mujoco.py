"""Pose the generated Minerva MJCF kinematically: an image, or MuJoCo's viewer.

Never steps physics: the model's masses are exporter placeholders, so only forward
kinematics is meaningful. Positions are URDF joint coordinates (arm radians, jaw
metres); mimic jaws follow their source. No sockets, devices or motor commands.
"""
import argparse
import json
from pathlib import Path
import time

import mujoco

from robot_platform.models import ModelProfile, UrdfModel

ROOT = Path(__file__).resolve().parents[1]


def pose(model, data, urdf, positions):
    q = {name: urdf.by_name[name]['default'] for name in urdf.independent}
    unknown = set(positions) - set(q)
    if unknown:
        raise SystemExit(f'Unknown joints: {sorted(unknown)}')
    q.update(positions)
    for name, value in urdf.positions(q).items():
        data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
    mujoco.mj_kinematics(model, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=ROOT / 'urdf/minerva/mujoco/scene.xml')
    parser.add_argument('--profile', type=Path, default=ROOT / 'urdf/minerva/model.json')
    parser.add_argument('--pose', help='JSON object of joint positions, for example {"left_arm_j2": 0.5}')
    parser.add_argument('--render', type=Path, help='write a PNG instead of opening the viewer')
    parser.add_argument('--azimuth', type=float, default=90., help='degrees; 90 looks at the front')
    parser.add_argument('--elevation', type=float, default=-15.)
    parser.add_argument('--collision', action='store_true', help='show source collision meshes, not visuals')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    urdf = UrdfModel(ModelProfile.load(args.profile), apply_review=False)
    pose(model, data, urdf, json.loads(args.pose) if args.pose else {})
    groups = (0, 3) if args.collision else (0, 2)  # Floor is group 0.
    if args.render:
        from PIL import Image
        options = mujoco.MjvOption()
        options.geomgroup[:] = [int(g in groups) for g in range(len(options.geomgroup))]
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.azimuth, camera.elevation = args.azimuth, args.elevation
        with mujoco.Renderer(model, 960, 1280) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(args.render)
        print(args.render)
        return 0
    from mujoco import viewer as mujoco_viewer
    # Joint sliders edit qpos; only kinematics is recomputed, never integrated.
    with mujoco_viewer.launch_passive(model, data) as viewer:
        for group in range(len(viewer.opt.geomgroup)):
            viewer.opt.geomgroup[group] = int(group in groups)
        while viewer.is_running():
            with viewer.lock():
                mujoco.mj_kinematics(model, data)
            viewer.sync()
            time.sleep(1 / 60)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
