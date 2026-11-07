#!/usr/bin/env python3
"""Render the task scene's head camera, depth and colour, with the robot hidden and the cube at each given place.

    MUJOCO_GL=egl python scripts/sim/render_head_depth.py <out dir> [--workcell FILE] [--block X Y YAW ...]
        [--blocks-from depth_check.json]

Builds the scene from the workcell (its head camera with the D435's pixel intrinsics, at their resolution) and renders
only geom group 0: the floor, the table, the box, the tags and the cube, not the robot (whose placement relative to the
tags is still a placeholder, and which would stand in the view). Each --block puts the scene's cube at (X, Y) metres
in the workcell's tag rectangle frame, turned YAW degrees, resting on the table; --blocks-from takes every block a
head_depth_check.py report found (the cube fitted to its depth: centre and turn). The renders are merged, the nearest
surface winning, so several blocks show at once. Writes sim_depth.npy (metres along the optical axis, 0 where
nothing is), sim_rgb.png and sim_render.json. Renders with MuJoCo: use a GPU's EGL (MUJOCO_GL=egl and
MUJOCO_EGL_DEVICE_ID), or a desktop GL context.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from robot_platform.simulation.task_scene import MinervaTaskScene
from robot_platform.simulation.workcell import load_workcell

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = ROOT / 'tests/fixtures/minerva_task_contact_path.json'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out', type=Path)
    parser.add_argument('--workcell', type=Path, default=WORKCELL)
    parser.add_argument('--block', type=float, nargs=3, action='append', default=[], metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--blocks-from', type=Path, help='a head_depth_check.py report (depth_check.json): its blocks')
    parser.add_argument('--visual-mode', choices=('cad', 'collision_proxy'), default='collision_proxy')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.blocks_from:
        report = json.loads(args.blocks_from.read_text(encoding='utf-8'))
        args.block += [
            [*(v / 1000 for v in o['centre_rectangle_mm']), o['yaw_deg']]
            for o in report['objects']
            if o['kind'] == 'block'
        ]
    workcell = load_workcell(args.workcell)
    width, height = (int(v) for v in workcell.head_camera['intrinsics']['resolution'])
    names = json.loads(FIXTURE.read_text())['joint_names']
    joint_map = {n: dict(actuator=n, scale=1.0, offset=0.0) for n in names if 'arm' in n}
    for side in ('left', 'right'):
        joint_map[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=0.01, offset=0.0)
    scene = MinervaTaskScene(
        joint_map, names, width=width, height=height, visual_mode=args.visual_mode, workcell=workcell
    )
    rectangle = workcell.fiducials['rectangle']
    (ox, oy), turn = rectangle['origin'], math.pi + rectangle['yaw']
    c, s = math.cos(turn), math.sin(turn)
    depth = rgb = None
    placed = []
    try:
        scene._q = np.zeros(len(names))
        scene.reset()
        options = mujoco.MjvOption()
        options.geomgroup[:] = [1, 0, 0, 0, 0, 0]  # The workcell and the cube; no robot.
        with mujoco.Renderer(scene.model, height=height, width=width) as renderer:
            for x, y, yaw in args.block or [(None, None, None)]:
                if x is not None:
                    base = (ox + c * x - s * y, oy + s * x + c * y, workcell.table_top + workcell.cube_half)
                    angle = math.radians(yaw) + turn
                    address = scene.cube_qadr
                    scene.data.qpos[address : address + 3] = base
                    scene.data.qpos[address + 3 : address + 7] = (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))
                    placed.append({'rectangle_m': [x, y], 'yaw_deg': yaw, 'base_m': list(base)})
                mujoco.mj_forward(scene.model, scene.data)
                renderer.update_scene(scene.data, camera='head', scene_option=options)
                renderer.enable_depth_rendering()
                d = renderer.render().copy()
                renderer.disable_depth_rendering()
                renderer.update_scene(scene.data, camera='head', scene_option=options)
                colour = renderer.render().copy()
                far = d >= scene.model.vis.map.zfar * scene.model.stat.extent * 0.999
                d[far] = 0.0
                if depth is None:
                    depth, rgb = d, colour
                else:
                    nearer = (d > 0) & ((depth == 0) | (d < depth))
                    depth[nearer], rgb[nearer] = d[nearer], colour[nearer]
    finally:
        scene.close()
    np.save(args.out / 'sim_depth.npy', depth.astype(np.float32))
    Image.fromarray(rgb).save(args.out / 'sim_rgb.png')
    (args.out / 'sim_render.json').write_text(
        json.dumps(
            {
                'workcell': str(args.workcell),
                'head_camera': workcell.head_camera,
                'size': [width, height],
                'blocks': placed,
                'visual_mode': args.visual_mode,
                'geom_groups': [0],
                'mujoco': mujoco.__version__,
            },
            indent=1,
            default=float,
        ),
        encoding='utf-8',
    )
    print(f'wrote {args.out}: depth {np.count_nonzero(depth)} px, {len(placed)} block(s)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
