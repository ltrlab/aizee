#!/usr/bin/env python3
"""Render the task scene's table with its fiducial tags: from straight above, and through the scene's head camera.

    python scripts/sim/render_fiducial_views.py <out dir> [--workcell FILE] [--visual-mode cad|collision_proxy]

Writes top.png (looking straight down on the table, the robot at the bottom, as the layout picture draws it) and
head.png (the scene's head camera at the real head frame's 640 x 480), with the robot hanging. Renders with MuJoCo,
so it needs a GL context (the GPU on Windows).
"""
from __future__ import annotations

import argparse
import json
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
    parser.add_argument('--visual-mode', choices=('cad', 'collision_proxy'), default='cad')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    names = json.loads(FIXTURE.read_text())['joint_names']
    joint_map = {n: dict(actuator=n, scale=1., offset=0.) for n in names if 'arm' in n}
    for side in ('left', 'right'):
        joint_map[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    workcell = load_workcell(args.workcell)
    scene = MinervaTaskScene(joint_map, names, width=640, height=480, visual_mode=args.visual_mode, workcell=workcell)
    try:
        scene._q = np.zeros(len(names))
        scene.reset()
        head, _ = scene.render(('head',))
        Image.fromarray(head['head']).save(args.out / 'head.png')
        options = mujoco.MjvOption()
        options.geomgroup[:] = scene.visual_groups
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        # Straight down on the table and the robot in front of it; the image's up is away from the robot (-y).
        cx, cy = workcell.table_center
        camera.lookat[:] = (cx, cy + .12, workcell.table_top)
        camera.distance, camera.azimuth, camera.elevation = 1.9, -90., -89.99
        with mujoco.Renderer(scene.model, height=960, width=1280) as renderer:
            renderer.update_scene(scene.data, camera=camera, scene_option=options)
            for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
                renderer.scene.flags[flag] = False
            Image.fromarray(renderer.render()).save(args.out / 'top.png')
    finally:
        scene.close()
    print(f'wrote {args.out / "top.png"} and {args.out / "head.png"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
