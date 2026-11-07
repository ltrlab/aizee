"""Render the task scene's look: the head view beside a real head frame, both wrist views mid-grasp, a wide 3/4 view of
the whole scene, and crops of a gripper over the table, with how bright the grippers are against the table.

    .venv-dev/Scripts/python.exe scripts/sim/render_look.py capture <states.npz> [--workcell <file>]
    .venv-dev/Scripts/python.exe scripts/sim/render_look.py render <states.npz> <out dir> [--real <head.jpg>]
    .venv-dev/Scripts/python.exe scripts/sim/render_look.py compare <before dir> <after dir> <out dir> [--real <head.jpg>]

`capture` runs the nominal handoff trial through the receiver with driven grippers and keeps the scene's whole state
(mj_getState, mjSTATE_INTEGRATION: the arms' mocap poses, the jaws, the block) every 0.1 s of scene time, with the
demonstrator's moves, the drives and the workcell, so that the same moments can be rendered by any later look of the
scene: the physics does not depend on it. `render` builds this checkout's CAD scene for those states and renders, at
640 x 480: the start (head), the left closing on the block (head, left wrist) and the right closing on it (head, right
wrist), and a wide 3/4 view of the whole scene; it writes the images and looks.json, each view's mean brightness of the
grippers', the table top's and the other things' pixels (sRGB luma, 0-255, from a segmentation render). `compare` lays two renders side by side.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
WIDTH, HEIGHT = 640, 480
# The moments rendered: a stage the demonstrator starts, and how long after its start.
MOMENTS = {'start': (None, .5), 'left grasp': ('left closes', .25), 'right take': ('right closes', .25)}
VIEWS = {'start': ('head', 'wide'), 'left grasp': ('head', 'left_wrist', 'wide'),
         'right take': ('head', 'right_wrist', 'wide')}
# The wide view: a free camera behind the robot's right shoulder, taking in the robot, the table, the box and the room.
WIDE = {'lookat': (0., -.7, .6), 'distance': 1.6, 'azimuth': -35., 'elevation': -20., 'fovy': 75.}


def capture(out, workcell):
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.grid import GridSpec, cube_poses
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.evaluation.runner import RunConfig, TrialRunner
    from robot_platform.simulation import task_scene
    from robot_platform.simulation.workcell import load_workcell
    kept = []
    original = task_scene.MinervaTaskScene.step_driven

    def traced(self, *args, **kwargs):
        state = original(self, *args, **kwargs)
        self._look_steps = getattr(self, '_look_steps', 0) + 1
        if self._look_steps % 40 == 0:  # 0.1 s of 2.5 ms steps
            spec = self.mj.mjtState.mjSTATE_INTEGRATION
            buffer = np.empty(self.mj.mj_stateSize(self.model, spec))
            self.mj.mj_getState(self.model, self.data, buffer, spec)
            kept.append((float(self.data.time), buffer))
        return state
    task_scene.MinervaTaskScene.step_driven = traced
    loaded = load_workcell(workcell)
    grid = GridSpec.for_workcell(loaded)
    with tempfile.TemporaryDirectory() as directory:
        runner = TrialRunner(RunConfig(grippers='driven', timeout_s=70., workcell=str(workcell), grid=grid),
                             stack=derive_stack(), directory=directory)
        demo = HandoffDemonstrator()
        try:
            drives = runner._open_plant().gripper_drives()
            result = runner.run_trial(demo, cube_poses(grid)[0], 0)
            log = [list(entry) for entry in demo.log]
        finally:
            runner.close()
            demo.close()
    task_scene.MinervaTaskScene.step_driven = original
    np.savez_compressed(out, times=np.array([t for t, _ in kept]), states=np.array([s for _, s in kept]),
                        log=json.dumps(log), drives=json.dumps(drives), workcell=json.dumps(loaded.describe()),
                        outcome=result['outcome'])
    print(f"{len(kept)} states, {result['outcome']}; moves: " + ', '.join(f'{name} {t:.2f} s' for name, t in log))


def _luma(pixels):
    pixels = np.asarray(pixels, dtype=float)
    return pixels[..., 0] * .2126 + pixels[..., 1] * .7152 + pixels[..., 2] * .0722


def _flags(scene, renderer):
    """The flags the scene renders with (task_scene.RENDER_FLAGS where it has them)."""
    from robot_platform.simulation import task_scene
    flags = getattr(task_scene, 'RENDER_FLAGS', None)
    if flags is None:
        flags = {name: False for name in ('mjRND_SHADOW', 'mjRND_REFLECTION', 'mjRND_FOG')}
    for name, value in flags.items():
        renderer.scene.flags[getattr(scene.mj.mjtRndFlag, name)] = value


def render(states, out, real=None):
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.simulation.task_scene import JAW_BODIES, WRISTS, MinervaTaskScene
    from robot_platform.simulation.workcell import Workcell
    from PIL import Image
    saved = np.load(states)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    stack = derive_stack()
    workcell = Workcell.from_description(json.loads(str(saved['workcell'])))
    scene = MinervaTaskScene(stack['derived']['joint_map'], stack['names'], width=WIDTH, height=HEIGHT,
                             visual_mode='cad', workcell=workcell, grippers=json.loads(str(saved['drives'])))
    mj, model, data = scene.mj, scene.model, scene.data
    log = {name: t for name, t in json.loads(str(saved['log']))}
    spec = mj.mjtState.mjSTATE_INTEGRATION
    gripper_bodies = {*WRISTS.values(), *(jaw for pair in JAW_BODIES.values() for jaw in pair)}
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
    body = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or '' for g in range(model.ngeom)]
    classes = {'gripper': [g for g in range(model.ngeom) if body[g] in gripper_bodies],
               'table': [g for g in range(model.ngeom) if names[g] == 'task/table'],
               'trim': [g for g in range(model.ngeom) if names[g].startswith('task/table/trim/')],
               'box': [g for g in range(model.ngeom) if names[g].startswith('task/bucket/')],
               'block': [scene._id('geom', 'task/cube/geom')],
               'tags': [g for g in range(model.ngeom) if names[g].startswith('task/fiducial/')]}
    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    options = mj.MjvOption()
    options.geomgroup[:] = scene.visual_groups
    wide = mj.MjvCamera()
    wide.type = mj.mjtCamera.mjCAMERA_FREE
    wide.lookat[:], wide.distance = WIDE['lookat'], WIDE['distance']
    wide.azimuth, wide.elevation = WIDE['azimuth'], WIDE['elevation']
    looks = {}
    try:
        for moment, (stage, after) in MOMENTS.items():
            at = (log[stage] if stage else 0.) + after
            index = int(np.argmin(np.abs(saved['times'] - at)))
            mj.mj_setState(model, data, saved['states'][index], spec)
            mj.mj_forward(model, data)
            for view in VIEWS[moment]:
                camera = wide if view == 'wide' else view
                fovy = None
                if view == 'wide':
                    fovy = float(model.vis.global_.fovy)
                    model.vis.global_.fovy = WIDE['fovy']
                try:
                    renderer.disable_segmentation_rendering()
                    renderer.update_scene(data, camera=camera, scene_option=options)
                    _flags(scene, renderer)
                    image = renderer.render().copy()
                    renderer.enable_segmentation_rendering()
                    renderer.update_scene(data, camera=camera, scene_option=options)
                    segments = renderer.render().copy()
                finally:
                    if fovy is not None:
                        model.vis.global_.fovy = fovy
                name = f"{moment.replace(' ', '-')}-{view}"
                Image.fromarray(image).save(out / f'{name}.png')
                geoms = np.where(segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM), segments[..., 0], -1)
                stats = {}
                for key, ids in classes.items():
                    mask = np.isin(geoms, ids)
                    if mask.sum() >= 20:
                        stats[key] = {'pixels': int(mask.sum()), 'luma': round(float(_luma(image[mask]).mean()), 1)}
                        if key == 'gripper':
                            rows, cols = np.nonzero(mask)
                            stats[key]['box'] = [int(rows.min()), int(rows.max()), int(cols.min()), int(cols.max())]
                stats['time_s'] = round(float(saved['times'][index]), 2)
                looks[name] = stats
        if real:
            sim = Image.open(out / 'start-head.png').convert('RGB')
            truth = Image.open(real).convert('RGB').resize(sim.size)
            pair = Image.new('RGB', (sim.width * 2 + 8, sim.height), (255, 255, 255))
            pair.paste(sim, (0, 0))
            pair.paste(truth, (sim.width + 8, 0))
            pair.save(out / 'head-sim-beside-real.png')
    finally:
        renderer.close()
        scene.close()
    (out / 'looks.json').write_text(json.dumps(looks, indent=1) + '\n', encoding='utf-8')
    for name, stats in looks.items():
        parts = [f"{key} {value['luma']}" for key, value in stats.items() if isinstance(value, dict)]
        print(f'{name} (t {stats["time_s"]} s): ' + ', '.join(parts))


def compare(before, after, out, real=None):
    """Side by side: each view before and after; and a crop around the gripper in the head and wrist views."""
    from PIL import Image, ImageDraw
    before, after, out = Path(before), Path(after), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    looks = json.loads((after / 'looks.json').read_text(encoding='utf-8'))
    old = json.loads((before / 'looks.json').read_text(encoding='utf-8'))
    for name in looks:
        a, b = Image.open(before / f'{name}.png'), Image.open(after / f'{name}.png')
        pair = Image.new('RGB', (a.width * 2 + 8, a.height + 22), (255, 255, 255))
        pair.paste(a, (0, 22))
        pair.paste(b, (a.width + 8, 22))
        draw = ImageDraw.Draw(pair)
        for x, label, stats in ((4, 'before', old.get(name, {})), (a.width + 12, 'after', looks[name])):
            text = ', '.join(f"{key} {value['luma']}" for key, value in stats.items() if isinstance(value, dict)
                             and key in ('gripper', 'table'))
            draw.text((x, 4), f'{label}: {name}; mean luma {text}', fill=(0, 0, 0))
        pair.save(out / f'{name}-before-after.png')
        box = looks[name].get('gripper', {}).get('box')
        if box and name.split('-')[-1] in ('head', 'wrist'):
            top, bottom, left, right = box
            pad = 30
            crop = (max(left - pad, 0), max(top - pad, 0), min(right + pad, a.width), min(bottom + pad, a.height))
            ca, cb = a.crop(crop), b.crop(crop)
            scale = max(1, 320 // max(ca.width, 1))
            ca, cb = (c.resize((c.width * scale, c.height * scale), Image.NEAREST) for c in (ca, cb))
            pair = Image.new('RGB', (ca.width * 2 + 8, ca.height), (255, 255, 255))
            pair.paste(ca, (0, 0))
            pair.paste(cb, (ca.width + 8, 0))
            pair.save(out / f'{name}-gripper-crop-before-after.png')
    if real and (after / 'start-head.png').exists():
        sim = Image.open(after / 'start-head.png').convert('RGB')
        truth = Image.open(real).convert('RGB').resize(sim.size)
        pair = Image.new('RGB', (sim.width * 2 + 8, sim.height), (255, 255, 255))
        pair.paste(sim, (0, 0))
        pair.paste(truth, (sim.width + 8, 0))
        pair.save(out / 'head-sim-beside-real.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    commands = parser.add_subparsers(dest='command', required=True)
    c = commands.add_parser('capture')
    c.add_argument('states')
    c.add_argument('--workcell', default=str(WORKCELL))
    r = commands.add_parser('render')
    r.add_argument('states')
    r.add_argument('out')
    r.add_argument('--real')
    p = commands.add_parser('compare')
    p.add_argument('before')
    p.add_argument('after')
    p.add_argument('out')
    p.add_argument('--real')
    args = parser.parse_args()
    if args.command == 'capture':
        capture(args.states, args.workcell)
    elif args.command == 'render':
        render(args.states, args.out, args.real)
    else:
        compare(args.before, args.after, args.out, args.real)


if __name__ == '__main__':
    main()
