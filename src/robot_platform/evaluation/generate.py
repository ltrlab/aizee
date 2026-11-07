"""Synthetic demonstrations: run a demonstrator through the simulated receiver and keep the episodes it completes.

Each trial runs exactly as the evaluation harness runs it (runner.TrialRunner: the native receiver, the armplant
joints, the contact scene), and an EpisodeRecorder writes what a real capture would: the receiver's measured and
commanded motor coordinates, its efforts, and the head and wrist cameras, through the shared TaskEpisodeWriter
(format 9; the simulated head stays at the neck's zero, task_scene.SIMULATED_HEAD). Only a trial the scene's own events call a success (pickup, then handoff, then the cube settled in the
bucket, eligible) is published; anything else is discarded. See docs/simulation/SIM_GENERATION.md.

Generated episodes say what they are: data_source 'simulation', authority 'policy', collection_mode
'scripted_demonstrator', the demonstrator's description, the scene's capabilities (workcell, gripper drives) and the
plant's provenance. Whether they may train a policy is the dataset contract's decision; the writer leaves
training_promoted false.

With a layout sampler (simulation.layouts, `generate --randomize <file>`), each attempt draws its own layout: the
cube's size, pose and mass, the bucket's size and pose, the table group's placement relative to the robot and the head
camera's pose. A layout the demonstrator cannot plan is drawn again, up to the file's cap, and every rejected draw is
counted in the manifest and the episode's metadata (`layout`).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from robot_platform.data.roster import HEAD_AXES
from robot_platform.simulation.task_scene import CAMERAS, SIMULATED_HEAD, with_fixed_head

LANGUAGE = 'Pick up the cube, pass it to the other gripper, then drop it into the bucket'


APPEARANCE = {'light': .3, 'color': .1, 'camera_shift': .01, 'camera_turn_deg': 2.}


class EpisodeRecorder:
    """Records one trial at `hz` (scene time) into `directory`; publishes it only if the trial succeeded.

    `appearance` (MinervaTaskScene.randomize_appearance's bounds) varies each episode's lights, colours and camera
    places, from a generator seeded by `seed` and the trial; None keeps the scene's own look."""

    def __init__(self, directory, *, hz=15., camera_roles=CAMERAS, metadata=None, appearance=None, seed=0):
        if not 0 < hz <= 50:
            raise ValueError('record rate must be in (0, 50] Hz')
        self.directory, self.hz, self.camera_roles = Path(directory), float(hz), tuple(camera_roles)
        self.metadata = dict(metadata or {})
        self.appearance, self.seed, self.scene = appearance, int(seed), None
        self.trial_metadata = {}  # annotate(): this trial's own metadata (its drawn layout), cleared by finish().
        self.writer = None
        self.next_at = 0.
        self.frames = 0
        self.last_path, self.last_frames = None, 0

    def annotate(self, **values):
        """Add `values` to the next recorded trial's metadata only (a drawn layout, say)."""
        self.trial_metadata.update(values)

    def start(self, scene, plant, context, policy):
        from robot_platform.data.task_capture import TaskEpisodeWriter
        self.scene = scene
        looks = None
        if self.appearance is not None:
            cube = context.cube_start
            rng = np.random.default_rng([self.seed, int(context.index), int(round(cube['x'] * 1e6) % 2**31),
                                         int(round(cube['y'] * 1e6) % 2**31)])
            looks = scene.randomize_appearance(rng, **self.appearance)
        metadata = dict(self.metadata, **self.trial_metadata, data_source='simulation', evidence='synthetic',
                        run_id=scene.run_id, appearance=looks,
                        collection_mode='scripted_demonstrator', language_instruction=LANGUAGE,
                        scene=scene.capabilities, plant=plant.describe(),
                        demonstrator=getattr(policy, 'describe', lambda: {})(),
                        cube_start=dict(context.cube_start), trial=context.index,
                        head=SIMULATED_HEAD,
                        sample_policy='receiver feedback and command rows, rendered at the record rate in scene time')
        self.writer = TaskEpisodeWriter(self.directory, tuple(context.joint_names) + HEAD_AXES,
                                        json.loads(json.dumps(metadata, default=_plain)), hz=self.hz,
                                        camera_roles=self.camera_roles)
        self.next_at, self.frames = float(scene.data.time), 0

    def sample(self, scene, plant):
        """Record a frame when the scene's clock reaches the next record time."""
        if self.writer is None or scene.data.time < self.next_at - 1e-9:
            return
        row = plant._row
        if row is None:
            return
        observation = plant.observation()
        images, stamps = scene.render(self.camera_roles)
        snapshot = scene.snapshot()
        snapshot['events'] = snapshot['events'][-8:]  # The full history stays in the trial record.
        self.writer.append(**with_fixed_head(dict(q=observation['q'], qcmd=observation['qcmd'],
                                                  torques=observation['effort'])),
                           source_time=row[0] / 1000., scene_time=float(scene.data.time), images=images,
                           camera_times=stamps, task_state=snapshot, authority='policy')
        self.frames += 1
        while self.next_at <= scene.data.time + 1e-9:
            self.next_at += 1 / self.hz

    def finish(self, record):
        """The published episode's path for a successful, eligible trial; None (and no file) otherwise. Also kept as
        `last_path`, with the frame count as `last_frames`."""
        writer, self.writer = self.writer, None
        self.trial_metadata = {}
        # A trial whose policy failed before the recording started (its plan, say) recorded nothing.
        self.last_path, self.last_frames = None, self.frames if writer is not None else 0
        if self.scene is not None:
            self.scene.restore_appearance()
            self.scene = None
        if writer is None:
            return None
        if record.get('outcome') == 'success' and record.get('eligible') and self.frames >= 2:
            self.last_path = writer.finish()
            return self.last_path
        writer.abort(f"trial {record.get('outcome')}: {record.get('detail') or 'not a success'}")
        writer.wait_closed(timeout=30.)
        Path(writer.partial_path).unlink(missing_ok=True)
        return None


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f'not JSON: {type(value).__name__}')


class HeadCameraClearance:
    """Whether a drawn layout's head camera stands clear of the robot, as the real one does: with the arms hanging, no
    ray through its image meets the robot, and its optical centre is at least `margin` from the robot's surface (a
    D435 is 25 mm deep). A layout moves the camera with the table group and about its calibrated pose, so near the
    robot a draw can put it against or inside the torso. Rays only, against the robot's collision proxies posed at
    `home`; built once, then a few thousand rays a draw."""

    def __init__(self, stack, *, margin=.02, grid=(48, 36), directions=1000, seed=0):
        from robot_platform.simulation.task_scene import MinervaTaskScene
        self.scene = MinervaTaskScene(stack['derived']['joint_map'], stack['names'], width=64, height=48)
        mj = self.scene.mj
        self.scene._pose(np.asarray(stack['derived']['initial'], dtype=float))
        mj.mj_kinematics(self.scene.model, self.scene.data)
        self.margin, self.grid = float(margin), tuple(grid)
        self.groups = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)  # The robot's collision proxies only.
        directions = np.random.default_rng(seed).normal(size=(directions, 3))
        self.directions = directions / np.linalg.norm(directions, axis=1, keepdims=True)

    def _nearest(self, start, rays):
        mj, scene, nearest = self.scene.mj, self.scene, np.inf
        geom = np.array([-1], dtype=np.int32)
        for ray in rays:
            distance = mj.mj_ray(scene.model, scene.data, start, ray, self.groups, 1, -1, geom)
            if geom[0] >= 0:
                nearest = min(nearest, distance)
        return nearest

    def __call__(self, workcell):
        """None if the camera is clear, else why not."""
        from robot_platform.simulation.workcell import head_pinhole
        head = workcell.head_camera
        start = np.asarray(head['pos'], dtype=float)
        x_axis = np.asarray(head['xyaxes'][:3], dtype=float)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.asarray(head['xyaxes'][3:], dtype=float)
        y_axis -= (y_axis @ x_axis) * x_axis
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)  # MuJoCo's camera looks along -z.
        width, height = head['intrinsics']['resolution'] if head.get('intrinsics') else (640, 480)
        fx, fy, cx, cy = head_pinhole(head, width, height)
        us, vs = np.meshgrid(np.linspace(0, width - 1, self.grid[0]), np.linspace(0, height - 1, self.grid[1]))
        rays = (-z_axis[None] + x_axis[None] * ((us.ravel() - cx) / fx)[:, None]
                - y_axis[None] * ((vs.ravel() - cy) / fy)[:, None])
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        seen = self._nearest(start, rays)
        if np.isfinite(seen):
            return f'head camera: the robot fills part of its view, {seen * 1000:.0f} mm away (arms hanging)'
        around = self._nearest(start, self.directions)
        if around < self.margin:
            return f'head camera: {around * 1000:.0f} mm from the robot, under {self.margin * 1000:.0f} mm'
        return None

    def close(self):
        self.scene.close()


class BucketInView:
    """Whether a drawn layout's head camera shows the bucket's whole opening: the four corners of its inside rim, at
    the walls' top, each at least `margin_px` inside a width x height image (workcell.bucket_opening_pixels), with the
    layout's own head camera pose, its drawn jitter included. A randomization file asks for it
    (`head_view.bucket_opening`)."""

    def __init__(self, image, margin_px):
        self.width, self.height = (int(v) for v in image)
        self.margin = float(margin_px)

    def slack(self, workcell):
        """How far inside the image the opening's nearest corner to an edge is, in pixels (negative: outside; -inf: a
        corner behind the camera)."""
        from robot_platform.simulation.workcell import bucket_opening_pixels
        corners = bucket_opening_pixels(workcell, self.width, self.height)
        if any(corner is None for corner in corners):
            return -math.inf
        return min(min(u, self.width - 1 - u, v, self.height - 1 - v) for u, v in corners)

    def __call__(self, workcell):
        """None if the opening is in view, else why not."""
        slack = self.slack(workcell)
        if slack >= self.margin:
            return None
        where = 'behind the camera' if slack == -math.inf else f'{-slack:.1f} px outside' if slack < 0 else \
            f'only {slack:.1f} px inside'
        return (f"head camera: the bucket's opening is not whole in its {self.width} x {self.height} image with "
                f"{self.margin:g} px to spare (a corner {where})")

    def close(self):
        pass


class LayoutChecks:
    """A drawn layout's checks before the demonstrator plans it, in order: the first problem refuses the draw."""

    def __init__(self, checks):
        self.checks = list(checks)

    def __call__(self, workcell):
        for check in self.checks:
            problem = check(workcell)
            if problem is not None:
                return problem
        return None

    def close(self):
        for check in self.checks:
            check.close()


def layout_checks(stack, randomization):
    """The checks a drawn layout passes before planning: its head camera clear of the robot (HeadCameraClearance) and,
    when the randomization file asks for it, the bucket's whole opening in the head image (BucketInView)."""
    checks = [HeadCameraClearance(stack)]
    if randomization.bucket_view is not None:
        checks.append(BucketInView(**randomization.bucket_view))
    return LayoutChecks(checks)


def draw_layout(runner, policy, sampler, seed, camera=None):
    """A layout `policy` can plan (simulation.layouts.LayoutSampler, drawn from seed + [draw]), and every draw rejected
    on the way: [{'draw', 'reason', 'layout'}]. (None, rejections) when none of the file's `plan_draws` plans. With
    `camera` (a check such as layout_checks gives: None, or why not), a draw it refuses is rejected too."""
    rejections = []
    for draw in range(sampler.randomization.plan_draws):
        key = [*seed, draw]
        try:
            layout = sampler.draw(np.random.default_rng(key), seed=key)
        except ValueError as exc:
            rejections.append({'draw': draw, 'reason': f'layout: {exc}', 'layout': None})
            continue
        problem = camera(layout.workcell) if camera is not None else None
        if problem is not None:
            rejections.append({'draw': draw, 'reason': problem, 'layout': layout.describe()})
            continue
        problem = runner.plan(policy, dict(layout.cube_start, cell='layout', index=0), layout.workcell)
        if problem is None:
            return layout, rejections
        rejections.append({'draw': draw, 'reason': problem, 'layout': layout.describe()})
    return None, rejections


def generate(policy, config, out, *, episodes, stack=None, hz=15., appearance=APPEARANCE, progress=None,
             layouts=None, attempts=None):
    """Run trials until `episodes` succeed or `attempts` (default `max(2 * episodes, 10)`) have been made. Writes the
    episodes and manifest.jsonl under `out`.

    Without `layouts`, the trials cycle over `config.grid`'s cube poses (with fresh seeds once a grid is used up) on
    the configured workcell. With `layouts` (a simulation.layouts.LayoutSampler), attempt k draws its layout from
    [config.grid.seed, k] (draw_layout): the cube's start is the layout's, and the head camera's pose is the layout's
    too, so the appearance variation moves only the wrist cameras. A draw whose head camera does not stand clear of
    the robot, or (when the randomization file asks for it) does not show the bucket's whole opening, is drawn again
    (layout_checks), as one the demonstrator cannot plan."""
    from dataclasses import replace
    from .grid import cube_poses
    from .runner import TrialRunner, git_identity
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    progress = progress or (lambda line: None)
    if layouts is not None and appearance is not None:
        appearance = dict(appearance, cameras=tuple(role for role in CAMERAS if role != 'head'))
    runner = TrialRunner(config, stack=stack, directory=out / 'receiver', progress=progress)
    recorder = EpisodeRecorder(out / 'episodes', hz=hz, appearance=appearance, seed=config.grid.seed,
                               metadata={'generator': {'git': git_identity(), 'run_config': config.describe(),
                                                       'record_hz': hz, 'appearance_bounds': appearance,
                                                       'layouts': None if layouts is None else
                                                       layouts.randomization.describe()}})
    cap = max(2 * episodes, 10) if attempts is None else int(attempts)
    made, attempts, seed = [], 0, config.grid.seed
    manifest = (out / 'manifest.jsonl').open('a', encoding='utf-8')
    camera = layout_checks(runner.stack, layouts.randomization) if layouts is not None else None

    def note(entry, record, path):
        entry.update(outcome=record['outcome'], detail=record.get('detail'), episode=str(path) if path else None,
                     duration_s=record.get('duration_s'), frames=recorder.last_frames)
        manifest.write(json.dumps(entry, default=_plain) + '\n')
        manifest.flush()
        if path:
            made.append(path)
        progress(f"attempt {entry['attempt']}: {record['outcome']}" + (f' -> {Path(path).name}' if path else ''))
    try:
        while layouts is not None and len(made) < episodes and attempts < cap:
            attempts += 1
            layout, rejections = draw_layout(runner, policy, layouts, [seed, attempts], camera=camera)
            entry = {'attempt': attempts, 'layout_seed': [seed, attempts], 'rejected_draws': len(rejections),
                     'rejections': [{'draw': r['draw'], 'reason': r['reason']} for r in rejections]}
            if layout is None:
                note(entry, {'outcome': 'layout_rejected', 'detail': f'no layout planned in '
                             f'{layouts.randomization.plan_draws} draws'}, None)
                continue
            entry.update(cube_start=layout.cube_start, layout=layout.describe())
            runner.use_workcell(layout.workcell)
            recorder.annotate(layout=dict(layout.describe(), rejected_draws=rejections))
            pose = dict(layout.cube_start, cell='layout', index=attempts)
            record = runner.run_trial(policy, pose, attempts, recorder=recorder)
            note(entry, record, recorder.last_path)
        while layouts is None and len(made) < episodes and attempts < cap:
            grid = replace(config.grid, seed=seed)
            for pose in cube_poses(grid):
                if len(made) >= episodes or attempts >= cap:
                    break
                attempts += 1
                runner.config = replace(config, grid=grid)
                record = runner.run_trial(policy, pose, pose['index'], recorder=recorder)
                note({'attempt': attempts, 'grid_seed': seed, 'cube_start': {k: pose[k] for k in ('x', 'y', 'yaw')}},
                     record, recorder.last_path)
            seed += 1
    finally:
        manifest.close()
        runner.close()
        if camera is not None:
            camera.close()
    return made, attempts
