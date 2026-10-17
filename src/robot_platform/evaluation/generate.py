"""Synthetic demonstrations: run a demonstrator through the simulated receiver and keep the episodes it completes.

Each trial runs exactly as the evaluation harness runs it (runner.TrialRunner: the native receiver, the armplant
joints, the contact scene), and an EpisodeRecorder writes what a real capture would: the receiver's measured and
commanded motor coordinates, its efforts, and the head and wrist cameras, through the shared TaskEpisodeWriter
(format 8). Only a trial the scene's own events call a success (pickup, then handoff, then the cube settled in the
bucket, eligible) is published; anything else is discarded. See docs/development/SIM_GENERATION.md.

Generated episodes say what they are: data_source 'simulation', authority 'policy', collection_mode
'scripted_demonstrator', the demonstrator's description, the scene's capabilities (workcell, gripper drives) and the
plant's provenance. Whether they may train a policy is the dataset contract's decision; the writer leaves
training_promoted false.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from robot_platform.simulation.task_scene import CAMERAS

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
        self.writer = None
        self.next_at = 0.
        self.frames = 0
        self.last_path, self.last_frames = None, 0

    def start(self, scene, plant, context, policy):
        from robot_platform.data.task_capture import TaskEpisodeWriter
        self.scene = scene
        looks = None
        if self.appearance is not None:
            cube = context.cube_start
            rng = np.random.default_rng([self.seed, int(context.index), int(round(cube['x'] * 1e6) % 2**31),
                                         int(round(cube['y'] * 1e6) % 2**31)])
            looks = scene.randomize_appearance(rng, **self.appearance)
        metadata = dict(self.metadata, data_source='simulation', evidence='synthetic', run_id=scene.run_id,
                        appearance=looks,
                        collection_mode='scripted_demonstrator', language_instruction=LANGUAGE,
                        scene=scene.capabilities, plant=plant.describe(),
                        demonstrator=getattr(policy, 'describe', lambda: {})(),
                        cube_start=dict(context.cube_start), trial=context.index,
                        sample_policy='receiver feedback and command rows, rendered at the record rate in scene time')
        self.writer = TaskEpisodeWriter(self.directory, context.joint_names, json.loads(json.dumps(metadata,
                                        default=_plain)), hz=self.hz, camera_roles=self.camera_roles)
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
        self.writer.append(q=observation['q'], qcmd=observation['qcmd'], source_time=row[0] / 1000.,
                           scene_time=float(scene.data.time), images=images, camera_times=stamps,
                           task_state=snapshot, authority='policy', torques=observation['effort'])
        self.frames += 1
        while self.next_at <= scene.data.time + 1e-9:
            self.next_at += 1 / self.hz

    def finish(self, record):
        """The published episode's path for a successful, eligible trial; None (and no file) otherwise. Also kept as
        `last_path`, with the frame count as `last_frames`."""
        writer, self.writer = self.writer, None
        self.last_path, self.last_frames = None, self.frames
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


def generate(policy, config, out, *, episodes, stack=None, hz=15., appearance=APPEARANCE, progress=None):
    """Run trials over `config.grid`'s cube poses (cycling with fresh seeds) until `episodes` succeed or every pose
    of `max(2 * episodes, 10)` attempts has been tried. Writes the episodes and manifest.jsonl under `out`."""
    from dataclasses import replace
    from .grid import cube_poses
    from .runner import TrialRunner, git_identity
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    progress = progress or (lambda line: None)
    runner = TrialRunner(config, stack=stack, directory=out / 'receiver', progress=progress)
    recorder = EpisodeRecorder(out / 'episodes', hz=hz, appearance=appearance, seed=config.grid.seed,
                               metadata={'generator': {'git': git_identity(), 'run_config': config.describe(),
                                                       'record_hz': hz, 'appearance_bounds': appearance}})
    made, attempts, seed = [], 0, config.grid.seed
    manifest = (out / 'manifest.jsonl').open('a', encoding='utf-8')
    try:
        while len(made) < episodes and attempts < max(2 * episodes, 10):
            grid = replace(config.grid, seed=seed)
            for pose in cube_poses(grid):
                if len(made) >= episodes or attempts >= max(2 * episodes, 10):
                    break
                attempts += 1
                runner.config = replace(config, grid=grid)
                record = runner.run_trial(policy, pose, pose['index'], recorder=recorder)
                path = recorder.last_path
                entry = {'attempt': attempts, 'grid_seed': seed, 'cube_start': {k: pose[k] for k in ('x', 'y', 'yaw')},
                         'outcome': record['outcome'], 'detail': record.get('detail'), 'episode': str(path) if path else None,
                         'duration_s': record.get('duration_s'), 'frames': recorder.last_frames}
                manifest.write(json.dumps(entry) + '\n')
                manifest.flush()
                if path:
                    made.append(path)
                progress(f"attempt {attempts}: {record['outcome']}" + (f' -> {Path(path).name}' if path else ''))
            seed += 1
    finally:
        manifest.close()
        runner.close()
    return made, attempts
