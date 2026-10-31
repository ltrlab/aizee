"""Minerva's splat maps for the headset (docs/development/HEADSET_3D_PLAN.md, phase 4): the head scene captures they
are trained from, the trained maps Helm serves to the headset (xr_splat.js), and Minerva's pose in each map's world.

Under <root> (on Minerva ~/minerva/helm/maps):
- captures/<stamp>/: a head scene capture in the splat trainer's format (scene.json, color/, depth/), recorded on the
  robot (helm.scene_capture) in a session running the live head, as the operator looks around in the headset;
- maps/<name>/: a trained map, map.splat (the trainer's compact Gaussians) and map.json (its report, naming the world
  it is in), put there by the operator station's trainer (scripts/splat/train_on_station.py).
A map's world is the table's board ('board') once the head camera is aligned on its tags (the head's camera file holds
the board's place in Minerva's frame, helm.head_alignment), else Minerva's own frame ('robot'). Minerva's pose in the
board's world is fixed while her base stands; a moving base will give it here from odometry, and nothing else
changes: captures record it with every frame, the headset places a map by it. Nothing here moves a motor.
"""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import threading
import time

import numpy as np

from robot_platform.neck import head
from robot_platform.neck.head_pose import aligned_mount, pose7
from robot_platform.vision.geometry import invert

from .calibration import run_remote
from .head_alignment import check_live_head
from .scene_capture import CAPTURE_ROOT, MAX_SECONDS

NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
LISTING_S = 5.  # The folders are listed again at most this often for the page's state.
FPS = 3.  # Frames a second a capture keeps at most.


def column_major(matrix):
    """A 4 x 4 transform as the 16 numbers three.js reads (Matrix4.fromArray)."""
    return [float(v) for v in np.asarray(matrix, float).flatten(order='F')]


def _rigid(matrix):
    m = np.asarray(matrix, float)
    return (m.shape == (4, 4) and np.isfinite(m).all() and np.allclose(m[3], (0., 0., 0., 1.))
            and np.allclose(m[:3, :3] @ m[:3, :3].T, np.eye(3), atol=1e-5) and np.linalg.det(m[:3, :3]) > 0)


class MapLibrary:
    """Call capture from a worker thread. `teleop()` is the teleop session's snapshot (helm.teleop); `files()` the
    head's files (neck.head.files)."""

    def __init__(self, profile, *, address, teleop, root, files, remote=None, clock=time.time):
        self.profile, self.address, self.teleop, self.files = profile, address, teleop, files
        self.root = Path(os.path.expanduser(str(root)))
        self.remote = remote or (lambda request, timeout_s: run_remote(
            self.profile, self.address, request, timeout_s=timeout_s, script='scene_capture.py'))
        self.clock = clock
        self._lock = threading.Lock()
        self.busy, self.error, self.last = False, None, None
        self._listed, self._listing = -math.inf, None

    # -- the worlds ---------------------------------------------------------------------------------------------

    def worlds(self):
        """Minerva's pose in each world ({'robot': 4 x 4, 'board': 4 x 4 or None}): the board's once the head camera's
        alignment holds for the head reference in use, its placement inverted."""
        poses = {'robot': np.eye(4), 'board': None}
        try:
            files = self.files()
            document, reference = head.load_document(files['camera']), head.load_document(files['reference'])
            if document is not None and reference is not None:
                aligned_mount(document, document.get('link'), reference)  # Made through this head reference.
                board = np.asarray(document['board_in_robot']['matrix'], float)
                if _rigid(board):
                    poses['board'] = invert(board)
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return poses

    # -- the folders --------------------------------------------------------------------------------------------

    def _maps(self):
        maps = []
        for folder in sorted((self.root / 'maps').glob('*'), reverse=True) if (self.root / 'maps').is_dir() else []:
            splat = folder / 'map.splat'
            if not (NAME.match(folder.name) and splat.is_file()):
                continue
            try:
                report = json.loads((folder / 'map.json').read_text(encoding='utf-8'))
            except (OSError, ValueError):
                report = {}
            maps.append(dict(name=folder.name, world=report.get('world', 'robot'), gaussians=report.get('gaussians'),
                             frames=report.get('frames'), psnr=report.get('psnr_held_out'),
                             capture=report.get('capture'), bytes=splat.stat().st_size,
                             trained_at=datetime.fromtimestamp(splat.stat().st_mtime, timezone.utc).isoformat()))
        return maps

    def _captures(self):
        captures = []
        folders = (self.root / 'captures').glob('*') if (self.root / 'captures').is_dir() else []
        for folder in sorted(folders, reverse=True):
            try:
                scene = json.loads((folder / 'scene.json').read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            captures.append(dict(name=folder.name, frames=len(scene.get('frames', [])), world=scene.get('world'),
                                 captured_at=scene.get('captured_at'),
                                 trained=(self.root / 'maps' / folder.name / 'map.splat').is_file()))
        return captures

    def listing(self, fresh=True):
        """The maps (newest first) as the headset reads them, with Minerva's pose in each world (column-major), and
        the captures."""
        if fresh or self._listing is None or self.clock() - self._listed > LISTING_S:
            poses = self.worlds()
            self._listing = dict(maps=self._maps(), captures=self._captures(),
                                 robot_in_world={world: None if pose is None else column_major(pose)
                                                 for world, pose in poses.items()},
                                 world='board' if poses['board'] is not None else 'robot')
            self._listed = self.clock()
        return self._listing

    def splat(self, name):
        """A map's .splat file."""
        if not isinstance(name, str) or not NAME.match(name):
            raise ValueError('Not a map name')
        path = self.root / 'maps' / name / 'map.splat'
        if not path.is_file():
            raise FileNotFoundError(f'No map {name}')
        return path

    # -- the capture --------------------------------------------------------------------------------------------

    @contextmanager
    def _claim(self):
        if not self._lock.acquire(blocking=False):
            raise ValueError('A map capture is running; wait for it')
        try:
            yield
        finally:
            self._lock.release()

    def capture(self, seconds):
        """A head scene capture of `seconds`, recorded on the robot as the operator looks around."""
        if type(seconds) not in (int, float) or not 5 <= seconds <= MAX_SECONDS:
            raise ValueError(f'A capture lasts 5 to {MAX_SECONDS} seconds')
        with self._claim():
            self.busy, self.error = True, None
            try:
                check_live_head(self.teleop(), 'Map captures', 'engage it (T) and look around slowly in the headset')
                poses = self.worlds()
                world = 'board' if poses['board'] is not None else 'robot'
                stamp = datetime.fromtimestamp(self.clock(), timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                reply = self.remote({'operation': 'scene', 'seconds': seconds, 'fps': FPS, 'world': world,
                                     'folder': f'{CAPTURE_ROOT}/{stamp}',
                                     'robot_in_world': pose7(poses[world]) if world == 'board' else None},
                                    seconds + 60.)
                self.last = reply
                self._listing = None
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.busy = False
        return self.snapshot()  # Once it is no longer busy.

    def snapshot(self):
        listing = self.listing(fresh=False)
        return dict(busy=self.busy, error=self.error, last=self.last, world=listing['world'],
                    maps=listing['maps'][:8], captures=listing['captures'][:8])
