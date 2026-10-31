"""Helm's head camera alignment (docs/development/HEADSET_3D_PLAN.md, Alignment): stills of the table's tags from the
head camera in a session running the live head, the solve, and saving the measured mount for the head's pose feed.

A still needs the live head held where its camera sees the board (T holds it). helm.calibration_still on the robot
averages a stack of fresh frames, finds the tags, and reads the neck's measured angles over the same frames from the
head's pose feed (neck.head_pose); the still is kept only if no neck joint moved more than `still_rad` across them.
Each still names the head reference its angles were read through. Solve runs vision.head_alignment in its own
process. Save writes the solved mount and the board's place into the head's camera file, beside its reference and
limits (neck.head.files), where the next session's head reads it (display only: the headset's 3D view and model, the
camera relay's stamps); the file it replaces is kept in the run's folder. Nothing here moves a motor or changes a
camera setting.
"""
from contextlib import contextmanager
from datetime import datetime, timezone
import base64
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

from robot_platform.control.joint_calibration import save_json
from robot_platform.neck import head
from robot_platform.neck.head_pose import reference_digest
from robot_platform.vision.board import load_board
from robot_platform.vision.head_alignment import CAMERA, DEFAULT_RIG, ROOT, alignment_document, load_rig, summary

from .calibration import RECORDING, SOLVE_TIMEOUT_S, _still_summary, run_remote


def check_live_head(view, what, then):
    """Refused (ValueError) unless the teleop snapshot `view` runs the live head, enabled, and nothing records: `what`
    names what needs it ('Head stills'), `then` what to do once the head is enabled."""
    view = view or {}
    if view.get('phase') != 'running':
        raise ValueError(f'{what} need a running session with the live head')
    if (view.get('receiver_choice') or {}).get('mode') != 'live':
        raise ValueError(f'{what} need the real head (a live session), not a virtual one')
    if ((view.get('head') or {}).get('state')) not in ('HOLD', 'TELEOP'):
        raise ValueError(f'Enable the head (E), then {then}')
    recording = (view.get('task') or {}).get('recording') or {}
    if recording.get('active') or recording.get('state') in RECORDING:
        raise ValueError(f'Stop the recording first: {what.lower()} would share its CPUs')


class HeadAlignment:
    """Call capture/solve/save from worker threads. `teleop()` is the teleop session's snapshot (helm.teleop);
    `files()` the head's files (neck.head.files); `wrists` the wrist calibration's runs, to compare the board."""

    def __init__(self, profile, *, address, teleop, root, files, wrists=None, rig=DEFAULT_RIG, remote=None,
                 solver=None, clock=time.time):
        self.config = load_rig(rig)
        self.board = load_board(self.config['board'])
        self.profile, self.address, self.teleop, self.files = profile, address, teleop, files
        self.root = Path(os.path.expanduser(str(root)))
        self.wrists = Path(os.path.expanduser(str(wrists))) if wrists else None
        self.remote = remote or (lambda request: run_remote(self.profile, self.address, request))
        self.solver = solver or self._solve_process
        self.clock = clock
        self._lock = threading.Lock()
        self.busy = self.solving = False
        self.error, self.last, self.result, self.run, self.stills = None, None, None, None, []
        self._thumbnail = None
        runs = sorted(p for p in self.root.glob('*') if (p / 'stills.jsonl').is_file()) if self.root.is_dir() else []
        if runs:
            self._load(runs[-1])

    def _load(self, folder):
        self.run = folder
        self.stills = [json.loads(line) for line in (folder / 'stills.jsonl').read_text(encoding='utf-8').splitlines()
                       if line.strip()]
        result = folder / 'result.json'
        self.result = summary(json.loads(result.read_text(encoding='utf-8'))) if result.is_file() else None

    @contextmanager
    def _claim(self):
        if not self._lock.acquire(blocking=False):
            raise ValueError('A head still, solve or save is running; wait for it')
        try:
            yield
        finally:
            self._lock.release()

    def new_run(self):
        with self._claim():
            self._new_run()
            return self.snapshot()

    def _new_run(self):
        stamp = datetime.fromtimestamp(self.clock(), timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        folder, k = self.root / stamp, 1
        while folder.exists():
            k += 1
            folder = self.root / f'{stamp}-{k}'
        folder.mkdir(parents=True)
        (folder / 'stills.jsonl').write_text('', encoding='utf-8')
        self.run, self.stills, self.result, self.last, self.error, self._thumbnail = folder, [], None, None, None, None

    # -- capture ------------------------------------------------------------------------------------------------

    def _reference(self):
        reference = head.load_document(self.files()['reference'])
        if reference is None:
            raise ValueError("Set the head's reference first (Limits tab, Head)")
        return reference

    def capture(self):
        with self._claim():
            self.busy, self.error = True, None
            try:
                self._capture()
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.busy = False
        return self.snapshot()  # Once it is no longer busy.

    def _capture(self):
        check_live_head(self.teleop(), 'Head stills', "hold it (T) where its camera sees the table's tags")
        digest = reference_digest(self._reference())
        if self.run is None:
            self._new_run()
        still = self.config['still']
        name = f'{len(self.stills) + 1:03d}-{CAMERA}'
        reply = self.remote({'operation': 'still', 'role': CAMERA, 'frames': still['frames'],
                             'family': self.board.family, 'tags': sorted(self.board.tags),
                             'path': str(self.run / 'images' / f'{name}.png'),
                             'head_pose': {'margin_s': still['margin_s']}})
        pose = reply.pop('head', None)
        if not pose:
            raise RuntimeError('The robot sent no head pose with the still; is its release current?')
        if pose['moved_rad'] > still['still_rad']:
            raise ValueError(f"The head moved {pose['moved_rad'] * 1e3:.1f} mrad while the frames were taken; hold it "
                             'still (T) and capture again')
        if not isinstance(reply.get('intrinsics'), dict):
            raise ValueError('The head camera published no intrinsics with its frames; its colour registration update '
                             'adds them')
        thumbnail = reply.pop('thumbnail', None)
        record = dict(name=name, camera=CAMERA, captured_at=datetime.fromtimestamp(self.clock(), timezone.utc)
                      .isoformat(), joints=pose['joints'], moved_rad=pose['moved_rad'], pose_samples=pose['samples'],
                      link=self.config['link'], reference_sha256=digest, **reply)
        with open(self.run / 'stills.jsonl', 'a', encoding='utf-8') as file:
            file.write(json.dumps(record) + '\n')
        self.stills.append(record)
        self._thumbnail = (name, base64.b64decode(thumbnail)) if thumbnail else None
        self.last = dict(_still_summary(record), thumbnail=self._thumbnail is not None,
                         joints_deg={k: round(math.degrees(v), 2) for k, v in pose['joints'].items()})

    def thumbnail(self):
        return self._thumbnail[1] if self._thumbnail else None

    def undo(self):
        """Drop the latest still (its image stays on the robot)."""
        with self._claim():
            if not self.stills:
                raise ValueError('No head still to undo')
            removed = self.stills.pop()
            (self.run / 'stills.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in self.stills),
                                                   encoding='utf-8')
            if self.last and self.last.get('name') == removed['name']:
                self.last = self._thumbnail = None
            return self.snapshot()

    # -- solve and save -----------------------------------------------------------------------------------------

    def solve(self):
        with self._claim():
            if self.run is None or not self.stills:
                raise ValueError('Capture head stills first')
            self.solving, self.error = True, None
            try:
                self.result = summary(self.solver(self.run))
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.solving = False
        return self.snapshot()

    def _solve_process(self, folder):
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        env['PYTHONPATH'] = os.pathsep.join(filter(None, [str(ROOT / 'src'), env.get('PYTHONPATH')]))
        command = [sys.executable, '-m', 'robot_platform.vision.head_alignment', str(folder)]
        if self.wrists is not None:
            command += ['--wrists', str(self.wrists)]
        if os.name == 'posix':
            command = ['nice', '-n', '10'] + command
        done = subprocess.run(command, env=env, capture_output=True, text=True, timeout=SOLVE_TIMEOUT_S)
        if not (folder / 'result.json').is_file() or done.returncode not in (0, 1):
            raise RuntimeError('Solve failed: ' + (done.stderr.strip().splitlines() or ['no output'])[-1][:500])
        return json.loads((folder / 'result.json').read_text(encoding='utf-8'))

    def save(self):
        """The run's solve as the head's camera file; the one it replaces is kept in the run's folder."""
        with self._claim():
            path = self.run / 'result.json' if self.run is not None else None
            if path is None or not path.is_file() or not self.result or self.result.get('error'):
                raise ValueError('Solve the run first')
            result = json.loads(path.read_text(encoding='utf-8'))
            if result.get('reference_sha256') != reference_digest(self._reference()):
                raise ValueError("The head's reference changed since these stills: start a new run")
            now = datetime.fromtimestamp(self.clock(), timezone.utc)
            target = self.files()['camera']
            if target.is_file():
                shutil.copy2(target, self.run / f"replaced-{now.strftime('%Y%m%dT%H%M%SZ')}-{target.name}")
            target.parent.mkdir(parents=True, exist_ok=True)
            save_json(target, alignment_document(result, saved_at=now.isoformat()))
            return self.snapshot()

    # -- state --------------------------------------------------------------------------------------------------

    def _saved(self):
        """The head's camera file as the next session's head reads it: which run, how well it fit, and whether it
        holds for the current head reference."""
        try:
            document = head.load_document(self.files()['camera'])
            if document is None:
                return None
            reference = head.load_document(self.files()['reference'])
            current = reference is not None and document.get('reference_sha256') == reference_digest(reference)
            return dict(run=document.get('run'), saved_at=document.get('saved_at'), fit=document.get('fit'),
                        mount_mm=[round(v * 1e3, 1) for v in document['mount']['xyz_m']], current=current)
        except (OSError, ValueError, KeyError, TypeError):
            return {'error': 'The head camera file is unreadable'}

    def snapshot(self):
        tagged = sum(1 for s in self.stills if s.get('tags'))
        return dict(run=self.run.name if self.run else None, stills=len(self.stills), tagged=tagged, busy=self.busy,
                    solving=self.solving, error=self.error, last=self.last, result=self.result, saved=self._saved(),
                    frames=self.config['still']['frames'])
