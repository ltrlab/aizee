"""Helm's wrist camera calibration: stills captured during a live teleop session, and the solve.

A still needs a session on the real arms with the camera's arm held still (T holds it). Helm reads that arm's
joints from the session's telemetry twice, `settle_s` apart, runs helm.calibration_still on the robot (a stack of
fresh frames averaged, the table's tags found, the light on them measured), then reads the joints again; the still
is kept only if no joint moved more than `still_rad` over all of it. Stills are appended to
<root>/<run>/stills.jsonl, beside the averaged images the robot saves, each with the arms' mechanical reference it
was read through (the digital twin's correction holds only for that reference, vision.twin). Solve runs vision.solve
in its own process (one BLAS thread, lowered priority) and keeps its result.json. Nothing here moves a motor or
changes a camera setting.
"""
import base64
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from robot_platform.vision.board import load_board
from robot_platform.vision.rig import DEFAULT_RIG, ROOT, load_rig

SOLVE_TIMEOUT_S = 300.
RECORDING = ('starting', 'recording', 'stopping', 'finishing')
MAX_REPLY = 1024 * 1024
# Run before a helper's remote_main, which starts its threads: they inherit the background CPUs (when the robot has
# the split, robot_platform.ops.cpu_partition) and the lower priority, as helm.episode_worker.background does. Helm's
# SSH sessions may run on every CPU, the arms' loop's too: a map capture there stretched the loop's cycles until an
# arm command expired (2026-11-01).
BACKGROUND = '''
import json as _json, os as _os
try:
    _os.sched_setaffinity(0, _json.load(open('/etc/minerva/cpu-partition.json'))['background'])
except Exception:
    pass
try:
    _os.nice(10)
except Exception:
    pass
'''


def run_remote(profile, address, request, *, open_ssh=None, timeout_s=15., script='calibration_still.py'):
    """Run a robot helper's remote_main(request) on the robot over Helm's pinned SSH (helm.calibration_still, or
    `script` beside it: helm.scene_capture), off the arms' loop's CPUs; its result."""
    from robot_platform.ops.ssh import connect, open_command
    client = channel = None
    try:
        client = (open_ssh or connect)(profile, address=address(), timeout=3.)
        source = (Path(__file__).with_name(script).read_text(encoding='utf-8') + BACKGROUND
                  + '\nremote_main(' + repr(request) + ')\n')
        stdin, out, _ = open_command(client, 'python3 -', timeout=3.)
        channel = out.channel
        channel.settimeout(3.)
        stdin.write(source)
        stdin.flush()
        channel.shutdown_write()
        deadline, stdout, stderr = time.monotonic() + timeout_s, bytearray(), bytearray()
        while True:
            if channel.recv_ready():
                stdout.extend(channel.recv(65536))
            if channel.recv_stderr_ready():
                stderr.extend(channel.recv_stderr(4096))
            if len(stdout) > MAX_REPLY or len(stderr) > 65536:
                raise RuntimeError('Calibration helper exceeded its output limit')
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{script} timed out on the robot')
            time.sleep(.01)
        if channel.recv_exit_status():
            raise RuntimeError('Calibration helper failed: ' + stderr.decode(errors='replace')[:1000])
        reply = json.loads(stdout)
        if reply.get('ok') is not True:
            raise RuntimeError(reply.get('error') or 'Calibration helper failed')
        return reply['result']
    finally:
        if channel is not None:
            channel.close()
        if client is not None:
            client.close()


class CameraCalibration:
    """Call capture/solve from worker threads. `teleop()` is the teleop session's snapshot (helm.teleop);
    `reference()` the arms' mechanical reference digest Helm's session binds."""

    def __init__(self, profile, *, address, teleop, root, reference=lambda: None, rig=DEFAULT_RIG, remote=None,
                 solver=None, clock=time.time, sleep=time.sleep):
        self.config = load_rig(rig)
        self.board = load_board(self.config['board'])
        self.profile, self.address, self.teleop, self.reference = profile, address, teleop, reference
        self.root = Path(os.path.expanduser(str(root)))
        self.remote = remote or (lambda request: run_remote(self.profile, self.address, request))
        self.solver = solver or self._solve_process
        self.clock, self.sleep = clock, sleep
        self._lock = threading.Lock()
        self.busy = self.solving = False
        self.error, self.last, self.result = None, None, None
        self.run = None
        self.stills = []
        self._thumbnail = None  # (still name, JPEG bytes) of the last still: the page fetches it once.
        self._open_latest()

    # -- runs ---------------------------------------------------------------------------------------------------

    def _open_latest(self):
        runs = sorted(p for p in self.root.glob('*') if (p / 'stills.jsonl').is_file()) if self.root.is_dir() else []
        if runs:
            self._load(runs[-1])

    def _load(self, folder):
        self.run = folder
        self.stills = [json.loads(line) for line in (folder / 'stills.jsonl').read_text(encoding='utf-8').splitlines()
                       if line.strip()] if (folder / 'stills.jsonl').is_file() else []
        result = folder / 'result.json'
        self.result = _summary(json.loads(result.read_text(encoding='utf-8'))) if result.is_file() else None

    def new_run(self):
        with self._claim():
            self._new_run()
            return self.snapshot()

    def _new_run(self):
        stamp = datetime.fromtimestamp(self.clock(), timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        folder, k = self.root / stamp, 1
        while folder.exists():  # Two runs within a second.
            k += 1
            folder = self.root / f'{stamp}-{k}'
        folder.mkdir(parents=True)
        (folder / 'stills.jsonl').write_text('', encoding='utf-8')
        self.run, self.stills, self.result, self.last, self.error = folder, [], None, None, None
        self._thumbnail = None

    @contextmanager
    def _claim(self):
        if not self._lock.acquire(blocking=False):
            raise ValueError('A calibration capture or solve is running; wait for it')
        try:
            yield
        finally:
            self._lock.release()

    # -- capture ------------------------------------------------------------------------------------------------

    def _joints(self, role):
        """The arm's joints from the running session: {name: position}, refused unless measured and fresh."""
        view = self.teleop() or {}
        if view.get('phase') != 'running' or view.get('state') not in ('HOLD', 'TELEOP'):
            raise ValueError('Calibration stills need a running teleop session on the arms (HOLD or TELEOP)')
        recording = (view.get('task') or {}).get('recording') or {}
        if recording.get('active') or recording.get('state') in RECORDING:
            # The still's decoding shares the camera CPUs with the recorder, whose capture bound is 30 ms.
            raise ValueError('Stop the recording first: a calibration still would share its CPUs')
        arm = self.config['cameras'][role]['arm']
        rows = [r for r in ((view.get('telemetry') or {}).get('rows') or [])
                if str(r.get('joint', '')).startswith(f'{arm}_arm_j')]
        if not rows:
            raise ValueError(f'The session reports no {arm} arm joints')
        if any(r.get('source') != 'measured' for r in rows):
            raise ValueError('Calibration stills need the real arms (a live receiver), not a virtual one')
        if any(r.get('stale') or r.get('position') is None for r in rows):
            raise ValueError(f'The {arm} arm\'s joint readings are stale; capture again')
        return {r['joint']: float(r['position']) for r in rows}

    def capture(self, role):
        if role not in self.config['cameras']:
            raise ValueError('Expected camera role ' + ' or '.join(self.config['cameras']))
        with self._claim():
            self.busy, self.error = True, None
            try:
                return self._capture(role)
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.busy = False

    def _capture(self, role):
        if self.run is None:
            self._new_run()
        still = self.config['still']
        first = self._joints(role)
        self.sleep(still['settle_s'])
        before = self._joints(role)
        _check_still([first, before], still['still_rad'], 'before the frames')
        index = len(self.stills) + 1
        name = f'{index:03d}-{role}'
        reply = self.remote({'operation': 'still', 'role': role, 'frames': still['frames'],
                             'family': self.board.family, 'tags': sorted(self.board.tags),
                             'path': str(self.run / 'images' / f'{name}.png')})
        after = self._joints(role)
        moved = _check_still([first, before, after], still['still_rad'], 'while the frames were taken')
        thumbnail = reply.pop('thumbnail', None)
        captured_at = datetime.fromtimestamp(self.clock(), timezone.utc).isoformat()
        record = dict(name=name, camera=role, captured_at=captured_at,
                      joints={k: (before[k] + after[k]) / 2 for k in sorted(before)}, moved_rad=moved,
                      link=self.config['cameras'][role]['link'], reference_sha256=self.reference(), **reply)
        with open(self.run / 'stills.jsonl', 'a', encoding='utf-8') as file:
            file.write(json.dumps(record) + '\n')
        self.stills.append(record)
        self._thumbnail = (name, base64.b64decode(thumbnail)) if thumbnail else None
        self.last = dict(_still_summary(record), thumbnail=self._thumbnail is not None)
        return self.snapshot()

    def thumbnail(self):
        """The last still's JPEG, its tags outlined, or None."""
        return self._thumbnail[1] if self._thumbnail else None

    def undo(self, role):
        """Drop the camera's latest still (its image stays on the robot)."""
        with self._claim():
            index = max((i for i, s in enumerate(self.stills) if s['camera'] == role), default=None)
            if index is None:
                raise ValueError(f'No {role} still to undo')
            removed = self.stills.pop(index)
            (self.run / 'stills.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in self.stills),
                                                   encoding='utf-8')
            if self.last and self.last.get('name') == removed['name']:
                self.last = self._thumbnail = None
            return self.snapshot()

    # -- solve --------------------------------------------------------------------------------------------------

    def solve(self):
        with self._claim():
            if self.run is None or not self.stills:
                raise ValueError('Capture stills first')
            self.solving, self.error = True, None
            try:
                result = self.solver(self.run)
                self.result = _summary(result)
                return self.snapshot()
            except Exception as exc:
                self.error = str(exc)
                raise
            finally:
                self.solving = False

    def _solve_process(self, folder):
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        env['PYTHONPATH'] = os.pathsep.join(filter(None, [str(ROOT / 'src'), env.get('PYTHONPATH')]))
        command = [sys.executable, '-m', 'robot_platform.vision.solve', str(folder)]
        if os.name == 'posix':
            command = ['nice', '-n', '10'] + command
        done = subprocess.run(command, env=env, capture_output=True, text=True, timeout=SOLVE_TIMEOUT_S)
        if done.returncode:
            raise RuntimeError('Solve failed: ' + (done.stderr.strip().splitlines() or ['no output'])[-1][:500])
        return json.loads((folder / 'result.json').read_text(encoding='utf-8'))

    # -- state --------------------------------------------------------------------------------------------------

    def snapshot(self):
        counts = {role: dict(stills=sum(1 for s in self.stills if s['camera'] == role),
                             tagged=sum(1 for s in self.stills if s['camera'] == role and s.get('tags')))
                  for role in self.config['cameras']}
        return dict(run=self.run.name if self.run else None, counts=counts, busy=self.busy, solving=self.solving,
                    error=self.error, last=self.last, result=self.result, cameras=list(self.config['cameras']),
                    frames=self.config['still']['frames'])


def _check_still(reads, limit, when):
    """The largest spread of any joint over the reads; refused beyond `limit`."""
    moved = max(max(read[k] for read in reads) - min(read[k] for read in reads) for k in reads[0])
    if moved > limit:
        raise ValueError(f'The arm moved {moved * 1e3:.1f} mrad {when}; hold it still (T) and capture again')
    return moved


def _still_summary(record):
    light = record.get('photometry') or {}
    whites = [p['white']['luma'] for p in light.values() if 'white' in p]
    blacks = [p['black']['luma'] for p in light.values() if 'black' in p]
    ratios = [(p['white'].get('r_over_g'), p['white'].get('b_over_g')) for p in light.values() if 'white' in p]
    middle = lambda values: sorted(values)[len(values) // 2] if values else None  # noqa: E731
    return dict(name=record['name'], camera=record['camera'], tags=sorted(int(t) for t in record.get('tags', {})),
                others=record.get('others', []), moved_mrad=round(record['moved_rad'] * 1e3, 2),
                white=middle(whites), black=middle(blacks),
                r_over_g=middle([r for r, _ in ratios if r is not None]),
                b_over_g=middle([b for _, b in ratios if b is not None]),
                noise=(record.get('noise') or {}).get('luma_sigma_median'))


def _summary(result):
    """What the page shows of a solve: per model (plain, with joint offsets) and camera, its errors and mount."""
    models = {}
    for key, report in (result.get('models') or {}).items():
        if 'error' in report:
            models[key] = dict(error=report['error'])
            continue
        cameras = {}
        for name, camera in report['cameras'].items():
            cross = (report.get('cross_validation') or {}).get('cameras', {}).get(name, {})
            cameras[name] = dict(rms_px=camera['rms_px'], holdout_px=cross.get('rms_px'),
                                 stills=len(camera['stills']), outliers=[s['name'] for s in camera['stills']
                                                                        if s.get('outlier')],
                                 focal=[camera['intrinsics']['fx'], camera['intrinsics']['fy']],
                                 centre=[camera['intrinsics']['cx'], camera['intrinsics']['cy']],
                                 distortion=camera['distortion'][:2],
                                 mount_mm=[round(v * 1e3, 1) for v in camera['mount']['xyz_m']],
                                 mount_rpy_deg=camera['mount']['rpy_deg'],
                                 mount_sigma_mm=camera['mount'].get('sigma_xyz_mm'),
                                 cells_seen=camera['coverage']['cells_seen'])
        placement = report['board']['placement']
        difference = (report.get('per_camera_placement') or {}).get('difference')
        models[key] = dict(cameras=cameras, board_mm=[round(v * 1e3, 1) for v in placement['xyz_m']],
                           board_rpy_deg=placement['rpy_deg'], left_vs_right=difference,
                           robot_in_board=report.get('robot_in_board'))
    return dict(models=models, recommended=result.get('recommended'), solved_at=result.get('solved_at'),
                link_check=result.get('link_check'), notes=result.get('notes', []))
