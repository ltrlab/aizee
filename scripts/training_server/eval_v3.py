"""Closed-loop evaluation of the train-v3 Minerva checkpoints on held-out randomized layouts, with the repo's harness.

Every piece that decides an outcome is the repository's (tree-9ebb37b): evaluation.runner.TrialRunner.run_trial (the
native receiver, the armplant joints, driven grippers, the contact scene and its own events), the generator's layout
draw (simulation.layouts.LayoutSampler with the v3 randomization file, generate.layout_checks and generate.draw_layout
with the scripted demonstrator planning each draw, so every evaluation layout is one the demonstrator could do),
policies.MinervaCheckpointPolicy and report.build_report. `python -m robot_platform.evaluation run` draws no layouts,
so this file wires them together, runs a few trial workers on GPU 1, and adds a finer stage ladder, videos and a
TensorBoard scalar.

The checkpoint runs as the laptop's v2 evaluation ran it (scratchpad simproof.checkpoint): on the GPU, one predicted
row per recorded frame (action_dt 1/15 s, the episodes' record rate, not the harness default 50 ms), half a chunk per
inference with RTC, from the mean of the largest cluster of the training episodes' start poses (within 0.1 rad on every
joint) instead of the checkpoint's ready_pose, which averages left J5's +pi and -pi branches into a pose no episode had.

    eval_v3.py layouts [--n 30] [--seed 9000]        draw the held-out layouts once (eval/v3/layouts.json)
    eval_v3.py layouts --from eval/v3/layouts.json   the same draws on another tree (its look drawn last, same geometry)
    eval_v3.py run --policy demonstrator|<checkpoint.pt> --out DIR [--workers 2] [--video] [--limit N]
    eval_v3.py watch                                 tmux eval-v3: epoch 5 now, each new epoch, then the final best
    eval_v3.py status

Another suite (v3.1: its checkpoints on the room-look layouts) sets EVAL_TREE, EVAL_DIR, EVAL_CKPT_DIR, EVAL_TB,
EVAL_TRAIN_LOG and EVAL_TRAIN_SESSION; EVAL_LAYOUTS_FROM (redraw another suite's layouts on this tree),
EVAL_DEMONSTRATOR (reuse a demonstrator run on the same geometry) and EVAL_BASELINES (name=checkpoint,... evaluated once
each, shown as references). Unset, everything is v3's.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys
import time

S = Path(os.environ.get('AIZEE_SIM', Path.home() / 'aizee-sim'))
TREE = Path(os.environ.get('EVAL_TREE') or S / 'tree-9ebb37b')
TRAIN_SRC = S / 'train-0a4d8ef'
WORKCELL = TREE / 'config/devices/simulation/minerva-production-01-workcell.json'
RANDOMIZATION = TREE / 'config/devices/simulation/minerva-production-01-randomization.json'
CKPT = Path(os.environ.get('EVAL_CKPT_DIR') or S / 'ckpt/train-v3')
EVAL = Path(os.environ.get('EVAL_DIR') or S / 'eval/v3')
LAYOUTS = EVAL / 'layouts.json'
TB = Path(os.environ.get('EVAL_TB') or S / 'tb/train-v3-eval')
PYTHON = S / '.venv-eval/bin/python'
TRAIN_LOG = Path(os.environ.get('EVAL_TRAIN_LOG') or S / 'logs/train-v3.log')
TRAIN_SESSION = (os.environ.get('EVAL_TRAIN_SESSION') or 'train-v3').replace('.', '_')  # tmux's own spelling
LAYOUTS_FROM = os.environ.get('EVAL_LAYOUTS_FROM') or None
DEMONSTRATOR = Path(os.environ.get('EVAL_DEMONSTRATOR') or EVAL / 'demonstrator')
BASELINES = dict(item.split('=', 1) for item in (os.environ.get('EVAL_BASELINES') or '').split(',') if '=' in item)
GPU = '1'
HZ = 15.  # The episodes' record rate: one chunk row per recorded frame.
TIMEOUT_S = 90.  # Scene seconds per trial (generation: 70; the laptop's v2 evaluation: 90).
SEED = 9000  # Held out: generation drew layouts from [1000 + 100 k, attempt] (k the worker).
LADDER = ('reached', 'grasped', 'lifted', 'handed_over', 'over_bucket', 'released', 'in_bucket')
MIN_AVAILABLE_GB = 8.  # Leave this much memory free on the shared box.
GB_PER_WORKER = 6.5  # A checkpoint worker peaked at 6.3 GB resident while loading (smoke test, 2026-10-18).
STAGGER_S = 45  # Workers start this far apart, so their loading peaks do not coincide.

ENV = {'PYTHONPATH': f'{TREE / "src"}:{TRAIN_SRC}', 'ROBOT_CONTROL_HARNESS': str(S / 'bin/control_harness'),
       'ROBOT_COLLISION_LIBRARY': str(S / 'bin/librobot_collision.so'), 'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
       'CUDA_VISIBLE_DEVICES': GPU, 'MUJOCO_GL': 'egl', 'MUJOCO_EGL_DEVICE_ID': GPU, 'PYOPENGL_PLATFORM': 'egl',
       'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1',
       'TMPDIR': str(S / 'tmp'), 'PYTHONUNBUFFERED': '1'}
os.environ.update(ENV)  # Before mujoco or torch is imported: EGL and CUDA on GPU 1 only.
sys.path[:0] = [str(TREE / 'src'), str(TRAIN_SRC)]


def log(*parts):
    print(time.strftime('%H:%M:%S'), *parts, flush=True)


def plain(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (Path, tuple, set)):
        return str(value) if isinstance(value, Path) else list(value)
    raise TypeError(f'not JSON: {type(value).__name__}')


def run_config(timeout_s=TIMEOUT_S):
    from robot_platform.evaluation.generate import APPEARANCE
    from robot_platform.evaluation.grid import GridSpec
    from robot_platform.evaluation.runner import RunConfig
    from robot_platform.simulation.workcell import resolve
    # As generation ran (gen-v3): 320x240 CAD views, driven grippers; the head camera stands where each layout puts it,
    # so the look varies lights, colours and the wrist cameras only. The grid only carries the seed here.
    grid = GridSpec.for_workcell(resolve(str(WORKCELL)), nx=6, ny=5, seed=SEED, jitter=1., yaw=.785398, nominal=False)
    return RunConfig(plant='armplant', timeout_s=timeout_s, grid=grid, plant_seed=9, camera_size=(320, 240),
                     visual_mode='cad', workcell=str(WORKCELL), grippers='driven', disturbance_seed=SEED,
                     appearance=dict(APPEARANCE, cameras=('left_wrist', 'right_wrist')))


def runner(directory, timeout_s=TIMEOUT_S):
    from robot_platform.evaluation.plants import derive_stack
    from robot_platform.evaluation.runner import TrialRunner
    return TrialRunner(run_config(timeout_s), stack=derive_stack(profile='minerva-teleop-direct.json'),
                       directory=directory, progress=log)


def start_mode(starts, within=.1):
    """The mean of the largest cluster of start poses (within `within` rad of each other on every joint), and its
    size."""
    import numpy as np
    starts = np.asarray(starts, dtype=float)
    near = (np.abs(starts[:, None, :] - starts[None, :, :]).max(axis=2) < within).sum(axis=1)
    centre = starts[int(np.argmax(near))]
    members = np.abs(starts - centre).max(axis=1) < within
    return starts[members].mean(axis=0), int(members.sum())


def make_policy(spec):
    import numpy as np
    if spec == 'demonstrator':
        from robot_platform.evaluation.demonstrator import HandoffDemonstrator
        return HandoffDemonstrator(), {}
    from robot_platform.evaluation.policies import MinervaCheckpointPolicy
    policy = MinervaCheckpointPolicy(spec, device='cuda', action_dt=1. / HZ, use_rtc=True)
    note = {'action_dt': 1. / HZ, 'execute_steps': policy.execute_steps, 'use_rtc': True, 'device': 'cuda (GPU 1)'}
    if 'start_poses' in policy.stats:
        starts = np.asarray(policy.stats['start_poses'], dtype=float)
        mode, members = start_mode(starts)
        names = list(policy.ckpt_joint_names)
        j5 = names.index('left_arm_j5') if 'left_arm_j5' in names else None
        note['start_pose'] = {'rule': 'mean of the largest start cluster (0.1 rad on every joint)',
                              'cluster': members, 'episodes': len(starts), 'pose': mode.round(4).tolist(),
                              'checkpoint_ready_pose': np.asarray(policy.stats['ready_pose'], float).round(4).tolist()}
        if j5 is not None:
            note['start_pose']['left_j5_branches'] = {'positive': int((starts[:, j5] > 0).sum()),
                                                      'negative': int((starts[:, j5] <= 0).sum())}
        policy.stats['ready_pose'] = mode.astype(np.float32)
    return policy, note


class Watch:
    """A run_trial recorder: the stage ladder from the scene's jaw contacts and the cube after every control cycle,
    and with `video` the left wrist, head and right wrist side by side at the record rate, piped to ffmpeg."""

    def __init__(self, video=None, hz=HZ):
        self.video, self.hz = (Path(video) if video else None), hz
        self.first, self.ffmpeg, self.frames, self.next_at = {}, None, 0, 0.
        self.plant, self.trip = None, None

    def start(self, scene, plant, context, policy):
        self.plant = plant
        self.next_at = float(scene.data.time)
        if self.video:
            self.video.parent.mkdir(parents=True, exist_ok=True)
            self.ffmpeg = subprocess.Popen(
                ['nice', '-n', '15', 'ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                 '-s', f'{3 * scene.width}x{scene.height}', '-r', f'{self.hz:g}', '-i', '-', '-c:v', 'libx264',
                 '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p', '-threads', '1', str(self.video)],
                stdin=subprocess.PIPE)

    def sample(self, scene, plant):
        import numpy as np
        from robot_platform.simulation.workcell import bucket_frame
        t = float(scene.data.time)
        jaws = getattr(scene, '_contacts', None) or scene.snapshot()['jaw_contacts']
        touching = any(jaws.values())
        if touching:
            self.first.setdefault('reached', t)
        if any(len(on) == 2 for on in jaws.values()):
            self.first.setdefault('grasped', t)
        if scene.handoff:
            w = scene.workcell
            x, y, _ = scene.data.xpos[scene.cube_id]
            along_x, along_y = bucket_frame(w, x, y)
            over = abs(along_x) < w.bucket_inner[0] / 2 and abs(along_y) < w.bucket_inner[1] / 2
            if over:
                self.first.setdefault('over_bucket', t)
                if not touching and 'over_bucket' in self.first:
                    self.first.setdefault('released', t)
        if self.ffmpeg is not None and t >= self.next_at - 1e-9:
            images, _ = scene.render(('left_wrist', 'head', 'right_wrist'))
            frame = np.concatenate([images['left_wrist'], images['head'], images['right_wrist']], axis=1)
            try:
                self.ffmpeg.stdin.write(np.ascontiguousarray(frame).tobytes())
                self.frames += 1
            except BrokenPipeError:
                self.ffmpeg = None
            while self.next_at <= t + 1e-9:
                self.next_at += 1 / self.hz

    def finish(self, record):
        if self.ffmpeg is not None:
            self.ffmpeg.stdin.close()
            self.ffmpeg.wait(timeout=120)
            self.ffmpeg = None
        if self.video and not self.frames:
            self.video.unlink(missing_ok=True)
        # What stopped the receiver, in its own words (its status's last_trip: joint, check, reading, limit).
        if record.get('outcome') == 'source_gap_or_abort' and self.plant is not None:
            try:
                from robot_platform.control.live_receiver import trip_text
                status = getattr(getattr(self.plant, 'controller', None), 'status', None) or {}
                trip = status.get('last_trip')
                if trip:
                    self.trip = dict(trip, text=trip_text(status) or None)
            except Exception as exc:  # Diagnostic only.
                self.trip = {'error': f'{type(exc).__name__}: {exc}'}
        self.plant = None

    def ladder(self, record):
        events = record.get('events') or []
        first = lambda kind: next((e['scene_time'] for e in events if e.get('kind') == kind), None)  # noqa: E731
        times = {'reached': self.first.get('reached'), 'grasped': self.first.get('grasped'), 'lifted': first('pickup'),
                 'handed_over': first('handoff'), 'over_bucket': self.first.get('over_bucket'),
                 'released': self.first.get('released'), 'in_bucket': first('bucket_settled')}
        reached, ok = {}, True
        for stage in LADDER:
            ok = ok and times[stage] is not None
            reached[stage] = ok
        return {'times': times, 'reached': reached,
                'last': next((s for s in reversed(LADDER) if reached[s]), None)}


def failure(record):
    outcome, detail = record['outcome'], record.get('detail') or ''
    if outcome == 'success':
        return None
    if outcome == 'source_gap_or_abort':
        if detail.startswith(('receiver ', 'lifecycle is', 'staging left', 'no receiver feedback')):
            return 'receiver_trip'
        if detail.startswith('policy error'):
            return 'policy_error'
        if detail.startswith('scene:'):
            return 'scene_numerical'
        return 'abort_other'
    return {'forbidden_penetration': 'forbidden_contact'}.get(outcome, outcome)


# -- layouts --------------------------------------------------------------------------------------------------------

def draw_layouts(n, seed, out):
    import numpy as np
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.generate import draw_layout, layout_checks
    from robot_platform.simulation.layouts import LayoutSampler
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    trials = runner(S / 'tmp' / f'eval-layouts-{seed}')
    demonstrator = HandoffDemonstrator()
    camera = layout_checks(trials.stack, sampler.randomization)
    accepted, refused, attempt, started = [], [], 0, time.time()
    try:
        while len(accepted) < n and attempt < 3 * n:
            attempt += 1
            layout, rejections = draw_layout(trials, demonstrator, sampler, [seed, attempt], camera=camera)
            reasons = [{'draw': r['draw'], 'reason': r['reason']} for r in rejections]
            if layout is None:
                refused.append({'attempt': attempt, 'rejections': reasons})
                log(f'attempt {attempt}: no layout planned in {sampler.randomization.plan_draws} draws')
                continue
            described = layout.describe()
            accepted.append({'index': len(accepted) + 1, 'key': described['seed'], 'cube_start': layout.cube_start,
                             'rejected_draws': reasons, 'demonstrator_ready': np.round(demonstrator.ready, 6).tolist(),
                             'layout': described})
            log(f"layout {len(accepted)}: key {described['seed']} ({len(reasons)} draws refused)")
    finally:
        trials.close()
        camera.close()
    out.parent.mkdir(parents=True, exist_ok=True)
    document = {'purpose': 'held-out evaluation layouts for train-v3 (generation drew [1000 + 100 k, attempt])',
                'seed': seed, 'requested': n, 'attempts': attempt, 'workcell': str(WORKCELL),
                'randomization': sampler.randomization.describe(), 'tree': str(TREE),
                'checks': 'generate.layout_checks (head camera clear, bucket opening in view) and '
                          'generate.draw_layout with the HandoffDemonstrator planning each draw',
                'layouts': accepted, 'refused_attempts': refused, 'wall_s': round(time.time() - started, 1)}
    out.write_text(json.dumps(document, indent=1, default=plain) + '\n', encoding='utf-8')
    return document


def redraw_layouts(source, out):
    """Another suite's accepted draws, drawn again by this tree's sampler: the same keys give the same table, bucket,
    block and camera (a look is drawn after everything else), now with this tree's look. Every layout's geometry is
    checked equal to the source's, so the demonstrator's plans and refusals carry over."""
    import numpy as np
    from robot_platform.simulation.layouts import LayoutSampler
    document = json.loads(Path(source).read_text(encoding='utf-8'))
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    geometry = lambda values: json.dumps({k: v for k, v in values.items() if k not in ('look', 'randomization')},  # noqa: E731
                                         sort_keys=True)
    layouts = []
    for entry in document['layouts']:
        key = entry['key']
        layout = sampler.draw(np.random.default_rng(key), seed=key)
        described = layout.describe()
        if geometry(described) != geometry(entry['layout']):
            raise SystemExit(f"layout {entry['index']} ({key}) draws a different geometry on {TREE.name}")
        layouts.append(dict(entry, cube_start=layout.cube_start, layout=described))
        log(f"layout {entry['index']}: key {key}, same geometry, look {described.get('look')}")
    redrawn = dict(document, purpose=f"{document.get('purpose')}; drawn again on {TREE.name} (same geometry, its look)",
                   tree=str(TREE), workcell=str(WORKCELL), randomization=sampler.randomization.describe(),
                   redrawn_from=str(source), layouts=layouts)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(redrawn, indent=1, default=plain) + '\n', encoding='utf-8')
    return redrawn


# -- one worker's share ---------------------------------------------------------------------------------------------

def selected(layouts, args):
    """The run's layouts: the first --limit, or the --only indices."""
    if getattr(args, 'only', None):
        wanted = {int(i) for i in str(args.only).split(',') if i.strip()}
        return [entry for entry in layouts if entry['index'] in wanted]
    return layouts[:args.limit or None]


def release_memory():
    """Hand freed heap back between trials: each trial builds its own scene, and the shard grew to 10 GB without."""
    import ctypes
    import gc
    gc.collect()
    try:
        ctypes.CDLL('libc.so.6').malloc_trim(0)
    except OSError:
        pass


def shard(args):
    import numpy as np
    from robot_platform.simulation.layouts import LayoutSampler
    out = Path(args.out)
    document = json.loads(Path(args.layouts).read_text(encoding='utf-8'))
    chosen = selected(document['layouts'], args)
    mine = [entry for i, entry in enumerate(chosen) if i % args.of == args.k]
    path = out / f'shard-{args.k}.jsonl'
    done = set()
    if path.exists():
        done = {json.loads(line)['trial'] for line in path.read_text(encoding='utf-8').splitlines() if line.strip()}
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    trials = runner(out / f'receiver-{args.k}', args.timeout)
    policy, note = make_policy(args.policy)
    if args.k == 0:
        (out / 'policy.json').write_text(json.dumps(dict(note, describe=getattr(policy, 'describe', dict)()),
                                                    indent=1, default=plain) + '\n', encoding='utf-8')
    try:
        with path.open('a', encoding='utf-8') as handle:
            for entry in mine:
                if entry['index'] in done:
                    continue
                key = entry['key']
                layout = sampler.draw(np.random.default_rng(key), seed=key)
                if json.dumps(layout.describe(), sort_keys=True) != json.dumps(entry['layout'], sort_keys=True):
                    raise SystemExit(f"layout {entry['index']} did not redraw identically from {key}")
                trials.use_workcell(layout.workcell)
                pose = dict(layout.cube_start, cell='layout', index=entry['index'])
                video = out / 'videos' / f"trial{entry['index']:02d}.mp4" if args.video else None
                watch = Watch(video)
                started = time.time()
                record = trials.run_trial(policy, pose, entry['index'], recorder=watch)
                record.update(layout_key=key, ladder=watch.ladder(record), failure=failure(record),
                              receiver_trip=watch.trip, video=str(video) if video and video.exists() else None,
                              trial_wall_s=round(time.time() - started, 1))
                handle.write(json.dumps(record, default=plain) + '\n')
                handle.flush()
                log(f"trial {entry['index']}: {record['outcome']} (last stage {record['ladder']['last']}, "
                    f"{record['duration_s']:.1f} s scene, {record['trial_wall_s']:.0f} s wall)"
                    + (f" {record['detail']}" if record.get('detail') else '')
                    + (f" [{watch.trip.get('text') or watch.trip}]" if watch.trip else ''))
                release_memory()
        if args.k == 0:
            meta = trials.meta(policy)
            (out / 'meta.json').write_text(json.dumps(meta, indent=1, default=plain) + '\n', encoding='utf-8')
    finally:
        trials.close()


# -- a run: workers, then the merged report -------------------------------------------------------------------------

def available_gb():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 2**20
    return 0.


PAUSE = S / 'eval' / 'PAUSE'  # While this file exists, no run starts workers (touch it to hold every eval).


def generating():
    """A generation (evaluation generate) is running on the box: its workers hold about 7 GB each."""
    return subprocess.run(['pgrep', '-u', str(os.getuid()), '-f', 'robot_platform.evaluation generate'],
                          capture_output=True).returncode == 0


def wait_for_memory(workers):
    """Until no generation runs, eval/PAUSE is absent, and MemAvailable has stayed over the floor plus each worker's
    need for 5 checks in a row, 30 s apart (a single reading once passed during a dip in others' use, 2026-10-18)."""
    need = MIN_AVAILABLE_GB + GB_PER_WORKER * workers
    steady, said = 0, 0.
    while steady < 5:
        blocked = PAUSE.exists() or generating()
        steady = 0 if blocked or available_gb() < need else steady + 1
        if steady == 0 and time.time() - said > 600:
            log(f"waiting: {available_gb():.1f} GB available, {need:.1f} GB wanted for {workers} workers"
                + ('; a generation is running' if generating() else '') + ('; eval/PAUSE' if PAUSE.exists() else ''))
            said = time.time()
        time.sleep(30)


def summarize(records, report, policy_note, out, started, spec, layouts, timeout_s):
    counted = [r for r in records if r['outcome'] != 'invalid_start']
    n = len(counted)
    from robot_platform.evaluation import stats
    successes = [r for r in counted if r['outcome'] == 'success']
    times = [r['stages']['bucket'] for r in successes]
    failures = {}
    for r in counted:
        if r['failure']:
            failures[r['failure']] = failures.get(r['failure'], 0) + 1
    ladder = {s: {'count': sum(r['ladder']['reached'][s] for r in counted)} for s in LADDER}
    trips = {}  # The receiver's own account of each trip: joint and check.
    for r in counted:
        trip = r.get('receiver_trip') or {}
        if r['failure'] == 'receiver_trip':
            key = f"{trip.get('joint', '?')} {trip.get('check', 'unrecorded')}"
            trips[key] = trips.get(key, 0) + 1
    for value in ladder.values():
        value['rate'] = value['count'] / n if n else None
    videos = {}
    # The failure shown is the first trial of the commonest failure; every trial's video stays in videos/.
    common = max(failures, key=failures.get) if failures else None
    for kind, pick in (('success', lambda r: r['outcome'] == 'success'),
                       ('failure', lambda r: r['outcome'] != 'success' and r['failure'] == common)):
        chosen = next((r for r in sorted(counted, key=lambda r: r['trial']) if pick(r) and r.get('video')), None)
        if chosen:
            name = f"{kind}_trial{chosen['trial']:02d}" + (f"_{chosen['failure']}" if kind == 'failure' else '') + '.mp4'
            target = out / name
            if not target.exists():
                os.link(chosen['video'], target)
            videos[kind] = str(target)
        else:
            videos[kind] = None
    epoch = None
    match = re.search(r'epoch_(\d+)', Path(spec).name)
    if match:
        epoch = int(match.group(1))
    return {
        'policy': report.get('policy'), 'checkpoint': None if spec == 'demonstrator' else str(spec), 'epoch': epoch,
        'sha256': (report.get('provenance') or {}).get('policy', {}).get('sha256'),
        'layouts': {'file': str(layouts), 'n': len(records), 'seed': SEED},
        'trials': {'counted': n, 'excluded_invalid_start': len(records) - n},
        'success': {'count': len(successes), 'rate': len(successes) / n if n else None,
                    'wilson95': list(stats.wilson_interval(len(successes), n))},
        'stage_ladder': ladder,
        'harness_stages': report['stage_completion'],
        'outcomes': report['outcomes'], 'failures': failures,
        'receiver_trips': trips,
        'failure_details': [{'trial': r['trial'], 'failure': r['failure'], 'detail': r.get('detail'),
                             'receiver_trip': (r.get('receiver_trip') or {}).get('text'),
                             'last_stage': r['ladder']['last'], 'duration_s': r['duration_s']}
                            for r in counted if r['failure']],
        'time_to_success_s': {'n': len(times), 'mean': statistics.fmean(times) if times else None,
                              'median': statistics.median(times) if times else None,
                              'min': min(times) if times else None, 'max': max(times) if times else None,
                              'definition': 'scene reset to the bucket_settled event, successes only'},
        'videos': videos, 'policy_settings': policy_note,
        'settings': {'timeout_s': timeout_s,
                     'record_hz': HZ, 'camera': '320x240 CAD', 'grippers': 'driven', 'gpu': GPU,
                     'look': 'lights, colours and wrist cameras varied (head camera from the layout)'},
        'wall_s': round(time.time() - started, 1), 'finished': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }


def run(args):
    """One run at a time on the box (every suite's watcher, any manual run): a lock, then the memory check, so two runs
    never both pass the check before either has loaded."""
    import fcntl
    lock = open(S / 'eval' / '.run.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        log('waiting for another evaluation run to finish (eval/.run.lock)')
        fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        return _run(args)
    finally:
        lock.close()


def launch_and_guard(args, spec, out, layouts):
    """Start the shard workers once memory allows, then watch MemAvailable every 15 s: under the floor twice in a row,
    stop them all (each resumes from its shard file on the next start) and return None. Otherwise their exit codes."""
    wait_for_memory(args.workers)
    log(f'{spec}: {args.workers} workers -> {out}')
    workers = []

    def stop():
        for process, handle in workers:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, 15)
                except ProcessLookupError:
                    pass
        for process, handle in workers:
            try:
                process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, 9)
                process.wait()
            handle.close()

    try:
        for k in range(args.workers):
            if k:
                for _ in range(STAGGER_S // 5):
                    time.sleep(5)
                    if available_gb() < MIN_AVAILABLE_GB + GB_PER_WORKER:
                        break
                if available_gb() < MIN_AVAILABLE_GB + GB_PER_WORKER:
                    log(f'not starting worker {k}: {available_gb():.1f} GB available')
                    stop()
                    return None
            command = ['nice', '-n', '10', str(PYTHON), str(Path(__file__).resolve()), 'shard', '--policy', spec,
                       '--out', str(out), '--layouts', str(layouts), '--k', str(k), '--of', str(args.workers),
                       '--timeout', str(args.timeout)] + (['--video'] if args.video else []) + \
                      (['--limit', str(args.limit)] if args.limit else []) + \
                      (['--only', str(args.only)] if args.only else [])
            handle = open(out / f'shard-{k}.log', 'a', encoding='utf-8')
            workers.append((subprocess.Popen(command, env=dict(os.environ, **ENV), stdout=handle,
                                             stderr=subprocess.STDOUT, start_new_session=True), handle))
        low = 0
        while any(process.poll() is None for process, _ in workers):
            time.sleep(15)
            low = low + 1 if available_gb() < MIN_AVAILABLE_GB else 0
            if low >= 2:
                log(f'{available_gb():.1f} GB available, under the {MIN_AVAILABLE_GB:g} GB floor: stopping the workers')
                stop()
                return None
    except BaseException:
        stop()
        raise
    codes = [process.wait() for process, _ in workers]
    for _, handle in workers:
        handle.close()
    return codes


def _run(args):
    from robot_platform.evaluation import report as reports
    spec = args.policy if args.policy == 'demonstrator' else str(Path(args.policy).resolve())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    layouts = Path(args.layouts)
    started = time.time()
    while True:
        codes = launch_and_guard(args, spec, out, layouts)
        if codes is not None:
            break
        log('stopped the workers: memory fell under the floor; they resume when it is back')
    records = []
    for k in range(args.workers):
        path = out / f'shard-{k}.jsonl'
        if path.exists():
            records += [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]
    records.sort(key=lambda r: r['trial'])
    expected = len(selected(json.loads(layouts.read_text(encoding='utf-8'))['layouts'], args))
    if any(codes) or len(records) != expected:
        log(f'workers exited {codes}; {len(records)} of {expected} trials recorded (see {out}/shard-*.log)')
        return 1
    meta = json.loads((out / 'meta.json').read_text(encoding='utf-8')) if (out / 'meta.json').exists() else {}
    document = json.loads(layouts.read_text(encoding='utf-8'))
    meta.setdefault('protocol', {})
    meta['protocol'].update(cube_poses=[dict(e['cube_start'], index=e['index']) for e in document['layouts']],
                            layouts={'file': str(layouts), 'seed': document['seed'], 'keys': [e['key'] for e in
                                                                                              document['layouts']]},
                            randomization=document['randomization'])
    report = reports.build_report(records, meta)
    reports.write(report, out)
    with (out / 'trials.jsonl').open('w', encoding='utf-8') as handle:
        for record in records:
            handle.write(json.dumps(record, default=plain) + '\n')
    note = json.loads((out / 'policy.json').read_text(encoding='utf-8')) if (out / 'policy.json').exists() else {}
    summary = summarize(records, report, note, out, started, spec, layouts, args.timeout)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1, default=plain) + '\n', encoding='utf-8')
    s = summary['success']
    log(f"{summary['policy']}: {s['count']}/{summary['trials']['counted']} succeeded "
        f"[Wilson 95% {s['wilson95'][0]:.2f}-{s['wilson95'][1]:.2f}]; ladder "
        + ', '.join(f"{k} {v['count']}" for k, v in summary['stage_ladder'].items())
        + f"; failures {summary['failures']}; {summary['wall_s']:.0f} s")
    return 0


# -- the watcher ----------------------------------------------------------------------------------------------------

def stable(path, quiet_s=90):
    try:
        first = path.stat()
        if time.time() - first.st_mtime < quiet_s:
            return False
        time.sleep(5)
        return path.stat().st_size == first.st_size
    except FileNotFoundError:
        return False


def training_done():
    """The suite's training run has started (its log exists) and ended: complete, or its tmux session gone."""
    if not TRAIN_LOG.exists():
        return False
    if 'Minerva training complete.' in TRAIN_LOG.read_text(encoding='utf-8', errors='replace'):
        return True
    return subprocess.run(['tmux', 'has-session', '-t', '=' + TRAIN_SESSION], capture_output=True).returncode != 0


class Board:
    """eval/... scalars per epoch in the suite's own TensorBoard run (EVAL_TB): each checkpoint's once, and each
    reference's rate (the demonstrator's, every baseline's) at that epoch once it exists."""

    def __init__(self):
        self.state_path = EVAL / 'tb_written.json'
        self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else {}
        self.writer = None

    def _writer(self):
        if self.writer is None:
            from torch.utils.tensorboard import SummaryWriter
            self.writer = SummaryWriter(str(TB))
        return self.writer

    def add(self, key, epoch, summary, references):
        if f'ckpt:{key}' not in self.state:
            w, s = self._writer(), summary['success']
            w.add_scalar('eval/success_rate', s['rate'], epoch)
            w.add_scalar('eval/success_wilson95_low', s['wilson95'][0], epoch)
            w.add_scalar('eval/success_wilson95_high', s['wilson95'][1], epoch)
            for stage, value in summary['stage_ladder'].items():
                w.add_scalar(f'eval_stages/{stage}', value['rate'], epoch)
            if summary['time_to_success_s']['mean'] is not None:
                w.add_scalar('eval/mean_time_to_success_s', summary['time_to_success_s']['mean'], epoch)
            w.flush()
            self.state[f'ckpt:{key}'] = epoch
        for name, reference in references.items():
            tag = 'demo' if name == 'demonstrator' else f'ref:{name}'
            if reference and f'{tag}:{epoch}' not in self.state:
                w = self._writer()
                w.add_scalar(f'eval/{name}_success_rate', reference['success']['rate'], epoch)
                w.flush()
                self.state[f'{tag}:{epoch}'] = reference['success']['rate']
        self.state_path.write_text(json.dumps(self.state, indent=1) + chr(10))


def evaluate_checkpoint(path, out, workers):
    command = [str(PYTHON), str(Path(__file__).resolve()), 'run', '--policy', str(path), '--out', str(out),
               '--workers', str(workers), '--video']
    return subprocess.run(command, env=dict(os.environ, **ENV)).returncode


def summary_of(out):
    path = Path(out) / 'summary.json'
    return json.loads(path.read_text()) if path.exists() else None


def watch(args):
    if not LAYOUTS.exists():
        if LAYOUTS_FROM:
            log(f'drawing the layouts of {LAYOUTS_FROM} again on {TREE.name}')
            redraw_layouts(LAYOUTS_FROM, LAYOUTS)
        else:
            log(f'drawing {args.n} held-out layouts from seed {SEED}')
            draw_layouts(args.n, SEED, LAYOUTS)
    demo = None
    if summary_of(DEMONSTRATOR) is None and DEMONSTRATOR.parent == EVAL:
        log('demonstrator reference: starting (1 worker, alongside)')
        demo = subprocess.Popen([str(PYTHON), str(Path(__file__).resolve()), 'run', '--policy', 'demonstrator',
                                 '--out', str(DEMONSTRATOR), '--workers', '1'], env=dict(os.environ, **ENV))
    elif DEMONSTRATOR.parent != EVAL:
        log(f'demonstrator reference: {DEMONSTRATOR} (same geometry; the demonstrator does not see the look)')
    board = Board()
    references = lambda: {'demonstrator': summary_of(DEMONSTRATOR),  # noqa: E731
                          **{name: summary_of(EVAL / name) for name in BASELINES}}
    finals, tries = set(), {}

    def attempt(path, out, label):
        if summary_of(out) is not None or tries.get(label, 0) >= 2 or not path.exists() or not stable(path):
            return
        tries[label] = tries.get(label, 0) + 1
        log(f'evaluating {label} (try {tries[label]})')
        log(f'{label}: exit {evaluate_checkpoint(path, out, args.workers)}')

    while True:
        for name, checkpoint in BASELINES.items():
            attempt(Path(checkpoint), EVAL / name, name)
        for path in sorted(CKPT.glob('minerva_epoch_*.pt')):
            attempt(path, EVAL / path.stem, path.name)
        if demo is not None and demo.poll() is not None:
            log(f'demonstrator reference: exit {demo.returncode}')
            demo = None
        for out in sorted(EVAL.glob('minerva_*')):
            summary = summary_of(out)
            if summary and summary.get('epoch'):
                board.add(out.name, summary['epoch'], summary, references())
        pending = [p.name for p in CKPT.glob('minerva_epoch_*.pt')
                   if summary_of(EVAL / p.stem) is None and tries.get(p.name, 0) < 2]
        pending += [name for name in BASELINES if summary_of(EVAL / name) is None and tries.get(name, 0) < 2]
        if training_done() and not pending:
            best = CKPT / 'minerva_best.pt'
            if best.exists() and 'best' not in finals and stable(best):
                finals.add('best')
                import torch
                epoch = int(torch.load(best, map_location='cpu', weights_only=False, mmap=True)['epoch']) + 1
                if summary_of(EVAL / f'minerva_epoch_{epoch:04d}') is None:
                    out = EVAL / f'minerva_best_epoch_{epoch:04d}'
                    log(f'final: minerva_best.pt is epoch {epoch}; evaluating it')
                    evaluate_checkpoint(best, out, args.workers)
                    summary = summary_of(out)
                    if summary:
                        summary['epoch'] = epoch
                        (out / 'summary.json').write_text(json.dumps(summary, indent=1) + chr(10))
                        board.add(out.name, epoch, summary, references())
                else:
                    log(f'final: minerva_best.pt is epoch {epoch}, already evaluated')
            if demo is None:
                log('all checkpoints evaluated; done')
                return 0
        time.sleep(120)


def status(args):
    rows = []
    outs = ([DEMONSTRATOR] + sorted(p for p in EVAL.iterdir() if p.is_dir() and p != DEMONSTRATOR)) \
        if EVAL.exists() else []
    for out in outs:
        path = out / 'summary.json'
        if not path.exists():
            shards = sum(len(p.read_text().splitlines()) for p in out.glob('shard-*.jsonl')) if out.exists() else 0
            if shards or out.name.startswith('minerva_'):
                rows.append(f'{out.name:28s} running or pending ({shards} trials recorded)')
            continue
        s = json.loads(path.read_text())
        rate, (low, high) = s['success']['rate'], s['success']['wilson95']
        ladder = ' '.join(f"{k}={v['count']}" for k, v in s['stage_ladder'].items())
        ttc = s['time_to_success_s']['mean']
        rows.append(f"{out.name:28s} success {s['success']['count']}/{s['trials']['counted']} = {rate:.0%} "
                    f"[{low:.0%}-{high:.0%}]  mean time {'-' if ttc is None else f'{ttc:.1f} s'}  {ladder}  "
                    f"failures {s['failures']}  trips {s.get('receiver_trips')}  videos {s['videos']}")
    print(chr(10).join(rows) if rows else 'nothing evaluated yet')
    return 0


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('layouts')
    a.add_argument('--n', type=int, default=30)
    a.add_argument('--seed', type=int, default=SEED)
    a.add_argument('--out', default=str(LAYOUTS))
    a.add_argument('--from', dest='source', help="another suite's layouts.json: the same draws on this tree")
    r = sub.add_parser('run')
    r.add_argument('--policy', required=True, help="'demonstrator' or a checkpoint path")
    r.add_argument('--out', required=True)
    r.add_argument('--layouts', default=str(LAYOUTS))
    r.add_argument('--workers', type=int, default=2)
    r.add_argument('--video', action='store_true')
    r.add_argument('--limit', type=int, default=0, help='only the first N layouts (smoke tests)')
    r.add_argument('--only', default='', help='only these layout indices, e.g. 4,6,8 (diagnostic re-runs)')
    r.add_argument('--timeout', type=float, default=TIMEOUT_S)
    h = sub.add_parser('shard')
    h.add_argument('--policy', required=True)
    h.add_argument('--out', required=True)
    h.add_argument('--layouts', required=True)
    h.add_argument('--k', type=int, required=True)
    h.add_argument('--of', type=int, required=True)
    h.add_argument('--video', action='store_true')
    h.add_argument('--limit', type=int, default=0)
    h.add_argument('--only', default='')
    h.add_argument('--timeout', type=float, default=TIMEOUT_S)
    w = sub.add_parser('watch')
    w.add_argument('--n', type=int, default=30)
    w.add_argument('--workers', type=int, default=2)
    sub.add_parser('status')
    args = p.parse_args()
    if args.command == 'layouts':
        if args.source:
            document = redraw_layouts(args.source, Path(args.out))
            log(f"{len(document['layouts'])} layouts drawn again from {args.source} -> {args.out}")
        else:
            document = draw_layouts(args.n, args.seed, Path(args.out))
            log(f"{len(document['layouts'])} layouts from {document['attempts']} attempts -> {args.out}")
        return 0
    return {'run': run, 'shard': shard, 'watch': watch, 'status': status}[args.command](args) or 0


if __name__ == '__main__':
    sys.exit(main())
