"""Live replay end to end on the virtual receiver: a recorded episode drives the actual Rust receiver through the
guarded lifecycle in place of the leaders. Engage aligns to the first frame, the arms then follow the recorded
commands, the end holds, and Engage again replays from the start."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.data.task_capture import TaskEpisodeWriter
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import REPLAY_SETTLE_S, TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
SESSION = 'live-20261022T200000Z'
MOVING = {'left_arm_j1': 0.12, 'right_arm_j1': -0.12, 'left_arm_j4': 0.08}  # Peak offsets from the reference pose.
NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        not default_binary().is_file() or not library_path().is_file(),
        reason='Build control_harness and the robot_collision library first',
    ),
]


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def write_episode(root, names, home, *, seconds=1.5, hz=15.0):
    """Each moving joint sways out from its reference and back over the episode; the others, and the neck, hold."""
    roster = (*names, 'head_pan', 'head_tilt')  # An episode records the whole robot; a replay drives the arms by name.
    writer = TaskEpisodeWriter(
        root / SESSION / 'episodes', roster, {'data_source': 'real', 'camera_roles': ['head']}, hz=hz, max_pending=64
    )
    for i in range(int(seconds * hz) + 1):
        t = i / hz
        q = [home.get(name, 0.0) + MOVING.get(name, 0.0) * math.sin(math.pi * t / seconds) for name in roster]
        writer.append(
            q=q,
            qcmd=q,
            source_time=50.0 + t,
            scene_time=None,
            images={'head': np.zeros((4, 6, 3), np.uint8)},
            camera_times={'head': 50.0 + t - 0.01},
            task_state={'i': i},
            authority='human',
        )
    path = Path(writer.finish())
    return f'{SESSION}/{path.name[: -len(".hdf5")]}'


# The guarded profile, and the direct profile with Motor tuning that Minerva's terminal runs (its lifecycle takes
# targets in its own joint order: a replay built with the tuning block's set of names faulted, 2026-10-22).
PROFILES = ('minerva-teleop.json', 'minerva-teleop-direct-tuning.json')


@pytest.fixture(scope='module', params=PROFILES)
def replay(request, tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('replay')
    helm = HelmSession(
        DEVICES / 'minerva-joint-calibration.json',
        runs=tmp_path / 'runs',
        limits=DEVICES / 'minerva-production-01-joint-limits.json',
        seed=SEED,
    )
    assert helm.saved_limits is not None, helm.saved_limits_error
    session = TeleopSession(
        helm,
        DEVICES / request.param,
        runs=tmp_path / 'runs',
        leaders=SimpleNamespace(saved=None, missing=lambda routes: []),
    )
    if session.manual is not None:  # As the operator's preset for teleop: every joint follows, at its full speed.
        session.manual.accept(
            [
                dict(row, follow=True, travel_rad=None, speed=template['bounds']['speed'][1])
                for row, template in zip(session.manual.active, session.manual.rows)
            ]
        )
    bundle = session._bundle()
    # The probe: any complete episode with the joints, to build the receiver and learn its reference pose.
    probe = write_episode(tmp_path, NAMES, {name: 0.0 for name in NAMES})
    clock = Clock()
    core = TeleopCore(
        dict(bundle, source={'kind': 'episode', 'episode': probe, 'speed': 1.0, 'root': str(tmp_path)}), clock=clock
    )
    core.build()
    names = [joint['name'] for joint in core.derived['config']['joints']]
    home = {row['motor']: row['home'] for row in core.derived['report']}
    episode = write_episode(tmp_path, names, home)
    core.source_spec = dict(core.source_spec, episode=episode)
    core.source = core._open_source(names)
    yield SimpleNamespace(core=core, clock=clock, names=names, home=home, episode=episode)
    core.close()
    helm.close()


def run(core, clock, key=None, *, until=None, seconds=None, limit=4000):
    cycles = limit if seconds is None else max(1, round(seconds / core.period_s))
    for _ in range(cycles):
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and until(core):
            return
    assert until is None, f'still {core.session.lifecycle.state}: {core.message}'


def measured(core):
    last = core.harness.last
    names = last['status'].get('joint_names') or [joint['name'] for joint in core.derived['config']['joints']]
    return {name: row['position'] for name, row in zip(names, last['feedback'])}


def test_engage_aligns_to_the_first_frame_then_the_arms_follow_the_recording_and_hold_at_its_end(replay):
    core, clock = replay.core, replay.clock
    state = lambda c: c.session.lifecycle.state
    run(core, clock, 'I', until=lambda c: state(c) == 'IDLE')
    run(core, clock, 'E', until=lambda c: state(c) == 'HOLD')
    assert core.status_view()['replay']['time_s'] == 0.0
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP')  # Aligned to frame 0 at the alignment speed.
    start = core.source.position_at(0.0)
    assert all(abs(measured(core)[n] - q) < 0.02 for n, q in zip(replay.names, start))
    errors, peak = [], 0.0
    while state(core) == 'TELEOP':
        run(core, clock, seconds=0.1)
        if state(core) != 'TELEOP' or core.source.finished:
            break
        expected = dict(zip(replay.names, core.source.position_at(core.source.t)))
        errors.append(max(abs(measured(core)[n] - expected[n]) for n in MOVING))
        peak = max(peak, abs(measured(core)['left_arm_j1'] - replay.home['left_arm_j1']))
    assert (
        len(errors) >= 10 and max(errors) < 0.05
    )  # The arms track the recorded commands (within the lifecycle's lag).
    assert peak > 0.1  # And they moved: left J1 swung most of its recorded 0.12 rad out from the reference.
    run(core, clock, until=lambda c: state(c) == 'HOLD', limit=round((REPLAY_SETTLE_S + 2.0) / core.period_s))
    assert core.source.finished and 'Replay finished' in core.message
    end = dict(zip(replay.names, core.source.position_at(core.source.duration)))
    assert all(abs(measured(core)[n] - end[n]) < 0.02 for n in MOVING)  # Held at the last frame: back near home.
    assert core.status_view()['replay']['finished'] is True


def test_engage_after_the_end_replays_from_the_start_and_a_replay_is_never_recorded(replay):
    core, clock = replay.core, replay.clock
    state = lambda c: c.session.lifecycle.state
    if state(core) != 'HOLD':
        pytest.skip('needs the previous test to leave the replay held at its end')
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP')
    assert not core.source.finished and core.source.t < 0.2  # Rewound to the first frame.
    run(core, clock, seconds=0.5)
    assert core.source.t > 0.3
    with pytest.raises(ValueError, match='has no leaders to zero'):
        core.set_zero()
    with pytest.raises(ValueError, match='no leader routes'):
        core.route('flip', 'left_arm_j1')
    core.task_capture = SimpleNamespace(command=lambda action: pytest.fail('a replay was recorded'))
    try:
        with pytest.raises(ValueError, match='not recorded'):
            core.task_action('start_record')
    finally:
        core.task_capture = None
    run(core, clock, 'X', until=lambda c: state(c) == 'DISABLED')  # Return lowers the arms as in teleop.
