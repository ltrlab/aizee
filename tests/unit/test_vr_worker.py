"""The headset's grab and drag in its own process (teleop.vr_worker): it builds both arms' chains, updates on its own
grid from the newest hands and seed, names the seed each answer comes from, counts B/Y presses, says where a trigger
is before any seed, and says why when it cannot start. The real process starts and closes too."""

import json
import math
import time

import pytest

from robot_platform.ops.release import ROOT
from robot_platform.teleop.vr_worker import VRWorker

WORKING = {'left': [-0.1, 0.6, -1.6, -0.6, -2.8, 0.2], 'right': [0.6, -0.4, -1.7, -1.0, 2.7, -0.4]}
HOME = [0.0, 1.2, -0.4]


def setup(**changes):
    swept = json.loads((ROOT / 'config/devices/minerva-production-01-joint-limits.json').read_text())['joints']
    limits = {
        arm: {
            f'{arm}_arm_j{n}': (swept[f'{arm}_arm_j{n}']['min_rad'], swept[f'{arm}_arm_j{n}']['max_rad'])
            for n in range(1, 7)
        }
        for arm in ('left', 'right')
    }
    return dict(
        {'model': str(ROOT / 'urdf/minerva/model.json'), 'verified': {}, 'limits': limits, 'period_s': 0.02}, **changes
    )


def hand(position=HOME, grip=1.0, trigger=0.0, b=False, ui=False):
    """A controller as Helm checked it (helm.xr.hand_sample)."""
    return {
        'position': tuple(position),
        'orientation': (0.0, 0.0, 0.0, 1.0),
        'grip': grip,
        'trigger': trigger,
        'hold': b,
        'ui': ui,
    }


def wait_for(worker, predicate, timeout=5.0, hands=None):
    """The first answer `predicate` accepts. With `hands`, sent again each poll as the headset streams them: a hand
    sent once goes stale if a loaded machine holds the worker back past its staleness limit."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if hands is not None:
            worker.hands(hands, 1, 0.0)
        latest = worker.latest()
        if latest is not None and predicate(latest[0]):
            return latest[0]
        time.sleep(0.005)
    raise AssertionError(f'no such answer; the last: {worker.latest()}')


@pytest.fixture
def worker():
    worker = VRWorker(setup(), in_process=True)
    yield worker
    worker.close()


def test_the_worker_drags_an_arm_from_its_seed_and_names_the_seed(worker):
    assert worker.ready() == {'hello': 1, 'arms': ['left', 'right']}
    # Before any seed it still says where each fresh trigger is, for the gripper Engage aligns to.
    first = wait_for(
        worker, lambda m: m['commands']['left']['trigger'] == 0.7, hands={'left': hand(grip=0.0, trigger=0.7)}
    )
    assert first['seed'] == 0 and first['commands']['left']['positions'] is None
    assert first['commands']['right']['trigger'] is None  # No right hand.
    worker.seed(1, WORKING)
    seeded = wait_for(worker, lambda m: m['seed'] == 1)
    assert seeded['commands']['left']['positions'] == pytest.approx(WORKING['left'])
    # Grip, then move 10 cm down: the left arm follows; the right, with no hand, holds.
    wait_for(worker, lambda m: m['commands']['left']['gripped'], hands={'left': hand(grip=1.0)})
    moved = wait_for(
        worker,
        lambda m: m['commands']['left']['positions'] != pytest.approx(WORKING['left']),
        hands={'left': hand([HOME[0], HOME[1] - 0.1, HOME[2]])},
    )
    assert moved['seed'] == 1 and moved['commands']['right']['positions'] == pytest.approx(WORKING['right'])
    # A sanity check only, loaded machines included: the Jetson's timing is in teleop.vr_worker's docstring.
    assert moved['compute_ms'] < 250 and math.isfinite(moved['commands']['left']['sigma'])


def test_b_or_y_presses_are_counted_so_none_is_lost_between_answers(worker):
    worker.ready()
    worker.seed(1, WORKING)
    for b in (False, True, False, True):
        worker.hands({'left': hand(b=b)}, 1, 0.0)
        wait_for(worker, lambda m: m['seed'] == 1)
        time.sleep(0.05)  # A few of its updates each.
    assert wait_for(worker, lambda m: m['commands']['left']['holds'] == 2)['commands']['right']['holds'] == 0


def test_a_worker_that_cannot_build_says_why():
    worker = VRWorker(setup(limits={'left': {'left_arm_j1': (0.0, 1.0)}}), in_process=True)
    try:
        with pytest.raises(RuntimeError, match='could not start: ValueError: Limits are needed'):
            worker.ready()
    finally:
        worker.close()


def test_the_worker_process_starts_answers_and_closes():
    worker = VRWorker(setup())
    try:
        assert worker.ready()['arms'] == ['left', 'right']
        worker.seed(1, WORKING)
        assert wait_for(worker, lambda m: m['seed'] == 1, timeout=10.0)['commands']['left'][
            'positions'
        ] == pytest.approx(WORKING['left'])
    finally:
        worker.close()
    assert not worker.alive
