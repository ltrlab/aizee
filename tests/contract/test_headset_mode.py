"""The Headset input end to end on the virtual receiver: the controllers' grab and drag (teleop.vr_arms, as the worker
runs it) drives the actual Rust receiver through the lifecycle in place of the leaders. Engage raises the hanging
arms to their ready pose, a held grip drags its arm's tool as the hand moved (in the robot's frame) without swinging
the shoulder, the trigger closes the gripper, lost hands hold the arm where it was, and B/Y holds; Engage again
leaves the arms where they are."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path
from robot_platform.teleop.headset_source import READY
from robot_platform.teleop.vr_arms import HandSample
from robot_platform.teleop.vr_worker import _answer, build

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
HOME = (0., 1.2, -.4)  # The hand in the headset's space: chest height, 40 cm ahead.

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class ScriptedVR:
    """VRWorker's interface, deterministic: the worker's own VRArms (built from the setup the loop sends), updated
    each time the loop asks for the newest answer, on the loop's clock."""

    def __init__(self, setup, clock):
        self.setup, self.clock, self.vr = setup, clock, build(setup)
        self.samples, self.seeded, self.closed = {}, 0, False
        self.holds = {arm: 0 for arm in self.vr.arms}

    def ready(self, timeout_s=None):
        return {'hello': 1, 'arms': sorted(self.vr.arms)}

    def hands(self, hands, epoch, forward):
        for arm, value in hands.items():
            self.samples[arm] = HandSample(tuple(value['position']), tuple(value['orientation']), value['grip'],
                                           value['trigger'], value['hold'], self.clock(), value['ui'])

    def seed(self, number, positions):
        self.seeded = number
        for arm, q in positions.items():
            self.vr.seed(arm, q)

    def latest(self):
        now = self.clock()
        commands = self.vr.update(now, self.samples)
        for arm, command in commands.items():
            self.holds[arm] += bool(command.hold)
        triggers = {arm: s.trigger for arm, s in self.samples.items() if now - s.at <= self.vr.settings.stale_s}
        return {'seed': self.seeded, 'commands': _answer(commands, self.holds, triggers), 'compute_ms': 1.}, now

    def close(self):
        self.closed = True


@pytest.fixture(scope='module')
def headset(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('headset')
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    assert helm.saved_limits is not None, helm.saved_limits_error
    session = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path / 'runs',
                            leaders=SimpleNamespace(saved=None, missing=lambda routes: []))
    clock, workers = Clock(), []

    def open_vr(setup):
        assert set(setup['limits']) == {'left', 'right'} and setup['period_s'] == pytest.approx(.02)
        workers.append(ScriptedVR(setup, clock))
        return workers[-1]
    core = TeleopCore(dict(session._bundle(), source={'kind': 'headset'}), clock=clock, open_vr=open_vr)
    core.build()
    yield SimpleNamespace(core=core, clock=clock, worker=workers[0])
    core.close()
    helm.close()
    assert workers[0].closed


def state(core):
    return core.session.lifecycle.state


def run(core, clock, key=None, *, until=None, seconds=None, limit=4000, hands=None):
    cycles = limit if seconds is None else max(1, round(seconds / core.period_s))
    for _ in range(cycles):
        if hands is not None:
            core.headset_hands({'hands': hands(clock()), 'epoch': 1, 'forward': 0.})
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and until(core):
            return
    assert until is None, f'still {state(core)}: {core.message}'


def measured(core):
    last = core.harness.last
    names = last['status'].get('joint_names') or [joint['name'] for joint in core.derived['config']['joints']]
    return {name: row['position'] for name, row in zip(names, last['feedback'])}


def tool(core, worker, arm):
    """The arm's tool point now, from the receiver's measured motors through the CAD map."""
    return worker.vr.arms[arm].kinematics.pose(core.source._cad(arm, measured(core)))[:3, 3]


def raised(share=1., circle=0.):
    """The hand `share` of the way to 6 cm up and 6 cm ahead of HOME (the headset's space: y up, -z ahead), then
    `circle` radians round a 3 cm circle across the robot's front (sideways and up) from there."""
    return (HOME[0] + .03 * math.sin(circle), HOME[1] + .06 * share + .03 * (1 - math.cos(circle)),
            HOME[2] - .06 * share)


def hand(position=HOME, grip=1., trigger=0., b=False):
    """A controller as Helm checked it (helm.xr.hand_sample)."""
    return {'position': tuple(position), 'orientation': (0., 0., 0., 1.), 'grip': grip, 'trigger': trigger,
            'hold': b, 'ui': False}


def test_engage_raises_the_hanging_arms_and_a_held_grip_drags_from_there_without_swinging(headset):
    core, clock, worker = headset.core, headset.clock, headset.worker
    run(core, clock, 'I', until=lambda c: state(c) == 'IDLE')
    run(core, clock, 'E', until=lambda c: state(c) == 'HOLD')
    open_left = core.source.grippers['left'][1]
    # Engage: both arms hang, so both rise to the ready pose at the alignment speed; the worker is seeded there.
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP', hands=lambda t: {'left': hand(grip=0.)})
    run(core, clock, seconds=1., hands=lambda t: {'left': hand(grip=0.)})
    after = measured(core)
    assert after['left_gripper'] == pytest.approx(open_left, abs=.02)  # The trigger is released: open.
    for arm in ('left', 'right'):
        np.testing.assert_allclose(core.source._cad(arm, after), READY[arm], atol=.02)
        np.testing.assert_allclose(worker.vr.arms[arm].q, READY[arm], atol=1e-9)
    view = core.status_view()['headset']
    assert view['following'] and view['raised'] == ['left', 'right']
    # Grip, then raise the hand 6 cm and move it 6 cm ahead over half a second: the tool rises 6 cm and moves 6 cm
    # to the robot's front (-y).
    start = tool(core, worker, 'left')
    right = tool(core, worker, 'right')
    run(core, clock, seconds=.2, hands=lambda t: {'left': hand()})
    t0 = clock()
    run(core, clock, seconds=1., hands=lambda t: {'left': hand(raised(min(1., (t - t0) / .5)))})
    # Then a 3 cm circle across the front, as near the hanging pose swung J1 and J5 by over a radian: from the ready
    # pose no joint turns far.
    ready, swing = core.source._cad('left', measured(core)), np.zeros(6)
    t0 = clock()
    for _ in range(round(6. / core.period_s)):
        run(core, clock, seconds=core.period_s,
            hands=lambda t: {'left': hand(raised(circle=min(2 * math.pi, 1.2 * (t - t0))))})
        swing = np.maximum(swing, np.abs(np.subtract(core.source._cad('left', measured(core)), ready)))
    assert swing.max() < .5 and swing[0] < .2, swing
    np.testing.assert_allclose(tool(core, worker, 'left') - start, (0., -.06, .06), atol=.006)
    np.testing.assert_allclose(tool(core, worker, 'right'), right, atol=1e-3)  # No right hand: it holds.
    arms = core.status_view()['headset']['arms']
    assert arms['left']['gripped'] and arms['left']['converged'] and not arms['right']['gripped']
    assert not arms['left']['near_singular']


def test_the_trigger_closes_the_gripper_and_lost_hands_hold_the_arm_where_it_was(headset):
    core, clock, worker = headset.core, headset.clock, headset.worker
    assert state(core) == 'TELEOP'
    run(core, clock, seconds=1.5, hands=lambda t: {'left': hand(raised(), trigger=1.)})
    assert measured(core)['left_gripper'] == pytest.approx(core.source.grippers['left'][2], abs=.05)
    held = tool(core, worker, 'left')
    run(core, clock, seconds=1.)  # The headset goes quiet: the hand lets go, the arm holds, the session follows on.
    assert state(core) == 'TELEOP' and not core.status_view()['headset']['arms']['left']['tracking']
    np.testing.assert_allclose(tool(core, worker, 'left'), held, atol=1e-3)
    assert measured(core)['left_gripper'] == pytest.approx(core.source.grippers['left'][2], abs=.05)


def test_b_or_y_holds_and_engage_follows_again_from_where_the_arms_are(headset):
    core, clock, worker = headset.core, headset.clock, headset.worker
    seed = core.source.seed
    run(core, clock, until=lambda c: state(c) == 'HOLD', limit=20, hands=lambda t: {'left': hand(grip=0., b=True)})
    assert 'Held from the headset' in core.message
    run(core, clock, seconds=.2, hands=lambda t: {'left': hand(grip=0.)})
    assert state(core) == 'HOLD'  # The press is not taken again, nor as an Engage.
    held = measured(core)
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP', hands=lambda t: {'left': hand(grip=0.)})
    run(core, clock, seconds=.2, hands=lambda t: {'left': hand(grip=0.)})
    assert core.source.seed == seed + 1 and core.source.raised == ()  # Not hanging: nothing rises.
    q = core.source._cad('left', measured(core))
    np.testing.assert_allclose(q, core.source._cad('left', held), atol=2e-3)  # The arm stayed where it was held,
    np.testing.assert_allclose(worker.vr.arms['left'].q, q, atol=2e-3)  # and the drag starts there.
