"""Policy mode end to end on the virtual receiver: a policy's chunks drive the actual Rust receiver through the
lifecycle in place of the leaders. Enable previews its first action, Engage aligns to it, the arms then follow the
chunks in real time despite inference latency, Hold pauses, and a policy that stops answering leaves the arms held."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
PROFILES = ('minerva-teleop.json', 'minerva-teleop-direct-tuning.json')
SWAY = {'left_arm_j1': .1, 'right_arm_j1': -.1, 'left_arm_j4': .08}  # Amplitude around the reference pose.
SWAY_PERIOD_S = 2.

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Sway:
    """A time-indexed trajectory, so successive chunks agree: each SWAY joint swings around its reference."""
    name = 'sway'
    action_dt = .1

    def reset(self, context):
        self.names, self.home = tuple(context.joint_names), np.asarray(context.home, dtype=float)

    def at(self, t):
        q = self.home.copy()
        for name, amplitude in SWAY.items():
            q[self.names.index(name)] += amplitude * math.sin(2 * math.pi * t / SWAY_PERIOD_S)
        return q

    def __call__(self, observation):
        assert len(observation['q']) == len(self.names) and len(observation['qcmd']) == len(self.names)
        return np.array([self.at(observation['t'] + k * self.action_dt) for k in range(20)])


class ScriptedWorker:
    """PolicyWorker's interface, deterministic: each answer arrives `delay` polls (loop cycles) after its
    observation, as inference latency; `silent` stops it answering."""

    def __init__(self, policy, names, context, *, delay=4):
        self.policy, self.delay, self.silent, self.closed = policy, delay, False, False
        policy.reset(SimpleNamespace(joint_names=tuple(names), index=0, **context))
        self.info = {'name': policy.name, 'action_dt': policy.action_dt, 'camera_roles': [], 'describe': None}
        self.queue = []

    def ready(self, timeout_s=None):
        return self.info

    def ask(self, request, obs_time, state):
        if not self.silent:
            chunk = self.policy(dict(state, t=obs_time))
            self.queue.append([self.delay, {'request': request, 'chunk': chunk, 'inference_ms': 1.}])

    def poll(self):
        for item in self.queue:
            item[0] -= 1
        ready = [reply for wait, reply in self.queue if wait <= 0]
        self.queue = [item for item in self.queue if item[0] > 0]
        return ready

    def close(self):
        self.closed = True


@pytest.fixture(scope='module', params=PROFILES)
def policy(request, tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('policy')
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    assert helm.saved_limits is not None, helm.saved_limits_error
    session = TeleopSession(helm, DEVICES / request.param, runs=tmp_path / 'runs',
                            leaders=SimpleNamespace(saved=None, missing=lambda routes: []))
    if session.manual is not None:  # As the operator's preset for teleop: every joint follows, at its full speed.
        session.manual.accept([dict(row, follow=True, travel_rad=None, speed=template['bounds']['speed'][1])
                               for row, template in zip(session.manual.active, session.manual.rows)])
    clock, workers, sway = Clock(), [], Sway()

    def open_policy(spec, names, context):
        assert spec == 'sway'
        workers.append(ScriptedWorker(sway, names, context))
        return workers[-1]
    core = TeleopCore(dict(session._bundle(), source={'kind': 'policy', 'policy': 'sway'}), clock=clock,
                      open_policy=open_policy)
    core.build()
    yield SimpleNamespace(core=core, clock=clock, worker=workers[0], sway=sway,
                          names=[joint['name'] for joint in core.derived['config']['joints']])
    core.close()
    helm.close()
    assert workers[0].closed


def state(core):
    return core.session.lifecycle.state


def run(core, clock, key=None, *, until=None, seconds=None, limit=4000):
    cycles = limit if seconds is None else max(1, round(seconds / core.period_s))
    for _ in range(cycles):
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


def test_enable_previews_engage_aligns_and_the_arms_follow_the_chunks_despite_latency(policy):
    core, clock, sway = policy.core, policy.clock, policy.sway
    run(core, clock, 'I', until=lambda c: state(c) == 'IDLE')
    assert core.source.requests == 0  # Nothing asked before Enable.
    run(core, clock, 'E', until=lambda c: state(c) == 'HOLD')
    run(core, clock, until=lambda c: c.source.aligned is not None, limit=20)  # Enable previews its first action.
    view = core.status_view()['policy']
    assert view['aligned'] and view['policy'] == 'sway' and view['spec'] == 'sway'
    preview = core.source.aligned.copy()
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP')  # Aligned to it at the alignment speed.
    assert all(abs(measured(core)[n] - q) < .02 for n, q in zip(policy.names, preview))
    run(core, clock, seconds=.5)  # Settle onto the rollout.
    errors = []
    for _ in range(30):  # 3 s, one and a half sways, each chunk answered four cycles after its observation.
        run(core, clock, seconds=.1)
        assert state(core) == 'TELEOP'
        expected = dict(zip(policy.names, sway.at(clock.now)))
        errors.append(max(abs(measured(core)[n] - expected[n]) for n in SWAY))
    assert max(errors) < .05  # The arms follow the policy's trajectory in real time, not its late chunks' starts.
    assert core.source.chunks > 30 and core.source.stale_cycles == 0  # Inference runs on through the rollout.
    rows = core.status_view()
    assert rows['policy']['chunk_age_s'] is not None and rows['replay'] is None


def test_hold_pauses_and_engage_aligns_anew_and_a_rollout_is_not_recorded(policy):
    core, clock = policy.core, policy.clock
    if state(core) != 'TELEOP':
        pytest.skip('needs the previous test to leave the policy following')
    run(core, clock, 'T', until=lambda c: state(c) == 'HOLD')
    run(core, clock, until=lambda c: c.source.aligned is not None, limit=20)  # A new preview from where they hold.
    run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP')
    with pytest.raises(ValueError, match='has no leaders to zero'):
        core.set_zero()
    with pytest.raises(ValueError, match='no leader routes'):
        core.route('flip', 'left_arm_j1')
    core.task_capture = SimpleNamespace(command=lambda action: pytest.fail('a rollout was recorded'))
    try:
        with pytest.raises(ValueError, match='not recorded yet'):
            core.task_action('start_record')
    finally:
        core.task_capture = None


def test_a_policy_that_stops_answering_leaves_the_arms_held(policy):
    core, clock, worker = policy.core, policy.clock, policy.worker
    if state(core) != 'TELEOP':
        pytest.skip('needs the previous test to leave the policy following')
    worker.silent = True
    run(core, clock, until=lambda c: state(c) == 'HOLD', limit=round(3. / core.period_s))  # Stale chunks: held,
    assert core.fault is None and core.source.stale_cycles > 0  # as for lost leaders, without a fault.
    worker.silent = False
    run(core, clock, until=lambda c: c.source.aligned is not None, limit=40)
    run(core, clock, 'X', until=lambda c: state(c) == 'DISABLED')  # Return lowers the arms as in teleop.
