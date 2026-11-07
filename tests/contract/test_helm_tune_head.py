"""Helm's Tune on the head: its jog (never a sweep: turned and tilted down together, the head reaches its shoulder,
the user 2026-10-29), on the arms' sweep engine and Tune loop over the head's receiver, under the direct policy. Each
joint alone goes slowly from its reference to each end and back. The receiver the live jog opens is the native
harness, so the roster, the enable, every direct update and the evidence run for real without servos."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.helm.tune import TuneCore
from robot_platform.neck import head
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary

ROOT = Path(__file__).resolve().parents[2]
SHORT = {
    'amplitudes': [0.1, 0.3],
    'kp_ladder': [1.0, 0.5],
    'damping': [1.0, 0.6],
    'speeds': [0.4, 0.8],
    'decelerations': [4.0],
    'hold_s': 0.2,
    'rest_s': 0.1,
}

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not default_binary().is_file(), reason='Build control_harness first'),
]


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Receiver:
    """The head receiver's interface (exchange, last, transport hook, close) over the actual Rust core."""

    def __init__(self, config, reference, clock, start=None):
        roster = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
        start = start or {name: q + 0.02 for name, q in roster.items()}
        self.harness = NativeControlHarness(config, [start[joint['name']] for joint in config['joints']])
        self.harness.step(0)
        self.clock, self.epoch, self.config, self.requests = clock, clock(), config, []
        self.transport = SimpleNamespace(on_result=None)

    @property
    def last(self):
        return self.harness.last

    def exchange(self, request):
        self.requests.append(request['type'])
        now_ms = max(self.harness.now_ms, int((self.clock() - self.epoch) * 1000))
        result = self.harness.step(now_ms, request)
        if self.transport.on_result:
            self.transport.on_result(result)
        return result['reply']

    def close(self):
        self.harness.close()


@pytest.fixture
def bundle(tmp_path):
    profile = json.loads((ROOT / 'config/devices/minerva-head.json').read_text(encoding='utf-8'))
    profile['sweep']['defaults'] = SHORT
    profile['bus'] = {
        'device': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46083062-if00',
        'usb_serial': '5A46083062',
        'baud': 1000000,
    }
    path = tmp_path / 'head.json'
    path.write_text(json.dumps(profile))
    reference = head.virtual_reference(profile)
    return {
        'kind': 'tune',
        'mode': 'jog',
        'teleop_profile': str(path),
        'arm': 'head',
        'receiver': {'mode': 'live', 'arm': 'head'},
        'joints': None,
        'head_reference': reference,
        'limits': head.virtual_limits(reference, span_rad=0.8),
        'servo': None,
    }


def test_the_head_jog_takes_each_joint_alone_slowly_to_its_ends_and_back(bundle, tmp_path):
    clock = Clock()
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock))
        return receivers[-1]

    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    receiver, engine = receivers[0], core.engine
    joints = {joint['name']: joint for joint in receiver.config['joints']}
    assert set(joints) == {'head_pan', 'head_tilt'} and 'collision' not in receiver.config
    assert engine.sweep == ['head_pan', 'head_tilt'] and 'JOG' in core.message
    speed = engine.settings['head_pan']['characterize_speed']
    before = dict(engine.targets)
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        targets = dict(engine.targets)
        # Never both joints off their reference at once: one joint alone never reaches the shoulder.
        assert sum(abs(targets[name] - engine.home[name]) > 0.03 for name in targets) <= 1, targets
        if engine.task != 'starting':  # Starting, the plan takes up where the head rests: not a move.
            jumps = {name: abs(targets[name] - before[name]) for name in targets}
            assert max(jumps.values()) <= speed * core.period_s + 1e-9, (engine.task, jumps)
        before = targets
        if core.stage == 'finished':
            break
    assert core.stage == 'finished' and engine.task == 'finished', core.message
    assert engine.checker is None and 'move' not in receiver.requests
    assert receiver.requests.count('direct_update') > 100
    assert receiver.last['status']['state'] == 'disarmed' and not receiver.last['hardware_enabled']
    for name in ('head_pan', 'head_tilt'):
        result = engine.summary()[name]
        assert result['status'] == 'done' and result['recommendation'] is None
        jog = result['jog']
        assert jog['reached'] == {
            'lower': pytest.approx(jog['lower'], abs=0.03),
            'upper': pytest.approx(jog['upper'], abs=0.03),
        }
        assert jog['lower'] < -0.5 and jog['upper'] > 0.5 and jog['speed'] == speed
    core.close()
    assert core.evidence.name.startswith('jog-head-')
    assert json.loads((core.evidence / 'results.json').read_text())['task'] == 'finished'


def test_the_head_is_jogged_never_swept_and_only_the_head_is_jogged(bundle):
    for mode in ('tune', 'verify', 'return'):
        with pytest.raises(ValueError, match='jogged, never swept'):
            TuneCore(dict(bundle, mode=mode, start_pose={'head_pan': 3.0}))
    arms = dict(bundle, teleop_profile=str(ROOT / 'config/devices/minerva-teleop.json'))
    with pytest.raises(ValueError, match='head profile'):
        TuneCore(arms)
    with pytest.raises(ValueError, match='jogged, never swept'):
        TuneCore(dict(arms, arm='left', receiver={'mode': 'virtual', 'arm': 'left'}))


def session(tmp_path):
    from robot_platform.helm.tune import TuneSession

    helm = SimpleNamespace(
        profile_path=ROOT / 'config/devices/minerva-joint-calibration.json',
        asset_id='minerva-test',
        operator_files=tmp_path / 'operator',
    )
    (tmp_path / 'operator').mkdir(exist_ok=True)
    return TuneSession(helm, ROOT / 'config/devices/minerva-teleop.json')


def test_the_tune_session_offers_the_head_and_needs_it_set_up_to_run_live(tmp_path):
    tune = session(tmp_path)
    assert [m['name'] for m in tune.snapshot()['head']['motors']] == ['head_pan', 'head_tilt']
    tune.configure(arm='head', mode='virtual', target='pc', verify=True)
    virtual = tune._bundle()
    assert virtual['arm'] == 'head' and virtual['head_reference']['id'] == 'virtual-head'
    assert virtual['mode'] == 'jog'  # The head's only test, whatever was asked.
    assert Path(virtual['teleop_profile']).name == 'minerva-head.json' and 'calibration_profile' not in virtual
    tune.configure(arm='head', mode='live', target='jetson')
    with pytest.raises(ValueError, match='Set up the head first'):
        tune._bundle()
    with pytest.raises(ValueError, match='Only a tuning sweep leaves a drive out'):
        tune.configure(arm='head', mode='live', absent=['head_tilt'])


def test_a_head_jog_saves_nothing(tmp_path):
    tune = session(tmp_path)
    tune.results = {'head_pan': {'status': 'done', 'recommendation': None, 'jog': {'reached': {}}}}
    for motors in (['head_pan'], ['head_pan', 'left_arm_j1']):
        with pytest.raises(ValueError, match='saves nothing'):
            tune.save(motors)
    assert not tune.head_paths['servo'].exists()


def test_the_head_jog_raises_the_head_from_where_it_rests_and_lowers_it_back_there(bundle, tmp_path):
    # The head cannot stay straight ahead: limp, it droops (the user, 2026-10-30). The jog starts there, raises it
    # to its reference first, and ends back where it rested, so it is never dropped from straight ahead.
    clock = Clock()
    centre = {m['name']: m['raw_reference_rad'] for m in bundle['head_reference']['motors']}
    rest = {'head_pan': centre['head_pan'] + 0.02, 'head_tilt': centre['head_tilt'] - 0.6}
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock, start=rest))
        return receivers[-1]

    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    engine = core.engine
    speed = engine.settings['head_tilt']['characterize_speed']
    before, tasks = None, []
    for _ in range(40000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        targets = dict(engine.targets)
        if engine.task != 'starting':
            assert max(abs(targets[name] - before[name]) for name in targets) <= speed * core.period_s + 1e-9
            if not tasks or tasks[-1] != engine.task:
                tasks.append(engine.task)
        before = targets
        if core.stage == 'finished':
            break
    assert core.stage == 'finished' and engine.task == 'finished', core.message
    assert engine.rest == pytest.approx(rest)
    assert tasks[0] == 'to the reference pose' and 'down to where it rested' in tasks
    assert engine.targets == pytest.approx(rest)  # Back where it rested, then disabled there.
    assert receivers[0].last['feedback'][1]['position'] == pytest.approx(rest['head_tilt'], abs=0.02)
    assert not receivers[0].last['hardware_enabled'] and 'back where it rested' in core.message
    core.close()
