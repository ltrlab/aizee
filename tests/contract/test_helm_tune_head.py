"""Helm's servo sweep on the head: the arms' sweep engine and Tune loop on the head's receiver, under the direct policy
(no collision envelopes: the arms keep out of everywhere the head can reach). The receiver the live sweep opens is
the native harness, so the roster, the raised ceilings, the enable, every direct update and the evidence run for
real without servos."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.helm.tune import TuneCore
from robot_platform.neck import head
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary

ROOT = Path(__file__).resolve().parents[2]
SHORT = {'amplitudes': [.1, .3], 'kp_ladder': [1., .5], 'damping': [1., .6], 'speeds': [.4, .8],
         'decelerations': [4.], 'hold_s': .2, 'rest_s': .1}

pytestmark = [pytest.mark.integration, pytest.mark.skipif(not default_binary().is_file(),
                                                          reason='Build control_harness first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Receiver:
    """The head receiver's interface (exchange, last, transport hook, close) over the actual Rust core."""

    def __init__(self, config, reference, clock):
        roster = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
        self.harness = NativeControlHarness(config, [roster[joint['name']] + .02 for joint in config['joints']])
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
    profile['bus'] = {'device': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46083062-if00',
                      'usb_serial': '5A46083062', 'baud': 1000000}
    path = tmp_path / 'head.json'
    path.write_text(json.dumps(profile))
    reference = head.virtual_reference(profile)
    return {'kind': 'tune', 'mode': 'tune', 'teleop_profile': str(path), 'arm': 'head',
            'receiver': {'mode': 'live', 'arm': 'head'}, 'joints': ['head_pan'], 'head_reference': reference,
            'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}


def test_a_head_sweep_runs_the_arms_engine_to_the_end_with_direct_updates(bundle, tmp_path):
    clock = Clock()
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock))
        return receivers[-1]
    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    receiver = receivers[0]
    joints = {joint['name']: joint for joint in receiver.config['joints']}
    assert set(joints) == {'head_pan', 'head_tilt'} and 'collision' not in receiver.config
    assert joints['head_pan']['max_kp'] == 64.  # The sweep's ceiling, in the servo's P register units.
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if core.stage == 'finished':
            break
    assert core.stage == 'finished' and core.engine.task == 'finished', core.message
    assert core.engine.checker is None and 'move' not in receiver.requests
    assert receiver.requests.count('direct_update') > 100
    assert receiver.last['status']['state'] == 'disarmed' and not receiver.last['hardware_enabled']
    result = core.engine.summary()['head_pan']
    assert result['status'] == 'done' and result['recommendation'] is not None
    assert result['verification']['passed']
    core.close()
    assert core.evidence.name.startswith('sweep-head-')
    assert json.loads((core.evidence / 'results.json').read_text())['task'] == 'finished'


def test_the_head_sweep_needs_the_head_profile_and_cannot_return(bundle):
    with pytest.raises(ValueError, match='no return'):
        TuneCore(dict(bundle, mode='return', start_pose={'head_pan': 3.}))
    arms = dict(bundle, teleop_profile=str(ROOT / 'config/devices/minerva-teleop.json'))
    with pytest.raises(ValueError, match='head profile'):
        TuneCore(arms)
