"""Helm's live teleop mode for one arm, with the actual Rust core standing in for the physical service.

Minerva's calibration, saved limits and keep-out model; the receiver the live
mode opens is the native harness here, so the one-arm roster, the parked other
arm, the routes, the single-use enable permit and closing all run for real
without motors or CAN.
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import library_path
from robot_platform.teleop.leader_ranges import KIND, SCHEMA, UNITS, pins
from robot_platform.teleop.openrb import LeaderSample

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
CENTERS = {'left': [4.67, 3.07, 1.2, 2.2, 3.2, .4, 5.1], 'right': [1.69, 4.76, 2.1, .9, 5.9, 3.3, 6.2]}
SPANS = [2., 2., 2., 2., 2., 2., .8]
HANG = .3
LIVE = {'reference_tolerance_rad': .1, 'period_us': 2500, 'maximum_lateness_us': 10000}

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Leaders:
    class inputs:
        max_age_s, max_skew_s = .05, .02

    def __init__(self, clock):
        self.clock, self.sequence, self.closed = clock, 0, False
        self.offsets = {name: [HANG] * 7 for name in CENTERS}

    def poll(self):
        self.sequence += 1
        now = self.clock()
        return {name: LeaderSample(tuple((c + d) % (2 * math.pi) for c, d in zip(CENTERS[name], self.offsets[name])),
                                   (0,) * 7, self.sequence, now - .005, now - .002, (0, 0, 1, 1), 1, 2)
                for name in CENTERS}

    def close(self):
        self.closed = True


class Receiver:
    """The live receiver's interface (exchange, last, close) over the actual Rust core."""

    def __init__(self, config, reference, clock):
        roster = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
        initial = [min(joint['upper'], max(joint['lower'], roster[joint['name']])) for joint in config['joints']]
        self.harness = NativeControlHarness(config, initial)
        self.harness.step(0)
        self.clock, self.epoch, self.closed = clock, clock(), False

    @property
    def last(self):
        return self.harness.last

    def exchange(self, request):
        now_ms = max(self.harness.now_ms, int((self.clock() - self.epoch) * 1000))
        return self.harness.step(now_ms, request)['reply']

    def close(self):
        self.closed = True
        self.harness.close()


@pytest.fixture
def bundle(tmp_path):
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    leader_profile = json.loads((DEVICES / 'gello-dual-viewer.json').read_text(encoding='utf-8'))
    ranges = {'schema': SCHEMA, 'kind': KIND, 'profile': leader_profile['id'], 'pins': pins(leader_profile),
              'units': UNITS, 'runtime_approved': False, 'leaders': {
                  name: {joint['name']: {'center_rad': center, 'span_rad': span}
                         for joint, center, span in zip(item['joints'], CENTERS[name], SPANS)}
                  for name, item in leader_profile['leaders'].items()},
              'reference': {'captured_at': 'fixture', 'leaders': {
                  name: {joint['name']: (center + HANG) % (2 * math.pi)
                         for joint, center in zip(item['joints'], CENTERS[name])}
                  for name, item in leader_profile['leaders'].items()}}}
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'), live=LIVE)
    profile_path = tmp_path / 'teleop.json'
    profile_path.write_text(json.dumps(profile))
    result = TeleopSession(helm, profile_path, runs=tmp_path / 'runs',
                           leaders=SimpleNamespace(saved=ranges, missing=lambda routes: []))._bundle()
    yield result, profile_path
    helm.close()


def run(core, clock, key=None, *, until=None, limit=1):
    for _ in range(limit):
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and core.session.lifecycle.state == until:
            return
    assert until is None, (f'still {core.session.lifecycle.state}, expected {until}: '
                           f'{core.message} ({core.input_error})')


def test_a_live_left_arm_session_drives_only_that_arm_and_spends_its_enable_permit(bundle):
    data, _profile_path = bundle
    data = dict(data, receiver={'mode': 'live', 'arm': 'left', 'motor_models': {}})
    clock, leaders, opened = Clock(), None, []

    def open_receiver(config, device, reference):
        opened.append((config, device, reference))
        return Receiver(config, reference, clock)

    leaders = Leaders(clock)
    core = TeleopCore(data, open_leaders=lambda _profile: leaders, open_receiver=open_receiver, clock=clock)
    try:
        core.build()
        config, device, reference = opened[0]
        names = [joint['name'] for joint in config['joints']]
        assert names == [f'left_arm_j{i}' for i in range(1, 7)] + ['left_gripper']
        assert {m['bus'] for m in reference['motors']} == {'canL'} and device['asset_id'] == 'minerva-production-01'
        assert core.compiler.fixed and all(name.startswith('right_') for name in core.compiler.fixed)
        view = core.view()
        assert len(view['joints']) == 7 and view['live']['arm'] == 'left' and 'LIVE LEFT ARM' in view['message']
        grip = next(row for row in view['joints'] if row['motor'] == 'left_gripper')
        assert grip['usable'][0] < 0  # Jaws close past the model's closed limit.
        run(core, clock, 'I')
        assert core.session.lifecycle.state == 'IDLE'
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=400)
        assert core.compiler.last_verdict['advisory'] and core.guide()['badge'].startswith('LIVE LEFT ARM · TELEOP')
        run(core, clock, 'H')
        assert core.session.lifecycle.state == 'DISABLED' and 'the arm is limp' in core.message
        run(core, clock, 'I')  # The enable permit was spent: a new session checks the pose again.
        assert core.session.lifecycle.state == 'DISABLED' and 'start a new one' in core.message
    finally:
        core.close()
    assert leaders.closed and 'live receiver disabled the arm' in core.message


def test_live_needs_the_profiles_live_block_and_advisory_checks(bundle, tmp_path):
    data, profile_path = bundle
    profile = json.loads(profile_path.read_text())
    del profile['live']
    plain = tmp_path / 'plain.json'
    plain.write_text(json.dumps(profile))
    core = TeleopCore(dict(data, teleop_profile=str(plain), receiver={'mode': 'live', 'arm': 'right'}),
                      open_receiver=lambda *args: pytest.fail('must not open a receiver'))
    with pytest.raises(ValueError, match="needs the profile's live block"):
        core.build()
    core.close()
    # One arm keeps advisory checks against the other, parked arm.
    profile = json.loads(profile_path.read_text())
    profile['receiver']['collision_mode'] = 'enforced'
    enforced = tmp_path / 'enforced.json'
    enforced.write_text(json.dumps(profile))
    core = TeleopCore(dict(data, teleop_profile=str(enforced), receiver={'mode': 'live', 'arm': 'right'}),
                      open_receiver=lambda *args: pytest.fail('must not open a receiver'))
    with pytest.raises(ValueError, match='one arm needs advisory collision checks'):
        core.build()
    core.close()
    # Both arms always enforce the two-arm (motion) bound.
    assert TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'both'})).collision_mode == 'motion'
    with pytest.raises(ValueError, match='virtual, or live on the left, right or both arms'):
        TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'middle'}))
