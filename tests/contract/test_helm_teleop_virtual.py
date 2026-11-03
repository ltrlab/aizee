"""Helm teleop end to end on the virtual receiver: Minerva's calibration and saved
limits, the native keep-out model and the actual Rust receiver (native harness),
driven through the GUI keys with scripted leaders on a test clock. Each leader
channel's saved range scales straight onto its robot joint's usable range, and
the leader zero maps onto the robot's reference pose."""
from collections import deque
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace

import pytest

from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import PAUSE_ATTEMPTS, TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path
from robot_platform.teleop.leader_ranges import KIND, SCHEMA, UNITS, pins
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
# Saved leader ranges: centers (the right trigger's range crosses the encoder seam) and spans.
CENTERS = {'left': [4.67, 3.07, 1.2, 2.2, 3.2, .4, 5.1], 'right': [1.69, 4.76, 2.1, .9, 5.9, 3.3, 6.2]}
SPANS = [2., 2., 2., 2., 2., 2., .8]
HANG = .3  # The leaders hang 0.3 rad above their range centers; that pose is the zero.

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Leaders:
    """Both leaders, offset per channel from their reference; stamped on the test clock."""
    class inputs:
        max_age_s, max_skew_s = .05, .02

    def __init__(self, clock):
        self.clock, self.sequence, self.failure, self.closed = clock, 0, None, False
        self.offsets = {name: [HANG] * 7 for name in CENTERS}

    def poll(self):
        if self.failure:
            raise self.failure
        self.sequence += 1
        now = self.clock()
        # The encoders report within one turn.
        return {name: LeaderSample(tuple((c + d) % (2 * math.pi) for c, d in zip(CENTERS[name], self.offsets[name])),
                                   (0,) * 7, self.sequence, now - .005, now - .002, (0, 0, 1, 1), 1, 2)
                for name in CENTERS}

    def close(self):
        self.closed = True


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('helm-teleop')
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    assert helm.saved_limits is not None, helm.saved_limits_error
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
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'))
    profile_path = tmp_path / 'teleop.json'
    profile_path.write_text(json.dumps(profile))
    saved = SimpleNamespace(saved=ranges, missing=lambda routes: [])
    bundle = TeleopSession(helm, profile_path, runs=tmp_path / 'runs', leaders=saved)._bundle()
    current = []

    def rig():
        # Loading the keep-out model dominates, so tests share a build unless one faulted it.
        if not current or current[0][0].fault:
            while current:
                current.pop()[0].close()
            clock = Clock()
            leaders = Leaders(clock)
            core = TeleopCore(bundle, open_leaders=lambda _profile: leaders, clock=clock)
            current.append((core, clock, leaders))
            core.build()
        return current[0]
    yield rig
    while current:
        current.pop()[0].close()
    helm.close()


@pytest.fixture
def rig(built):
    core, clock, leaders = built()
    leaders.failure = None
    leaders.offsets = {name: [HANG] * 7 for name in CENTERS}
    run(core, clock, 'H')
    assert state(core) == ('DISABLED', 'disarmed')
    return core, clock, leaders


def run(core, clock, key=None, *, until=None, limit=1, seconds=None):
    """`limit` cycles, or `seconds` of them at the loop's rate (the profile's)."""
    if seconds is not None:
        limit = max(1, round(seconds / core.period_s))
    for _ in range(limit):
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and core.session.lifecycle.state == until:
            return
    assert until is None, f'still {core.session.lifecycle.state}, expected {until}'


def ramp(core, clock, leaders, side, channel, value):
    """Move a leader channel at 1 rad/s; an instant jump is rejected as an encoder glitch."""
    step = core.period_s
    while abs(leaders.offsets[side][channel] - value) > 1e-12:
        current = leaders.offsets[side][channel]
        leaders.offsets[side][channel] = current + max(-step, min(step, value - current))
        run(core, clock)


def state(core):
    return core.session.lifecycle.state, core.view()['receiver']


def expected_motor(core, motor, offset, sign, span=2., zero=HANG):
    """Arm routes: the zero maps to the robot's reference; one straight scale reaching both robot ends."""
    row = next(r for r in core.derived['joint_map'].values() if r['actuator'] == motor)
    report = next(r for r in core.derived['report'] if r['motor'] == motor)
    (low, high), home = report['usable'], report['home']
    direction = 1 if sign * row['scale'] > 0 else -1
    near, far = (home - low, high - home) if direction > 0 else (high - home, home - low)
    scale = max(near / (zero + span / 2), far / (span / 2 - zero))
    return min(high, max(low, home + direction * scale * (offset - zero)))


def test_gui_keys_drive_the_virtual_receiver_through_the_whole_lifecycle(rig):
    core, clock, leaders = rig
    view = core.view()
    assert view['phase'] == 'running' and len(view['joints']) == 14
    assert view['collision']['mode'] == 'advisory'
    assert all(row['usable'][0] < row['usable'][1] for row in view['joints'])
    run(core, clock, 'I')
    assert state(core) == ('IDLE', 'idle')
    run(core, clock, 'E')
    assert state(core) == ('HOLD', 'holding')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    assert core.compiler.last_verdict['advisory'] and core.compiler.last_verdict['clear']
    with pytest.raises(ValueError, match='Hold'):
        core.route('flip', 'left_arm_j2')

    joints = {row['motor']: row for row in core.view()['joints']}
    homes = {row['motor']: row['home'] for row in core.derived['report']}
    for motor in ('left_arm_j1', 'right_arm_j3'):  # Leaders hanging: robot at its reference.
        assert joints[motor]['command'] == pytest.approx(homes[motor], abs=1e-6)
    grip = joints['right_gripper']  # Grippers map end to end, not anchored.
    fraction = (HANG + .4) / .8
    assert grip['command'] == pytest.approx(grip['usable'][0] + fraction * (grip['usable'][1] - grip['usable'][0]))
    ramp(core, clock, leaders, 'left', 1, -.5)  # Left J2's route sign is -1.
    run(core, clock, seconds=4)  # Speeding up within its braking (8 rad/s2), it arrives within 4 s.
    joints = {row['motor']: row for row in core.view()['joints']}
    target = expected_motor(core, 'left_arm_j2', -.5, -1)
    assert joints['left_arm_j2']['command'] == pytest.approx(target, abs=1e-6)
    assert joints['left_arm_j2']['measured'] == pytest.approx(target, abs=1e-6)
    for side in ('left', 'right'):  # Triggers to the low end of their range: grippers close.
        ramp(core, clock, leaders, side, 6, -.4)
    run(core, clock, seconds=6.4)
    joints = {row['motor']: row for row in core.view()['joints']}
    for motor in ('left_gripper', 'right_gripper'):
        assert joints[motor]['command'] == pytest.approx(joints[motor]['usable'][0], abs=1e-6)
    assert state(core)[0] == 'TELEOP'

    run(core, clock, 'T')
    assert state(core)[0] == 'HOLD'
    core.route('flip', 'left_arm_j2')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    run(core, clock, seconds=.4)
    flipped = {row['motor']: row for row in core.view()['joints']}['left_arm_j2']
    assert flipped['command'] == pytest.approx(expected_motor(core, 'left_arm_j2', -.5, 1), abs=1e-6)

    run(core, clock, 'X', until='DISABLED', seconds=32)
    assert state(core) == ('DISABLED', 'disarmed')
    measured = core.view()['joints']
    assert all(row['measured'] == pytest.approx(home, abs=.05)
               for row, home in zip(measured, core.derived['initial']))


def test_engage_needs_live_leaders_and_a_lost_leader_holds(rig):
    core, clock, leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    leaders.failure = LeaderSampleSkipped('Leader pair arrived late')
    run(core, clock, 'T')
    assert state(core)[0] == 'HOLD'
    assert core.message.startswith('[T] Engage needs live leaders: Leader pair arrived late')
    leaders.failure = None
    run(core, clock, 'T', until='TELEOP', seconds=16)
    leaders.failure = LeaderSampleSkipped('Leader pair arrived late')
    run(core, clock, seconds=.12)
    assert state(core)[0] == 'HOLD'
    assert 'Leader input lost' in core.message
    run(core, clock, 'H')
    assert state(core) == ('DISABLED', 'disarmed')


def test_a_stalled_cycle_faults_with_the_receivers_reason_then_lowers_the_held_arm(rig):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    clock.now += .1  # Past the 80 ms collision verdict: the receiver brakes and revokes the lease.
    core.tick(None, clock())
    assert re.match(r'Receiver stop(ping|ped): collision_expired · ', core.fault), core.fault
    assert core.session.lifecycle.state == 'FAULT'
    fault = core.fault
    # The receiver holds the arm: the session takes it back there and lowers it, rather than dropping it.
    for _ in range(2000):
        clock.now += core.period_s
        core.tick('T', clock())  # Keys do not bring following back.
        if core.session.lifecycle.state == 'DISABLED':
            break
    assert core.fault == fault
    assert core.session.lifecycle.state == 'DISABLED' and core.harness.last['status']['state'] == 'disarmed'
    assert 'lowered to the reference and disabled' in core.message
    assert 'Stop the session and start it again' in core.message


class Drops:
    """A camera watch (helm.camera_watch) reporting the given cameras dropping off USB, one a poll."""

    def __init__(self, *cameras):
        self.cameras = list(cameras)

    def poll(self):
        return self.cameras.pop(0) if self.cameras else None

    def close(self):
        pass


def watch(core, monkeypatch, *cameras):
    """Cameras dropping off USB on the shared rig's core, its camera state restored after the test."""
    monkeypatch.setattr(core, 'camera_watch', Drops(*cameras))
    monkeypatch.setattr(core, '_camera_dropped_at', None)
    monkeypatch.setattr(core, '_pauses', deque(maxlen=PAUSE_ATTEMPTS))


def test_a_camera_dropping_off_usb_pauses_the_following_arm_and_engage_follows_again(rig, monkeypatch):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    watch(core, monkeypatch, 'the camera at USB 1-2.2 (05a3:9230)')
    run(core, clock)
    assert core.session.lifecycle.state == 'HOLD'
    assert core.message.startswith('Paused: the camera at USB 1-2.2 (05a3:9230) dropped off USB'), core.message
    run(core, clock, 'T', until='TELEOP', seconds=16)


def test_a_late_cycle_just_after_a_camera_dropped_holds_the_arm_where_it_stopped(rig, monkeypatch):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    watch(core, monkeypatch, 'the camera at USB 1-2.2 (05a3:9230)')
    clock.now += .1  # The reconnect's hotplug burst: past the 80 ms collision verdict, the receiver brakes.
    core.tick(None, clock())
    assert re.match(r'Receiver stop(ping|ped): collision_expired · ', core.fault), core.fault
    assert 'PAUSED' in core.message and 'taking it back there' in core.message, core.message
    stopped = [joint['position'] for joint in core.harness.last['status']['joints']]
    for _ in range(400):
        clock.now += core.period_s
        core.tick(None, clock())
        if core.fault is None:
            break
    # Held where the receiver braked it, not lowered: the fault clears and Engage follows again.
    assert core.fault is None and core.session.lifecycle.state == 'HOLD', core.message
    assert 'PAUSED' in core.message and 'Engage (T) follows again' in core.message, core.message
    held = [joint['position'] for joint in core.harness.last['status']['joints']]
    assert max(abs(a - b) for a, b in zip(held, stopped)) < .05
    run(core, clock, 'T', until='TELEOP', seconds=16)


def over_speed(core, harness, monkeypatch, joint, readings=3):
    """`joint` read moving at 1.5 times its receiver cap toward its lower end, `readings` readings in a row: the
    receiver's sustained_speed check (left J4 lurching into its lower end, 2026-11-03), not its one-reading check
    at three times the allowed speed. Returns the joint's index and how far its stop may carry it."""
    joints = core.derived['config']['joints']
    index = [j['name'] for j in joints].index(joint)
    cap, braking = joints[index]['max_velocity'], joints[index]['stop_acceleration']
    step, left = harness.step, [readings]

    def stepped(now_ms, request=None, **kwargs):
        if left[0] > 0:
            left[0] -= 1
            measured = [{key: row[key] for key in ('position', 'velocity', 'effort')}
                        for row in harness.last['feedback']]
            measured[index]['velocity'] = -1.5 * cap
            kwargs['feedback'] = measured
        return step(now_ms, request, **kwargs)
    monkeypatch.setattr(harness, 'step', stepped)
    monkeypatch.setattr(core, '_pauses', deque(maxlen=PAUSE_ATTEMPTS))
    return index, cap * cap / (2 * braking)


def tick_until(core, clock, done, limit=400):
    for _ in range(limit):
        core.tick(None, clock())
        clock.now += core.period_s
        if done():
            return


def test_a_joint_over_its_speed_is_held_where_the_receiver_stopped_it_and_engage_follows_again(rig, monkeypatch):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    joint = next(name for name in core.session.lifecycle.names if name.endswith('_j4'))
    index, travel = over_speed(core, core.harness, monkeypatch, joint)
    before = [row['position'] for row in core.harness.last['feedback']]
    tick_until(core, clock, lambda: core.fault, limit=10)
    assert re.match(r'Receiver stop(ping|ped): limit_violation \(.* for three readings', core.fault), core.fault
    assert 'PAUSED' in core.message and 'taking it back there' in core.message, core.message
    tick_until(core, clock, lambda: core.fault is None)
    # Held where the receiver braked it, not lowered or disabled: the fault clears and Engage follows again.
    assert core.fault is None and core.session.lifecycle.state == 'HOLD', core.message
    assert 'PAUSED: the receiver stopped the arms' in core.message, core.message
    assert 'Engage (T) follows again' in core.message and core.harness.last['hardware_enabled']
    held = [row['position'] for row in core.harness.last['feedback']]
    assert all(abs(a - b) < (travel + .01 if i == index else .05)
               for i, (a, b) in enumerate(zip(held, before))), (held, before)
    run(core, clock, 'T', until='TELEOP', seconds=16)


def test_a_joint_past_its_limit_still_lowers_the_arm(rig, monkeypatch):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    joints = core.derived['config']['joints']
    index = next(i for i, j in enumerate(joints) if j['name'].endswith('_j4'))
    step, left = core.harness.step, [1]

    def stepped(now_ms, request=None, **kwargs):
        if left[0] > 0:
            left[0] -= 1
            measured = [{key: row[key] for key in ('position', 'velocity', 'effort')}
                        for row in core.harness.last['feedback']]
            measured[index]['position'] = joints[index]['upper'] + .01
            kwargs['feedback'] = measured
        return step(now_ms, request, **kwargs)
    monkeypatch.setattr(core.harness, 'step', stepped)
    monkeypatch.setattr(core, '_pauses', deque(maxlen=PAUSE_ATTEMPTS))
    tick_until(core, clock, lambda: core.fault, limit=10)
    assert re.match(r'Receiver stop(ping|ped): limit_violation \(', core.fault), core.fault
    assert 'lowering to the reference, then disabling' in core.message, core.message


def test_a_drive_that_left_run_mode_is_named_while_the_receiver_holds_the_arm(rig, monkeypatch):
    from robot_platform.control.guarded import GuardRejected
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    clock.now += .1
    core.tick(None, clock())
    assert core.session.lifecycle.state == 'FAULT'
    # A drive rebooted: the live adapter withholds its readings and names it; a re-arm is refused while it is quiet.
    step = core.harness.step

    def dropped(*args, **kwargs):
        result = step(*args, **kwargs)
        core.harness.last = dict(result, hardware_dropped='right_arm_j4: left Run mode while driven')
        return core.harness.last
    monkeypatch.setattr(core.harness, 'step', dropped)

    def refused():
        raise GuardRejected('feedback_stale', core.session.controller.poll())
    monkeypatch.setattr(core.session, 'recover', refused)
    for _ in range(3):
        clock.now += core.period_s
        core.tick(None, clock())
    assert core.landing and core.harness.last['status']['state'] == 'stopped'
    assert 'right_arm_j4: left Run mode while driven. The receiver holds the arm' in core.message, core.message
    assert 'press H' in core.message


def test_h_disables_a_faulted_arm_at_once_rather_than_lowering_it(rig):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    clock.now += .1
    core.tick(None, clock())
    assert core.fault
    clock.now += core.period_s
    core.tick('H', clock())
    assert not core.landing and 'Disabled at your request' in core.message
    assert not core.harness.last['hardware_enabled']


def test_closing_a_following_arm_lowers_it_before_the_receiver_closes(rig):
    core, clock, _leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    assert core.land() and core.landing
    for _ in range(2000):
        clock.now += core.period_s
        core.tick(None, clock())
        if not core.landing:
            break
    assert core.session.lifecycle.state == 'DISABLED' and core.harness.last['status']['state'] == 'disarmed'
    assert not core.land()  # Nothing left to lower.


def test_setting_the_zero_while_engaged_holds_then_follows_from_the_new_zero(rig):
    core, clock, leaders = rig
    run(core, clock, 'I')
    run(core, clock, 'E')
    run(core, clock, 'T', until='TELEOP', seconds=16)
    sign = core.routes()['left_arm_j2']['sign']  # Earlier tests may have flipped it.
    ramp(core, clock, leaders, 'left', 1, -.5)
    run(core, clock, seconds=4)
    moved = {row['motor']: row for row in core.view()['joints']}['left_arm_j2']
    assert moved['command'] == pytest.approx(expected_motor(core, 'left_arm_j2', -.5, sign), abs=1e-6)
    zero = core.set_zero()
    assert zero['left']['channel_2'] == pytest.approx(CENTERS['left'][1] - .5)
    run(core, clock)
    held = {row['motor']: row for row in core.view()['joints']}['left_arm_j2']
    assert state(core)[0] == 'HOLD' and held['command'] == pytest.approx(moved['command'], abs=1e-6)
    assert 'Leader zero set' in core.message
    run(core, clock, 'T', until='TELEOP', seconds=16)  # Aligns slowly to the new mapping.
    home = next(r['home'] for r in core.derived['report'] if r['motor'] == 'left_arm_j2')
    now = {row['motor']: row for row in core.view()['joints']}['left_arm_j2']
    assert now['command'] == pytest.approx(home, abs=1e-6)
    assert now['command'] == pytest.approx(expected_motor(core, 'left_arm_j2', -.5, sign, zero=-.5), abs=1e-6)
    core.set_zero()  # Back to the hanging zero for the tests that share this build.
    ramp(core, clock, leaders, 'left', 1, HANG)
    core.set_zero()
    run(core, clock, 'H')


def test_the_loop_takes_its_triggers_from_the_leader_profile_it_runs_with(rig):
    core, _clock, _leaders = rig
    # gello-dual-viewer.json, as every teleop profile names it: only these leaders' triggers are ever driven.
    assert core.grip_triggers == {'left': (7, 1), 'right': (7, 1)}
