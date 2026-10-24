"""Gripper force feedback without hardware: the squeeze law, the leader request field, the PC's trigger frames,
the board write order, and the live switch through the session loop."""
import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace

import pytest

from robot_platform.teleop import leader_bridge
from robot_platform.teleop.grip_feedback import (FF_DISABLE, GripFeedback, crc8, ff_frame, trigger_config,
                                                 trigger_currents, validate_config)
from robot_platform.teleop.leader_bridge import (GRIP_REFRESH_S, GRIP_STALE_S, LatestLeaderInputs,
                                                 LeaderSnapshotClient, grip_request, sample_reply)
from robot_platform.teleop.openrb import OpenRBInput, crc8 as openrb_crc8

ROOT = Path(__file__).resolve().parents[2]
PROFILE = {'leaders': {'left': {'usb_serial': 'L', 'trigger': {'channel': 7, 'open_sign': 1}},
                       'right': {'usb_serial': 'R', 'trigger': {'channel': 7, 'open_sign': -1}}},
           'max_age_s': .05, 'max_skew_s': .02}


def unpack(frame):
    assert frame[0] == 0xCC and frame[1] == 7 and len(frame) == 17
    assert frame[-1] == crc8(frame[:-1])
    return list(struct.unpack('<7h', frame[2:16]))


# -- the frame the firmware accepts ---------------------------------------------------------------------------

def test_the_ff_frame_matches_the_firmware_layout_and_crc():
    frame = ff_frame([FF_DISABLE] * 6 + [-150])
    assert frame[:2] == bytes((0xCC, 7)) and len(frame) == 17
    assert unpack(frame) == [FF_DISABLE] * 6 + [-150]
    # firmware/openrb_leader accept_ff_current: CRC8 (0x31) over the command byte, the count and the currents.
    assert frame[-1] == openrb_crc8(frame[:16])
    for bad in ([0] * 6, [0] * 7 + [0], [201] + [0] * 6, [-201] + [0] * 6, [1.5] + [0] * 6, [True] + [0] * 6):
        with pytest.raises(ValueError):
            ff_frame(bad)


def test_a_trigger_is_the_only_servo_energised_and_only_toward_open():
    assert trigger_currents(80, (7, 1)) == [FF_DISABLE] * 6 + [80]
    assert trigger_currents(80.4, (3, -1)) == [FF_DISABLE] * 2 + [-80] + [FF_DISABLE] * 4
    assert trigger_currents(900, (7, 1))[6] == 200  # The firmware's cap.
    for bad in (-1, math.nan, math.inf, '5', None):
        with pytest.raises(ValueError):
            trigger_currents(bad, (7, 1))


def test_triggers_come_only_from_the_leader_profile():
    assert trigger_config(PROFILE) == {'left': (7, 1), 'right': (7, -1)}
    assert trigger_config({'leaders': {'left': {'usb_serial': 'L'}}}) == {}
    for trigger in ({'channel': 0, 'open_sign': 1}, {'channel': 8, 'open_sign': 1}, {'channel': 7, 'open_sign': 0},
                    {'channel': 7}, {'channel': 7.0, 'open_sign': 1}, 'seven'):
        with pytest.raises(ValueError, match='trigger'):
            trigger_config({'leaders': {'left': {'usb_serial': 'L', 'trigger': trigger}}})
    # The file teleop reads: every teleop profile's leader_profile. On 2026-10-21 the triggers sat in
    # minerva-leaders-01.json, which teleop never reads; the PC dropped every strength while Helm showed feedback on.
    import json
    profiles = sorted((ROOT / 'config/devices').glob('minerva-teleop*.json'))
    assert profiles
    for path in profiles:
        teleop = json.loads(path.read_text(encoding='utf-8'))
        leaders = json.loads((path.parent / teleop['leader_profile']).read_text(encoding='utf-8'))
        assert trigger_config(leaders) == {'left': (7, 1), 'right': (7, 1)}, path.name
    bench = json.loads((ROOT / 'config/devices/minerva-leaders-01.json').read_text(encoding='utf-8'))
    assert trigger_config(bench) == {}  # One place only: the bench scripts' identity file names no triggers.


# -- the squeeze law on the robot ------------------------------------------------------------------------------

def rows(left=(0., 0.), right=(0., 0.)):
    return [dict(name='left_gripper', effort=left[0], velocity=left[1]),
            dict(name='right_gripper', effort=right[0], velocity=right[1]),
            dict(name='left_arm_j1', effort=-9., velocity=0.)]


def settle(feedback, sample, *, engaged=True, cycles=60):
    for _ in range(cycles):
        out = feedback.update(sample, engaged=engaged)
    return out


def test_feedback_is_off_until_switched_on_and_needs_teleop():
    feedback = GripFeedback(None)
    assert feedback.update(rows(right=(-2., 0.)), engaged=True) is None
    feedback.set(enabled=True)
    assert settle(feedback, rows(right=(-2., 0.)), engaged=False) == {'left': 0., 'right': 0.}
    assert settle(feedback, rows(right=(-2., 0.)))['right'] == pytest.approx(150., abs=.5)


def test_only_a_held_squeeze_above_the_deadband_pushes_back():
    feedback = GripFeedback({'enabled': True})
    # Measured on Minerva: a held cube near -2 Nm; free air within 0.3 Nm; closing effort is negative.
    assert settle(feedback, rows(right=(-.5, 0.)))['right'] == pytest.approx((.5 - .3) * 400, abs=.5)  # A light grip.
    assert settle(feedback, rows(right=(-.3, 0.))) == {'left': 0., 'right': 0.}  # Free-air level.
    assert settle(feedback, rows(right=(-1.1, 0.)))['right'] == 150.  # The lightest held cube measured: the maximum.
    assert settle(feedback, rows(right=(-2., 1.2))) == {'left': 0., 'right': 0.}  # Closing fast: not a hold.
    assert settle(feedback, rows(right=(+2., 0.))) == {'left': 0., 'right': 0.}  # Pushed open: never feedback.
    assert settle(feedback, rows(left=(-9., 0.)))['left'] == 150.  # Capped.
    assert settle(feedback, [dict(name='right_gripper', effort=math.nan, velocity=0.)]) == {'left': 0., 'right': 0.}
    assert settle(feedback, []) == {'left': 0., 'right': 0.}


def test_strength_scales_and_smoothing_ramps_instead_of_stepping():
    feedback = GripFeedback({'enabled': True})
    feedback.set(strength=.5)
    first = feedback.update(rows(right=(-.5, 0.)), engaged=True)['right']
    assert 0 < first < 40  # (0.5 - 0.3) Nm x 400 mA/Nm x 0.5 = 40 mA, approached smoothly.
    assert settle(feedback, rows(right=(-.5, 0.)))['right'] == pytest.approx(40., abs=.5)
    assert 0 < feedback.update(rows(), engaged=True)['right'] < 40  # Released: it decays, also smoothly.
    for bad in (-.1, 1.1, math.nan, '1'):
        with pytest.raises(ValueError):
            feedback.set(strength=bad)
    with pytest.raises(ValueError):
        feedback.set(enabled='yes')


@pytest.mark.parametrize('block', [{'gain': 1}, {'cap_ma': 250}, {'deadband_nm': -1}, {'closing_effort_sign': 0},
                                   {'enabled': 1}, {'smoothing': 0}, 'on'])
def test_profile_settings_stay_inside_their_bounds(block):
    with pytest.raises(ValueError):
        validate_config(block)
    assert validate_config({'cap_ma': 200, 'gain_ma_per_nm': 50})['gain_ma_per_nm'] == 50.


# -- the request and the PC's sampler ----------------------------------------------------------------------------

class FakeInputs:
    pins = {'left': 'L', 'right': 'R'}
    max_age_s, max_skew_s = .05, .02
    triggers = {'left': (7, 1), 'right': (7, -1)}

    def __init__(self, clock):
        self.clock, self.calls = clock, []

    def connect(self):
        pass

    def poll(self, feedback=None):
        self.calls.append(feedback)
        now = self.clock()
        return {name: SimpleNamespace(sampled_at=now, raw_positions=(0,) * 7, sequence=len(self.calls),
                                      joystick=(0, 0, 0, 0), board_sample_ms=1, board_reply_ms=2)
                for name in self.pins}

    def close(self):
        pass


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def test_the_sampler_sends_triggers_when_due_and_turns_them_off_once():
    clock = Clock()
    sampler = LatestLeaderInputs(FakeInputs(clock), rate_hz=200, clock=clock)
    sampler.set_grip({'left': 60., 'right': 0., 'chest': 9.})  # Unknown leader ignored.
    frames = sampler._feedback_frames(clock())
    assert unpack(frames['left']) == [FF_DISABLE] * 6 + [60]
    assert unpack(frames['right']) == [FF_DISABLE] * 6 + [0]  # On at 0 mA: the mode switch happens once.
    assert sampler._feedback_frames(clock()) == {}  # Nothing new, not due.
    clock.now += GRIP_REFRESH_S + 1e-6
    assert set(sampler._feedback_frames(clock())) == {'left', 'right'}  # Refreshed for the firmware watchdog.
    sampler.set_grip({'left': 61., 'right': 0.})
    assert sampler._feedback_frames(clock()) == {}  # Under 2 mA of change.
    sampler.set_grip({'left': 90., 'right': 0.})
    assert set(sampler._feedback_frames(clock())) == {'left'}
    clock.now += GRIP_STALE_S + .001  # The loop stopped asking: every trigger off, once.
    off = sampler._feedback_frames(clock())
    assert {name: unpack(frame) for name, frame in off.items()} == {name: [FF_DISABLE] * 7 for name in ('left', 'right')}
    assert sampler._feedback_frames(clock()) == {}
    sampler.set_grip({'left': 30., 'right': 0.})
    assert set(sampler._feedback_frames(clock())) == {'left', 'right'}
    sampler.set_grip(None)
    assert {name: unpack(frame)[6] for name, frame in sampler._feedback_frames(clock()).items()} == {
        'left': FF_DISABLE, 'right': FF_DISABLE}
    with pytest.raises(ValueError):
        sampler.set_grip({'left': -1.})


def test_the_sampler_thread_hands_frames_to_the_board_polls():
    inputs = FakeInputs(time.perf_counter)
    sampler = LatestLeaderInputs(inputs, rate_hz=200)
    sampler.connect()
    try:
        sampler.grip = {'left': 40., 'right': 10.}
        deadline = time.monotonic() + 2.
        while not any(call for call in inputs.calls[1:]) and time.monotonic() < deadline:
            time.sleep(.005)
        sent = next(call for call in inputs.calls[1:] if call)
        assert unpack(sent['left'])[6] == 40 and unpack(sent['right'])[6] == -10
        assert sampler.grip == {'left': 40., 'right': 10.}
    finally:
        sampler.close()


def test_a_request_carries_or_withholds_strengths_and_a_bad_one_turns_feedback_off():
    seen = []
    inputs = SimpleNamespace(set_grip=seen.append, pins={'left': 'L'},
                             poll=lambda: {'left': SimpleNamespace(raw_positions=(0,) * 7, sequence=1, sampled_at=0.,
                                                                   joystick=(0, 0, 0, 0), board_sample_ms=1,
                                                                   board_reply_ms=2)})
    nonce = 'a' * 32
    reply = sample_reply({'type': 'sample', 'nonce': nonce, 'grip': {'left': 25}}, inputs, 'epoch', clock=lambda: 0.)
    assert reply['nonce'] == nonce and seen[-1] == {'left': 25.}
    sample_reply({'type': 'sample', 'nonce': nonce}, inputs, 'epoch', clock=lambda: 0.)
    assert seen[-1] is None
    for bad in ({'left': -3}, {'left': math.nan}, {'left': '9'}, {}, [1], {7: 1.}):
        sample_reply({'type': 'sample', 'nonce': nonce, 'grip': bad}, inputs, 'epoch', clock=lambda: 0.)
        assert seen[-1] is None
    with pytest.raises(ValueError, match='fresh input sample'):
        sample_reply({'type': 'sample', 'nonce': nonce, 'torque': 1}, inputs, 'epoch', clock=lambda: 0.)
    assert grip_request({'left': 1000}) == {'left': 1000.} and grip_request({'left': 1001}) is None


def test_the_robot_side_client_sends_the_loop_strengths_with_each_request():
    requests = []

    def exchange(request):
        requests.append(request)
        raise leader_bridge.LeaderSampleSkipped('not answering in this test')
    client = LeaderSnapshotClient(exchange, PROFILE)
    with pytest.raises(leader_bridge.LeaderSampleSkipped):
        client.poll()
    client.grip = {'left': 12.5, 'right': 0.}
    with pytest.raises(leader_bridge.LeaderSampleSkipped):
        client.poll()
    assert 'grip' not in requests[0] and requests[1]['grip'] == {'left': 12.5, 'right': 0.}


# -- the board: write order and close ---------------------------------------------------------------------------

class FakeSerial:
    def __init__(self, reply):
        self.reply, self.writes, self.closed = bytearray(reply), [], False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(bytes(data))

    def read(self, size):
        data, self.reply = bytes(self.reply[:size]), self.reply[size:]
        return data

    def flush(self):
        pass

    def close(self):
        self.closed = True


def fresh_frame():
    data = b'\xa6\x07' + struct.pack('<7ihhBBIIIB', *range(2000, 2007), 0, 0, 1, 1, 1, 100, 101, 1)
    return data + bytes([openrb_crc8(data)])


def test_a_board_gets_its_trigger_frame_before_the_read_and_an_off_frame_when_it_closes():
    board = OpenRBInput('COM9', usb_serial='L')
    board.serial = FakeSerial(fresh_frame())
    frame = ff_frame(trigger_currents(70, (7, 1)))
    sample = board.poll(frame)
    assert board.serial.writes == [frame, b'\xa6'] and sample.sequence == 1
    serial = board.serial
    board.close()
    assert unpack(serial.writes[-1]) == [FF_DISABLE] * 7 and serial.closed
    quiet = OpenRBInput('COM9', usb_serial='L')
    quiet.serial = FakeSerial(fresh_frame())
    quiet.poll()
    serial = quiet.serial
    quiet.close()
    assert serial.writes == [b'\xa6']  # A board that never got feedback gets no frame at all.


# -- the live switch through the session loop -------------------------------------------------------------------

class FakeCore:
    period_s = .01
    clock = staticmethod(time.perf_counter)
    landing = False

    def __init__(self):
        self.phase, self.fault = 'starting', None
        self.feedback = GripFeedback(None)

    def build(self):
        self.phase = 'running'

    def tick(self, key, started):
        pass

    def set_grip_feedback(self, payload):
        from robot_platform.helm.teleop import TeleopCore
        return TeleopCore.set_grip_feedback(SimpleNamespace(grip_feedback=self.feedback), payload)  # The real rules.

    def view(self):
        return {'phase': self.phase, 'fault': self.fault, 'grip_feedback': self.feedback.snapshot()}

    def guide(self):
        return None

    def close(self):
        self.phase = 'stopped'

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason

    def land(self):
        return self.landing


def test_helm_switches_feedback_live_in_the_running_loop(tmp_path):
    from robot_platform.helm.teleop import TeleopSession
    helm = SimpleNamespace(profile_path=ROOT / 'config/devices/minerva-joint-calibration.json', bindings={},
                           mapping={}, saved_limits={'joints': {}}, asset_id='minerva-production-01')
    fake = FakeCore()
    leaders = SimpleNamespace(saved=None, missing=lambda routes: [], store_reference=lambda zero: None)
    session = TeleopSession(helm, ROOT / 'config/devices/minerva-teleop.json', runs=tmp_path, leaders=leaders,
                            core_factory=lambda bundle: fake)
    with pytest.raises(ValueError, match='Start a teleop session'):
        session.grip_feedback({'enabled': True})
    session.start()
    try:
        deadline = time.monotonic() + 5.
        while session.snapshot()['phase'] != 'running':
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert session.grip_feedback({'enabled': True, 'strength': .6}) == dict(fake.feedback.snapshot())
        assert fake.feedback.enabled and fake.feedback.strength == .6
        with pytest.raises(ValueError, match='enabled, strength and cap_ma'):
            session.grip_feedback({'gain': 3})
        session.grip_feedback({'cap_ma': 120})
        assert fake.feedback.config['cap_ma'] == 120.  # Live in the running loop.
    finally:
        session.stop()
    # Saved for every later session; the switch itself is not.
    assert session.grip_settings == {'strength': .6, 'cap_ma': 120.}
    again = TeleopSession(helm, ROOT / 'config/devices/minerva-teleop.json', runs=tmp_path, leaders=leaders,
                          core_factory=lambda bundle: fake)
    assert again.grip_settings == {'strength': .6, 'cap_ma': 120.}
    assert again.grip_feedback({'cap_ma': 90, 'strength': 1}) == {'strength': 1., 'cap_ma': 90.}  # No session.
    with pytest.raises(ValueError, match='0-200 mA'):
        again.grip_feedback({'cap_ma': 250})
    with pytest.raises(ValueError, match='Start a teleop session'):
        again.grip_feedback({'enabled': True})


def test_the_loop_reads_gripper_effort_from_the_receiver_feedback_and_only_feeds_back_in_teleop():
    from robot_platform.helm import teleop
    core = object.__new__(teleop.TeleopCore)
    core.grip_feedback = GripFeedback({'enabled': True, 'smoothing': 1.})
    core.leaders = SimpleNamespace(grip='unset')
    core.fault, core.derived = None, None
    core.grip_triggers = {'left': (7, 1), 'right': (7, 1)}
    names = ['left_arm_j1', 'left_gripper', 'right_gripper']
    feedback = [dict(position=0., velocity=0., effort=-5.), dict(position=1.4, velocity=0., effort=-.4),
                dict(position=1.3, velocity=.01, effort=-2.)]
    core.harness = SimpleNamespace(last={'status': {'joint_names': names}, 'feedback': feedback})
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip == {'left': 40., 'right': 150.}  # (0.4 - 0.3) x 400; (2.0 - 0.3) x 400 over the 150 cap.
    core._send_grip_feedback('HOLD')
    assert core.leaders.grip == {'left': 0., 'right': 0.}
    feedback[2] = dict(feedback[2], fault=True)
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip == {'left': 40., 'right': 0.}  # A faulted drive's reading is never felt.
    core.grip_feedback.set(enabled=False)
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip is None  # Off: the PC turns the triggers off.
    assert core._grip_status()['available'] is True
    core.grip_feedback.set(enabled=True)
    core.grip_triggers = {'right': (7, 1)}
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip == {'right': 0.}  # Only a leader with a trigger is sent a strength.
    core.grip_triggers = {}
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip is None and core._grip_status()['available'] is False  # No triggers: Helm says so.
    assert core.grip_feedback.snapshot()['levels']['right'] == 0.  # The bars still show the squeeze.
    core.leaders = SimpleNamespace()  # Scripted leaders: nothing to send, nothing to fail.
    core._send_grip_feedback('TELEOP')
    assert core._grip_status()['available'] is False


def test_strengths_cross_the_signed_link_and_reach_the_board_polls():
    from robot_platform.teleop.leader_link import LeaderLinkInput, LeaderLinkServer
    key = bytes(range(32))
    inputs = FakeInputs(time.perf_counter)
    sampler = LatestLeaderInputs(inputs, rate_hz=200)
    sampler.connect()
    robot = LeaderLinkInput(key, PROFILE, bind=('127.0.0.1', 0), timeout_s=.2)
    server = LeaderLinkServer(sampler, key, robot.address, local=('127.0.0.1', 0)).start()
    try:
        robot.wait_for_pc(5.)
        robot.grip = {'left': 0., 'right': 44.}
        robot.poll()
        assert sampler.grip == {'left': 0., 'right': 44.}
        deadline = time.monotonic() + 2.
        while not any(call and 'right' in call for call in inputs.calls) and time.monotonic() < deadline:
            time.sleep(.005)
        sent = next(call for call in inputs.calls if call and 'right' in call)
        assert unpack(sent['right'])[6] == -44  # The right trigger opens with negative current in PROFILE.
        robot.grip = None
        robot.poll()
        assert sampler.grip is None
    finally:
        server.close()
        robot.close()
        sampler.close()


def test_the_force_bars_follow_the_squeeze_against_the_maximum_even_with_feedback_off():
    feedback = GripFeedback({'smoothing': 1.})
    assert feedback.update(rows(right=(-.5, 0.)), engaged=True) is None  # Off: no strengths sent...
    snap = feedback.snapshot()
    assert snap['levels']['right'] == pytest.approx(80 / 150, abs=1e-3)  # ...but the bar shows (0.5 - 0.3) x 400 of 150 mA.
    assert snap['squeeze_nm']['right'] == .5 and snap['currents_ma'] == {'left': 0., 'right': 0.}
    feedback.set(strength=.25)
    feedback.update(rows(right=(-.5, 0.)), engaged=False)
    assert feedback.snapshot()['levels']['right'] == pytest.approx(80 / 150, abs=1e-3)  # Strength scales the push, not the bar.
    feedback.set(enabled=True, cap_ma=50)
    out = feedback.update(rows(right=(-.5, 0.)), engaged=True)
    assert out['right'] == 20. and feedback.snapshot()['levels']['right'] == 1.  # Capped at 50: 80 x .25 = 20 mA.
    feedback.set(cap_ma=0)
    assert feedback.update(rows(right=(-1.5, 0.)), engaged=True)['right'] == 0.
    assert feedback.snapshot()['levels']['right'] == 1.  # Squeezing past a zero maximum: the bar is full.
    assert feedback.snapshot()['cap_ma'] == 0.


def test_saved_settings_stay_within_bounds():
    from robot_platform.teleop.grip_feedback import validate_settings
    assert validate_settings({'strength': 1, 'cap_ma': 200}) == {'strength': 1., 'cap_ma': 200.}
    assert validate_settings({}) == {}
    for bad in ({'cap_ma': 201}, {'cap_ma': -1}, {'strength': 2}, {'strength': math.nan}, {'gain': 1}, [1]):
        with pytest.raises(ValueError):
            validate_settings(bad)


def test_a_core_built_without_leaders_or_feedback_skips_force_feedback():
    from robot_platform.helm import teleop
    core = object.__new__(teleop.TeleopCore)
    core._send_grip_feedback('TELEOP')  # Nothing to send and nothing to fail.
    assert core._grip_status() is None
