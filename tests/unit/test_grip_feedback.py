"""Gripper force feedback without hardware: the squeeze law, the leader request field, the PC's trigger frames,
the board write order, and the live switch through the session loop."""

import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace

import pytest

from robot_platform.teleop import leader_bridge
from robot_platform.teleop.grip_feedback import (
    FF_DISABLE,
    GripFeedback,
    crc8,
    ff_frame,
    trigger_config,
    trigger_currents,
    validate_config,
)
from robot_platform.teleop.leader_bridge import (
    GRIP_REFRESH_S,
    GRIP_STALE_S,
    WALL_ONSET_S,
    LatestLeaderInputs,
    LeaderSnapshotClient,
    grip_request,
    sample_reply,
)
from robot_platform.teleop.openrb import OpenRBInput, crc8 as openrb_crc8

ROOT = Path(__file__).resolve().parents[2]
PROFILE = {
    'leaders': {
        'left': {'usb_serial': 'L', 'trigger': {'channel': 7, 'open_sign': 1}},
        'right': {'usb_serial': 'R', 'trigger': {'channel': 7, 'open_sign': -1}},
    },
    'max_age_s': 0.05,
    'max_skew_s': 0.02,
}


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
    for trigger in (
        {'channel': 0, 'open_sign': 1},
        {'channel': 8, 'open_sign': 1},
        {'channel': 7, 'open_sign': 0},
        {'channel': 7},
        {'channel': 7.0, 'open_sign': 1},
        'seven',
    ):
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


def rows(left=(0.0, 0.0), right=(0.0, 0.0)):
    return [
        dict(name='left_gripper', effort=left[0], velocity=left[1]),
        dict(name='right_gripper', effort=right[0], velocity=right[1]),
        dict(name='left_arm_j1', effort=-9.0, velocity=0.0),
    ]


# A hold: the trigger 1.3 degrees past the wall where the jaws stopped (reading direction +1 opens it), against
# walls of 2 degrees for the whole 150 mA. The default is stiffer (test_the_default_wall_suits_the_holds_measured).
WALL = {'right': (3.0, 3.0 - math.radians(1.3), 1)}
K = 150 / math.radians(2)


def settle(feedback, sample, *, engaged=True, cycles=60, walls=WALL):
    for _ in range(cycles):
        out = feedback.update(sample, engaged=engaged, walls=walls)
    return out


def test_feedback_is_off_until_switched_on_and_needs_teleop():
    feedback = GripFeedback({'wall_deg': 2.0, 'cap_ma': 150.0})
    assert feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls=WALL) is None
    feedback.set(enabled=True)
    assert settle(feedback, rows(right=(-2.0, 0.0)), engaged=False) == {'left': 0.0, 'right': 0.0}
    assert settle(feedback, rows(right=(-2.0, 0.0)))['right'] == pytest.approx({'wall': 3.0, 'k': K, 'cap': 150.0})


def test_a_still_squeeze_grips_after_the_hold_time_and_puts_up_the_wall():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    # Measured on Minerva: a held cube squeezes about 2 Nm with the gripper still; closing effort is negative.
    out = [feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls=WALL)['right'] for _ in range(12)]
    assert out[:6] == [0.0] * 6  # 70 ms still first (10 ms cycles).
    assert out[6] == pytest.approx({'wall': 3.0, 'k': K, 'cap': 150.0}) and feedback.gripping['right']
    # What the loop estimates the trigger feels: 1.3 degrees past the wall, of 2 for the whole 150 mA.
    assert feedback.pushes['right'] == pytest.approx(150 * 1.3 / 2) and feedback.levels['right'] == pytest.approx(0.65)


def test_the_start_of_a_free_close_and_the_rest_level_never_put_up_a_wall():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    # Measured: a free close starts with the gripper stuck for 30-100 ms, then it moves at 0.7-2 rad/s.
    for _ in range(9):
        assert feedback.update(rows(right=(-0.8, 0.0)), engaged=True, walls=WALL)['right'] == 0.0
    for _ in range(30):
        assert feedback.update(rows(right=(-1.0, 1.5)), engaged=True, walls=WALL)['right'] == 0.0
    # The gripper's own friction at rest: 0.30-0.35 Nm for seconds at a time.
    assert all(feedback.update(rows(right=(-0.35, 0.05)), engaged=True, walls=WALL)['right'] == 0.0 for _ in range(300))
    assert not feedback.gripping['right']


def test_the_push_grows_with_the_trigger_past_the_wall_never_before_it():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    settle(feedback, rows(right=(-2.0, 0.0)))
    for past_deg, push in ((0.0, 0.0), (0.5, 37.5), (2.0, 150.0), (5.0, 150.0), (-1.0, 0.0)):
        feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls={'right': (3.1, 3.1 - math.radians(past_deg), 1)})
        assert feedback.pushes['right'] == pytest.approx(push)
    # A trigger whose reading opens the other way is past its wall on the other side.
    feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls={'right': (3.1, 3.1 + math.radians(1.0), -1)})
    assert feedback.pushes['right'] == pytest.approx(75.0)
    # Readings a whole turn apart are the same angle.
    feedback.update(
        rows(right=(-2.0, 0.0)), engaged=True, walls={'right': (3.1 + 2 * math.pi, 3.1 - math.radians(1.0), 1)}
    )
    assert feedback.pushes['right'] == pytest.approx(75.0)


def test_dips_and_blips_inside_a_grip_keep_the_wall():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    settle(feedback, rows(right=(-2.0, 0.0)))
    # A light hold lets the squeeze dip to nothing for 0.1 s; the jaws twitch, or race for 20 ms.
    samples = (
        [rows(right=(-2.0, 0.6))] * 2
        + [rows(right=(-0.05, 0.05))] * 10
        + [rows(right=(-2.0, 1.5))] * 2
        + [rows(right=(-2.0, 0.0))] * 10
    )
    assert all(isinstance(feedback.update(sample, engaged=True, walls=WALL)['right'], dict) for sample in samples)


def test_a_grip_ends_once_the_squeeze_stays_near_nothing_or_the_gripper_moves_fast():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    settle(feedback, rows(right=(-2.0, 0.0)))
    let_go = [feedback.update(rows(right=(0.2, -0.2)), engaged=True, walls=WALL)['right'] for _ in range(40)]
    assert isinstance(let_go[20], dict) and let_go[-1] == 0.0  # About a quarter second after the squeeze is gone.
    assert not feedback.gripping['right'] and feedback.pushes['right'] == 0.0
    settle(feedback, rows(right=(-2.0, 0.0)))
    for _ in range(4):  # Moving fast for more than 30 ms: it opened, or the object slipped.
        feedback.update(rows(right=(-1.0, 1.5)), engaged=True, walls=WALL)
    assert not feedback.gripping['right']


def test_nothing_is_felt_without_a_reading_a_wall_or_teleop():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    assert settle(feedback, rows(right=(+2.0, 0.0))) == {'left': 0.0, 'right': 0.0}  # Pushed open: never a grip.
    settle(feedback, rows(right=(-2.0, 0.0)))
    assert settle(feedback, [dict(name='right_gripper', effort=math.nan, velocity=0.0)]) == {'left': 0.0, 'right': 0.0}
    assert not feedback.gripping['right']
    settle(feedback, rows(right=(-2.0, 0.0)))
    assert feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls={})['right'] == 0.0  # No route: no wall.
    assert feedback.gripping['right'] and feedback.pushes['right'] == 0.0
    assert feedback.update(rows(right=(-2.0, 0.0)), engaged=False, walls=WALL)['right'] == 0.0  # Outside TELEOP...
    assert feedback.pushes['right'] > 0  # ...the bars still show it.
    for step in (math.nan, math.inf, -1.0):  # A bad cycle time counts as at most 100 ms, never backwards.
        assert isinstance(feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls=WALL, dt=step)['right'], dict)


def test_strength_and_every_setting_apply_live_within_bounds():
    feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})
    feedback.set(strength=0.5)
    assert settle(feedback, rows(right=(-2.0, 0.0)))['right'] == pytest.approx({'wall': 3.0, 'k': 0.5 * K, 'cap': 75.0})
    assert feedback.pushes['right'] == pytest.approx(0.5 * 150 * 1.3 / 2)
    feedback.set(cap_ma=100, wall_deg=4.0, grip_nm=1.0, hold_ms=0.0)
    feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls=WALL)
    assert feedback.pushes['right'] == pytest.approx(0.5 * 100 * 1.3 / 4)
    assert feedback.settings() == {'strength': 0.5, 'cap_ma': 100.0, 'grip_nm': 1.0, 'hold_ms': 0.0, 'wall_deg': 4.0}
    for bad in (
        {'strength': -0.1},
        {'strength': 1.1},
        {'strength': math.nan},
        {'strength': '1'},
        {'grip_nm': 0.05},
        {'hold_ms': 600},
        {'wall_deg': 0.1},
        {'wall_deg': 25},
        {'cap_ma': 201},
        {'contact_push': 0.4},
        {'gain': 1},
    ):
        with pytest.raises(ValueError):
            feedback.set(**bad)
    with pytest.raises(ValueError):
        feedback.set(enabled='yes')


def test_the_default_wall_suits_the_holds_measured():
    # live-20261024T172641Z: walled holds sat 0.08-0.15 degrees past the wall (medians, 0.37-0.45 p90), at about
    # 2.5 Nm a degree. Asked for more, the default reaches the boards' 200 mA at 0.5 degrees, within a 70 ms hold.
    feedback = GripFeedback({'enabled': True})
    assert {key: feedback.config[key] for key in ('cap_ma', 'wall_deg', 'hold_ms', 'grip_nm')} == {
        'cap_ma': 200.0,
        'wall_deg': 0.5,
        'hold_ms': 70.0,
        'grip_nm': 0.5,
    }
    settle(feedback, rows(right=(-2.0, 0.0)))
    for past_deg, push in ((0.1, 40.0), (0.15, 60.0), (0.4, 160.0), (0.5, 200.0), (1.0, 200.0)):
        feedback.update(rows(right=(-2.0, 0.0)), engaged=True, walls={'right': (3.0, 3.0 - math.radians(past_deg), 1)})
        assert feedback.pushes['right'] == pytest.approx(push, abs=0.1)


@pytest.mark.parametrize(
    'block',
    [
        {'gain': 1},
        {'deadband_nm': 0.3},
        {'contact_push': 0.4},
        {'cap_ma': 250},
        {'grip_nm': -1},
        {'closing_effort_sign': 0},
        {'enabled': 1},
        {'hold_ms': 1000},
        {'slip_rad_s': 0},
        {'wall_deg': 0},
        'on',
    ],
)
def test_profile_settings_stay_inside_their_bounds(block):
    with pytest.raises(ValueError):
        validate_config(block)
    assert validate_config({'cap_ma': 200, 'grip_nm': 0.8, 'release_nm': 0.1})['grip_nm'] == 0.8


GRIP_ROUTES = [
    dict(
        actuator='left_gripper',
        leader='left',
        channel=6,
        raw_min=4.13,
        raw_max=5.07,
        follower_min=0.0,
        follower_max=3.6,
        direction=1,
    ),
    dict(
        actuator='right_gripper',
        leader='right',
        channel=6,
        raw_min=2.57,
        raw_max=3.45,
        follower_min=0.0,
        follower_max=2.9,
        direction=1,
    ),
]


def test_a_trigger_wall_is_the_reading_its_route_maps_to_the_gripper_position():
    from robot_platform.teleop.absolute_mapping import follow, reading_for

    route = GRIP_ROUTES[1]
    for variant in (route, dict(route, direction=-1), dict(route, raw_zero=3.0, follower_zero=1.0, gain=3.3)):
        for position in (0.0, 0.7, 2.9):
            assert follow(variant, reading_for(variant, position)) == pytest.approx(position)
    # Past the closed end, extended straight rather than clamped: where the jaws pressed past their swept end.
    assert reading_for(route, -0.29) == pytest.approx(2.57 - 0.088)


# -- the request and the PC's sampler ----------------------------------------------------------------------------


class FakeInputs:
    pins = {'left': 'L', 'right': 'R'}
    max_age_s, max_skew_s = 0.05, 0.02
    triggers = {'left': (7, 1), 'right': (7, -1)}

    def __init__(self, clock):
        self.clock, self.calls = clock, []

    def connect(self):
        pass

    def poll(self, feedback=None):
        self.calls.append(feedback)
        now = self.clock()
        return {
            name: SimpleNamespace(
                sampled_at=now,
                raw_positions=(0,) * 7,
                positions_rad=(0.0,) * 7,
                sequence=len(self.calls),
                joystick=(0, 0, 0, 0),
                board_sample_ms=1,
                board_reply_ms=2,
            )
            for name in self.pins
        }

    def close(self):
        pass


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_the_sampler_sends_triggers_when_due_and_turns_them_off_once():
    clock = Clock()
    sampler = LatestLeaderInputs(FakeInputs(clock), rate_hz=200, clock=clock)
    sampler.set_grip({'left': 60.0, 'right': 0.0, 'chest': 9.0})  # Unknown leader ignored.
    frames = sampler._feedback_frames(clock())
    assert unpack(frames['left']) == [FF_DISABLE] * 6 + [60]
    assert unpack(frames['right']) == [FF_DISABLE] * 6 + [0]  # On at 0 mA: the mode switch happens once.
    assert sampler._feedback_frames(clock()) == {}  # Nothing new, not due.
    clock.now += GRIP_REFRESH_S + 1e-6
    assert set(sampler._feedback_frames(clock())) == {'left', 'right'}  # Refreshed for the firmware watchdog.
    sampler.set_grip({'left': 61.0, 'right': 0.0})
    assert sampler._feedback_frames(clock()) == {}  # Under 2 mA of change.
    sampler.set_grip({'left': 90.0, 'right': 0.0})
    assert set(sampler._feedback_frames(clock())) == {'left'}
    clock.now += GRIP_STALE_S + 0.001  # The loop stopped asking: every trigger off, once.
    off = sampler._feedback_frames(clock())
    assert {name: unpack(frame) for name, frame in off.items()} == {
        name: [FF_DISABLE] * 7 for name in ('left', 'right')
    }
    assert sampler._feedback_frames(clock()) == {}
    sampler.set_grip({'left': 30.0, 'right': 0.0})
    assert set(sampler._feedback_frames(clock())) == {'left', 'right'}
    sampler.set_grip(None)
    assert {name: unpack(frame)[6] for name, frame in sampler._feedback_frames(clock()).items()} == {
        'left': FF_DISABLE,
        'right': FF_DISABLE,
    }
    with pytest.raises(ValueError):
        sampler.set_grip({'left': -1.0})


def test_the_sampler_thread_hands_frames_to_the_board_polls():
    inputs = FakeInputs(time.perf_counter)
    sampler = LatestLeaderInputs(inputs, rate_hz=200)
    sampler.connect()
    try:
        sampler.grip = {'left': 40.0, 'right': 10.0}
        deadline = time.monotonic() + 2.0
        while not any(call for call in inputs.calls[1:]) and time.monotonic() < deadline:
            time.sleep(0.005)
        sent = next(call for call in inputs.calls[1:] if call)
        assert unpack(sent['left'])[6] == 40 and unpack(sent['right'])[6] == -10
        assert sampler.grip == {'left': 40.0, 'right': 10.0}
    finally:
        sampler.close()


def test_a_request_carries_or_withholds_strengths_and_a_bad_one_turns_feedback_off():
    seen = []
    inputs = SimpleNamespace(
        set_grip=seen.append,
        pins={'left': 'L'},
        poll=lambda: {
            'left': SimpleNamespace(
                raw_positions=(0,) * 7,
                sequence=1,
                sampled_at=0.0,
                joystick=(0, 0, 0, 0),
                board_sample_ms=1,
                board_reply_ms=2,
            )
        },
    )
    nonce = 'a' * 32
    reply = sample_reply({'type': 'sample', 'nonce': nonce, 'grip': {'left': 25}}, inputs, 'epoch', clock=lambda: 0.0)
    assert reply['nonce'] == nonce and seen[-1] == {'left': 25.0}
    sample_reply({'type': 'sample', 'nonce': nonce}, inputs, 'epoch', clock=lambda: 0.0)
    assert seen[-1] is None
    for bad in ({'left': -3}, {'left': math.nan}, {'left': '9'}, {}, [1], {7: 1.0}):
        sample_reply({'type': 'sample', 'nonce': nonce, 'grip': bad}, inputs, 'epoch', clock=lambda: 0.0)
        assert seen[-1] is None
    with pytest.raises(ValueError, match='fresh input sample'):
        sample_reply({'type': 'sample', 'nonce': nonce, 'torque': 1}, inputs, 'epoch', clock=lambda: 0.0)
    assert grip_request({'left': 1000}) == {'left': 1000.0} and grip_request({'left': 1001}) is None


def test_the_robot_side_client_sends_the_loop_strengths_with_each_request():
    requests = []

    def exchange(request):
        requests.append(request)
        raise leader_bridge.LeaderSampleSkipped('not answering in this test')

    client = LeaderSnapshotClient(exchange, PROFILE)
    with pytest.raises(leader_bridge.LeaderSampleSkipped):
        client.poll()
    client.grip = {'left': 12.5, 'right': 0.0}
    with pytest.raises(leader_bridge.LeaderSampleSkipped):
        client.poll()
    assert 'grip' not in requests[0] and requests[1]['grip'] == {'left': 12.5, 'right': 0.0}


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
    period_s = 0.01
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
    import json
    from robot_platform.helm.teleop import TeleopSession

    helm = SimpleNamespace(
        profile_path=ROOT / 'config/devices/minerva-joint-calibration.json',
        bindings={},
        mapping={},
        saved_limits={'joints': {}},
        asset_id='minerva-production-01',
    )
    fake = FakeCore()
    leaders = SimpleNamespace(saved=None, missing=lambda routes: [], store_reference=lambda zero: None)
    session = TeleopSession(
        helm,
        ROOT / 'config/devices/minerva-teleop.json',
        runs=tmp_path,
        leaders=leaders,
        core_factory=lambda bundle: fake,
    )
    defaults = GripFeedback(None).settings()
    assert session.grip_settings == defaults
    with pytest.raises(ValueError, match='Start a teleop session'):
        session.grip_feedback({'enabled': True})
    session.start()
    try:
        deadline = time.monotonic() + 5.0
        while session.snapshot()['phase'] != 'running':
            assert time.monotonic() < deadline
            time.sleep(0.01)
        assert session.grip_feedback({'enabled': True, 'strength': 0.6}) == dict(fake.feedback.snapshot())
        assert fake.feedback.enabled and fake.feedback.strength == 0.6
        with pytest.raises(ValueError, match='Force feedback takes enabled, reset and strength'):
            session.grip_feedback({'gain': 3})
        session.grip_feedback({'cap_ma': 120})
        session.grip_feedback({'hold_ms': 60, 'grip_nm': 0.7})
        assert fake.feedback.config['cap_ma'] == 120.0 and fake.feedback.config['hold_ms'] == 60.0  # Live.
        assert fake.feedback.enabled  # A setting leaves the switch as it was.
    finally:
        session.stop()
    # Saved for every later session, only what the operator changed; the switch itself is not.
    saved = {'strength': 0.6, 'cap_ma': 120.0, 'hold_ms': 60.0, 'grip_nm': 0.7}
    assert session.grip_saved == saved and session.grip_settings == dict(defaults, **saved)
    assert json.loads(session.grip_settings_path.read_text(encoding='utf-8')) == saved
    again = TeleopSession(
        helm,
        ROOT / 'config/devices/minerva-teleop.json',
        runs=tmp_path,
        leaders=leaders,
        core_factory=lambda bundle: fake,
    )
    assert again.grip_settings == dict(defaults, **saved)
    assert again._bundle()['grip_feedback'] == dict(defaults, **saved)  # What a session starts with.
    assert again.grip_feedback({'cap_ma': 90, 'strength': 1})['cap_ma'] == 90.0  # No session running.
    assert again.grip_feedback({'reset': True}) == defaults  # Defaults: nothing saved.
    assert json.loads(again.grip_settings_path.read_text(encoding='utf-8')) == {}
    with pytest.raises(ValueError, match='0-200 mA'):
        again.grip_feedback({'cap_ma': 250})
    with pytest.raises(ValueError, match='reset is true'):
        again.grip_feedback({'reset': False})
    with pytest.raises(ValueError, match='Start a teleop session'):
        again.grip_feedback({'enabled': True})


def wall_core(position=1.3, past_deg=1.3):
    """A loop holding a cube in the right gripper at `position`, the trigger `past_deg` past the wall."""
    from robot_platform.helm import teleop
    from robot_platform.teleop.absolute_mapping import reading_for

    core = object.__new__(teleop.TeleopCore)
    core.grip_feedback = GripFeedback({'enabled': True, 'hold_ms': 0.0, 'wall_deg': 2.0, 'cap_ma': 150.0})
    core.leaders = SimpleNamespace(grip='unset')
    core.fault, core.derived = None, None
    core.grip_triggers = {'left': (7, 1), 'right': (7, 1)}
    core.mapper = SimpleNamespace(routes=GRIP_ROUTES)
    readings = {
        'left': reading_for(GRIP_ROUTES[0], 0.6) - math.radians(0.5),
        'right': reading_for(GRIP_ROUTES[1], position) - math.radians(past_deg),
    }
    core._grip_samples = {
        name: SimpleNamespace(positions_rad=(0.0,) * 6 + (value,)) for name, value in readings.items()
    }
    names = ['left_arm_j1', 'left_gripper', 'right_gripper']
    feedback = [
        dict(position=0.0, velocity=0.0, effort=-5.0),
        dict(position=0.6, velocity=0.0, effort=-0.6),
        dict(position=position, velocity=0.01, effort=-2.0),
    ]
    core.harness = SimpleNamespace(last={'status': {'joint_names': names}, 'feedback': feedback})
    return core, feedback


def test_the_loop_puts_each_wall_where_the_jaws_stopped_and_only_in_teleop():
    from robot_platform.teleop.absolute_mapping import reading_for

    core, feedback = wall_core()
    for _ in range(40):
        core._send_grip_feedback('TELEOP')
    assert set(core.leaders.grip) == {'left', 'right'}
    assert core.leaders.grip['left'] == pytest.approx({'wall': reading_for(GRIP_ROUTES[0], 0.6), 'k': K, 'cap': 150.0})
    assert core.leaders.grip['right'] == pytest.approx({'wall': reading_for(GRIP_ROUTES[1], 1.3), 'k': K, 'cap': 150.0})
    status = core._grip_status()
    assert status['currents_ma'] == pytest.approx({'left': 37.5, 'right': 97.5}, abs=0.1)  # The loop's estimate.
    core._send_grip_feedback('HOLD')
    assert core.leaders.grip == {'left': 0.0, 'right': 0.0}
    feedback[2] = dict(feedback[2], fault=True)
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip['right'] == 0.0 and isinstance(core.leaders.grip['left'], dict)  # A faulted drive: none.
    core.grip_feedback.set(enabled=False)
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip is None  # Off: the PC turns the triggers off.
    assert core._grip_status()['available'] is True
    core.grip_feedback.set(enabled=True)
    core.grip_triggers = {'left': (7, 1)}
    core._send_grip_feedback('TELEOP')
    assert set(core.leaders.grip) == {'left'}  # Only a leader with a trigger is sent a wall.
    core.grip_triggers = {}
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip is None and core._grip_status()['available'] is False  # No triggers: Helm says so.
    core.leaders = SimpleNamespace()  # Scripted leaders: nothing to send, nothing to fail.
    core._send_grip_feedback('TELEOP')
    assert core._grip_status()['available'] is False


def test_a_route_and_trigger_that_disagree_on_opening_get_no_wall():
    core, _ = wall_core()
    core.mapper = SimpleNamespace(routes=[GRIP_ROUTES[0], dict(GRIP_ROUTES[1], direction=-1)])
    for _ in range(40):
        core._send_grip_feedback('TELEOP')
    # The right gripper grips, but its wall would sit on the wrong side and push the trigger open at once.
    assert core.grip_feedback.gripping['right'] and core.leaders.grip['right'] == 0.0
    assert isinstance(core.leaders.grip['left'], dict)


def test_the_loop_runs_the_law_on_its_own_clock():
    core, _ = wall_core()
    core.grip_feedback = GripFeedback({'enabled': True, 'wall_deg': 2.0, 'cap_ma': 150.0})  # The default 70 ms hold.
    clock = Clock()
    core.clock = clock
    core._send_grip_feedback('TELEOP')
    clock.now += 0.05
    core._send_grip_feedback('TELEOP')
    assert core.leaders.grip['right'] == 0.0  # 50 ms held still: not yet a grip.
    clock.now += 0.05
    core._send_grip_feedback('TELEOP')
    assert isinstance(core.leaders.grip['right'], dict)  # 100 ms: a grip, however few cycles it took.


def test_the_sampler_renders_a_wall_from_its_own_trigger_readings():
    clock = Clock()
    sampler = LatestLeaderInputs(FakeInputs(clock), rate_hz=200, clock=clock)  # Triggers: left opens +1, right -1.

    def trigger(reading):
        return SimpleNamespace(positions_rad=(0.0,) * 6 + (reading,))

    def currents():
        return {name: unpack(frame)[6] for name, frame in sampler._feedback_frames(clock()).items()}

    sampler._samples = {'left': trigger(3.0 - math.radians(1.0)), 'right': trigger(1.0 + math.radians(1.0))}
    walls = {'left': {'wall': 3.0, 'k': K, 'cap': 150.0}, 'right': {'wall': 1.0, 'k': K, 'cap': 150.0}}
    sampler.set_grip(walls)
    assert currents() == {'left': 0, 'right': 0}  # A new wall rises from nothing...
    clock.now += WALL_ONSET_S / 2
    assert currents() == {'left': 38, 'right': -38}
    clock.now += WALL_ONSET_S
    assert currents() == {'left': 75, 'right': -75}  # ...to 1 degree past it of 2 for the whole 150 mA, open-signed.
    sampler._samples = {'left': trigger(3.0 - math.radians(5.0)), 'right': trigger(1.0)}
    sampler.set_grip(walls)
    clock.now += GRIP_REFRESH_S + 1e-6  # Every trigger refreshed, changed or not.
    assert currents() == {'left': 150, 'right': 0}  # Squeezed further: capped. Back at the wall: nothing.
    sampler._samples = {'left': trigger(3.0 + math.radians(3.0)), 'right': trigger(1.0 - math.radians(3.0))}
    sampler.set_grip(walls)
    clock.now += GRIP_REFRESH_S + 1e-6
    assert currents() == {'left': 0, 'right': 0}  # Opened past the wall: nothing, never a pull.
    sampler.set_grip({'left': 0.0, 'right': 0.0})  # The grip ended: free.
    sampler._samples = {'left': trigger(3.0 - math.radians(1.0)), 'right': trigger(1.0)}
    clock.now += GRIP_REFRESH_S + 1e-6
    assert currents() == {'left': 0, 'right': 0}
    sampler._samples = {'left': SimpleNamespace(), 'right': trigger(math.nan)}  # Unusable readings: no push,
    sampler.set_grip(walls)  # never a stopped sampler.
    clock.now += WALL_ONSET_S
    assert currents() == {'left': 0, 'right': 0}
    sampler.set_grip({'left': 0.0, 'right': 0.0})
    sampler._samples = {'left': trigger(3.0 - math.radians(1.0)), 'right': trigger(1.0)}
    clock.now += GRIP_REFRESH_S + 1e-6
    currents()
    sampler.set_grip(walls)  # A new grip: its wall rises from nothing again, from the sampler's next frame.
    assert currents().get('left', 0) == 0
    clock.now += WALL_ONSET_S / 2
    assert currents()['left'] == 38


def test_a_wall_request_is_checked_and_a_bad_one_turns_feedback_off():
    wall = {'wall': 3.0, 'k': 4000.0, 'cap': 150.0}
    assert grip_request({'left': wall, 'right': 0}) == {'left': wall, 'right': 0.0}
    for bad in (
        {'wall': 3.0, 'k': 4000.0},
        dict(wall, extra=1),
        dict(wall, wall=math.nan),
        dict(wall, k=-1.0),
        dict(wall, cap=1001.0),
        dict(wall, wall='3'),
        dict(wall, wall=2000.0),
        dict(wall, k=True),
    ):
        assert grip_request({'left': bad}) is None
    with pytest.raises(ValueError):
        LatestLeaderInputs(FakeInputs(Clock()), rate_hz=200).set_grip({'left': {'wall': 3.0}})


def test_strengths_cross_the_signed_link_and_reach_the_board_polls():
    from robot_platform.teleop.leader_link import LeaderLinkInput, LeaderLinkServer

    key = bytes(range(32))
    inputs = FakeInputs(time.perf_counter)
    sampler = LatestLeaderInputs(inputs, rate_hz=200)
    sampler.connect()
    robot = LeaderLinkInput(key, PROFILE, bind=('127.0.0.1', 0), timeout_s=0.2)
    server = LeaderLinkServer(sampler, key, robot.address, local=('127.0.0.1', 0)).start()
    try:
        robot.wait_for_pc(5.0)
        robot.grip = {'left': 0.0, 'right': 44.0}
        robot.poll()
        assert sampler.grip == {'left': 0.0, 'right': 44.0}
        deadline = time.monotonic() + 2.0
        while not any(call and 'right' in call for call in inputs.calls) and time.monotonic() < deadline:
            time.sleep(0.005)
        sent = next(call for call in inputs.calls if call and 'right' in call)
        assert unpack(sent['right'])[6] == -44  # The right trigger opens with negative current in PROFILE.

        def next_sample():  # A request needs a newer sample than the last one it got.
            seen = len(inputs.calls)
            while len(inputs.calls) <= seen and time.monotonic() < deadline:
                time.sleep(0.002)

        wall = {'wall': 3.0, 'k': 4000.0, 'cap': 150.0}
        robot.grip = {'left': 0.0, 'right': wall}
        next_sample()
        robot.poll()
        assert sampler.grip == {'left': 0.0, 'right': wall}  # A wall crosses too.
        robot.grip = None
        next_sample()
        robot.poll()
        assert sampler.grip is None
    finally:
        server.close()
        robot.close()
        sampler.close()


def test_the_force_bars_show_the_push_against_the_maximum_even_with_feedback_off():
    feedback = GripFeedback({'wall_deg': 2.0, 'cap_ma': 150.0})
    assert settle(feedback, rows(right=(-2.0, 0.0))) is None  # Off: no walls sent...
    snap = feedback.snapshot()
    assert snap['levels']['right'] == pytest.approx(0.65, abs=1e-3)  # ...but the bar shows 1.3 degrees of 2.
    assert snap['gripping'] == {'left': False, 'right': True} and snap['squeeze_nm']['right'] == pytest.approx(2.0)
    assert snap['currents_ma'] == pytest.approx({'left': 0.0, 'right': 97.5})
    feedback.set(strength=0.25)
    feedback.update(rows(right=(-2.0, 0.0)), engaged=False, walls=WALL)
    assert feedback.snapshot()['levels']['right'] == pytest.approx(
        0.65, abs=1e-3
    )  # Strength scales the push, not the bar.
    assert feedback.snapshot()['currents_ma']['right'] == pytest.approx(97.5 / 4, abs=0.1)
    feedback.set(cap_ma=0)
    feedback.update(rows(right=(-2.0, 0.0)), engaged=False, walls={'right': (3.0, 3.0 - math.radians(3.0), 1)})
    assert feedback.snapshot()['levels']['right'] == 1.0 and feedback.snapshot()['currents_ma']['right'] == 0.0


def test_saved_settings_stay_within_bounds():
    from robot_platform.teleop.grip_feedback import validate_settings

    assert validate_settings({'strength': 1, 'cap_ma': 200, 'hold_ms': 0, 'wall_deg': 2}) == {
        'strength': 1.0,
        'cap_ma': 200.0,
        'hold_ms': 0.0,
        'wall_deg': 2.0,
    }
    assert validate_settings({}) == {}
    for bad in (
        {'cap_ma': 201},
        {'cap_ma': -1},
        {'strength': 2},
        {'strength': math.nan},
        {'gain': 1},
        {'grip_nm': True},
        {'hold_ms': '80'},
        {'contact_push': 0.4},
        {'wall_deg': 30},
        [1],
    ):
        with pytest.raises(ValueError):
            validate_settings(bad)


def test_an_earlier_laws_saved_settings_are_dropped_not_fatal(tmp_path):
    import json
    from robot_platform.helm.teleop import TeleopSession

    helm = SimpleNamespace(
        profile_path=ROOT / 'config/devices/minerva-joint-calibration.json',
        bindings={},
        mapping={},
        saved_limits={'joints': {}},
        asset_id='minerva-production-01',
    )
    saved = tmp_path / 'minerva-production-01-teleop' / 'grip-feedback.json'
    saved.parent.mkdir(parents=True)
    saved.write_text(json.dumps({'contact_push': 0.4, 'smoothing_ms': 40, 'hold_ms': 70}), encoding='utf-8')
    leaders = SimpleNamespace(saved=None, missing=lambda routes: [], store_reference=lambda zero: None)
    session = TeleopSession(
        helm,
        ROOT / 'config/devices/minerva-teleop.json',
        runs=tmp_path,
        leaders=leaders,
        core_factory=lambda bundle: None,
    )
    assert session.grip_saved == {'hold_ms': 70.0}


def test_every_live_setting_has_a_slider_within_its_bounds():
    import re
    from robot_platform.teleop.grip_feedback import BOUNDS, SETTINGS

    static = ROOT / 'src/robot_platform/helm/static'
    table = re.findall(
        r"\{\s*key: '(\w+)',\s*id: '([\w-]+)',\s*label: '[^']+',\s*scale: (\d+)",
        (static / 'grip.js').read_text(encoding='utf-8'),
    )
    assert sorted(key for key, _, _ in table) == sorted(SETTINGS)
    page = (static / 'index.html').read_text(encoding='utf-8')
    bounds = dict(BOUNDS, strength=(0.0, 1.0))
    for key, element, scale in table:
        slider = re.search(rf'<input\s+id="{element}"\s+type="range"\s+min="([\d.]+)"\s+max="([\d.]+)"', page)
        assert slider, element
        low, high = (float(value) / int(scale) for value in slider.groups())
        assert bounds[key][0] <= low < high <= bounds[key][1], key


def test_a_core_built_without_leaders_or_feedback_skips_force_feedback():
    from robot_platform.helm import teleop

    core = object.__new__(teleop.TeleopCore)
    core._send_grip_feedback('TELEOP')  # Nothing to send and nothing to fail.
    assert core._grip_status() is None
