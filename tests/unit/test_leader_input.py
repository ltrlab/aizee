from dataclasses import replace
import struct

import pytest

from robot_platform.teleop.openrb import crc8, decode_frame, LeaderSample, LeaderSampleSkipped
from robot_platform.teleop.leader_mapping import LeaderTargetMapper


def frame(*, sequence=1, sample_ms=100, reply_ms=103, valid=1):
    data = b"\xa6\x07" + struct.pack("<7ihhBBIIIB", *range(2000, 2007), 0, 0, 1, 1,
                                   sequence, sample_ms, reply_ms, valid)
    return data + bytes([crc8(data)])


def test_source_age_includes_board_read_time_and_host_round_trip():
    sample = decode_frame(frame(), requested_at=10., received_at=10.005)
    assert sample.sampled_at == pytest.approx(9.997)
    assert sample.raw_positions[0] == 2000
    assert sample.positions_rad[0] == pytest.approx(2000 * 2 * 3.141592653589793 / 4096)
    assert sample.joystick[-1] == 1  # absent joystick is not an engaged deadman


@pytest.mark.parametrize("bad", [frame(valid=0), frame(sequence=0), frame(sample_ms=40),
                                 frame()[:-1] + b"\0", b"\xa5\x07" + frame()[2:37]])
def test_cached_partial_corrupt_and_legacy_leader_frames_are_rejected(bad):
    with pytest.raises(ValueError):
        decode_frame(bad, requested_at=10., received_at=10.005)


@pytest.mark.parametrize("raw,skipped", [(frame(valid=0), True), (frame(sample_ms=40), True),
                                         (frame(sequence=0), False), (frame()[:-1] + b"\0", False)])
def test_only_complete_late_or_partial_servo_frames_are_skippable(raw, skipped):
    """Skipped frames were fully consumed; corrupt frames must still close the port."""
    with pytest.raises(ValueError) as error:
        decode_frame(raw, requested_at=10., received_at=10.005)
    assert isinstance(error.value, LeaderSampleSkipped) is skipped


def test_repeated_source_and_reboot_are_rejected_even_when_serial_frames_are_new():
    previous = decode_frame(frame(sequence=5), requested_at=10., received_at=10.005)
    for raw in (frame(sequence=5), frame(sequence=4), frame(sequence=6, sample_ms=1, reply_ms=3)):
        with pytest.raises(ValueError):
            decode_frame(raw, requested_at=10.01, received_at=10.015, previous=previous)
    wrapped = replace(previous, sequence=0xfffffffe, board_reply_ms=0xfffffffd)
    assert decode_frame(frame(sequence=1, sample_ms=0, reply_ms=3), requested_at=10.01,
                        received_at=10.015, previous=wrapped).sequence == 1


def sample(position=0., sequence=1, now=10.):
    return LeaderSample((position,), (0,), sequence, now, now, (0, 0, 1, 1), 0, 0)


def mapper():
    return LeaderTargetMapper(
        [{"name": name, "lower": -2., "upper": 2., "max_velocity": .1} for name in ("a", "b")],
        [{"actuator": "a", "leader": "left", "channel": 0, "scale": -1.},
         {"actuator": "b", "leader": "right", "channel": 0, "scale": 2.}])


def test_registration_never_snaps_and_targets_are_slew_limited_in_joint_order():
    mapping = mapper()
    mapping.register({"left": sample(1.), "right": sample(-1.)}, [.3, -.3], now=10.)
    same = {"left": sample(1., 2, 10.04), "right": sample(-1., 2, 10.04)}
    assert mapping.propose(same, now=10.04, engaged=True) == [.3, -.3]
    moved = {"left": sample(1.1, 3, 10.08), "right": sample(-.9, 3, 10.08)}
    assert mapping.propose(moved, now=10.08, engaged=True) == pytest.approx([.296, -.296])


def test_failed_new_registration_cannot_leave_old_registration_active():
    mapping = mapper()
    inputs = {"left": sample(), "right": sample()}
    mapping.register(inputs, [.3, -.3], now=10.)
    with pytest.raises(ValueError):
        mapping.register(inputs, [float("nan"), 0.], now=10.)
    assert not mapping.registered


@pytest.mark.parametrize("failure", ["missing", "stale", "skew", "frozen", "jump", "stall", "release"])
def test_any_leader_or_operator_failure_latches_until_explicit_registration(failure):
    mapping = mapper()
    mapping.register({"left": sample(), "right": sample()}, [.3, -.3], now=10.)
    now = 10.04
    inputs = {"left": sample(.01, 2, now), "right": sample(.01, 2, now)}
    if failure == "missing":
        inputs.pop("left")
    elif failure == "stale":
        inputs["left"] = sample(.01, 2, 9.9)
    elif failure == "skew":
        inputs["left"] = sample(.01, 2, 10.01)
    elif failure == "frozen":
        inputs["left"] = sample(.01, 1, now)
    elif failure == "jump":
        inputs["left"] = sample(6.28, 2, now)
    elif failure == "stall":
        now = 10.1
    with pytest.raises(ValueError):
        mapping.propose(inputs, now=now, engaged=failure != "release")
    assert not mapping.registered
    with pytest.raises(ValueError, match="registration"):
        mapping.propose(inputs, now=now + .001, engaged=True)
