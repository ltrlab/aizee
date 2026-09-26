"""Authenticated UDP leader link over loopback: freshness, skips, faults and keys."""
import time

import pytest

from robot_platform.teleop.leader_link import LeaderLinkInput, LeaderLinkProcess, LeaderLinkServer, seal, unseal
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped

PROFILE = {'leaders': {'left': {'usb_serial': 'A1'}, 'right': {'usb_serial': 'B2'}},
           'max_age_s': .05, 'max_skew_s': .02}
KEY = b'k' * 32


class Inputs:
    pins = {'left': 'A1', 'right': 'B2'}

    def __init__(self):
        self.sequence, self.failure = 0, None

    def poll(self):
        if self.failure:
            raise self.failure
        self.sequence += 1
        now = time.perf_counter()
        return {name: LeaderSample(tuple(.1 * i for i in range(7)), tuple(range(7)), self.sequence, now - .002,
                                   now - .001, (0, 0, 1, 1), 10 * self.sequence, 10 * self.sequence + 1)
                for name in self.pins}


@pytest.fixture
def link():
    robot = LeaderLinkInput(KEY, PROFILE, bind=('127.0.0.1', 0), timeout_s=.2)
    inputs = Inputs()
    server = LeaderLinkServer(inputs, KEY, robot.address, local=('127.0.0.1', 0)).start()
    robot.wait_for_pc(5.)
    yield robot, server, inputs
    server.close()
    robot.close()


def test_the_robot_gets_fresh_authenticated_pairs(link):
    robot, server, _inputs = link
    first, second = robot.poll(), robot.poll()
    assert set(first) == {'left', 'right'} and second['left'].sequence > first['left'].sequence
    assert first['left'].raw_positions == tuple(range(7))
    assert 0 <= time.perf_counter() - second['left'].sampled_at < .05  # Source age plus the round trip.
    assert server.served == 2 and server.rejected == 0


def test_a_skip_on_the_pc_is_a_skipped_sample_not_a_fault(link):
    robot, _server, inputs = link
    inputs.failure = LeaderSampleSkipped('Leader pair age or skew exceeded the input profile')
    with pytest.raises(LeaderSampleSkipped, match='Leader PC: Leader pair age'):
        robot.poll()
    inputs.failure = None
    assert robot.poll()['left'].sequence > 0 and robot.fault is None


def test_a_failed_pc_input_faults_the_robot_side(link):
    robot, server, inputs = link
    inputs.failure = RuntimeError('Leader left is missing or ambiguous')
    with pytest.raises(RuntimeError, match='Leader PC: Leader left is missing'):
        robot.poll()
    assert robot.fault and server.failure
    with pytest.raises(RuntimeError, match='faulted'):
        robot.poll()


def test_another_key_is_ignored_and_the_robot_times_out(link):
    robot, server, _inputs = link
    robot.key = b'x' * 32
    with pytest.raises(LeaderSampleSkipped, match='No leader reply within'):
        robot.poll()
    assert server.rejected == 1 and robot.late == 1 and robot.fault is None


def test_nothing_is_requested_before_the_pc_says_hello():
    robot = LeaderLinkInput(KEY, PROFILE, bind=('127.0.0.1', 0))
    try:
        with pytest.raises(LeaderSampleSkipped, match='Waiting for the leader PC'):
            robot.poll()
        with pytest.raises(TimeoutError):
            robot.wait_for_pc(.05)
    finally:
        robot.close()


def test_seal_rejects_tampering():
    packet = seal(KEY, b'\x81\xa4type\xa5hello')
    assert unseal(KEY, packet) == {'type': 'hello'}
    assert unseal(KEY, packet[:-1] + b'x') is None and unseal(b'y' * 32, packet) is None
    with pytest.raises(ValueError, match='session key'):
        LeaderLinkInput(b'short', PROFILE)


def test_the_pc_end_runs_in_its_own_responsive_process():
    robot = LeaderLinkInput(KEY, PROFILE, bind=('127.0.0.1', 0), timeout_s=.5)
    pc = LeaderLinkProcess(PROFILE, KEY, robot.address, local=('127.0.0.1', 0), real=False)
    try:
        pc.start()
        robot.wait_for_pc(10.)
        first, second = robot.poll(), robot.poll()
        assert second['left'].sequence > first['left'].sequence and robot.fault is None
    finally:
        counts = pc.close()
        robot.close()
    assert counts['served'] == 2 and counts['failure'] is None and counts['rejected'] == 0
    assert pc.close() is counts  # Closing twice is harmless.
