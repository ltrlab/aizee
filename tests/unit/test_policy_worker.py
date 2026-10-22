"""The policy's own process: it loads a policy, answers observations with chunks matched to their request, reports
failures instead of dying, and a live session runs only trained checkpoints or hold_still."""
import time

import numpy as np
import pytest

from robot_platform.teleop.policy_source import validate_request
from robot_platform.teleop.policy_worker import PolicyWorker

NAMES = ('a', 'b')
CONTEXT = {'home': [.1, -.2], 'lower': [-1., -1.], 'upper': [1., 1.], 'period_s': .02, 'joint_map': {}}


def answers(worker, count=1, timeout=30.):
    replies, deadline = [], time.monotonic() + timeout
    while len(replies) < count:
        assert time.monotonic() < deadline, 'the worker did not answer'
        replies += worker.poll()
        time.sleep(.01)
    return replies


class Doubling:
    name = 'doubling'
    action_dt = .05

    def reset(self, context):
        self.names = context.joint_names

    def __call__(self, observation):
        if observation['q'][0] < 0:
            raise RuntimeError('negative')
        return [[2 * q for q in observation['q']]] * 3


def test_a_worker_answers_each_observation_with_its_chunk_and_reports_failures():
    worker = PolicyWorker(Doubling(), NAMES, context=CONTEXT, in_process=True)
    try:
        info = worker.ready(10.)
        assert (info['name'], info['action_dt'], info['camera_roles']) == ('doubling', .05, [])
        worker.ask(1, 12.5, {'q': [.5, 1.], 'qcmd': [.5, 1.]})
        reply, = answers(worker)
        assert reply['request'] == 1 and np.allclose(reply['chunk'], [[1., 2.]] * 3) and reply['inference_ms'] >= 0
        worker.ask(2, 13., {'q': [-1., 0.], 'qcmd': [-1., 0.]})
        reply, = answers(worker)
        assert reply == {'request': 2, 'error': 'RuntimeError: negative'}  # One failed answer; the worker lives on.
        worker.ask(3, 13.5, {'q': [.25, 0.], 'qcmd': [.25, 0.]})
        assert answers(worker)[0]['request'] == 3
    finally:
        worker.close()


def test_a_policy_process_loads_a_reference_policy_and_defaults_its_row_spacing_to_the_loop_period():
    worker = PolicyWorker('hold_still', NAMES, context=CONTEXT)
    try:
        info = worker.ready(60.)
        assert info['name'] == 'hold_still' and info['action_dt'] == .02  # It names none: the loop's period.
        worker.ask(1, 5., {'q': [0., 0.], 'qcmd': [0., 0.]})
        reply, = answers(worker)
        assert np.allclose(reply['chunk'], [CONTEXT['home']])  # The reference pose, to hold.
    finally:
        worker.close()
    assert not worker.process.is_alive()


def test_a_policy_that_cannot_load_says_why():
    worker = PolicyWorker('no_such_policy', NAMES, context=CONTEXT)
    try:
        with pytest.raises(RuntimeError, match='could not start: ValueError: policy is one of'):
            worker.ready(60.)
    finally:
        worker.close()


def test_the_arms_run_only_trained_checkpoints_or_hold_still():
    for spec in ('hold_still', 'checkpoint:C:/runs/v3/best.pt', 'lerobot:/home/minerva/policies/act-001'):
        assert validate_request({'policy': spec}, live=True) == {'kind': 'policy', 'policy': spec}
    for spec in ('penetrating', 'fixture_replay', 'tests.policies:Sway'):
        with pytest.raises(ValueError, match='for the simulation'):
            validate_request({'policy': spec}, live=True)
        assert validate_request({'policy': spec}, live=False)['policy'] == spec  # The virtual receiver may.
    for bad in ({}, {'policy': ''}, {'policy': 3}, {'policy': 'hold_still', 'speed': 2}, {'kind': 'episode', 'policy': 'x'}):
        with pytest.raises(ValueError, match='names its policy'):
            validate_request(bad, live=False)
