"""The policy's own process: it loads a policy, answers observations with chunks matched to their request, reports
failures instead of dying, reads the robot's cameras as the task capture does, and a live session runs only trained
checkpoints or hold_still."""
from io import BytesIO
import threading
import time

import numpy as np
from PIL import Image
import pytest
import zmq

from robot_platform.helm.task_capture import CAMERAS
from robot_platform.teleop.policy_source import validate_request
from robot_platform.teleop.policy_worker import CaptureCameras, PolicyWorker
from robot_platform.transport.wire import pack_camera

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
    assert not worker.alive


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


def test_the_cameras_reach_the_policy_as_the_recorder_stores_them_and_stale_ones_give_no_observation():
    context, publishers, endpoints = zmq.Context(), {}, {}
    for role in CAMERAS:
        publisher = context.socket(zmq.PUB)
        publisher.setsockopt(zmq.LINGER, 0)
        publisher.bind('tcp://127.0.0.1:*')
        endpoints[role], publishers[role] = publisher.getsockopt(zmq.LAST_ENDPOINT).decode(), publisher
    config = dict(schema=1, camera_roles=list(CAMERAS), recording_image_size=[32, 24],
                  cameras={role: dict(endpoint=endpoints[role], camera_id=role, timestamp_clock='local_unix',
                                      timestamp_semantics='host_read_complete') for role in CAMERAS},
                  head=dict(endpoint='tcp://127.0.0.1:5596', motors=['head_pan', 'head_tilt']))  # Unused here.
    cameras = CaptureCameras(config)
    with pytest.raises(ValueError, match='lacks'):
        cameras.open(('top',))
    reader, stop = cameras.open(('head', 'right_wrist')), threading.Event()

    def publish():  # 64x48 JPEGs, each camera its own grey, every 30 ms: twice the recording size, as live.
        frame = 0
        while not stop.is_set():
            stamp, frame = time.time(), frame + 1
            for value, role in enumerate(CAMERAS):
                payload = BytesIO()
                Image.fromarray(np.full((48, 64, 3), 40 + 60 * value, np.uint8)).save(payload, format='JPEG', quality=95)
                header = dict(camera_id=role, timestamp=stamp, frame_number=frame,
                              color=dict(format='jpeg', width=64, height=48))
                publishers[role].send_multipart(pack_camera(header, payload.getvalue()))
            time.sleep(.03)
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 10.
        while True:
            try:
                images, ages = reader.latest()
                break
            except ValueError:
                assert time.monotonic() < deadline, 'no coherent camera set arrived'
                time.sleep(.05)
        assert set(images) == {'head', 'right_wrist'}  # The policy's cameras, at the recording size.
        assert all(image.shape == (24, 32, 3) and image.dtype == np.uint8 for image in images.values())
        assert abs(int(images['head'][0, 0, 0]) - 160) < 6 and abs(int(images['right_wrist'][0, 0, 0]) - 100) < 6
        assert all(0 <= age < .2 for age in ages.values())
        stop.set()
        thread.join()
        time.sleep(.3)  # Past the profile's 0.2 s camera age.
        with pytest.raises(ValueError, match='no fresh camera frame'):
            reader.latest()
    finally:
        stop.set()
        reader.close()
        for publisher in publishers.values():
            publisher.close(0)
        context.term()
