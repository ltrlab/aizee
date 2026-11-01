"""Where the head camera looks, for the headset's world-locked 3D view (neck.head_pose): the colour camera's pose from
the neck's CAD angles, and the head's measured pose on its local feed at the Jetson's wall time."""
import json
import math

import numpy as np
import pytest
import zmq

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.neck import head
from robot_platform.neck.head_pose import CameraPose, HeadPoseFeed, pose7
from robot_platform.ops.release import ROOT
from robot_platform.vision.geometry import rotation


@pytest.fixture(scope='module')
def camera_pose():
    profile = head.load_profile(ROOT / head.PROFILE)
    return CameraPose(UrdfModel(ModelProfile.load(ROOT / profile['model_profile'])), profile['camera'])


def matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_at_zero_the_head_camera_looks_along_the_robots_front_level(camera_pose):
    # The AWG_RGB_CAMERA part's centre (the head model's README: 32.8, -110.5, 1269.9 mm in the robot frame at zero);
    # optical axes: x right (the robot's right is -x: its front is -y), y down, z ahead along the front.
    zero = camera_pose.zero
    assert camera_pose.joints == ['neck_z', 'neck_y']
    np.testing.assert_allclose(zero[:3, 3], [.0328, -.1105, 1.2699], atol=2e-4)
    np.testing.assert_allclose(zero[:3, :3], [[-1, 0, 0], [0, 0, -1], [0, -1, 0]], atol=1e-5)


def test_panning_turns_the_camera_left_and_tilting_raises_its_view(camera_pose):
    # As the headset's yaw (left positive) and pitch (up positive) steer neck_z and neck_y (helm.xr).
    # (The URDF's quarter turns are 1.5708: a few millionths off.)
    panned = camera_pose.pose({'neck_z': .5})[:3, 2]
    np.testing.assert_allclose(panned, [math.sin(.5), -math.cos(.5), 0], atol=1e-5)
    raised = camera_pose.pose({'neck_y': .3})[:3, 2]
    np.testing.assert_allclose(raised, [0, -math.cos(.3), math.sin(.3)], atol=1e-5)
    # Its x axis stays level: the camera never rolls.
    both = camera_pose.pose({'neck_z': -.4, 'neck_y': .2})
    assert abs(both[2, 0]) < 1e-5


def test_a_pose_is_its_position_and_unit_quaternion(camera_pose):
    rng = np.random.default_rng(3)
    turns = [rotation(rng.normal(size=3) * 1.5) for _ in range(20)]
    # Half turns and near them, where a quaternion's signs are easily lost: the camera's own pose at zero is one.
    turns += [camera_pose.zero[:3, :3]] + [rotation(axis / np.linalg.norm(axis) * (math.pi - offset))
                                           for axis, offset in zip(rng.normal(size=(10, 3)), np.geomspace(1e-9, .1, 10))]
    for turn in turns:
        transform = np.eye(4)
        transform[:3, :3] = turn
        transform[:3, 3] = rng.normal(size=3)
        pose = pose7(transform)
        np.testing.assert_allclose(pose[:3], transform[:3, 3])
        assert np.linalg.norm(pose[3:]) == pytest.approx(1)
        np.testing.assert_allclose(matrix(pose[3:]), transform[:3, :3], atol=1e-6)


class Socket:
    def __init__(self, full=False):
        self.sent, self.full = [], full

    def send_string(self, text, flags=0):
        assert flags == zmq.NOBLOCK  # The head's thread never waits for a reader.
        if self.full:
            raise zmq.Again()
        self.sent.append(json.loads(text))


AXES = {'head_pan': ('neck_z', 1., -3.), 'head_tilt': ('neck_y', -1., 3.2)}


def reply(now_ms, *times):
    """The head service's reply: its clock, and its rows (ms, then position, velocity, effort and command for each
    servo, then the command block)."""
    rows = [[t, 3. + t / 1e4, 0., 0., 0., 3.2 - t / 2e4, 0., 0., 0.] + [0.] * 8 for t in times]
    return {'status': {'now_ms': now_ms}, 'recent': rows}


def test_the_feed_sends_each_row_once_at_the_jetsons_wall_time(camera_pose):
    socket = Socket()
    feed = HeadPoseFeed(camera_pose, AXES, ['head_pan', 'head_tilt'], socket)
    assert feed.publish(reply(1000, 980, 990, 1000), wall_now=5000.003)
    assert not feed.publish(reply(1000, 990, 1000), wall_now=5000.004)  # Nothing new.
    assert feed.publish(reply(1010, 1000, 1010), wall_now=5000.013)
    first, second = socket.sent
    assert first['joints'] == ['neck_z', 'neck_y'] and first['zero'] == pytest.approx(pose7(camera_pose.zero))
    assert [row[0] for row in first['samples'] + second['samples']] == pytest.approx(
        [4999.983, 4999.993, 5000.003, 5000.013])
    t, pan, tilt, *pose = second['samples'][0]
    assert (pan, tilt) == pytest.approx((1010 / 1e4, -(3.2 - 1010 / 2e4) + 3.2))
    assert pose == pytest.approx(pose7(camera_pose.pose({'neck_z': pan, 'neck_y': tilt})))


def test_the_feeds_clock_takes_the_quickest_reply_and_rises_only_slowly(camera_pose):
    socket = Socket()
    feed = HeadPoseFeed(camera_pose, AXES, ['head_pan', 'head_tilt'], socket)
    feed.publish(reply(1000, 1000), wall_now=10.)  # A reply read late: 9.0 s ahead of the service's clock.
    feed.publish(reply(1010, 1010), wall_now=10.002)  # A quicker one: 8.992.
    assert feed.offset == pytest.approx(8.992)
    feed.publish(reply(1020, 1020), wall_now=10.050)  # A slow one moves it only a little.
    assert feed.offset == pytest.approx(8.9921)
    assert not HeadPoseFeed(camera_pose, AXES, ['head_pan', 'head_tilt'], Socket(full=True)).publish(
        reply(1000, 1000), wall_now=10.)  # No reader keeps up: it waits for nobody.
    assert not feed.publish({'status': {}}, wall_now=11.)  # A reply without the service's clock.


def test_the_twins_offsets_turn_the_pose_and_leave_the_published_angles_measured(camera_pose):
    profile = head.load_profile(ROOT / head.PROFILE)
    turned = CameraPose(UrdfModel(ModelProfile.load(ROOT / profile['model_profile'])), profile['camera'],
                        offsets={'neck_z': .05})
    np.testing.assert_allclose(turned.pose({'neck_z': .2, 'neck_y': -.3}),
                               camera_pose.pose({'neck_z': .25, 'neck_y': -.3}), atol=1e-12)
    np.testing.assert_allclose(turned.cad, camera_pose.cad)  # The CAD mount is the neck's zero's, as before.
    np.testing.assert_allclose(turned.zero, camera_pose.pose({'neck_z': .05}), atol=1e-12)
    socket = Socket()
    HeadPoseFeed(turned, AXES, ['head_pan', 'head_tilt'], socket).publish(reply(1000, 1000), wall_now=10.)
    _t, pan, tilt, *pose = socket.sent[0]['samples'][0]
    assert pan == pytest.approx(.1)  # As measured: head stills are fitted on these.
    assert pose == pytest.approx(pose7(camera_pose.pose({'neck_z': pan + .05, 'neck_y': tilt})))
    assert socket.sent[0]['zero'] == pytest.approx(pose7(turned.zero))
