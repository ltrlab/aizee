"""Where the head camera looks, for the headset's world-locked 3D view (docs/development/HEADSET_3D_PLAN.md).

The colour camera's pose in the robot frame from the neck's CAD angles: forward kinematics on the head model's URDF,
then the camera's mount in the head link. The mount is the one measured on the table's tags (Helm's head camera
alignment, saved as the head's camera file) when there is one for this head reference; else the CAD guess, the head
profile's `camera`: its centre, and its axes from the model's front, which the camera looks along, level, with the
neck at zero. Optical axes: x right, y down, z ahead. The digital twin's correction of the pan's zero (vision.twin),
when one holds, is added to the measured angles the pose is worked out on; the feed still publishes them as measured.

The head's teleop publishes the head's measured pose on a local feed (ENDPOINT, ZMQ PUB): every one of the head
service's 100 Hz rows, at the Jetson's wall time, as CAD angles and that camera pose, with each row's raw motor
readings (position, velocity, effort, command). Helm's camera relay (helm.camera_video_worker) stamps each head depth
frame with the sample nearest its capture, and the task capture (helm.task_capture) records the head's axes and
camera pose with every episode frame: both clocks are the Jetson's.
"""
import hashlib
import json
import math

import numpy as np

from robot_platform.vision.geometry import rotation, rotvec, rpy_matrix, transform

ENDPOINT = 'tcp://127.0.0.1:5596'
UP = np.array([0., 0., 1.])
ALIGNMENT_KIND = 'head_camera_alignment'
MAX_MOUNT_M = .5  # A mount farther than this from its link's origin is not the head camera's.


def cad_mount(link_at_zero, front_direction, origin_m):
    """The camera's CAD mount in its link (4x4): at `origin_m`, looking along the model's front, level, with the neck
    at zero (`link_at_zero`: the link's pose in the robot frame there)."""
    front = np.asarray(front_direction, dtype=float)
    ahead = front / np.linalg.norm(front)
    right = np.cross(ahead, UP)
    right /= np.linalg.norm(right)
    down = np.cross(ahead, right)
    return transform(link_at_zero[:3, :3].T @ np.column_stack([right, down, ahead]), origin_m)


def reference_digest(reference):
    """A head reference's identity: the neck's CAD angles, and so a measured mount, hold only for the reference they
    were read through."""
    return hashlib.sha256(json.dumps(reference, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def aligned_mount(document, link, reference):
    """The measured mount (4x4) in a saved head camera alignment (helm.head_alignment), for the camera on `link` and
    the head reference in use; ValueError says why it does not apply."""
    if not isinstance(document, dict) or document.get('kind') != ALIGNMENT_KIND or document.get('schema') != 1:
        raise ValueError('not a head camera alignment')
    if document.get('link') != link:
        raise ValueError(f"the alignment is for the camera on {document.get('link')}, not {link}")
    if document.get('reference_sha256') != reference_digest(reference):
        raise ValueError("the alignment was made through another head reference: align the head camera again")
    mount = np.asarray((document.get('mount') or {}).get('matrix'), dtype=float)
    if (mount.shape != (4, 4) or not np.isfinite(mount).all() or not np.allclose(mount[3], (0., 0., 0., 1.))
            or not np.allclose(mount[:3, :3] @ mount[:3, :3].T, np.eye(3), atol=1e-6)
            or np.linalg.det(mount[:3, :3]) < 0 or np.linalg.norm(mount[:3, 3]) > MAX_MOUNT_M):
        raise ValueError('the alignment\'s mount is not a rigid transform near its link')
    return mount


def pose7(transform):
    """[x, y, z, qx, qy, qz, qw] of a 4x4 transform. Through its rotation vector, which holds through a half turn:
    the camera's own at the head's zero is one."""
    v = rotvec(transform[:3, :3])
    theta = float(np.linalg.norm(v))
    xyz = v / theta * math.sin(theta / 2) if theta > 1e-12 else v / 2
    return [float(c) for c in (*transform[:3, 3], *xyz, math.cos(theta / 2))]


class CameraPose:
    """The colour camera's optical pose in the robot frame for the neck's CAD angles ({joint: rad}; the others in
    its chain at zero). `model`: the head model (models.UrdfModel); `camera`: the head profile's `camera`; `mount`:
    a measured mount (aligned_mount) in place of the CAD one; `offsets`: the digital twin's correction of the neck's
    zeros ({joint: rad}, vision.twin.head_offsets), added to the angles the pose is worked out on."""

    def __init__(self, model, camera, mount=None, offsets=None):
        by_child = {joint['child']: joint for joint in model.joints}
        chain, link = [], camera['link']
        while link != model.root:
            joint = by_child[link]
            chain.append((joint['name'], transform(rpy_matrix(*joint['origin']['rpy']), joint['origin']['xyz']),
                          joint['axis'], joint['type']))
            link = joint['parent']
        self.chain = chain[::-1]
        self.joints = [name for name, _origin, _axis, kind in self.chain if kind in ('revolute', 'continuous')]
        self.cad = cad_mount(self._link({}), model.profile.front_direction, camera['origin_m'])
        self.mount = self.cad if mount is None else np.asarray(mount, dtype=float)
        self.offsets = dict(offsets or {})
        self.zero = self.pose({})

    def _link(self, positions):
        result = np.eye(4)
        for name, origin, axis, kind in self.chain:
            result = result @ origin
            if kind in ('revolute', 'continuous'):
                result = result @ transform(rotation(np.asarray(axis) * positions.get(name, 0.)), (0., 0., 0.))
        return result

    def pose(self, positions):
        """The camera's 4x4 optical pose in the robot frame, for the neck's measured angles."""
        drawn = {name: positions.get(name, 0.) + self.offsets.get(name, 0.) for name in self.joints}
        return self._link(drawn) @ self.mount


class HeadPoseFeed:
    """The head's measured pose on the local feed: each new row of the head service's reply (`recent`: the service's
    ms, then position, velocity, effort and command for each motor), at the Jetson's wall time (the service's clock
    plus the smallest gap between the two seen, slowly let rise), as CAD angles through the head's axes ({motor:
    (joint, scale, offset)}) and the camera's pose. `socket`: a ZMQ PUB, bound by the caller."""

    def __init__(self, camera_pose, axes, motors, socket):
        self.camera_pose, self.axes, self.motors, self.socket = camera_pose, axes, list(motors), socket
        self.offset = None
        self.last_ms = -math.inf
        self.zero = pose7(camera_pose.zero)

    def publish(self, last, wall_now):
        """Publish the rows of `last` (the head receiver's latest output) not yet sent; True when any went."""
        status = (last or {}).get('status') or {}
        now_ms, rows = status.get('now_ms'), (last or {}).get('recent') or []
        if type(now_ms) not in (int, float):
            return False
        gap = wall_now - now_ms / 1000
        self.offset = gap if self.offset is None else min(gap, self.offset + 1e-4)
        samples, raw = [], []
        for row in rows:
            if row[0] <= self.last_ms:
                continue
            angles = {}
            for index, motor in enumerate(self.motors):
                joint, scale, offset = self.axes[motor]
                angles[joint] = row[1 + 4 * index] * scale + offset
            pose = pose7(self.camera_pose.pose(angles))
            samples.append([row[0] / 1000 + self.offset, *(angles.get(j, 0.) for j in self.camera_pose.joints), *pose])
            # The row as the service recorded it, for the episodes (helm.task_capture): per motor, its position,
            # velocity, effort (N m) and the command it was given, in `motors` order.
            raw.append([float(v) for v in row[1:1 + 4 * len(self.motors)]])
            self.last_ms = row[0]
        if not samples:
            return False
        import zmq
        try:
            self.socket.send_string(json.dumps({'joints': self.camera_pose.joints, 'zero': self.zero,
                                                'samples': samples, 'motors': self.motors, 'raw': raw}), zmq.NOBLOCK)
        except zmq.Again:
            return False  # Nobody keeps up: the next rows go with the next reply.
        return True
