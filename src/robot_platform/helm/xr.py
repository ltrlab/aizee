"""The operator's headset steering the head (helm.head_teleop): a WebXR page (static/xr.html), open in the browser of
a Meta Quest Pro, streams the headset's orientation to Helm, which turns it into the head's yaw and pitch from a zero
the operator sets, for the running teleop session.

A pose is the headset's orientation in WebXR's 'local' space as a unit quaternion (x, y, z, w): Y is up (the space is
level) and -Z is forward where the XR session started. Yaw turns about Y, left positive; pitch is the gaze's
elevation, up positive; roll is ignored. The zero starts as that heading, level (a new XR session, or a recenter in
the headset, starts it again), and moves to wherever the operator looks when they zero. One headset steers at a time:
another device's is refused while it is connected; the same device reconnecting takes its stream back.

A pose may carry the controllers (`hands`: {'left', 'right'}), for the arms' grab and drag (teleop.vr_arms, VR arm
control): each its grip's position `p` (metres) and orientation `q` in the same space, `grip` and `trigger` (0 to 1),
`b` (B or Y held) and `ui` (it is working the headset's screens). Helm keeps each hand's latest with the time it came
(this clock), and counts the XR sessions and recenters (`epoch`): a hand from before one is in another space.

The headset shows the wrist cameras' pictures on the model's wrists (static/xr_wrists.js), each where its camera is:
wrist_mounts gives each camera's mount on its link, from the newest wrist calibration's solve, else the nominal one.
"""
import json
import math
from pathlib import Path
import time

MAX_RATE_HZ = 120.  # Poses beyond this rate are dropped (a page renders at 72-90 Hz and sends at most 60).
FRESH_S = .25  # A pose this recent counts as the headset tracking.
HANDS = ('left', 'right')
HAND_KEYS = frozenset({'p', 'q', 'grip', 'trigger', 'b', 'ui'})
REACH_M = 10.  # A controller farther than this from where the session started is not a controller in the room.
# A wrist camera's mount on its link until a solve gives one (simulation/task_scene.py's wrist camera: 35 mm along the
# link's y and z, looking 37 degrees below the link's y; optical axes x right, y down, z ahead), and its vertical field
# of view.
NOMINAL_MOUNT = ((1., 0., 0., 0.), (0., -.606, .795, .035), (0., -.795, -.606, .035), (0., 0., 0., 1.))
NOMINAL_FOV_Y_DEG = 85.


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def hand_sample(message):
    """A controller's fields as {position, orientation (unit), grip, trigger, hold, ui}; ValueError if malformed."""
    if not isinstance(message, dict) or set(message) - HAND_KEYS or not {'p', 'q', 'grip', 'trigger'} <= set(message):
        raise ValueError('A controller is p, q, grip and trigger, with b and ui')
    position, orientation = message['p'], message['q']
    if (not isinstance(position, list) or len(position) != 3 or not all(_number(v) for v in position)
            or not all(abs(v) <= REACH_M for v in position)):
        raise ValueError(f'A controller position is three numbers within {REACH_M:g} m')
    if not isinstance(orientation, list) or len(orientation) != 4 or not all(_number(v) for v in orientation):
        raise ValueError('A controller orientation is four numbers x, y, z, w')
    norm = math.sqrt(sum(v * v for v in orientation))
    if not .9 < norm < 1.1:
        raise ValueError('A controller orientation is a unit quaternion')
    if not all(_number(message[k]) and 0 <= message[k] <= 1 for k in ('grip', 'trigger')):
        raise ValueError('A controller grip and trigger are 0 to 1')
    if any(type(message.get(k, False)) is not bool for k in ('b', 'ui')):
        raise ValueError('A controller b and ui are true or false')
    return {'position': tuple(float(v) for v in position), 'orientation': tuple(float(v) / norm for v in orientation),
            'grip': float(message['grip']), 'trigger': float(message['trigger']), 'hold': message.get('b', False),
            'ui': message.get('ui', False)}


def yaw_pitch(quaternion):
    """(yaw, pitch) in radians of the gaze of an orientation (x, y, z, w) in a Y-up, -Z-forward space."""
    if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
        raise ValueError('An orientation is four numbers x, y, z, w')
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or not .9 < norm < 1.1:
        raise ValueError('An orientation is a unit quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # The gaze is -Z turned by the orientation: minus the rotation matrix's third column.
    gaze = (-2 * (x * z + w * y), -2 * (y * z - w * x), -(1 - 2 * (x * x + y * y)))
    return math.atan2(-gaze[0], -gaze[2]), math.asin(max(-1., min(1., gaze[1])))


def wrapped(angle):
    """An angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Headset:
    """The headset steering the head: its connection, latest pose, rate and zero."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.connection = 0  # The live connection's number; 0: none.
        self.owner = self.device = None  # Its login session (None: Helm without logins) and device name.
        self._connections = 0
        self.zero_yaw, self.zero_pitch = 0., 0.
        self.raw = self.at = self.sequence = None
        self.poses = 0
        self._rate = None
        self.hands = {}  # Each controller's latest (hand_sample, with 'at').
        self.epoch = 0  # XR sessions and recenters so far.

    def connect(self, owner=None, device=None):
        """A connection from the login session `owner`, which steers from now on; its number. While another
        device's headset is connected it is refused (ValueError); the same device's replaces its old one."""
        if self.connection and owner != self.owner:
            raise ValueError(f"{self.device or 'Another headset'} is steering the head: stop head tracking there "
                             'first')
        self._connections += 1
        if owner != self.owner:
            self._restart()
        self.connection, self.owner, self.device = self._connections, owner, device
        self.sequence, self._rate = None, None
        return self.connection

    def disconnect(self, connection):
        if connection == self.connection:
            self.connection = 0

    def start(self, connection):
        """A new XR session (or a recenter) on `connection`: its space starts at the headset's heading, level,
        which is the zero again."""
        self._current(connection)
        self._restart()

    def _restart(self):
        self.zero_yaw, self.zero_pitch = 0., 0.
        self.raw = self.at = self.sequence = None
        self._rate = None
        self.hands = {}
        self.epoch += 1

    def _current(self, connection):
        if connection is not None and connection != self.connection:
            raise ValueError('Another connection is steering the head')

    def receive(self, connection, message):
        """A pose message ({'type': 'pose', 'seq': n, 'q': [x, y, z, w]}, optionally 'hands') from `connection`: (yaw,
        pitch) from the zero, or None for one that is dropped (another connection's, an old sequence, too fast). A
        malformed orientation or controller refuses the whole message (ValueError)."""
        if connection != self.connection or not isinstance(message, dict) or message.get('type') != 'pose':
            return None
        sequence = message.get('seq')
        if type(sequence) is not int or sequence < 1 or (self.sequence is not None and sequence <= self.sequence):
            return None
        yaw, pitch = yaw_pitch(message.get('q'))
        hands = message.get('hands', {})
        if not isinstance(hands, dict) or set(hands) - set(HANDS):
            raise ValueError('Controllers are {left, right}')
        hands = {hand: hand_sample(value) for hand, value in hands.items()}
        now = self.clock()
        if self.at is not None and now - self.at < 1 / MAX_RATE_HZ:
            return None
        if self.at is not None and now > self.at:
            rate = 1 / (now - self.at)
            self._rate = rate if self._rate is None else .9 * self._rate + .1 * rate
        self.raw, self.at, self.sequence = (yaw, pitch), now, sequence
        self.poses += 1
        for hand, sample in hands.items():
            self.hands[hand] = dict(sample, at=now)
        return self.relative()

    def hand_samples(self):
        """({hand: its latest sample with 'at'} while connected, else {}; epoch): for the arms' grab and drag, which
        lets go of a hand whose sample is stale or from an earlier epoch."""
        return ({hand: dict(sample) for hand, sample in self.hands.items()} if self.connection else {}), self.epoch

    def relative(self):
        yaw, pitch = self.raw
        return wrapped(yaw - self.zero_yaw), pitch - self.zero_pitch

    def zero(self, connection=None):
        """Wherever the headset looks now becomes the head's zero (straight ahead, level); from the teleop tab
        (`connection` None) or the steering headset's own connection."""
        self._current(connection)
        if not self.fresh():
            raise ValueError('No headset pose to zero from: start head tracking on the headset first')
        self.zero_yaw, self.zero_pitch = self.raw
        return self.status()

    def fresh(self):
        return self.connection != 0 and self.at is not None and self.clock() - self.at <= FRESH_S

    def angles(self):
        """Where the headset looks, as the neck's CAD angles (yaw is neck_z's, pitch neck_y's, as the head follows
        them: helm.head_teleop) while it tracks; None otherwise."""
        if not self.fresh():
            return None
        yaw, pitch = self.relative()
        return {'neck_z': yaw, 'neck_y': pitch}

    def status(self):
        fresh = self.fresh()
        yaw, pitch = self.relative() if self.raw is not None else (None, None)
        now = self.clock()
        hands = {hand: {'tracking': self.connection != 0 and now - sample['at'] <= FRESH_S, 'grip': sample['grip'],
                        'trigger': sample['trigger'], 'ui': sample['ui']} for hand, sample in self.hands.items()}
        return {'connected': self.connection != 0, 'device': self.device if self.connection else None,
                'tracking': fresh, 'poses': self.poses, 'age_s': None if self.at is None else now - self.at,
                'rate_hz': self._rate if fresh else None, 'yaw': yaw, 'pitch': pitch,
                'zero': {'yaw': self.zero_yaw, 'pitch': self.zero_pitch}, 'hands': hands, 'epoch': self.epoch}


def _solved(root):
    """The newest wrist calibration run under `root` with a solve: (its recommended model's cameras, the run's name),
    or None."""
    folder = Path(root).expanduser() if root else None
    if folder is None or not folder.is_dir():
        return None
    for result in sorted(folder.glob('*/result.json'), reverse=True):
        try:
            data = json.loads(result.read_text(encoding='utf-8'))
            return data['models'][data['recommended']]['cameras'], result.parent.name
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return None


def _matrix(rows):
    if not (isinstance(rows, list) and len(rows) == 4 and all(isinstance(row, list) and len(row) == 4 for row in rows)
            and all(_number(value) for row in rows for value in row)):
        raise ValueError('A mount is four rows of four numbers')
    return rows


def wrist_mounts(cameras, root=None):
    """Each wrist camera's mount for the headset: {role: {link, matrix (T_link_camera, four rows of four), fov_deg
    ([horizontal, vertical]), source}}, from the recommended model of the newest solved run under `root` (Helm's
    wrist calibration runs), else the nominal mount on the rig's link (`cameras`: the rig file's, vision.rig)."""
    solved = _solved(root)
    mounts = {}
    for role, camera in cameras.items():
        width, height = camera['size']
        horizontal = math.degrees(2 * math.atan(math.tan(math.radians(NOMINAL_FOV_Y_DEG) / 2) * width / height))
        mounts[role] = dict(link=camera['link'], matrix=[list(row) for row in NOMINAL_MOUNT],
                            fov_deg=[round(horizontal, 2), NOMINAL_FOV_Y_DEG], source='nominal')
        fitted = solved[0].get(role) if solved else None
        try:
            if fitted:
                view = fitted['field_of_view_deg']
                mounts[role] = dict(link=fitted.get('link') or camera['link'], matrix=_matrix(fitted['mount']['matrix']),
                                    fov_deg=[float(view['horizontal']), float(view['vertical'])],
                                    source=f'calibration {solved[1]}')
        except (KeyError, TypeError, ValueError):
            pass  # A solve without this camera's mount: the nominal stays.
    return mounts
