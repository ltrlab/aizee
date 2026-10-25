"""The operator's headset steering the head (helm.head_teleop): a WebXR page (static/xr.html), open in the browser of
a Meta Quest Pro, streams the headset's orientation to Helm, which turns it into the head's yaw and pitch from a zero
the operator sets, for the running teleop session.

A pose is the headset's orientation in WebXR's 'local' space as a unit quaternion (x, y, z, w): Y is up (the space is
level) and -Z is forward where the XR session started. Yaw turns about Y, left positive; pitch is the gaze's
elevation, up positive; roll is ignored. The zero starts as that heading, level (a new XR session, or a recenter in
the headset, starts it again), and moves to wherever the operator looks when they zero. One headset steers at a time:
another device's is refused while it is connected; the same device reconnecting takes its stream back.
"""
import math
import time

MAX_RATE_HZ = 120.  # Poses beyond this rate are dropped (a page renders at 72-90 Hz and sends at most 60).
FRESH_S = .25  # A pose this recent counts as the headset tracking.


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

    def _current(self, connection):
        if connection is not None and connection != self.connection:
            raise ValueError('Another connection is steering the head')

    def receive(self, connection, message):
        """A pose message ({'type': 'pose', 'seq': n, 'q': [x, y, z, w]}) from `connection`: (yaw, pitch) from the
        zero, or None for one that is dropped (another connection's, an old sequence, too fast)."""
        if connection != self.connection or not isinstance(message, dict) or message.get('type') != 'pose':
            return None
        sequence = message.get('seq')
        if type(sequence) is not int or sequence < 1 or (self.sequence is not None and sequence <= self.sequence):
            return None
        yaw, pitch = yaw_pitch(message.get('q'))
        now = self.clock()
        if self.at is not None and now - self.at < 1 / MAX_RATE_HZ:
            return None
        if self.at is not None and now > self.at:
            rate = 1 / (now - self.at)
            self._rate = rate if self._rate is None else .9 * self._rate + .1 * rate
        self.raw, self.at, self.sequence = (yaw, pitch), now, sequence
        self.poses += 1
        return self.relative()

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

    def status(self):
        fresh = self.fresh()
        yaw, pitch = self.relative() if self.raw is not None else (None, None)
        return {'connected': self.connection != 0, 'device': self.device if self.connection else None,
                'tracking': fresh, 'poses': self.poses, 'age_s': None if self.at is None else self.clock() - self.at,
                'rate_hz': self._rate if fresh else None, 'yaw': yaw, 'pitch': pitch,
                'zero': {'yaw': self.zero_yaw, 'pitch': self.zero_pitch}}
