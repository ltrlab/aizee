"""A recorded episode's commands as the teleop loop's target, in place of the leaders (Helm's live replay).

The source gives the lifecycle what the leader mapper gives it: one row per actuator, its position in receiver motor
radians, the time it was sampled and a sequence that advances every cycle. The positions are the episode's recorded
receiver commands (`observations/qcmd`), taken by joint name in the receiver's order, interpolated linearly between
recorded frames and never extrapolated.

Its playback clock runs only while the lifecycle follows (TELEOP), at 0.25, 0.5 or 1x. Engage therefore first aligns
the arms to the current frame at the lifecycle's alignment speed (the first frame, or where a pause left off), and a
pause (HOLD) stops the clock. At the end the last frame holds; `finished_at` lets the loop disengage once the arms
have had a moment to reach it. Rewinding returns to the first frame. Everything the arms then do is the lifecycle's
and the receiver's: alignment and joint speed limits, the collision mode, Hold, Return and Disable.
"""

import math

SPEEDS = (0.25, 0.5, 1.0)


class EpisodeSource:
    def __init__(self, names, times, positions, *, speed=1.0, episode=None):
        import numpy as np

        if speed not in SPEEDS:
            raise ValueError(f'Replay runs at {", ".join(f"{s:g}x" for s in SPEEDS)}')
        times = np.asarray(times, dtype=np.float64)
        positions = np.asarray(positions, dtype=np.float64)
        if times.ndim != 1 or len(times) < 2 or positions.shape != (len(times), len(names)):
            raise ValueError('An episode needs at least two frames with a position for every joint')
        if not np.all(np.diff(times) > 0) or not np.all(np.isfinite(times)) or not np.all(np.isfinite(positions)):
            raise ValueError("The episode's times must increase and its positions be finite")
        self.names, self.episode, self.speed = tuple(names), episode, float(speed)
        self.times, self.positions = times - times[0], positions
        self.duration = float(self.times[-1])
        self.t, self.sequence = 0.0, 0
        self.finished_at = None  # The loop clock when playback reached the end.
        self._last = None  # The loop clock at the last following cycle.

    @classmethod
    def load(cls, path, names, *, speed=1.0, episode=None):
        """The source for `names` (the receiver's joints) from a complete task capture episode."""
        import h5py
        import numpy as np

        def text(value):
            return value.decode(errors='replace') if isinstance(value, bytes) else str(value)

        with h5py.File(path, 'r') as f:
            attrs = f.attrs
            if text(attrs.get('complete', False)) != 'True':
                raise ValueError('Only a complete episode can be replayed')
            joints = [name for name in text(attrs.get('minerva_joints', '')).split(',') if name]
            missing = [name for name in names if name not in joints]
            if missing:
                raise ValueError(f'The episode has no {", ".join(missing)}: it cannot drive this receiver')
            columns = [joints.index(name) for name in names]
            positions = np.asarray(f['observations/qcmd'][:], dtype=np.float64)[:, columns]
            times = np.asarray(f['timestamps/telem'][:], dtype=np.float64)
        return cls(names, times, positions, speed=speed, episode=episode)

    @property
    def finished(self):
        return self.finished_at is not None

    def rewind(self):
        self.t, self.finished_at, self._last = 0.0, None, None

    def position_at(self, t):
        import numpy as np

        t = min(max(t, 0.0), self.duration)
        i = int(np.searchsorted(self.times, t, side='right') - 1)
        if i >= len(self.times) - 1:
            return self.positions[-1]
        fraction = (t - self.times[i]) / (self.times[i + 1] - self.times[i])
        return self.positions[i] + fraction * (self.positions[i + 1] - self.positions[i])

    def target(self, state, now):
        """This cycle's target rows; the clock advances only between consecutive following (TELEOP) cycles."""
        if state == 'TELEOP' and self._last is not None and self.finished_at is None:
            self.t = min(self.duration, self.t + (now - self._last) * self.speed)
            if self.t >= self.duration:
                self.finished_at = now
        self._last = now if state == 'TELEOP' else None
        self.sequence += 1
        return [
            dict(name=name, position=float(q), sampled_at=now, sequence=self.sequence)
            for name, q in zip(self.names, self.position_at(self.t))
        ]

    def snapshot(self):
        import numpy as np

        frame = int(np.searchsorted(self.times, self.t, side='right') - 1)
        return dict(
            episode=self.episode,
            time_s=round(self.t, 3),
            duration_s=round(self.duration, 3),
            frame=min(frame, len(self.times) - 1),
            frames=len(self.times),
            speed=self.speed,
            finished=self.finished,
        )


def validate_request(payload):
    """Helm's replay request: {'episode': '<session>/<name>', 'speed': 0.25 | 0.5 | 1}."""
    if not isinstance(payload, dict) or set(payload) - {'episode', 'speed'} or 'episode' not in payload:
        raise ValueError('A replay names its episode, and optionally its speed')
    speed = payload.get('speed', 1.0)
    if type(speed) not in (int, float) or not math.isfinite(speed) or float(speed) not in SPEEDS:
        raise ValueError(f'Replay runs at {", ".join(f"{s:g}x" for s in SPEEDS)}')
    if not isinstance(payload['episode'], str) or payload['episode'].count('/') != 1:
        raise ValueError('An episode is named <session>/<name>')
    return {'kind': 'episode', 'episode': payload['episode'], 'speed': float(speed)}
