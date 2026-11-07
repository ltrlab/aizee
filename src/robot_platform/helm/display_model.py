"""The model Helm's 3D view draws: the head model (urdf/minerva/onshape-to-robot-head-v1), Minerva with her two-joint
neck, so the view shows the head.

The view only draws it. The guard, the receivers and every guide keep the active model, whose collision model has no
head yet (docs/robots/minerva/HEAD_NECK_MODEL.md: that is the motion-contract owner's). Each guide (the stopped
reader's, a session's, a sweep's) is drawn on the head model by joint name: the arms as the guide has them, the neck
at the head's measured angles while a session drives it, else at its zero. The joint telemetry the page hands the
view is stamped for the head model the same way: its rows name joints both models have.
"""
import math

ORIGIN_M = 1e-5  # The head export's arm frames match the active model's within 0.005 mm (HEAD_NECK_MODEL.md).
ROTATION = 1e-5


class DisplayModel:
    """`active`, `shown`: UrdfModels. The shown model is the active one plus joints of its own: every active joint
    with the same type, links, axis, frame, bounds and coupling, so a pose on one is the same pose on the other."""

    def __init__(self, active, shown):
        differ = [name for name, joint in active.by_name.items() if not _same(joint, shown.by_name.get(name))]
        if differ:
            raise ValueError(f"{shown.profile.title} does not extend {active.profile.title}: "
                             f"{', '.join(differ[:4])}{' …' if len(differ) > 4 else ''} differ")
        self.active, self.shown = active, shown
        self.extra = tuple(name for name in shown.independent if name not in active.by_name)

    @classmethod
    def load(cls, active, path, *, log=print):
        """The shown model at `path` for `active`; None (the view draws the active model) when it is missing, does not
        extend it or cannot be read: drawing the head never stops Helm."""
        from robot_platform.models import ModelProfile, UrdfModel
        try:
            return cls(active, UrdfModel(ModelProfile.load(path)))
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            log(f'3D view: drawing the active model, without the head ({exc})')
            return None

    @property
    def profile(self):
        return self.shown.profile

    def _active(self, value):
        return isinstance(value, dict) and value.get('model') == self.active.profile.id \
            and value.get('urdf_sha256') == self.active.sha256

    def guide(self, pose, joints=None):
        """`pose`, a guide for the active model, drawn on the shown model: its own joints at `joints` ({name: rad},
        within their bounds) or at their defaults. Anything else (None, another model's guide) passes unchanged."""
        if not self._active(pose):
            return pose
        own = {}
        for name in self.extra:
            joint, value = self.shown.by_name[name], (joints or {}).get(name)
            if type(value) not in (int, float) or not math.isfinite(value):
                value = joint['default']
            low, high = joint['bounds'] or (-math.inf, math.inf)
            own[name] = min(max(float(value), low), high)
        return dict(pose, model=self.shown.profile.id, urdf_sha256=self.shown.sha256,
                    positions={**pose['positions'], **own})

    def preview(self, pose, joints, sequence, *, source='headset', note='headset preview, no motors move'):
        """`pose` drawn with its own joints at `joints` (None: at their defaults): where the headset looks, or the
        head setup's readings (`source`, `note` for the badge). Each of them (`sequence`, its count) is a pose of its
        own to the view, which applies a pose once per stream and sequence, and keeps its framing for it."""
        drawn = self.guide(pose, joints)
        if drawn is pose:
            return pose
        badge = drawn.get('badge') or ('LIVE · motors disabled' if drawn.get('live') else 'Target pose')
        return dict(drawn, stream_id=f"{drawn.get('stream_id') or 'guide'}+{source}{sequence}", view='keep',
                    badge=f'{badge} · head: {note}')

    def telemetry(self, packet):
        """A joint telemetry packet (helm.telemetry) for the active model, stamped for the shown one."""
        return dict(packet, model=self.shown.profile.id, urdf_sha256=self.shown.sha256) if self._active(packet) \
            else packet


def _same(joint, other):
    if other is None or any(joint[key] != other[key] for key in ('type', 'parent', 'child', 'axis', 'bounds', 'mimic')):
        return False
    if any(abs(a - b) > ORIGIN_M for a, b in zip(joint['origin']['xyz'], other['origin']['xyz'])):
        return False
    # Compared as rotations: the export writes some angles as -pi where the active model has pi.
    return all(abs(a - b) <= ROTATION for a, b in zip(_rotation(joint['origin']['rpy']), _rotation(other['origin']['rpy'])))


def _rotation(rpy):
    """URDF's fixed-axis roll, pitch, yaw as a row-major rotation matrix (Rz(yaw) Ry(pitch) Rx(roll))."""
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr)
