"""Operator presets for direct teleop, separate from measured servo calibration.

This module validates data and stores deliberate manual presets; it never opens
hardware or certifies a manually chosen gain as a measured calibration.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

FIELDS = {'motor', 'kp', 'kd', 'speed', 'follow', 'travel_rad'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def enabled(profile):
    return profile.get('manual_tuning') is True


def receiver_rows(derived, models, labels=None):
    """Explicit manual mode permits gains within each motor's encoding range.

    Position, measured speed, effort, stopping and tracking caps stay unchanged.
    Following speed remains bounded by the original derived lifecycle cap.
    """
    from robot_platform.control.commissioning import wire_ranges
    labels = labels or {}
    rows = []
    for joint, lifecycle in zip(derived['config']['joints'], derived['lifecycle']):
        name = joint['name']
        if name not in models:
            raise ValueError(f'{name}: a known motor model is required for manual tuning')
        wire = wire_ranges(models[name])
        joint['max_kp'], joint['max_kd'] = wire['kp'], wire['kd']
        rows.append(dict(motor=name, label=labels.get(name, name), kp=joint['hold_kp'], kd=joint['hold_kd'],
            speed=min(.05, lifecycle['max_velocity']), follow=name == 'left_arm_j1',
            travel_rad=.03 if name == 'left_arm_j1' else None,
            bounds={'kp': [0., joint['max_kp']], 'kd': [0., joint['max_kd']],
                    'speed': [.001, lifecycle['max_velocity']],
                    'travel_rad': [.001, lifecycle['upper'] - lifecycle['lower']]},
            limits={'feedback_speed': joint['max_velocity'], 'effort': joint['max_effort'],
                    'tracking_error': joint['max_tracking_error'], 'stop_acceleration': joint['stop_acceleration']}))
    return rows


def following(lifecycle, loop_s):
    """What sets a joint's top following speed, for the panel's estimate (teleop.lifecycle `_approach`): a
    command leads the joint's position, predicted to when it is computed, by at most `max_lead`, against the
    friction its servo sweep measured (0 when not measured). The drive sees it `command_lag_s` later on
    average (its way to the drive and half the loop it holds, about one loop, plus the receiver's
    interpolation), by when the joint has closed part of the lead. With velocity fed forward (the preset's switch)
    the drive's damping acts about the command's speed, not against it. A gripper's lead spans its whole range."""
    return dict(max_lead=lifecycle['max_lead'], friction_nm=lifecycle['friction_nm'],
                command_lag_s=loop_s + lifecycle['interpolation_s'], gripper=lifecycle.get('gripper') is True)


def values(rows):
    return [{key: row[key] for key in FIELDS} for row in rows]


def validate(joints, rows):
    if (not isinstance(joints, list) or len(joints) != len(rows)
            or any(not isinstance(row, dict) or set(row) != FIELDS for row in joints)
            or [row['motor'] for row in joints] != [row['motor'] for row in rows]):
        raise ValueError('Provide each motor exactly once in the displayed order')
    result = deepcopy(joints)
    for row, template in zip(result, rows):
        if type(row['follow']) is not bool:
            raise ValueError(f"{row['motor']}: Follow must be a boolean")
        for key in ('kp', 'kd', 'speed', 'travel_rad'):
            value = row[key]
            if key == 'travel_rad' and value is None:
                continue
            low, high = template['bounds'][key]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{row['motor']}: {key} must be between {low:g} and {high:g}")
            row[key] = float(value)
    return result


def install_initial(derived, settings):
    """Set initial gains before receiver construction; native caps were already fixed."""
    table = {row['motor']: row for row in settings}
    for joint, lifecycle in zip(derived['config']['joints'], derived['lifecycle']):
        selected = table[joint['name']]
        for key, source in (('hold_kp', 'kp'), ('hold_kd', 'kd')):
            joint[key] = lifecycle[key] = selected[source]


class ManualTuning:
    def __init__(self, helm, profile, runs, servo):
        from robot_platform.control.commissioning import motor_models
        from robot_platform.control.guarded_config import derive_receiver, fit_motor_ranges
        derived = derive_receiver(reference=helm.reference, steps=helm.steps, directions=helm.mapping,
            transmissions=helm.transmissions, model=helm.model, limits=helm.saved_limits,
            profile=profile, backend=None, servo=servo)
        models = motor_models(helm.device, Path(helm.profile_path).resolve().parents[2])
        from .teleop import loop_period_s
        self.rows = receiver_rows(derived, models, helm.labels)
        for row, lifecycle in zip(self.rows, derived['lifecycle']):
            row['following'] = following(lifecycle, loop_period_s(profile))
        fit_motor_ranges(derived['config']['joints'], models)
        law = ('direct-velocity-feedforward' if any(j.get('velocity_feedforward') for j in derived['lifecycle'])
               else 'direct-position-only')
        self.binding = dict(asset=helm.asset_id, profile=profile['id'], control=law,
            calibration=helm.bindings, profile_sha256=digest(profile), servo_sha256=digest(servo),
            motors=[{k: motor[k] for k in ('name', 'uid', 'firmware')} for motor in helm.reference['motors']])
        self.path = Path(runs) / (helm.asset_id + '-teleop') / (profile['id'] + '-manual.json')
        self.session_id = str(uuid.uuid4())
        self.revision = 0
        self.saved = self._load()
        self.active = deepcopy(self.saved['joints']) if self.saved else values(self.rows)
        # The operator's velocity feed-forward switch, taken at the next session start (helm.teleop TeleopCore):
        # off is position-only. Saved with the gains; a preset saved before it had the switch is off.
        self.feedforward = bool(self.saved) and self.saved.get('velocity_feedforward') is True

    def _load(self):
        if not self.path.is_file():
            return None
        document = json.loads(self.path.read_text(encoding='utf-8'))
        if (not isinstance(document, dict) or type(document.get('schema')) is not int
                or document['schema'] != 1 or document.get('kind') != 'manual_teleop_tuning'
                or document.get('binding') != self.binding or document.get('measured_calibration') is not False):
            raise ValueError('Saved manual tuning belongs to another profile, calibration or motor; review it before use')
        if type(document.get('velocity_feedforward', False)) is not bool:
            raise ValueError('Saved manual tuning has an invalid velocity feed-forward switch')
        document['joints'] = validate(document.get('joints'), self.rows)
        return document

    def check_revision(self, session_id, revision):
        if session_id != self.session_id or type(revision) is not int or revision != self.revision:
            raise ValueError('Tuning changed or the session restarted. Revert edits to review current settings.')

    def bundle(self):
        return dict(session_id=self.session_id, revision=self.revision, joints=deepcopy(self.active),
                    velocity_feedforward=self.feedforward)

    def new_session(self):
        self.session_id = str(uuid.uuid4())

    def accept(self, candidate, feedforward=None):
        """Take validated gains and, between sessions, the velocity feed-forward switch (None: unchanged)."""
        if feedforward is not None and type(feedforward) is not bool:
            raise ValueError('Velocity feed-forward is on or off')
        self.active = validate(candidate, self.rows)
        if feedforward is not None:
            self.feedforward = feedforward
        self.revision += 1
        return self.bundle()

    def save(self):
        document = dict(schema=1, kind='manual_teleop_tuning', measured_calibration=False,
            provenance='operator-selected values; not an automatic servo sweep result', binding=self.binding,
            updated_at=datetime.now(timezone.utc).isoformat(), joints=deepcopy(self.active),
            velocity_feedforward=self.feedforward)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.path)
        self.saved = document

    def snapshot(self, *, can_apply, reason):
        rows = [dict(template, **row) for template, row in zip(self.rows, self.active)]
        return dict(enabled=True, session_id=self.session_id, revision=self.revision,
            saved_revision=self.saved.get('updated_at') if self.saved else None,
            dirty=(self.saved is None or self.active != self.saved['joints']
                   or self.feedforward != (self.saved.get('velocity_feedforward') is True)),
            can_apply=can_apply, can_save=can_apply, reason=reason, rows=rows, velocity_feedforward=self.feedforward,
            control='direct-velocity-feedforward' if self.feedforward else 'direct-position-only')
