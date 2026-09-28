"""The servo calibration: per joint, the settings the live servo sweep measured and the operator saved.

It lives next to the joint limits (config/devices/<asset>-servo.json). Each joint
is bound to its motor's UID and firmware; guarded_config.derive_receiver takes
that joint's gains, speed, braking and tracking bound from it, and refuses a
joint whose motor has changed since.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path

from .guarded_config import SERVO_KEYS


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def servo_path(profile_path, asset_id):
    """The servo calibration, next to the calibration profile and joint limits."""
    return Path(profile_path).resolve().parent / f'{asset_id}-servo.json'


def load_servo(path):
    path = Path(path)
    if not path.is_file():
        return None
    document = json.loads(path.read_text(encoding='utf-8'))
    if document.get('schema') != 1 or document.get('kind') != 'servo_calibration':
        raise ValueError(f'{path.name} is not a servo calibration')
    return document


def evidence_name(session):
    """The sweep's evidence directory as the robot's minerva tree names it, without its home directory:
    the calibration is kept in the public repository."""
    text = str(session)
    marker = '/minerva/evidence/'
    return '~' + text[text.index(marker):] if marker in text else text


def save_servo(path, *, asset_id, reference, results, motors, session, existing=None):
    """Write the confirmed joints' recommended settings, bound to their motor identities."""
    roster = {motor['name']: motor for motor in reference['motors']}
    document = deepcopy(existing) if existing else {'schema': 1, 'kind': 'servo_calibration', 'joints': {}}
    document.update(asset_id=asset_id, reference_id=reference['id'], updated_at=utc_now())
    for motor in motors:
        result = results.get(motor) or {}
        recommendation = result.get('recommendation')
        if not recommendation or not recommendation.get('gains_met'):
            raise ValueError(f'{motor}: no recommended settings to save')
        if not (result.get('verification') or {}).get('passed'):
            raise ValueError(f'{motor}: the verification pass did not pass')
        entry = {'uid': roster[motor]['uid'], 'firmware': roster[motor]['firmware'],
                 'confirmed_at': utc_now(), 'session': evidence_name(session),
                 'measured': {'torque': result.get('torque'), 'verified': result.get('verified'),
                              'verification': result.get('verification')},
                 'notes': recommendation.get('notes', [])}
        entry.update({key: recommendation[key] for key in SERVO_KEYS if recommendation.get(key) is not None})
        document['joints'][motor] = entry
    Path(path).write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
    return document


def coverage(document, reference):
    """(calibrated, missing, changed) motor names: changed = swept with another motor or firmware."""
    joints = (document or {}).get('joints') or {}
    calibrated, missing, changed = [], [], []
    for motor in reference['motors']:
        entry = joints.get(motor['name'])
        if entry is None:
            missing.append(motor['name'])
        elif entry.get('uid') != motor.get('uid') or entry.get('firmware') != motor.get('firmware'):
            changed.append(motor['name'])
        else:
            calibrated.append(motor['name'])
    return calibrated, missing, changed
