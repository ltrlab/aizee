"""Bound operator settings and a searchable catalog; never opens hardware.

Profile definitions, measured calibration and receiver hard limits remain separate
from this stopped-session overlay. Native settings are always derived again from
the validated effective profile, never patched on an existing receiver.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import uuid

from .manual_tuning import digest

JOINT_FIELDS = ('max_velocity', 'align_speed', 'shutdown_speed')
SOURCE_DEFAULTS = dict(feedback_max_age_s=.03, target_max_age_s=.05, max_skew_s=.02,
                       max_step_s=.08, settle_time_s=.1)
RECEIVER_FIELDS = ('lease_ms', 'feedback_timeout_ms', 'settled_velocity',
                   'direct.command_ms', 'direct.velocity_timeout_ms')
RETURN_BUDGET_S = 40.
STOP_TIMEOUT_S = 45.


def _get(value, path):
    for key in path.split('.'):
        value = value[key]
    return value


def _put(value, path, replacement):
    parts = path.split('.')
    for key in parts[:-1]:
        value = value[key]
    value[parts[-1]] = replacement


def _number(value, low, high, label, *, integer=False):
    if (type(value) not in ((int,) if integer else (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f'{label} must be {"an integer " if integer else ""}between {low:g} and {high:g}')
    return value if integer else float(value)


def _source_defaults(profile):
    from .teleop import loop_period_s, source_misses
    return dict(SOURCE_DEFAULTS, max_source_misses=source_misses(loop_period_s(profile)))


def _header(profile, payload):
    if (not isinstance(payload, dict)
            or set(payload) != {'schema', 'revision', 'profile_sha256', 'joints', 'source', 'receiver'}
            or type(payload['schema']) is not int or payload['schema'] != 1
            or type(payload['revision']) is not int or payload['revision'] < 0
            or payload['profile_sha256'] != digest(profile)):
        raise ValueError('Operator settings must match this exact base profile and schema')


def resolve_profile(base_profile, payload):
    """Return a validated effective profile before calling derive_receiver()."""
    from .teleop import loop_period_s, validate_profile
    _header(base_profile, payload)
    if base_profile['receiver'].get('collision_mode') != 'disabled':
        raise ValueError('Editable operator settings currently require the direct teleop profile')
    receiver = payload['receiver']
    if not isinstance(receiver, dict):
        raise ValueError('Receiver settings must be a complete object')
    baseline = base_profile['receiver']
    checked = deepcopy(baseline)
    period_ms = round(loop_period_s(base_profile) * 1000)
    for path in RECEIVER_FIELDS:
        try:
            old, value = _get(baseline, path), _get(receiver, path)
        except (KeyError, TypeError):
            raise ValueError('Provide the complete receiver settings') from None
        minimum = 2 * period_ms + 1 if path == 'direct.command_ms' else (
            max(1, baseline.get('interpolation_ms', 0)) if path == 'direct.velocity_timeout_ms' else
            .001 if path == 'settled_velocity' else 1)
        _put(checked, path, _number(value, minimum, old, 'receiver.' + path,
                                   integer=path != 'settled_velocity'))
    if receiver != checked:
        raise ValueError('Receiver hard limits, mode, loop rate and braking policy are read-only')
    profile = deepcopy(base_profile)
    profile['receiver'] = checked
    validate_profile(profile)
    return profile


def _shutdown_min(joint, report, settle_time):
    from robot_platform.teleop.lifecycle import SETTLE_WINDOW_S
    usable = report.get('usable', (joint['lower'], joint['upper']))
    distance = max(abs(endpoint - joint['zero_position']) for endpoint in usable)
    return max(.001, distance / (RETURN_BUDGET_S - settle_time - SETTLE_WINDOW_S))


def validate_bundle(base_profile, derived, payload, manual=None):
    """Validate an overlay against freshly derived limits, returning owned data.

    ``derived`` may contain one arm; the bundle still names the full profile
    roster. Call resolve_profile first and derive from its result. The returned
    lifecycle rows precede configure_manual(), which retains its existing caps.
    """
    from .teleop import loop_period_s
    from robot_platform.teleop.lifecycle import TeleopLifecycle
    profile = resolve_profile(base_profile, payload)
    source = payload['source']
    defaults = _source_defaults(base_profile)
    if not isinstance(source, dict) or set(source) != set(defaults):
        raise ValueError('Provide the complete source freshness and settling settings')
    source = deepcopy(source)
    for key, default in defaults.items():
        low, high = (default, 5.) if key == 'settle_time_s' else (
            (0, default) if key == 'max_source_misses' else
            (loop_period_s(profile), default) if key == 'max_step_s' else (1e-6, default))
        source[key] = _number(source[key], low, high, 'source.' + key,
                              integer=key == 'max_source_misses')
    if source['feedback_max_age_s'] * 1000 > profile['receiver']['feedback_timeout_ms'] + 1e-9:
        raise ValueError('Lifecycle feedback age cannot exceed the receiver feedback timeout')
    roster = list(base_profile['routes'])
    rows = payload['joints']
    if (not isinstance(rows, list) or len(rows) != len(roster)
            or any(not isinstance(row, dict) or set(row) != {'motor', *JOINT_FIELDS} for row in rows)
            or [row['motor'] for row in rows] != roster):
        raise ValueError('Provide each profile motor exactly once in profile order')
    table = {row['motor']: row for row in rows}
    lifecycle = deepcopy(derived['lifecycle'])
    if not lifecycle or any(joint['name'] not in table for joint in lifecycle):
        raise ValueError('Derived receiver joints do not match the operator settings')
    reports = {row['motor']: row for row in derived.get('report', [])}
    manual_rows = manual.active if hasattr(manual, 'active') else manual
    if isinstance(manual_rows, dict):
        manual_rows = manual_rows.get('joints')
    manual_table = {row['motor']: row for row in manual_rows or []}
    for joint in lifecycle:
        name = joint['name']
        row = table[name]
        for key in JOINT_FIELDS:
            # The return must reach the reference within its budget, and the ceiling bounds the return speed.
            low = (_shutdown_min(joint, reports.get(name, {}), source['settle_time_s'])
                   if key in ('shutdown_speed', 'max_velocity') else .001)
            joint[key] = _number(row[key], low, joint[key], name + '.' + key)
        if max(joint['align_speed'], joint['shutdown_speed']) > joint['max_velocity']:
            raise ValueError(f"{name}: its alignment and return speeds ({joint['align_speed']:g} and "
                             f"{joint['shutdown_speed']:g} rad/s) must not exceed its following ceiling "
                             f"({joint['max_velocity']:g} rad/s); lower them with it")
        if name in manual_table and manual_table[name]['speed'] > joint['max_velocity']:
            raise ValueError(f'{name}: reduce the current manual following speed before lowering its ceiling')
    TeleopLifecycle(lifecycle, **source, allow_repeated_source=True, trial=profile.get('trial'))
    return lifecycle, source


def _leaves(value, prefix=''):
    if isinstance(value, dict) and value:
        for key, item in value.items():
            yield from _leaves(item, f'{prefix}.{key}' if prefix else str(key))
    elif isinstance(value, list) and value:
        for index, item in enumerate(value):
            yield from _leaves(item, f'{prefix}.{index}')
    else:
        yield prefix, deepcopy(value)


def _unit(path):
    key = path.rsplit('.', 1)[-1]
    if key.endswith('_ms'): return 'ms'
    if key.endswith('_us'): return 'µs'
    if key.endswith('_s') or key == 'reaction_s': return 's'
    if key.endswith('_hz'): return 'Hz'
    if key in ('max_velocity', 'align_speed', 'shutdown_speed', 'settled_velocity', 'speed', 'velocity'): return 'rad/s'
    if key == 'stop_acceleration': return 'rad/s²'
    if key in ('hold_kp', 'max_kp', 'kp'): return 'N·m/rad'
    if key in ('hold_kd', 'max_kd', 'kd'): return 'N·m·s/rad'
    if key in ('max_effort', 'friction_nm'): return 'N·m'
    if key in ('max_temperature', 'temperature'): return '°C'
    if key.endswith('_rad') or key in ('lower', 'upper', 'zero_position', 'max_lead', 'tolerance', 'max_tracking_error', 'rest_tracking_error'): return 'rad'
    return ''


def _field(identifier, value, default, *, label=None, source='profile', configurable=False,
           can_apply=False, bounds=None, effect='read_only', reason='', description='', workflow=None):
    kind = 'boolean' if type(value) is bool else 'integer' if type(value) is int else 'number' if type(value) is float else 'string'
    key = identifier.rsplit('.', 1)[-1]
    for suffix in ('_ms', '_us', '_s', '_hz', '_rad', '_nm'):  # The unit is shown beside the label.
        if key.endswith(suffix) and len(key) > len(suffix):
            key = key[:-len(suffix)]
            break
    return dict(id=identifier, label=label or key.replace('_', ' ').capitalize(),
                value=deepcopy(value), default=deepcopy(default), source=source, unit=_unit(identifier),
                type=kind, configurable=configurable, editable=configurable and can_apply,
                effect=effect, bounds=bounds, reason=reason, description=description, workflow=workflow)


class OperatorSettings:
    """Profile-bound, atomic stopped-session persistence; caller serializes Start."""
    def __init__(self, helm, profile, runs, servo, manual=None):
        self.helm, self.profile, self.servo = helm, deepcopy(profile), deepcopy(servo)
        self.manual = manual
        self._lock = threading.RLock()
        self.session_id, self.revision = str(uuid.uuid4()), 0
        self.enabled = profile['receiver'].get('collision_mode') == 'disabled'
        self.binding = dict(asset=helm.asset_id, profile=profile['id'], profile_sha256=digest(profile),
            calibration=deepcopy(helm.bindings), servo_sha256=digest(servo),
            limits_sha256=digest(helm.saved_limits), directions_sha256=digest(helm.mapping),
            motors=[{key: motor[key] for key in ('name', 'uid', 'firmware')} for motor in helm.reference['motors']])
        self.path = Path(runs) / (helm.asset_id + '-teleop') / (profile['id'] + '-settings.json')
        self.saved_at = None
        self.base_derived = self._derive(profile) if self.enabled else None
        if self.enabled:
            table = {row['name']: row for row in self.base_derived['lifecycle']}
            self.defaults = dict(schema=1, revision=0, profile_sha256=digest(profile),
                joints=[dict(motor=name, **{key: table[name][key] for key in JOINT_FIELDS}) for name in profile['routes']],
                source=_source_defaults(profile), receiver=deepcopy(profile['receiver']))
            self.active = deepcopy(self.defaults)
            if self.path.is_file():
                document = json.loads(self.path.read_text(encoding='utf-8'))
                if (not isinstance(document, dict) or document.get('kind') != 'operator_teleop_settings'
                        or type(document.get('schema')) is not int or document['schema'] != 1
                        or document.get('measured_calibration') is not False or document.get('binding') != self.binding):
                    raise ValueError('Saved operator settings belong to another profile, calibration, limits or motor')
                self._validate(document.get('operator_settings'), manual)
                self.active = deepcopy(document['operator_settings'])
                self.revision = self.active['revision']
                self.saved_at = document.get('updated_at')
        else:
            self.defaults = self.active = None

    def _derive(self, profile):
        from robot_platform.control.guarded_config import derive_receiver, fit_motor_ranges
        from robot_platform.control.commissioning import motor_models
        from .manual_tuning import receiver_rows
        helm = self.helm
        derived = derive_receiver(reference=helm.reference, steps=helm.steps, directions=helm.mapping,
            transmissions=helm.transmissions, model=helm.model, limits=helm.saved_limits,
            profile=profile, backend=None, servo=self.servo)
        models = motor_models(helm.device, Path(helm.profile_path).resolve().parents[2])
        if profile.get('manual_tuning'):
            receiver_rows(derived, models, helm.labels)
        fit_motor_ranges(derived['config']['joints'], models)
        return derived

    def _validate(self, candidate, manual=None):
        profile = resolve_profile(self.profile, candidate)
        derived = self._derive(profile)
        return validate_bundle(self.profile, derived, candidate, self.manual if manual is None else manual)

    def bundle(self):
        with self._lock:
            return deepcopy(self.active)

    def new_session(self):
        with self._lock:
            self.session_id = str(uuid.uuid4())

    def validate_manual(self, rows):
        """Run after manual schema validation, before ACK or preset acceptance. A motor without an operator
        ceiling (no route in the profile) keeps only the receiver bound that manual validation applies."""
        if self.active is None:
            return
        caps = {row['motor']: row['max_velocity'] for row in self.active['joints']}
        for row in rows:
            if row['motor'] in caps and row['speed'] > caps[row['motor']]:
                raise ValueError(f"{row['motor']}: manual speed exceeds its operator following ceiling")

    def _edits(self, candidate):
        result = {}
        for index, row in enumerate(candidate['joints']):
            for key in JOINT_FIELDS:
                result[f"joint.{row['motor']}.{key}"] = (row, key)
        for key in candidate['source']:
            result['source.' + key] = (candidate['source'], key)
        for key in RECEIVER_FIELDS:
            container = candidate['receiver']['direct'] if key.startswith('direct.') else candidate['receiver']
            result['receiver.' + key] = (container, key.rsplit('.', 1)[-1])
        return result

    def update(self, action, payload, *, running=False, manual=None):
        with self._lock:
            if running:
                raise ValueError('Stop the teleop session before changing session settings')
            if not self.enabled:
                raise ValueError('This profile exposes a read-only catalog; operator edits require direct teleop')
            expected = {'session_id', 'revision', 'values'} if action == 'apply' else {'session_id', 'revision'}
            if action not in ('apply', 'reset') or not isinstance(payload, dict) or set(payload) != expected:
                raise ValueError('Expected settings action, session_id, revision and apply values')
            if (payload['session_id'] != self.session_id or type(payload['revision']) is not int
                    or payload['revision'] != self.revision):
                raise ValueError('Settings changed or the session restarted; reload current settings')
            candidate = deepcopy(self.active if action == 'apply' else self.defaults)
            if action == 'apply':
                if not isinstance(payload['values'], dict) or not payload['values']:
                    raise ValueError('Provide a nonempty object of settings values')
                slots = self._edits(candidate)
                for key, value in payload['values'].items():
                    if key not in slots:
                        raise ValueError(f'{key}: unknown or read-only setting; use its linked workflow')
                    container, field = slots[key]
                    container[field] = value
            candidate['revision'] = self.revision + 1
            self._validate(candidate, manual)
            updated_at = datetime.now(timezone.utc).isoformat()
            document = dict(schema=1, kind='operator_teleop_settings', measured_calibration=False,
                provenance='operator-selected session settings; not measured calibration',
                binding=self.binding, updated_at=updated_at, operator_settings=candidate)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(document, indent=2, allow_nan=False) + '\n', encoding='utf-8')
            temporary.replace(self.path)
            self.active, self.revision, self.saved_at = candidate, candidate['revision'], updated_at
            return self.bundle()

    def snapshot(self, *, can_apply, reason='', manual=None):
        with self._lock:
            can_apply = bool(can_apply and self.enabled)
            if not self.enabled:
                reason = 'This profile is read-only; operator session edits require direct teleop'
            sections = []
            effective = resolve_profile(self.profile, self.active) if self.active else self.profile
            manual_rows = manual if manual is not None else self.manual
            if hasattr(manual_rows, 'active'): manual_rows = manual_rows.active
            if isinstance(manual_rows, dict): manual_rows = manual_rows.get('joints')
            manual_table = {row['motor']: row for row in manual_rows or []}
            if self.active:
                reports = {row['motor']: row for row in self.base_derived.get('report', [])}
                base_joints = {row['name']: row for row in self.base_derived['lifecycle']}
                fields = []
                for row in self.active['joints']:
                    name, base = row['motor'], base_joints[row['motor']]
                    for key in JOINT_FIELDS:
                        low = (_shutdown_min(base, reports.get(name, {}), self.active['source']['settle_time_s'])
                               if key in ('shutdown_speed', 'max_velocity') else .001)
                        descriptions = {'max_velocity': 'Ceiling for manual following speed. Native overspeed and braking limits stay fixed.',
                            'align_speed': 'Alignment ceiling. Effective alignment is the smaller of this value and the manual following speed.',
                            'shutdown_speed': 'Planned return speed, independent of manual following speed. Explicit Disable and source-loss holds remain immediate.'}
                        fields.append(_field(f'joint.{name}.{key}', row[key], base[key],
                            label=f'{self.helm.labels.get(name, name)} · {key.replace("_", " ")}',
                            source='operator' if row[key] != base[key] else 'derived profile / servo',
                            configurable=True, can_apply=can_apply, bounds=[low, base[key]],
                            effect='next_session', reason=reason, description=descriptions[key]))
                    if name in manual_table:
                        manual_row = manual_table[name]
                        fields.append(_field(f'manual.{name}.effective_alignment_speed', min(row['align_speed'], manual_row['speed']),
                            min(base['align_speed'], manual_row['speed']), source='resolved manual settings',
                            label=f'{self.helm.labels.get(name, name)} · effective alignment',
                            reason='Minimum of the alignment ceiling and current manual following speed.', workflow='teleop'))
                        fields[-1]['unit'] = 'rad/s'
                        for key in ('kp', 'kd', 'speed', 'follow', 'travel_rad'):
                            fields.append(_field(f'manual.{name}.{key}', manual_row[key], None,
                                label=f'{self.helm.labels.get(name, name)} · manual {key}', source='manual tuning',
                                effect='hold_receiver_ack', reason='Use manual tuning: Hold → Apply → Engage; receiver acknowledgement is required.',
                                workflow={'tab': 'teleop', 'label': 'Manual tuning'}))
                sections.append(dict(id='motion', label='Motion and manual tuning', fields=fields))
                source_fields = []
                for key, value in self.active['source'].items():
                    default = self.defaults['source'][key]
                    low, high = (default, 5.) if key == 'settle_time_s' else (
                        (0, default) if key == 'max_source_misses' else
                        (1 / effective['receiver']['loop_hz'], default) if key == 'max_step_s' else (1e-6, default))
                    source_fields.append(_field('source.' + key, value, default, configurable=True,
                        can_apply=can_apply, bounds=[low, high], effect='next_session', reason=reason,
                        source='operator' if value != default else 'lifecycle default',
                        description='Freshness and miss bounds may only tighten; settling time may only increase.'))
                sections.append(dict(id='source', label='Source freshness and settling', fields=source_fields))
                receiver_fields = []
                for key, value in _leaves(effective['receiver']):
                    default = _get(self.profile['receiver'], key)
                    editable = key in RECEIVER_FIELDS
                    minimum = 2 * round(1000 / effective['receiver']['loop_hz']) + 1 if key == 'direct.command_ms' else (
                        max(1, effective['receiver'].get('interpolation_ms', 0)) if key == 'direct.velocity_timeout_ms' else .001 if key == 'settled_velocity' else 1)
                    receiver_fields.append(_field('receiver.' + key, value, default, configurable=editable,
                        can_apply=can_apply, bounds=[minimum, default] if editable else None,
                        effect='next_session' if editable else 'read_only',
                        source='operator' if value != default else 'profile',
                        reason=reason if editable else 'Qualified receiver policy; changing it requires a profile change and qualification.',
                        description='A smaller value tightens the bound. A new receiver is built on the next session.' if editable else ''))
                sections.append(dict(id='receiver', label='Receiver timing and policy', fields=receiver_fields))
                hard_fields = []
                for joint in self.base_derived['config']['joints']:
                    for key, value in _leaves(joint):
                        hard_fields.append(_field(f'guard.{joint["name"]}.{key}', value, value, source='derived receiver / motor encoding',
                            label=f'{self.helm.labels.get(joint["name"], joint["name"])} · {key}',
                            reason='Receiver hard limit. Braking, acceleration, tracking, geometry and encoded motor limits need their existing qualification workflow.',
                            workflow='tune' if key in ('stop_acceleration', 'max_tracking_error') else 'limits'))
                sections.append(dict(id='guards', label='Motor limits and braking guards', fields=hard_fields))
            profile_fields = []
            for path, value in _leaves(self.profile):
                workflow = 'teleop' if path.startswith('routes.') else 'leaders' if path.startswith('leader') else 'settings'
                profile_fields.append(_field('profile.' + path, value, value, label=path,
                    reason='Base profile definition. Resolved operating values appear above; route directions and calibration use their existing workflows.', workflow=workflow))
            sections.append(dict(id='profile', label='Complete base profile', fields=profile_fields))
            bindings = []
            for path, value in _leaves(self.binding):
                bindings.append(_field('binding.' + path, value, value, source='recorded identity / calibration',
                    label=path, reason='Measured calibration and hardware identity cannot be edited as operator settings.', workflow='check'))
            sections.append(dict(id='binding', label='Hardware and calibration binding', fields=bindings))
            from .teleop import CYCLE_BUDGET_S, RECORDING_BYTES, RECORDING_FLUSH_S
            from robot_platform.teleop.lifecycle import SETTLE_WINDOW_S
            constants = {'cycle_budget_s': CYCLE_BUDGET_S, 'stop_timeout_s': STOP_TIMEOUT_S,
                         'return_budget_s': RETURN_BUDGET_S, 'settle_window_s': SETTLE_WINDOW_S,
                         'recording_bytes': RECORDING_BYTES, 'recording_flush_s': RECORDING_FLUSH_S}
            sections.append(dict(id='runtime', label='Fixed runtime guards', fields=[
                _field('runtime.' + key, value, value, source='runtime contract',
                    reason='Fixed control/recording contract; changing it requires code review and timing qualification.')
                for key, value in constants.items()]))
            return dict(enabled=True, session_id=self.session_id, revision=self.revision, can_apply=can_apply,
                reason=reason, saved_at=self.saved_at, path=str(self.path), sections=sections)
