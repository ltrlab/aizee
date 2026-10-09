"""One robot in Helm: profile, mechanical reference, direction candidates and limits.

Reuses the stopped-only calibration plan, validation and direction-edit rules.
Nothing here opens hardware; the link delivers verified stopped-reader frames.
"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from robot_platform.control.joint_calibration import (digest, edit_preview, live_positions, load_plan,
    preview_transmission_row, save_json, seed_results, validate_preview_transmissions)
from robot_platform.ops.ssh import TETHER
from .limits import LimitRecorder, load_limits, utc_now


def process_alive(pid):
    """True when the process exists (or cannot be inspected). Never signals it."""
    if pid <= 0:
        return False
    if os.name == 'nt':
        import ctypes
        kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel32.OpenProcess.restype = ctypes.c_void_p
        kernel32.OpenProcess.argtypes = (ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong)
        kernel32.GetExitCodeProcess.argtypes = (ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong))
        kernel32.CloseHandle.argtypes = (ctypes.c_void_p,)
        handle = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return ctypes.get_last_error() == 5  # Access denied: it exists.
        try:
            code = ctypes.c_ulong()
            if not kernel32.GetExitCodeProcess(handle, ctypes.byref(code)):
                return True
            return code.value == 259  # STILL_ACTIVE
        finally:
            kernel32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def acquire_lock(path, *, alive=process_alive):
    """Exclusive session lock; a lock left by an exited process is taken over."""
    path = Path(path)
    stale = None
    try:
        handle = path.open('x')
    except FileExistsError:
        text = path.read_text().strip()
        if not text.isdigit():
            raise RuntimeError(f'{path} has no process id. Make sure no calibration tool is running, then delete it.')
        if alive(int(text)):
            raise RuntimeError(f'Another calibration process (PID {text}) is using {path.parent}. Close it first.')
        stale = int(text)
        path.unlink()
        handle = path.open('x')
    handle.write(str(os.getpid()))
    handle.flush()
    return handle, stale


class HelmSession:
    def __init__(self, profile, *, runs, limits=None, seed=None, clock=time.time):
        self.profile_path = Path(profile).resolve()
        (self.plan, self.paths, self.device, self.reference, self.model,
         self.pose, self.bindings) = load_plan(self.profile_path)
        self.asset_id = self.device['asset_id']
        # How Helm reaches the robot computer: the USB Ethernet tether, or its Wi-Fi
        # address from the device profile. Minerva Check and live teleop need the tether.
        self.connections = {'tether': TETHER, 'wifi': self.device['address']}
        self.connection = 'tether'
        self.steps = self.plan['steps']
        self.transmissions = validate_preview_transmissions(
            self.plan.get('preview_transmissions', []), self.steps, self.reference, self.model)
        self.clock = clock
        self.started_at = utc_now()
        self.lock = threading.RLock()
        self._lock_handle = None
        runs = Path(runs).resolve()
        directory = runs / (self.asset_id + '-directions')
        if (directory / 'session.json').is_file():
            existing = json.loads((directory / 'session.json').read_text())
            if existing.get('bindings') != self.bindings:
                directory = runs / (self.asset_id + '-directions-' + self.bindings['plan'][:8])
        directory.mkdir(parents=True, exist_ok=True)
        self.directions_dir = directory
        self._lock_handle, self.stale_lock_pid = acquire_lock(directory / 'session.lock')
        try:
            self.state_path = directory / 'session.json'
            self.state = self._load_state(seed)
            roster = {m['name']: m for m in self.reference['motors']}
            results = self.state['results']
            self.mapping = self.state.setdefault('preview_mapping', {
                s['joint']: {'motor': s['motor'], 'sign': results.get(s['joint'], {}).get('sign', 1),
                    'offset_rad': -results.get(s['joint'], {}).get('sign', 1) * roster[s['motor']]['raw_reference_rad'],
                    'direction_verified': s['joint'] in results, 'alignment_verified': False,
                    'scale_verified': False, 'ratio_assumed': 1., 'runtime_approved': False}
                for s in self.steps})
            if (set(self.mapping) != {s['joint'] for s in self.steps}
                    or any(r.get('sign') not in (-1, 1) or not math.isfinite(r.get('offset_rad', math.nan))
                           for r in self.mapping.values())):
                raise ValueError('Invalid saved direction mapping')
            self._persist_state()
            self.evidence = runs / 'helm' / time.strftime('%Y%m%dT%H%M%S', time.gmtime())
            self.evidence.mkdir(parents=True, exist_ok=True)
            self.limits_path = Path(limits).resolve() if limits else self.profile_path.parent / (self.asset_id + '-joint-limits.json')
            self.binding = dict(asset_id=self.asset_id, reference=self.reference,
                                reference_sha256=self.bindings['reference'])
            self.saved_limits, self.saved_limits_error = load_limits(self.limits_path, **self.binding)
            self.recorder = LimitRecorder(self.reference['motors'])
            self.labels = {s['motor']: s['label'] for s in self.steps}
            self.labels.update({t['motor']: t['motor'].replace('_', ' ').capitalize() for t in self.transmissions})
            self.raw, self.frame, self.received = None, None, 0.
            self.feed = dict(state='disconnected', message='Not connected. Press Connect to read the disabled motors.',
                             reconnections=0)
            self._guide = self._reference_guide()
        except BaseException:
            self.close()
            raise

    def _load_state(self, seed):
        if self.state_path.is_file():
            state = json.loads(self.state_path.read_text())
            allowed = {s['joint'] for s in self.steps}
            if (state.get('bindings') != self.bindings or state.get('runtime_approved') is not False
                    or set(state['results']) - allowed
                    or any(r.get('direction_verified') is not True or r.get('sign') not in (-1, 1)
                           for r in state['results'].values())):
                raise ValueError('Direction session does not match this profile revision')
            return state
        results = seed_results(seed, self.steps, self.reference) if seed else {}
        return {'schema': 1, 'asset_id': self.asset_id, 'reference': self.reference['id'],
                'bindings': self.bindings, 'results': results, 'directions_complete': False,
                'runtime_approved': False, 'physical_limits_verified': False,
                'scale_verified': False, 'captures': []}

    def _persist_state(self):
        save_json(self.state_path, self.state)

    def close(self):
        handle, self._lock_handle = self._lock_handle, None
        if handle:
            path = Path(handle.name)
            handle.close()
            path.unlink(missing_ok=True)

    # -- live feed ---------------------------------------------------------

    def _reference_guide(self):
        positions = {}
        for name, q in self.pose.items():
            # Display only: e.g. fully closed fingers sit just below the viewing range.
            bounds = self.model.by_name[name].get('control_bounds')
            positions[name] = min(max(q, bounds[0]), bounds[1]) if bounds else q
        return {'model': self.model.profile.id, 'urdf_sha256': self.model.sha256, 'positions': positions,
                'sequence': 0, 'view': 'fit', 'live': False, 'rows': [],
                'message': 'Reference pose shown. Helm is not connected to the robot.',
                'badge': 'NOT CONNECTED · reference pose shown'}

    def link_state(self, state, message, *, reconnections=0):
        with self.lock:
            self.feed = dict(state=state, message=message, reconnections=reconnections)
            if self._guide.get('live'):
                self._guide.update(feed_state=state if state != 'connecting' else 'waiting',
                                   message=message, reconnections=reconnections)
                self._guide['badge'] = {
                    'waiting': 'WAITING FOR MOTORS · last pose held',
                    'fault': 'MINERVA NOT REACHED · last pose held',
                    'reconnecting': 'LIVE · MOTORS DISABLED · renewing the reader'}.get(
                        state, 'NOT READING · last pose held')
            else:
                self._guide.update(message=message, badge={
                    'connecting': 'CONNECTING · checking host, CAN adapters and motor identities',
                    'waiting': 'WAITING FOR MOTORS · reference pose shown',
                    'fault': 'MINERVA NOT REACHED · reference pose shown'}.get(state, 'NOT CONNECTED · reference pose shown'))

    def on_frame(self, frame):
        """Validated stopped frame from the link worker thread."""
        raw = live_positions(frame, self.reference)
        with self.lock:
            self.recorder.observe(raw, sequence=frame['sequence'], stream=frame['request_id'])
            positions, rows = dict(self.pose), []
            for step in self.steps:
                joint, motor, row = step['joint'], step['motor'], self.mapping[step['joint']]
                positions[joint] = row['sign'] * raw[motor] + row['offset_rad']
                rows.append({'joint': joint, 'motor': motor, 'raw_rad': raw[motor], 'cad_rad': positions[joint],
                             'cad_value': positions[joint], 'cad_unit': 'rad', 'sign': row['sign'],
                             'offset_rad': row['offset_rad'], 'direction_verified': row['direction_verified']})
            for transmission in self.transmissions:
                row = preview_transmission_row(transmission, raw[transmission['motor']])
                positions[transmission['joint']] = row['cad_value']
                rows.append(row)
            self.raw, self.frame, self.received = raw, frame, self.clock()
            self.feed['state'] = 'reading'
            self._guide = {'model': self.model.profile.id, 'urdf_sha256': self.model.sha256,
                'sequence': frame['sequence'], 'positions': positions, 'live': True, 'rows': rows,
                # The viewer ages this against its own wall clock.
                'stream_id': frame['request_id'], 'updated_at': time.time(),
                'scan_seconds': frame['scan_seconds'], 'source_age_seconds': frame['roundtrip_seconds'],
                'feed_state': 'reading', 'reconnections': self.feed.get('reconnections', 0), 'view': 'fit',
                'badge': f"LIVE · MOTORS DISABLED · all {len(raw)} drives answering",
                'message': 'Live disabled motors. Directions are candidates; the model is not a safety check.'}
            with (self.evidence / 'frames.jsonl').open('a', encoding='utf-8') as out:
                out.write(json.dumps({'t': self.received, 'stream': frame['request_id'], 'sequence': frame['sequence'],
                    'scan_seconds': frame['scan_seconds'], 'roundtrip_seconds': frame['roundtrip_seconds'],
                    'raw_rad': raw}, separators=(',', ':')) + '\n')

    def guide(self):
        with self.lock:
            return deepcopy(self._guide)

    def fresh(self, limit=.75):
        with self.lock:
            return (self.frame is not None and self.feed['state'] == 'reading'
                    and self.clock() - self.received + self.frame['roundtrip_seconds'] <= limit)

    # -- operator actions --------------------------------------------------

    def direction(self, action, joint):
        if action not in ('flip', 'accept') or joint not in self.mapping:
            raise ValueError('Use flip or accept with a mapped arm joint')
        with self.lock:
            if not self.fresh():
                raise ValueError('No fresh live readings; direction edits need the robot connected')
            edit_preview(f'{action} {joint}', self.mapping, self.raw, self.steps)
            self.state.setdefault('preview_edits', []).append({'command': f'{action} {joint}', 'source': 'helm',
                'raw_positions': dict(self.raw), 'sequence': self.frame['sequence'], 'recorded_at': time.time()})
            self._persist_state()

    def limits(self, action, motor):
        with self.lock:
            if action == 'confirm':
                if not self.fresh():
                    raise ValueError('Confirm while connected, so the sweep reflects live readings')
                self.recorder.confirm(motor)
            elif action == 'unconfirm':
                self.recorder.unconfirm(motor)
            elif action == 'reset':
                self.recorder.reset(motor)
            else:
                raise ValueError('Unknown limits action')

    def save_limits(self):
        with self.lock:
            existing = self.saved_limits if self.saved_limits_error is None else None
            document = self.recorder.document(**self.binding, existing=existing,
                                              session=str(self.evidence.name))
            if self.limits_path.is_file() and existing is None:
                # Keep a copy of anything replaced; the evidence folder is local.
                save_json(self.evidence / ('replaced-' + self.limits_path.name),
                          json.loads(self.limits_path.read_text(encoding='utf-8')))
            save_json(self.limits_path, document)
            save_json(self.evidence / ('saved-' + uuid.uuid4().hex[:8] + '-' + self.limits_path.name), document)
            self.saved_limits, self.saved_limits_error = document, None
            self.recorder.confirmed.clear()
            return {'path': str(self.limits_path), 'joints': len(document['joints']),
                    'sha256': digest(self.limits_path.read_bytes())}

    # -- presentation ------------------------------------------------------

    @property
    def address(self):
        return self.connections[self.connection]

    def set_connection(self, key):
        if key not in self.connections:
            raise ValueError('Connect over the tether or Wi-Fi')
        self.connection = key

    def snapshot(self):
        with self.lock:
            saved = (self.saved_limits or {}).get('joints', {})
            directions = {row['motor']: dict(joint=joint, sign=row['sign'], verified=row['direction_verified'])
                          for joint, row in self.mapping.items()}
            for transmission in self.transmissions:
                directions[transmission['motor']] = dict(joint=transmission['joint'], editable=False,
                    sign=1 if transmission['metres_per_motor_rad'] > 0 else -1,
                    verified=transmission['direction_verified'])
            rows = []
            for row in self.recorder.rows():
                motor = row['name']
                rows.append({**row, 'label': self.labels.get(motor, motor),
                             'arm': 'left' if motor.startswith('left') else 'right' if motor.startswith('right') else 'other',
                             'direction': directions.get(motor), 'saved': deepcopy(saved.get(motor))})
            age = None
            if self.frame is not None:
                age = self.clock() - self.received + self.frame['roundtrip_seconds']
            return {'robot': {'asset_id': self.asset_id, 'address': self.address, 'connection': self.connection,
                              'connections': [{'key': key, 'address': address}
                                              for key, address in self.connections.items()],
                              'model': self.model.profile.title, 'reference': self.reference['id']},
                    'feed': {**self.feed, 'sequence': self.frame['sequence'] if self.frame else None,
                             'age_seconds': age, 'fresh': self.fresh(),
                             'scan_seconds': self.frame['scan_seconds'] if self.frame else None},
                    'limits': {'rows': rows, 'path': str(self.limits_path),
                               'saved_at': (self.saved_limits or {}).get('updated_at'),
                               'saved_joints': len(saved), 'saved_error': self.saved_limits_error,
                               'pending': len(self.recorder.confirmed)},
                    'session': {'directions': str(self.directions_dir), 'evidence': str(self.evidence),
                                'stale_lock_pid': self.stale_lock_pid, 'started_at': self.started_at}}
