"""Resumable terminal wizard for motor-to-CAD directions, with stopped captures."""
import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
import uuid


def digest(data):
    return hashlib.sha256(data).hexdigest()


def save_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8', newline='\n') as output:
        json.dump(data, output, indent=2, allow_nan=False)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())
    # Windows briefly denies replacement while the viewer has the old file
    # open. Keep the previous complete frame and retry for a bounded interval.
    for attempt in range(11):
        try:
            os.replace(temporary, path)
            break
        except PermissionError:
            if attempt == 10:
                raise
            time.sleep(.01)


def write_guide(path, data):
    # Append complete frames: Windows readers may hold a file open without
    # delete sharing. A live feed must not depend on repeatedly replacing it.
    with Path(path).open('a', encoding='utf-8', newline='\n') as output:
        output.write(json.dumps(data, allow_nan=False, separators=(',', ':')) + '\n')


def validate_preview_transmissions(transmissions, steps, reference, model):
    """Validate display-only motor-angle to independent linear-joint mappings."""
    if not isinstance(transmissions, (list, tuple)):
        raise ValueError('preview_transmissions must be an array')
    motors = {motor['name']: motor for motor in reference['motors']}
    names, joints = {s['motor'] for s in steps}, {s['joint'] for s in steps}
    validated = []
    for entry in transmissions:
        if not isinstance(entry, dict):
            raise ValueError('Each preview transmission must be an object')
        motor, joint = entry.get('motor'), entry.get('joint')
        if (not isinstance(motor, str) or not isinstance(joint, str)
                or motor not in motors or motor in names or joint in joints):
            raise ValueError('Duplicate or unknown preview transmission motor/joint')
        if joint not in model.independent or model.by_name[joint]['type'] != 'prismatic':
            raise ValueError('Preview transmissions require independent prismatic CAD joints')
        for key in ('motor_reference_rad', 'joint_reference_m', 'metres_per_motor_rad'):
            value = entry.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'Preview transmission {key} must be a finite number')
        raw_reference = motors[motor].get('raw_reference_rad')
        if (type(raw_reference) not in (int, float) or not math.isfinite(raw_reference)
                or entry['motor_reference_rad'] != raw_reference):
            raise ValueError('Preview transmission must match the recorded motor reference')
        scale = entry['metres_per_motor_rad']
        if scale == 0 or not math.isfinite(entry['joint_reference_m'] - scale * raw_reference):
            raise ValueError('Preview transmission requires a nonzero scale and finite offset')
        if (type(entry.get('direction_verified')) is not bool
                or entry.get('scale_verified') is not False or entry.get('runtime_approved') is not False):
            raise ValueError('Preview transmission requires explicit direction status and unapproved scale/runtime')
        if any(not isinstance(entry.get(key), str) or not entry[key].strip() for key in ('note', 'source')):
            raise ValueError('Preview transmission requires nonempty note and source strings')
        validated.append(dict(entry))
        names.add(motor); joints.add(joint)
    return validated


def preview_transmission_row(transmission, raw_rad):
    """Convert one measured angle without clamping or inferring a mechanism."""
    if type(raw_rad) not in (int, float) or not math.isfinite(raw_rad):
        raise ValueError('Nonfinite preview transmission position')
    scale = transmission['metres_per_motor_rad']
    value = transmission['joint_reference_m'] + scale * (raw_rad - transmission['motor_reference_rad'])
    if not math.isfinite(value):
        raise ValueError('Nonfinite preview transmission position')
    return {**transmission, 'raw_rad': raw_rad, 'cad_value': value, 'cad_unit': 'm',
            'sign': 1 if scale > 0 else -1,
            'offset_m': transmission['joint_reference_m'] - scale * transmission['motor_reference_rad'],
            'display_only': True}


def load_plan(path):
    from robot_platform.models import ModelProfile, UrdfModel
    path = Path(path).resolve()
    plan = json.loads(path.read_text())
    if plan.get('schema') != 1 or not plan.get('steps'):
        raise ValueError('Expected a calibration plan with steps')
    paths = {key: (path.parent / plan[key]).resolve() for key in ('device', 'reference', 'model')}
    device, reference = (json.loads(paths[key].read_text()) for key in ('device', 'reference'))
    model = UrdfModel(ModelProfile.load(paths['model']))
    if reference['asset_id'] != device['asset_id'] or reference['model_revision'] != model.sha256:
        raise ValueError('Device, reference and CAD revision must match')
    if not reference.get('power_cycle_verified'):
        raise ValueError('Mechanical reference requires a verified power cycle')
    motors = {m['name']: m for m in reference['motors']}
    if len(motors) != len(reference['motors']):
        raise ValueError('Duplicate motor names')
    names, joints = set(), set()
    transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'], reference, model)
    pose = {n: model.by_name[n]['default'] for n in model.independent}
    pose.update(plan['reference_positions'])
    for entry in transmissions:
        joint = entry['joint']
        if joint in plan['reference_positions'] and pose[joint] != entry['joint_reference_m']:
            raise ValueError('Preview transmission conflicts with the CAD reference pose')
        pose[joint] = entry['joint_reference_m']
    # URDF validity still applies to the reference; provisional viewer control
    # bounds do not. Live measured positions are displayed without clamping.
    model.positions(pose)
    for step in plan['steps']:
        name, joint, delta = step['motor'], step['joint'], step['demonstration_rad']
        if name not in motors or name in names or joint in joints:
            raise ValueError('Duplicate or unknown motor/joint')
        if joint not in model.independent or model.by_name[joint]['type'] not in ('revolute', 'continuous'):
            raise ValueError('Direction steps require independent rotary CAD joints')
        if type(delta) not in (int, float) or not math.isfinite(delta) or not .05 <= delta <= .4:
            raise ValueError('Demonstration must be a small positive angle')
        target = {**pose, joint: pose[joint] + delta}
        model.positions(target)
        bounds = model.by_name[joint]['control_bounds']
        if not bounds[0] <= target[joint] <= bounds[1]:
            raise ValueError('Demonstration is outside the reviewed viewer range')
        names.add(name); joints.add(joint)
    bindings = {'plan': digest(path.read_bytes()), 'device': digest(paths['device'].read_bytes()),
                'reference': digest(paths['reference'].read_bytes()), 'model': model.sha256,
                'model_profile': model.profile.profile_sha256}
    return plan, paths, device, reference, model, pose, bindings


def checked_positions(snapshot, reference):
    if (not snapshot.get('success') or snapshot.get('enable_requested') is not False
            or snapshot.get('parameter_writes_requested') is not False):
        raise ValueError('Stopped capture failed: ' + str(snapshot.get('error', 'invalid capture')))
    if snapshot.get('asset_id') != reference['asset_id']:
        raise ValueError('Capture device mismatch')
    rows = snapshot.get('motors', [])
    roster = reference['motors']
    if len(rows) != len(roster) or {r['name'] for r in rows} != {m['name'] for m in roster}:
        raise ValueError('Incomplete or duplicate motor readings')
    by_name = {r['name']: r for r in rows}
    result = {}
    for motor in roster:
        row = by_name[motor['name']]
        if any(row.get(k) != motor[k] for k in ('bus', 'id', 'uid', 'firmware')) or row.get('reset_confirmed') is not True:
            raise ValueError('Motor identity or Reset verification mismatch')
        samples, q = row.get('samples_rad', []), row['position_rad']
        if (len(samples) < 6 or not all(type(x) in (int, float) and math.isfinite(x) for x in [q, *samples])
                or max(samples) - min(samples) > .002 or not min(samples) <= q <= max(samples)):
            raise ValueError('Moving, incomplete or nonfinite capture')
        result[motor['name']] = q
    after = snapshot.get('after', {})
    if set(after) != {m['bus'] for m in roster} or any('UP' in r['flags'] for r in after.values()):
        raise ValueError('CAN cleanup was not verified')
    return result


def check_reference(positions, reference, tolerance=.05):
    outside = [f"{m['name']} ({positions[m['name']]:+.4f} rad)" for m in reference['motors']
               if abs(positions[m['name']] - m['raw_reference_rad']) > tolerance]
    if outside:
        raise ValueError('Return these joints to reference: ' + ', '.join(outside))


def direction_result(step, baseline, moved, reference):
    before, after = checked_positions(baseline, reference), checked_positions(moved, reference)
    check_reference(before, reference)
    deltas = {name: after[name] - before[name] for name in before}
    delta = deltas[step['motor']]
    if not .05 <= abs(delta) <= 1.0:
        raise ValueError(f"{step['motor']}: movement too small or large ({delta:+.4f} rad); use a modest movement")
    changed = [name for name, value in deltas.items() if name != step['motor'] and abs(value) > .05]
    if changed:
        raise ValueError('Other joints moved: ' + ', '.join(changed) + '. Return to reference and retry.')
    return {'motor': step['motor'], 'joint': step['joint'], 'sign': 1 if delta > 0 else -1,
            'raw_delta_rad': delta, 'other_deltas_rad': {n: v for n, v in deltas.items() if n != step['motor']},
            'demonstration_rad': step['demonstration_rad'], 'direction_verified': True,
            'scale_verified': False, 'ratio': None, 'runtime_approved': False}


def seed_results(path, steps, reference):
    data = json.loads(Path(path).read_text())
    # A reference revision that re-zeroes some motors names the earlier references whose direction checks it
    # keeps: a direction is a sign (raw motor change to CAD joint change), which a re-zero does not move.
    references = {reference['id'], *reference.get('direction_evidence_from', ())}
    if (data['asset_id'] != reference['asset_id'] or data['mechanical_reference'] not in references
            or data['model_revision'] != reference['model_revision']):
        raise ValueError('Seed evidence belongs to another reference/model/device')
    roster = {m['name']: m for m in reference['motors']}
    step_map = {s['joint']: s for s in steps}
    result = {}
    for check in data['checks']:
        step = step_map.get(check['joint'])
        if not step:
            continue
        motor = roster[step['motor']]
        sign = check['observed_motor_to_cad_sign']
        delta = check['raw_delta_rad']
        if (check['joint'] in result or check['motor_uid'] != motor['uid'] or check['firmware'] != motor['firmware']
                or check.get('direction_verified') is not True or type(sign) is not int or sign not in (-1, 1)
                or not math.isfinite(delta) or not .05 <= abs(delta) <= 1 or delta * sign <= 0
                or check.get('viewer_demonstration_rad', 0) <= 0):
            raise ValueError('Invalid seed direction evidence')
        result[check['joint']] = {'motor': step['motor'], 'joint': step['joint'], 'sign': sign,
            'direction_verified': True, 'scale_verified': False, 'ratio': None, 'runtime_approved': False,
            'seed_sha256': digest(Path(path).read_bytes()), 'source_check': check}
    return result


class SshCapture:
    """Stage only the stopped reader and pin the SSH host by fingerprint."""
    def __init__(self, connection, device, reference, plan_directory):
        import paramiko
        expected = connection['host_key_sha256']
        class PinnedHost(paramiko.MissingHostKeyPolicy):
            def missing_host_key(self, client, hostname, key):
                actual = 'SHA256:' + base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
                if actual != expected:
                    raise paramiko.SSHException('SSH host fingerprint mismatch')
                client.get_host_keys().add(hostname, key.get_name(), key)
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(PinnedHost())
        self.device, self.reference = device, reference
        try:
            self.client.connect(device['address'], username=connection['username'],
                key_filename=str((plan_directory / connection['key_file']).resolve()),
                look_for_keys=False, allow_agent=False, timeout=8, auth_timeout=8, banner_timeout=8)
            source = Path(__file__).parent
            files = {n: (source / n).read_bytes() for n in ('joint_capture.py', 'mechanical_zero.py')}
            self.source_revision = digest(b''.join(files.values()))
            with self.client.open_sftp() as sftp:
                sftp.get_channel().settimeout(10)
                with sftp.open('/etc/machine-id', 'rb') as f:
                    if f.read().strip().decode() != device['machine_id']:
                        raise RuntimeError('Wrong remote machine identity')
                base = connection['staging_directory'].rstrip('/')
                self.stage = base + '/joint-calibration-' + self.source_revision[:16]
                try:
                    sftp.stat(self.stage)
                except FileNotFoundError:
                    sftp.mkdir(self.stage)
                for name, content in files.items():
                    target = self.stage + '/' + name
                    try:
                        with sftp.open(target, 'rb') as f:
                            existing = f.read()
                    except FileNotFoundError:
                        with sftp.open(target, 'wb') as f:
                            f.write(content)
                        with sftp.open(target, 'rb') as f:
                            existing = f.read()
                    if existing != content:
                        raise RuntimeError('Staged reader hash mismatch')
        except BaseException:
            self.close()
            raise

    def close(self):
        self.client.close()

    def capture(self):
        token = uuid.uuid4().hex
        stdin, stdout, stderr = self.client.exec_command('python3 ' + shlex.quote(self.stage + '/joint_capture.py'), timeout=45)
        stdin.write(json.dumps({'device': self.device, 'reference': self.reference, 'request_id': token}))
        stdin.flush(); stdin.channel.shutdown_write()
        raw = stdout.read().decode()
        error = stderr.read().decode()
        if stdout.channel.recv_exit_status():
            raise RuntimeError('Stopped reader failed: ' + error)
        report = json.loads(raw)
        if report.get('request_id') != token:
            raise RuntimeError('Capture did not match this request')
        report['reader_source_revision'] = self.source_revision
        return report

    def stream(self, receive, stop):
        token = uuid.uuid4().hex
        # The first reply waits for the full identity preflight; each silent motor
        # adds about 0.4 s (one bounded retry). The remote reader keeps its own
        # heartbeat and scan limits, so a patient read here cannot hide a stall.
        stdin, stdout, stderr = self.client.exec_command(
            'python3 ' + shlex.quote(self.stage + '/joint_capture.py') + ' --live', timeout=20)
        stdin.write(json.dumps({'device': self.device, 'reference': self.reference, 'request_id': token}) + '\n')
        stdin.flush()
        last_sequence = 0
        while True:
            nonce = uuid.uuid4().hex
            began = time.perf_counter()
            stdin.write('stop\n' if stop.is_set() else 'heartbeat ' + nonce + '\n'); stdin.flush()
            line = stdout.readline()
            if not line:
                raise RuntimeError('Live reader disconnected: ' + stderr.read().decode())
            frame = json.loads(line)
            if frame.get('request_id') != token:
                raise RuntimeError('Live request identity mismatch')
            if frame.get('kind') == 'finished':
                return frame
            if frame.get('request_nonce') != nonce:
                raise RuntimeError('Live sample does not match the fresh request')
            if frame.get('kind') == 'scan_rejected':
                continue
            if frame.get('sequence', 0) <= last_sequence:
                raise RuntimeError('Repeated or reversed live sequence')
            last_sequence = frame['sequence']
            frame['roundtrip_seconds'] = time.perf_counter() - began
            if frame['roundtrip_seconds'] > .75:
                continue  # Delayed responses never freshen the displayed pose.
            receive(frame)


def live_positions(frame, reference):
    if (frame.get('kind') != 'positions' or frame.get('asset_id') != reference['asset_id']
            or frame.get('enable_requested') is not False or frame.get('parameter_writes_requested') is not False
            or not 0 <= frame.get('scan_seconds', math.inf) <= .5
            or not 0 <= frame.get('roundtrip_seconds', math.inf) <= .75):
        raise ValueError('Invalid or delayed live capture')
    rows = frame['motors']
    roster = reference['motors']
    by_name = {row['name']: row for row in rows}
    if len(rows) != len(roster) or len(by_name) != len(rows) or set(by_name) != {m['name'] for m in roster}:
        raise ValueError('Incomplete live roster')
    positions = {}
    for motor in roster:
        row = by_name[motor['name']]
        q = row['position_rad']
        if (any(row.get(k) != motor[k] for k in ('bus', 'id', 'uid', 'firmware'))
                or row.get('reset_confirmed') is not True or type(q) not in (int, float) or not math.isfinite(q)):
            raise ValueError('Live motor identity, Reset or position mismatch')
        positions[motor['name']] = q
    return positions


def edit_preview(command, mapping, latest, steps):
    """Explicit local edits produce a candidate only; never motor writes."""
    words = command.split()
    if len(words) < 2 or words[1] not in mapping:
        raise ValueError('Use flip JOINT, align JOINT CAD_RADIANS, or accept JOINT')
    action, joint = words[:2]
    row = mapping[joint]
    if action == 'flip' and len(words) == 2:
        row['sign'] *= -1
        row['offset_rad'] *= -1
        row['direction_verified'] = False
    elif action == 'align' and len(words) == 3:
        angle = float(words[2])
        if not math.isfinite(angle) or abs(angle) > math.pi:
            raise ValueError('Alignment must be a finite CAD angle within +/-pi')
        motor = next(s['motor'] for s in steps if s['joint'] == joint)
        row['offset_rad'] = angle - row['sign'] * latest[motor]
        row['alignment_source'] = 'operator visually matched current pose to entered CAD angle'
        row['alignment_verified'] = False
    elif action == 'accept' and len(words) == 2:
        row['direction_verified'] = True
        row['direction_source'] = 'operator confirmed live physical/CAD movement agreement'
    else:
        raise ValueError('Use flip JOINT, align JOINT CAD_RADIANS, or accept JOINT')
    row['runtime_approved'] = False


def preview_cleanup_verified(report, reference):
    """Check all stopped-reader identities and every bus's cleanup result."""
    after = report.get('after', {})
    return (report.get('asset_id') == reference['asset_id'] and report.get('identity_verified') is True
            and report.get('enable_requested') is False and report.get('parameter_writes_requested') is False
            and not report.get('cleanup_errors')
            and set(after) == {m['bus'] for m in reference['motors']}
            and all('UP' not in row.get('flags', ['UP']) for row in after.values()))


def preview_reconnect_allowed(report, reference):
    """Only stopped diagnostic timeouts with verified CAN-down cleanup may retry.

    This is deliberately outside the powered controller and cannot re-arm it.
    Protocol/identity failures, disconnects and unknown cleanup require an exit.
    """
    return (report.get('success') is False and report.get('error_code') == 'incomplete_scans'
            and preview_cleanup_verified(report, reference))


def run_live(client, steps, reference, state, directory, persist, model, pose, guide_path, *, transmissions=(), ask=input):
    transmissions = validate_preview_transmissions(transmissions, steps, reference, model)
    lock, stop = threading.RLock(), threading.Event()
    live = {'frame': None, 'received': 0., 'error': None, 'final': None, 'reconnections': 0}
    guide = {'model': model.profile.id, 'urdf_sha256': model.sha256, 'positions': dict(pose),
             'sequence': 0, 'stream_id': uuid.uuid4().hex, 'live': True, 'rows': [],
             'updated_at': 0., 'scan_seconds': 0., 'source_age_seconds': 0., 'view': 'fit'}
    def publish_status(status, message):
        with lock:
            guide.update(feed_state=status, message=message, reconnections=live['reconnections'])
            write_guide(guide_path, guide)
    publish_status('waiting', 'Connecting to stopped motors; waiting for a complete live pose.')
    roster = {m['name']: m for m in reference['motors']}
    mapping = state.setdefault('preview_mapping', {
        s['joint']: {'motor': s['motor'], 'sign': state['results'].get(s['joint'], {}).get('sign', 1),
            'offset_rad': -state['results'].get(s['joint'], {}).get('sign', 1) * roster[s['motor']]['raw_reference_rad'],
            'direction_verified': s['joint'] in state['results'], 'alignment_verified': False,
            'scale_verified': False, 'ratio_assumed': 1., 'runtime_approved': False}
        for s in steps})
    if set(mapping) != {s['joint'] for s in steps} or any(r.get('sign') not in (-1, 1) or not math.isfinite(r.get('offset_rad', math.nan)) for r in mapping.values()):
        raise ValueError('Invalid saved preview mapping')
    persist()
    def receive(frame):
        raw = live_positions(frame, reference)
        with lock:
            live.update(frame=frame, received=time.perf_counter(), raw=raw)
            positions = dict(pose)
            rows = []
            for step in steps:
                n, motor = step['joint'], step['motor']
                row = mapping[n]
                positions[n] = row['sign'] * raw[motor] + row['offset_rad']
                rows.append({'joint': n, 'motor': motor, 'raw_rad': raw[motor], 'cad_rad': positions[n],
                             'cad_value': positions[n], 'cad_unit': 'rad',
                             'sign': row['sign'], 'offset_rad': row['offset_rad'],
                             'direction_verified': row['direction_verified']})
            for transmission in transmissions:
                row = preview_transmission_row(transmission, raw[transmission['motor']])
                positions[transmission['joint']] = row['cad_value']
                rows.append(row)
            guide.update({'model': model.profile.id, 'urdf_sha256': model.sha256,
                'sequence': frame['sequence'], 'positions': positions, 'live': True, 'rows': rows,
                'stream_id': frame['request_id'],
                'updated_at': time.time(), 'scan_seconds': frame['scan_seconds'],
                'source_age_seconds': frame['roundtrip_seconds'],
                'feed_state': 'reading', 'reconnections': live['reconnections'],
                'message': ('Live stopped-motor preview. Unchecked signs and all 1:1 angle scales are provisional. '
                            + ('Linear transmission scales are unverified and for display only.' if transmissions
                               else 'Only configured rotary joints are mapped.')),
                'view': 'fit'})
            write_guide(guide_path, guide)
    def worker():
        try:
            attempt = 0
            while True:
                final = client.stream(receive, stop)
                with lock:
                    live['final'] = final
                save_json(directory / ('live-cleanup-' + uuid.uuid4().hex[:8] + '.json'), final)
                if final.get('success'):
                    if not preview_cleanup_verified(final, reference):
                        raise RuntimeError('Stopped-reader identity and CAN-down cleanup not confirmed')
                    if final.get('completion_reason') == 'duration_limit' and not stop.is_set():
                        # Keep each remote reader finite. A healthy expiry can
                        # renew after cleanup, without hiding faults or resetting
                        # the session's three-retry budget.
                        with lock:
                            live['frame'] = None
                        publish_status('reconnecting', 'Refreshing stopped-reader identity checks; last pose held.')
                        if not stop.wait(.1):
                            continue
                    publish_status('stopped', 'Live reader closed; CAN links are down. Restart the calibration script to reconnect.')
                    break
                if attempt == 3 or stop.is_set() or not preview_reconnect_allowed(final, reference):
                    raise RuntimeError(final.get('error', 'Live reader failed'))
                # A new worker command reruns the complete host, USB, UID,
                # firmware and Reset preflight. Never reuse a partial pose or
                # treat this preview-only recovery as transport qualification.
                with lock:
                    live['reconnections'] += 1
                    live['frame'] = None
                message = f"CAN replies stopped. Reconnecting ({attempt+1}/3); last pose held."
                publish_status('reconnecting', message)
                print('\n' + message, flush=True)
                if stop.wait(.5):
                    publish_status('stopped', 'Live reader closed; CAN links are down.')
                    break
                attempt += 1
        except Exception as exc:
            with lock:
                live['error'] = str(exc)
            print('\nLive reader stopped: ' + str(exc), flush=True)
            publish_status('stopped', 'Live reader stopped: ' + str(exc) + '. Restart the calibration script to retry.')
        finally:
            stop.set()
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    print('\nMove the robot by hand and compare the model. Rotary-joint commands:\n'
          '  flip JOINT            reverse one displayed direction\n'
          '  align JOINT 0         align the CURRENT physical pose to CAD angle 0\n'
          '  accept JOINT          mark its observed direction checked\n'
          '  status                list candidate mappings\n'
          '  q                     save and close; every edit also autosaves\n'
          'Use the joint names in the viewer. Angle scales and offsets remain candidates.\n'
          + ('Linear preview mappings come from the profile; their scales are unverified.\n' if transmissions else ''), flush=True)
    try:
        while not stop.is_set():
            command = ask('calibration> ').strip()
            if command.lower() in ('q', 'quit', 'exit'):
                break
            if not command:
                continue
            with lock:
                if command == 'status':
                    print(json.dumps({**mapping, **{entry['joint']: entry for entry in transmissions}}, indent=2)); continue
                if live['error'] or not live['frame'] or time.perf_counter() - live['received'] + live['frame']['roundtrip_seconds'] > .75:
                    print('No fresh complete readings; edit rejected.'); continue
                try:
                    edit_preview(command, mapping, live['raw'], steps)
                    state.setdefault('preview_edits', []).append({'command': command, 'raw_positions': dict(live['raw']),
                        'sequence': live['frame']['sequence'], 'recorded_at': time.time()})
                    persist()
                    print('Candidate saved. Hardware zeros and runtime configuration unchanged.')
                except (ValueError, KeyError) as exc:
                    print(str(exc))
    finally:
        stop.set()
        thread.join(timeout=8)
        if thread.is_alive():
            raise RuntimeError('Cleanup response missing; do not enable. Inspect CAN links on the device.')
        if live['error']:
            raise RuntimeError(live['error'])
        final = live['final']
        if not final or not final.get('success') or any('UP' in r['flags'] for r in final.get('after', {}).values()):
            raise RuntimeError('CAN-down cleanup not confirmed')
        print('Live preview closed; physical CAN links are down.', flush=True)


def run_wizard(steps, reference, state, capture, persist, guide, *, ask=input, say=print):
    """Operator presses Enter locally; every accepted step is checkpointed."""
    def prompt(text):
        if ask(text + '\n[Enter] capture   [q] save and quit: ').strip().lower() == 'q':
            raise KeyboardInterrupt

    def reference_capture():
        while True:
            description = reference.get('reference_description', 'Match the documented reference pose.')
            guide(None, description + ' Press Enter in the terminal when stationary.')
            prompt('Match the documented reference pose. Support the mechanism and keep every joint still.')
            snap, file = capture('reference')
            try:
                check_reference(checked_positions(snap, reference), reference)
                return snap, file
            except ValueError as exc:
                say(str(exc))

    baseline, baseline_file = reference_capture()
    for index, step in enumerate(steps, 1):
        if step['joint'] in state['results']:
            continue
        while True:
            text = f"[{index}/{len(steps)}] {step['label']}: {step['instruction']}"
            guide(step, text + ' Direction only; exact angle need not match. Capture in the terminal.')
            prompt(text + '\nMove ONLY this joint by hand in the shown positive direction; hold still. Angle need not be exact.')
            snap, file = capture(step['joint'])
            try:
                result = direction_result(step, baseline, snap, reference)
            except ValueError as exc:
                say('Capture rejected: ' + str(exc))
                baseline, baseline_file = reference_capture()
                continue
            result.update(baseline_capture=baseline_file, moved_capture=file)
            say(f"Observed sign {result['sign']:+d}; encoder change {result['raw_delta_rad']:+.4f} rad. Return to reference.")
            returned, returned_file = reference_capture()
            result['return_capture'] = returned_file
            state['results'][step['joint']] = result
            persist()
            baseline, baseline_file = returned, returned_file
            say(f"Saved {len(state['results'])}/{len(steps)} directions.")
            break
    state['directions_complete'] = True
    persist()
    guide(None, 'Direction checks complete. Motors disabled. Calibration candidate saved; runtime approval remains pending.')
    say('All direction checks saved. This does not approve ratios, physical limits, loaded stopping or teleoperation.')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='robot-platform calibrate', description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--output', type=Path, help='Session directory; every raw capture is preserved')
    parser.add_argument('--resume', action='store_true', help='Resume saved results after a fresh reference check')
    parser.add_argument('--seed', type=Path, help='Import compatible previously observed direction evidence')
    parser.add_argument('--plan', action='store_true', help='Validate and list steps without opening devices')
    parser.add_argument('--check', action='store_true', help='One stopped connectivity capture; no movement prompts')
    parser.add_argument('--no-viewer', action='store_true')
    parser.add_argument('--live', action='store_true', help='Live hand-movement preview with flip/align/accept commands')
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args(argv)
    client = viewer = log = lock = None
    try:
        plan, paths, device, reference, model, pose, bindings = load_plan(args.profile)
        seed = seed_results(args.seed, plan['steps'], reference) if args.seed else {}
        if args.plan:
            print(json.dumps({'asset_id': device['asset_id'], 'bindings': bindings,
                'steps': plan['steps'], 'preview_transmissions': plan.get('preview_transmissions', []),
                'seeded': list(seed), 'enable_requested': False}, indent=2))
            return 0
        if not 1 <= args.port <= 65535:
            raise ValueError('Viewer port must be in 1..65535')
        directory = (args.output or Path('calibration_runs') / (device['asset_id'] + '-directions')).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        lock = (directory / 'session.lock').open('x')
        lock.write(str(os.getpid())); lock.flush()
        state_path = directory / 'session.json'
        if state_path.exists():
            if not args.resume:
                raise ValueError('Session exists; use --resume or a new output directory')
            state = json.loads(state_path.read_text())
            if state['bindings'] != bindings or state.get('runtime_approved') is not False:
                raise ValueError('Session revision mismatch; start a new directory')
            allowed = {s['joint'] for s in plan['steps']}
            if set(state['results']) - allowed or any(r.get('direction_verified') is not True or r.get('sign') not in (-1, 1) for r in state['results'].values()):
                raise ValueError('Invalid saved direction results')
        else:
            state = {'schema': 1, 'asset_id': device['asset_id'], 'reference': reference['id'],
                     'bindings': bindings, 'results': seed, 'directions_complete': False,
                     'runtime_approved': False, 'physical_limits_verified': False,
                     'scale_verified': False, 'captures': []}
        def persist():
            save_json(state_path, state)
        persist()
        guide_path = directory / 'guide.jsonl'
        def guide(step, message):
            positions = dict(pose)
            if step:
                positions[step['joint']] += step['demonstration_rad']
            write_guide(guide_path, {'model': model.profile.id, 'urdf_sha256': model.sha256,
                'sequence': uuid.uuid4().hex, 'positions': positions, 'message': message,
                'joint': step['joint'] if step else None, 'view': step.get('view', 'fit') if step else 'fit'})
        guide(None, 'Calibration terminal ready. Follow its prompts; this is a target pose, not live motor feedback.')
        if not args.no_viewer and not args.check:
            log = (directory / 'viewer.log').open('w')
            viewer = subprocess.Popen([sys.executable, '-m', 'robot_platform', 'view', '--model', str(paths['model']),
                '--guide', str(guide_path), '--port', str(args.port)], stdout=log, stderr=log)
            time.sleep(.5)
            if viewer.poll() is not None:
                raise RuntimeError('Viewer failed to start; inspect ' + str(directory / 'viewer.log'))
            print(f'Calibration viewer: http://127.0.0.1:{args.port}/', flush=True)
        if args.live:
            saved = sum(row.get('direction_verified') is True for row in state['preview_mapping'].values()) if 'preview_mapping' in state else len(state['results'])
            direction_label = 'Saved live directions'
        else:
            saved, direction_label = len(state['results']), 'Saved directions'
        print('Stopped-only calibration: move by hand. No enable, zero, flash save or parameter writes.\n'
              f'{direction_label}: {saved}/{len(plan["steps"])}. Progress: {state_path}', flush=True)
        client = SshCapture(plan['connection'], device, reference, args.profile.resolve().parent)
        def capture(label):
            print('Reading stopped motors...', flush=True)
            report = client.capture()
            name = f'{len(state["captures"]):04d}-{label}-{uuid.uuid4().hex[:8]}.json'
            save_json(directory / name, report)
            state['captures'].append({'file': name, 'sha256': digest((directory / name).read_bytes())})
            persist()
            checked_positions(report, reference)
            return report, name
        if args.check:
            report, name = capture('connection-check')
            print(f'PASS: {len(report["motors"])} identities, stationary Reset readings and CAN-down cleanup. Saved {name}')
            return 0
        if args.live:
            run_live(client, plan['steps'], reference, state, directory, persist, model, pose, guide_path,
                     transmissions=plan.get('preview_transmissions', ()))
        else:
            run_wizard(plan['steps'], reference, state, capture, persist, guide)
        return 0
    except (KeyboardInterrupt, EOFError):
        print('\nStopped. Saved steps are preserved; run again with --resume.')
        return 130
    except Exception as exc:
        print(f'Calibration stopped: {exc}', file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()
        if viewer and viewer.poll() is None:
            viewer.terminate()
            try:
                viewer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                viewer.kill(); viewer.wait(timeout=2)
        if log:
            log.close()
        if lock:
            path = Path(lock.name)
            lock.close(); path.unlink()


if __name__ == '__main__':
    raise SystemExit(main())
