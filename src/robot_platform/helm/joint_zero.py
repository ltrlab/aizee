"""Helm's Joint zero: set arm joints' motor zeros at the six-joint zero pose, prove them by a power cycle, and record
the arms' new mechanical reference with what is bound to it.

A joint put back differently (a wrist structure remounted) reads off at the zero pose by the remount. The operator
holds the arm at the six-joint zero pose (the 3D view's reference pose: every arm joint at 0 rad), picks the joints
and confirms; the robot zeroes and saves those drives with every drive stopped (ops.probe zero_joints): nothing is
enabled. A drive's zero counts once a power cycle shows the drive kept it: the operator switches the motors' power
off and on, leaves the arm where it is and presses Verify, which reads every drive stopped (ops.probe motors). Until
then the zero is pending, and Helm starts nothing that moves the arms.

Verify then records the arms' new reference revision among Helm's operator files (control.joint_calibration
operator_reference), in place of the profile's: the same roster with the re-zeroed drives' evidence, so the operator
can redo it without a release. What was bound to the old revision moves to the new one: the saved joint limits less
the re-zeroed joints' (re-swept on the Limits tab: a joint put back differently may stop at other angles), the
direction session (a zero moves no sign) and the manual tuning presets. The old copies go to Helm's evidence folder,
and Helm restarts to load the new revision. Wrist camera stills and the twin correction already refuse anything
recorded on another reference.
"""

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import shutil
import threading
import time

from robot_platform.control.joint_calibration import arms_reference_name, digest, save_json
from robot_platform.ops.probe import ARM_JOINTS

KIND = 'minerva_joint_zero'
VERIFY_RAD = 0.01  # A kept zero reads within this at the pose it was set at: the zero's own readback limit.
TIMEOUT_S = 180.0


def pending_path(directory, asset_id):
    """The zeros waiting for their power cycle, beside the operator's other files."""
    return Path(directory) / f'{asset_id}-joint-zero.json'


def _iso(at):
    return datetime.fromtimestamp(at, timezone.utc).isoformat()


def _stamp(at):
    return time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(at))


def _write(path, data):
    """Replace `path` with exactly these bytes (the reference's digest is of its bytes)."""
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('wb') as output:
        output.write(data)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)


def reference_bytes(document):
    """The bytes a reference is written as, so its digest is known before it is."""
    return (json.dumps(document, indent=2, allow_nan=False) + '\n').encode('utf-8')


def next_reference(current, zeroed, scan, *, at, labels=None):
    """The revision of `current` (the arms' reference) after the zeros in `zeroed` ({motor: what the zero
    reported}), proven by `scan` (ops.probe motors after the power cycle). Every raw reference stays 0: the zeros were
    set at the reference pose."""
    labels = labels or {}
    names = ', '.join(labels.get(motor, motor) for motor in sorted(zeroed))
    rows = {row['name']: row for row in scan['motors']}
    document = deepcopy(current)
    document.update(
        id=f'{current["id"].rsplit("-", 1)[0]}-{_stamp(at)}',
        supersedes=current['id'],
        recorded_at=_iso(at),
        power_cycle_verified=True,
        reference_description=(
            f"Revision of {current['id']}: {names} re-zeroed by Helm's Joint zero, the operator "
            'holding the arm at the six-joint zero pose (every arm joint at 0 rad in the viewer). '
            'Every other motor keeps its zero.'
        ),
        # A zero moves no sign: the direction checks the old revision kept still hold.
        direction_evidence_from=sorted(set(current.get('direction_evidence_from') or []) | {current['id']}),
        rezeroed={
            'motors': sorted(zeroed),
            'at_utc': max(entry['at'] for entry in zeroed.values()),
            'reason': "Helm's Joint zero",
            'operator_report': 'Arm held at the six-joint zero pose; motor power switched off and on before '
            'Verify, the arm left where it was.',
            'before_zero_rad': {motor: entry['before_rad'] for motor, entry in sorted(zeroed.items())},
            'evidence': sorted({entry['evidence'] for entry in zeroed.values()}),
        },
        power_cycle_verification={
            'operator_report': 'Motor power switched off and on after the zero, the arm left at the zero pose '
            "(confirmed in Helm's Verify).",
            'capture_started_at': scan.get('started_at'),
            'remote_capture': scan.get('evidence'),
            'all_14_passed': bool(scan.get('success'))
            and len(rows) == len(current['motors'])
            and all(row.get('reset_confirmed') for row in scan['motors']),
            'rezeroed_max_absolute_readback_rad': max(abs(rows[motor]['position_rad']) for motor in zeroed),
            'motors': [
                {
                    key: row.get(key)
                    for key in ('name', 'position_rad', 'uid', 'firmware', 'reset_confirmed', 'zero_sta')
                }
                for row in scan['motors']
            ],
        },
        invalidated_calibrations=[
            f'Saved joint limits of {names}: re-sweep them on the Limits tab',
            "Wrist camera stills and the digital twin's correction recorded before this zero",
        ],
        limitations=list(current.get('limitations') or [])
        + [
            f"{names}: re-zeroed by Helm's Joint zero on {_iso(at)[:10]}. Their saved limits were cleared for a "
            're-sweep; servo settings and leader mapping carry over.'
        ],
    )
    for motor in document['motors']:
        entry = zeroed.get(motor['name'])
        if entry is not None:
            motor.update(
                after_save_samples_rad=[entry['after_save_rad']],
                zero_transmissions=1,
                save_transmissions=1 if entry['save_sent'] else 0,
                save_response=None,
                save_response_missing=entry['save_response_missing'],
                before_zero_rad=entry['before_rad'],
                zeroed_at=entry['at'],
            )
    return document


def rebind_limits(limits, reference, reference_sha256, drop, *, at):
    """Saved limits moved onto the new revision, less the re-zeroed joints' (re-swept afterwards); None for none."""
    if limits is None:
        return None
    document = deepcopy(limits)
    document['joints'] = {name: entry for name, entry in document['joints'].items() if name not in drop}
    document.update(reference=reference['id'], reference_sha256=reference_sha256, updated_at=_iso(at))
    return document


def rebind(document, old, new):
    """A file bound to the calibration's digests (`binding.calibration`), moved from `old` to `new`; None when it
    was bound to something else (it stays as it was)."""
    binding = (document or {}).get('binding')
    if not isinstance(binding, dict) or binding.get('calibration') != old:
        return None
    return dict(document, binding=dict(binding, calibration=dict(new)))


class JointZero:
    """`session` (helm.session.HelmSession): the reference, limits and direction session; its operator files keep
    the new revision. `runs`: Helm's runs folder (the teleop presets). `open_ssh(profile, address=...)`: the pinned
    SSH client to the robot."""

    def __init__(self, session, *, runs, open_ssh=None, remote_root=None, clock=time.time):
        from robot_platform.ops.check import CURRENT

        self.session, self.runs, self.open_ssh, self.clock = session, Path(runs), open_ssh, clock
        self.remote_root = remote_root or CURRENT
        self.lock = threading.Lock()
        self.running = None  # 'zero' or 'verify'
        self.last = None
        self.applied = None  # The revision Verify recorded, which Helm loads once it restarts.
        directory = getattr(session, 'operator_files', None)
        self.path = pending_path(directory, session.asset_id) if directory is not None else None

    def unavailable(self):
        if self.path is None:
            return (
                "Joint zero keeps the arms' new reference among Helm's operator files, which only Helm on Minerva "
                'has (helm --operator-files)'
            )
        return None

    def pending(self):
        """The zeros waiting for their power cycle ({'zeroed': {motor: ...}, ...}), or None."""
        if self.path is None or not self.path.is_file():
            return None
        document = json.loads(self.path.read_text(encoding='utf-8'))
        if document.get('kind') != KIND or document.get('asset_id') != self.session.asset_id:
            raise ValueError(f"{self.path.name} is not this robot's joint zero")
        return document if document.get('zeroed') else None

    def blocking(self):
        """Why nothing that moves the arms may start now, or None."""
        if self.applied is not None:
            return f"Helm is restarting to load the arms' new reference {self.applied['reference']}"
        try:
            pending = self.pending()
        except (OSError, ValueError) as exc:
            return f'The joint zero could not be read: {exc}'
        if pending:
            names = ', '.join(self._label(motor) for motor in sorted(pending['zeroed']))
            return (
                f"{names}: zeroed, waiting for the power cycle. Switch the motors' power off and on and press "
                'Verify in Limits → Joint zero first'
            )
        return None

    def _label(self, motor):
        return self.session.labels.get(motor, motor)

    def _claim(self, action):
        reason = self.unavailable()
        if reason:
            raise ValueError(reason)
        with self.lock:
            if self.running:
                raise ValueError('A joint zero is already running')
            if self.applied is not None:
                raise ValueError(self.blocking())
            self.running = action

    def _finish(self, report):
        report['finished_at'] = self.clock()
        with self.lock:
            self.running, self.last = None, report
        return report

    def _probe(self, args):
        """One robot-side probe's report (its 'ok' removed); raises when it failed to run."""
        from robot_platform.ops.ssh import DEFAULT_PROFILE, connect, run

        address = getattr(self.session, 'address', None) or self.session.device['address']
        client = (self.open_ssh or connect)(DEFAULT_PROFILE, address=address)
        try:
            command = (
                f'cd {shlex.quote(self.remote_root)} && PYTHONPATH=src python3 -u -m robot_platform.ops.probe '
                + ' '.join(shlex.quote(arg) for arg in args)
            )
            status, out, err = run(client, command, timeout=TIMEOUT_S)
        finally:
            client.close()
        if 'or zero_gripper MOTOR' in out:  # The usage line of a release without zero_joints.
            raise RuntimeError('The deployed release predates Joint zero: deploy it first')
        try:
            report = json.loads(out.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise RuntimeError(f'The robot did not answer: {err.strip()[-300:] or f"exit {status}"}') from None
        if not report.pop('ok', False):
            raise RuntimeError(report.get('error') or 'The robot could not run it')
        return report

    # -- the zero -----------------------------------------------------------------------------------------------

    def zero(self, motors, *, confirm=False):
        """Zero these arm joints' drives where the arm is (blocking: call it off the event loop). Returns the robot's
        report; every drive it sent a zero to waits for the power cycle, also when a later one failed."""
        motors = list(motors) if isinstance(motors, (list, tuple)) else []
        if not motors or len(set(motors)) != len(motors) or any(motor not in ARM_JOINTS for motor in motors):
            raise ValueError('Choose the arm joints to zero, each once')
        if confirm is not True:
            raise ValueError('Hold the arm at the zero pose (every joint at 0 in the 3D view), then confirm the zero')
        self._claim('zero')
        try:
            report = self._probe(['zero_joints', *motors])
            report['pending'] = self._record(report)
        except Exception as exc:
            report = {'motors': motors, 'success': False, 'error': str(exc) or type(exc).__name__}
        return self._finish(dict(report, action='zero'))

    def _record(self, report):
        """Add the drives the robot sent a zero to (whatever came after) to the zeros waiting for their power cycle;
        returns their motors."""
        pending = self.pending() or {'schema': 1, 'kind': KIND, 'asset_id': self.session.asset_id, 'zeroed': {}}
        sent = [joint for joint in report.get('joints') or [] if joint.get('zero_sent')]
        for joint in sent:
            pending['zeroed'][joint['motor']] = dict(
                {
                    key: joint.get(key)
                    for key in (
                        'before_rad',
                        'after_zero_rad',
                        'after_save_rad',
                        'uid',
                        'firmware',
                        'zero_acknowledged',
                        'save_sent',
                        'save_response_missing',
                    )
                },
                at=_iso(self.clock()),
                evidence=report.get('evidence'),
                reference=self.session.reference['id'],
                reference_sha256=self.session.bindings['reference'],
            )
        if sent:
            save_json(self.path, pending)
        return [joint['motor'] for joint in sent]

    # -- the power cycle ----------------------------------------------------------------------------------------

    def verify(self, *, confirm=False):
        """After the power cycle: read every drive stopped; when each zeroed drive still reads its zero, record the
        new reference revision and move what is bound to it (blocking). Returns the report; on success Helm must
        restart (`report['restart']`)."""
        if confirm is not True:
            raise ValueError("Switch the motors' power off and on, leave the arm at the zero pose, then confirm Verify")
        self._claim('verify')
        try:
            pending = self.pending()
            if not pending:
                raise ValueError('No joint zero is waiting for its power cycle')
            scan = self._probe(['motors'])
            if not scan.get('success'):
                raise RuntimeError(f'The stopped read failed: {scan.get("error") or "no reason given"}')
            rows = {row['name']: row for row in scan.get('motors') or []}
            readings = {motor: rows[motor]['position_rad'] for motor in pending['zeroed'] if motor in rows}
            missing = sorted(set(pending['zeroed']) - set(readings))
            if missing:
                raise RuntimeError(f'The stopped read has no reading of {", ".join(map(self._label, missing))}')
            off = {motor: value for motor, value in readings.items() if abs(value) > VERIFY_RAD}
            if off:
                report = {'success': False, 'readings': readings, 'error': self._off_text(off, pending)}
            else:
                report = dict(self._apply(pending, scan), success=True, readings=readings, restart=True)
        except Exception as exc:
            report = {'success': False, 'error': str(exc) or type(exc).__name__}
        return self._finish(dict(report, action='verify'))

    def _off_text(self, off, pending):
        parts = []
        for motor, value in sorted(off.items()):
            before = pending['zeroed'][motor].get('before_rad')
            kept_old = before is not None and abs(value - before) <= max(VERIFY_RAD, abs(before) / 2)
            parts.append(
                f'{self._label(motor)} reads {value:+.3f} rad'
                + (f', its old zero ({before:+.3f}): the drive did not keep the zero' if kept_old else '')
            )
        return (
            '; '.join(parts) + f'. A kept zero reads within {VERIFY_RAD} rad at the zero pose. If the arm moved, '
            'put it back at the zero pose and verify again; if a drive lost its zero, zero it again.'
        )

    def _apply(self, pending, scan):
        """Record the new revision and move what is bound to the old one. Everything is worked out first, the old
        copies kept, then the bound files written and the reference last."""
        session = self.session
        at = self.clock()
        zeroed = pending['zeroed']
        current, old = session.reference, dict(session.bindings)
        document = next_reference(current, zeroed, scan, at=at, labels=session.labels)
        data = reference_bytes(document)
        new = dict(old, reference=digest(data))
        target = session.operator_files / arms_reference_name(session.asset_id)
        valid = session.saved_limits if session.saved_limits_error is None else None
        limits = rebind_limits(valid, document, new['reference'], set(zeroed), at=at)
        if session.state.get('bindings') != old:
            raise RuntimeError('The direction session belongs to another calibration; nothing was changed')
        presets = {}
        for path in sorted((self.runs / f'{session.asset_id}-teleop').glob('*.json')):
            try:
                moved = rebind(json.loads(path.read_text(encoding='utf-8')), old, new)
            except (OSError, ValueError):
                moved = None
            if moved is not None:
                presets[path] = moved
        kept = session.evidence / f'joint-zero-{_stamp(at)}'
        kept.mkdir(parents=True, exist_ok=True)
        for path in [target, session.limits_path, session.state_path, *presets, self.path]:
            if Path(path).is_file():
                shutil.copy2(path, kept / f'replaced-{Path(path).name}')
        if limits is not None:
            save_json(session.limits_path, limits)
        with session.lock:  # Helm's own copy, so nothing it saves before the restart puts the old one back.
            session.state.update(bindings=new, reference=document['id'])
            save_json(session.state_path, session.state)
        for path, moved in presets.items():
            save_json(path, moved)
        _write(target, data)
        save_json(
            kept / 'applied.json',
            {
                'reference': document['id'],
                'supersedes': current['id'],
                'reference_sha256': new['reference'],
                'zeroed': zeroed,
                'limits_cleared': sorted(set(zeroed) & set((valid or {}).get('joints', {}))),
                'moved': [str(path) for path in presets],
                'scan': scan.get('evidence'),
            },
        )
        self.path.unlink()
        self.applied = {'reference': document['id'], 'at': _iso(at), 'kept': str(kept)}
        return {
            'reference': document['id'],
            'supersedes': current['id'],
            'zeroed': sorted(zeroed),
            'resweep': sorted(zeroed),
            'kept': str(kept),
        }

    # -- the page ----------------------------------------------------------------------------------------------

    def resweep(self):
        """Joints the active revision re-zeroed whose limits are not saved again yet."""
        rezeroed = (self.session.reference.get('rezeroed') or {}).get('motors') or []
        saved = (self.session.saved_limits or {}).get('joints', {})
        return [motor for motor in rezeroed if motor in ARM_JOINTS and motor not in saved]

    def snapshot(self):
        try:
            pending, error = self.pending(), None
        except (OSError, ValueError) as exc:
            pending, error = None, str(exc)
        with self.lock:
            running, last, applied = self.running, deepcopy(self.last), deepcopy(self.applied)
        return {
            'available': self.unavailable() is None,
            'reason': self.unavailable(),
            'running': running,
            'last': last,
            'applied': applied,
            'error': error,
            'reference': self.session.reference['id'],
            'verify_rad': VERIFY_RAD,
            'pending': None
            if not pending
            else [
                dict(
                    motor=motor,
                    label=self._label(motor),
                    **{key: entry.get(key) for key in ('before_rad', 'after_save_rad', 'at', 'save_response_missing')},
                )
                for motor, entry in sorted(pending['zeroed'].items())
            ],
            'resweep': [{'motor': motor, 'label': self._label(motor)} for motor in self.resweep()],
        }
