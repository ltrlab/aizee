"""The head's setup in Helm (Limits tab, Head), so the operator sets the head up, and sets it up again, from Helm:

1. read the neck's servos, limp, on Helm on Minerva's own serial port (the 3D view's head follows them);
2. set the head's reference: both servos centred with the head held straight ahead and level, then its reference
   file saved;
3. check each joint's direction against the 3D view (flip or accept it, in the reference);
4. sweep each joint's limits by hand (the arms' LimitRecorder: an end counts once two readings agree) and save them,
   which also sets each servo's own position limits MARGIN_RAD outside, a backstop under Helm's limits;
5. record the shoulder corners (neck.clearance): turned to a side and tilted down, the head reaches its shoulder, so
   with the head turned and tilted down until just clear, each corner is recorded, and they are saved with the limits.
   Live sessions leave the head out until its clearance is saved.

Every servo write goes through neck.setup's checked steps (limp servos, values read back, the EEPROM locked
again). The files go beside the arms' servo calibration (neck.head.files), where Tune and teleop read them; a file
replaced is kept in this setup's evidence folder. A new reference makes saved limits stale (they name their
reference): sweep again. The reader never enables torque, and stops for any session that needs the head.
"""

from copy import deepcopy
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time
import uuid

from robot_platform.control.joint_calibration import save_json
from robot_platform.neck import clearance, head, setup
from robot_platform.neck.feetech import RESOLUTION, FeetechError, alarms_of

from .limits import LimitRecorder

PERIOD_S = 0.1  # The reader polls both servos' positions at 10 Hz,
HEALTH_EVERY = 10  # and their torque, voltage, temperature and alarms once a second.
FRESH_S = 0.5  # An older reading is not fresh: no reference, confirm or save from it.
MARGIN_RAD = 0.05  # The servos' own limits sit this far outside the swept range, so Helm's limits trip first.
RAD_PER_STEP = head.RAD_PER_STEP


def _now():
    return datetime.now(timezone.utc)


class HeadSetup:
    """`directory`: the operator files, beside the arms' servo calibration; `evidence`: Helm's runs folder, where
    this setup keeps the files it replaced. `open_bus`: opens the head's bus (tests pass a simulated one)."""

    def __init__(self, directory, asset_id, evidence, *, profile=None, open_bus=None, clock=time.monotonic):
        if profile is None:
            from robot_platform.ops.release import ROOT

            profile = head.load_profile(ROOT / head.PROFILE)
        self.profile, self.asset_id, self.clock = profile, asset_id, clock
        self.motors = profile['motors']
        self.paths = head.files(directory, asset_id)
        self.evidence_root, self.evidence = Path(evidence), None
        self.device, self.usb_serial = profile['bus'].get('device'), profile['bus'].get('usb_serial')
        self.open_bus = open_bus or self._open_bus
        self.lock = threading.RLock()  # The bus and the state: the reader and each step take turns.
        self.bus = self.thread = None
        self.reading = False
        self.raw, self.health, self.received = {}, {}, None
        self.sequence, self.stream = 0, None
        self.error, self.message = None, None
        self.reference = head.load_document(self.paths['reference'])
        self.limits = head.load_document(self.paths['limits'])
        self.recorder = self._recorder()
        self.corners = []  # Shoulder corners recorded since the clearance was last saved ({pan, tilt}, rad).

    def _open_bus(self):
        if not self.device:
            raise ValueError("The head's bus is not pinned in its profile")
        from robot_platform.neck.probe import open_bus

        return open_bus(self.device, self.profile['bus']['baud'])

    def _recorder(self):
        """The limits sweep, in radians from the reference (so 0 is straight ahead and level): none before one."""
        if self.reference is None:
            return None
        return LimitRecorder([{'name': m['name'], 'raw_reference_rad': 0.0} for m in self.motors])

    # -- the reader ---------------------------------------------------------------------------

    def start(self):
        with self.lock:
            if self.reading:
                return
            self.bus = self.open_bus()
            self.reading, self.error = True, None
            self.stream = uuid.uuid4().hex[:8]  # The recorder never pairs readings across two runs.
            self.message = 'Reading the head: its servos stay limp.'
            self.thread = threading.Thread(target=self._run, name='helm-head-setup', daemon=True)
            self.thread.start()

    def stop(self):
        with self.lock:
            if not self.reading:
                return
            self.reading = False
            thread = self.thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2)
        with self.lock:
            if self.bus is not None:
                self.bus.transport.close()
            self.bus = self.thread = None
            self.message = 'Stopped reading the head.'

    def _run(self):
        count = 0
        while True:
            with self.lock:
                if not self.reading:
                    return
                try:
                    self._poll(count)
                    self.error = None
                except (FeetechError, OSError, ValueError) as exc:
                    self.error = f'Reading the head: {exc}'
            count += 1
            time.sleep(PERIOD_S)

    def _poll(self, count):
        ids = [m['id'] for m in self.motors]
        values = self.bus.sync_read(ids, ['Present_Position'])
        self.raw = {m['name']: values[m['id']]['Present_Position'] for m in self.motors}
        self.received = self.clock()
        self.sequence += 1
        if count % HEALTH_EVERY == 0:
            for m in self.motors:
                h = self.bus.read_many(m['id'], ['Torque_Enable', 'Present_Voltage', 'Present_Temperature', 'Status'])
                self.health[m['name']] = {
                    'torque': bool(h['Torque_Enable']),
                    'voltage_v': h['Present_Voltage'] / 10,
                    'temperature_c': h['Present_Temperature'],
                    'alarms': alarms_of(h['Status']),
                }
        if self.recorder is not None:
            self.recorder.observe(self._relative(), sequence=self.sequence, stream=self.stream)

    def fresh(self):
        with self.lock:
            return (
                self.reading
                and self.received is not None
                and len(self.raw) == len(self.motors)
                and self.clock() - self.received <= FRESH_S
            )

    def _need_fresh(self, what):
        if not self.fresh():
            raise ValueError(f'{what} needs fresh readings: read the head first')

    def _relative(self):
        """Each servo's reading in radians from its reference reading."""
        steps = {m['name']: m['raw_reference_steps'] for m in self.reference['motors']}
        return {name: (self.raw[name] - steps[name]) * RAD_PER_STEP for name in steps}

    def joints(self):
        """The neck's CAD angles ({joint: rad}) from fresh readings, for the 3D view; None before a reference."""
        with self.lock:
            if self.reference is None or not self.fresh():
                return None
            relative = self._relative()
            return {m['joint']: m['sign'] * relative[m['name']] for m in self.reference['motors']}

    # -- the steps ----------------------------------------------------------------------------

    def set_reference(self):
        """With the head held straight ahead and level: centre both servos there, then save the head's reference.
        Each servo's previous offset comes back if any of it fails. Directions checked under the last reference
        carry over for the same servos (same uid): the mounting is unchanged."""
        with self.lock:
            self._need_fresh('Setting the reference')
            previous = {m['id']: self.bus.read(m['id'], 'Homing_Offset') for m in self.motors}
            try:
                for m in self.motors:
                    setup.centre(self.bus, m['id'])
                document = setup.reference_document(
                    self.bus,
                    self.profile,
                    asset_id=self.asset_id,
                    usb_serial=self.usb_serial,
                    device=self.device,
                    now=_now(),
                )
            except BaseException:
                for servo_id, offset in previous.items():
                    try:
                        setup.restore_offset(self.bus, servo_id, offset)
                    except Exception:  # noqa: BLE001 - restore the others; the error that matters is raised
                        pass
                raise
            if self.reference is not None:
                before = {m['uid']: m for m in self.reference['motors']}
                for motor in document['motors']:
                    if motor['uid'] in before:
                        motor['sign'] = before[motor['uid']]['sign']
                        motor['direction_verified'] = before[motor['uid']]['direction_verified']
            self._replace(self.paths['reference'], document)
            self.reference = document
            self.recorder = self._recorder()  # A new sweep, from the new reference.
            self.corners = []  # Corners read against the old reference: recorded again after the sweep.
            self.message = 'Reference set: both servos read mid-turn with the head straight ahead and level.' + (
                '' if self.limits is not None else ' Now check the directions and sweep the limits.'
            )
            return document

    def direction(self, action, name):
        """`flip` the motor's direction against the CAD, or `accept` it as seen moving in the 3D view."""
        if action not in ('flip', 'accept'):
            raise ValueError('Use flip or accept')
        with self.lock:
            if self.reference is None:
                raise ValueError("Set the head's reference first")
            document = deepcopy(self.reference)
            motor = next((m for m in document['motors'] if m['name'] == name), None)
            if motor is None:
                raise ValueError(f'Unknown head motor: {name}')
            if action == 'accept':
                self._need_fresh('Accepting a direction')
                motor['direction_verified'] = True
            else:
                motor['sign'], motor['direction_verified'] = -int(motor['sign']), False
            self._replace(self.paths['reference'], document)
            self.reference = document

    def limit(self, action, name):
        with self.lock:
            if self.recorder is None:
                raise ValueError("Set the head's reference first")
            if action == 'confirm':
                self._need_fresh('Confirming a sweep')
                self.recorder.confirm(name)
            elif action == 'unconfirm':
                self.recorder.unconfirm(name)
            elif action == 'reset':
                self.recorder.reset(name)
            else:
                raise ValueError('Unknown limits action')

    def save_limits(self):
        """Both joints' confirmed sweeps: each servo's own limits MARGIN_RAD outside, then the limits file."""
        with self.lock:
            if self.recorder is None:
                raise ValueError("Set the head's reference first")
            missing = [m['label'] for m in self.motors if m['name'] not in self.recorder.confirmed]
            if missing:
                raise ValueError(f"Confirm {' and '.join(missing)} first: the head's limits are saved together")
            self._need_fresh('Saving the limits')
            reference = {m['name']: m for m in self.reference['motors']}
            seen = {}
            for m in self.motors:
                swept, home = self.recorder.confirmed[m['name']], reference[m['name']]['raw_reference_steps']
                seen[m['id']] = (
                    round(home + swept['min_rad'] / RAD_PER_STEP),
                    round(home + swept['max_rad'] / RAD_PER_STEP),
                )
            document = setup.limits_document(seen, self.profile, self.reference, now=_now())
            margin = math.ceil(MARGIN_RAD / RAD_PER_STEP)
            for m in self.motors:
                low, high = seen[m['id']]
                setup.set_limits(self.bus, m['id'], max(0, low - margin), min(RESOLUTION - 1, high + margin))
            self._replace(self.paths['limits'], document)
            self.limits = document
            self.recorder.confirmed.clear()
            self.message = (
                "Head limits saved, and each servo's own limits set just outside them. Now record the shoulder corners."
            )
            return {'path': str(self.paths['limits'])}

    def _matching_limits(self):
        return bool(self.limits and self.reference and self.limits.get('reference_id') == self.reference['id'])

    def corner(self, action):
        """`record` the head's pose now as a shoulder corner (turned to a side and tilted down until just clear of the
        shoulder), or `clear` the corners recorded so far."""
        with self.lock:
            if action == 'clear':
                self.corners = []
                return
            if action != 'record':
                raise ValueError('Use record or clear')
            if not self._matching_limits():
                raise ValueError("Save the head's limits first: its corners are saved with them")
            self._need_fresh('Recording a corner')
            relative = self._relative()
            joints = {m['joint']: m['name'] for m in self.reference['motors']}
            self.corners.append({'pan': relative[joints[clearance.PAN]], 'tilt': relative[joints[clearance.TILT]]})

    def save_clearance(self):
        """The corners recorded, as the shoulder clearance in the limits file (neck.clearance)."""
        with self.lock:
            if not self._matching_limits():
                raise ValueError("Save the head's limits first: its corners are saved with them")
            document = dict(
                deepcopy(self.limits), clearance=dict(clearance.section(self.corners), created_at=_now().isoformat())
            )
            clearance.from_limits(document, self.reference)  # The floor they make, checked before it is saved.
            self._replace(self.paths['limits'], document)
            self.limits, self.corners = document, []
            self.message = (
                'Shoulder clearance saved: the head keeps above its corners in the next session, and live '
                'sessions take the head.'
            )
            return {'path': str(self.paths['limits']), 'corners': len(document['clearance']['corners'])}

    def _replace(self, path, document):
        if self.evidence is None:
            self.evidence = self.evidence_root / f'head-setup-{_now().strftime("%Y%m%dT%H%M%SZ")}'
            self.evidence.mkdir(parents=True, exist_ok=True)
        tag = uuid.uuid4().hex[:8]
        if path.is_file():
            (self.evidence / f'replaced-{tag}-{path.name}').write_bytes(path.read_bytes())
        path.parent.mkdir(parents=True, exist_ok=True)
        save_json(path, document)
        save_json(self.evidence / f'saved-{tag}-{path.name}', document)

    # -- for the page -------------------------------------------------------------------------

    def snapshot(self):
        with self.lock:
            fresh, reference, limits = self.fresh(), self.reference, self.limits
            by_motor = {m['name']: m for m in reference['motors']} if reference else {}
            matches = bool(limits and reference and limits.get('reference_id') == reference['id'])
            rows = {row['name']: row for row in self.recorder.rows()} if self.recorder else {}
            servos, cards = [], []
            for m in self.motors:
                measured, steps = by_motor.get(m['name']), self.raw.get(m['name'])
                relative = (
                    None
                    if measured is None or steps is None
                    else (steps - measured['raw_reference_steps']) * RAD_PER_STEP
                )
                servos.append(
                    {
                        'name': m['name'],
                        'label': m['label'],
                        'joint': m['joint'],
                        'id': m['id'],
                        'steps': steps,
                        'relative_rad': relative,
                        **self.health.get(m['name'], {}),
                    }
                )
                if measured is None:
                    continue
                saved = limits['joints'].get(m['name']) if matches else None
                cards.append(
                    dict(
                        rows[m['name']],
                        label=m['label'],
                        arm='head',
                        saved=None
                        if saved is None
                        else {
                            'min_rad': saved['min_rad'] - measured['raw_reference_rad'],
                            'max_rad': saved['max_rad'] - measured['raw_reference_rad'],
                        },
                        direction={
                            'joint': m['name'],
                            'sign': measured['sign'],
                            'verified': measured['direction_verified'],
                            'editable': True,
                        },
                    )
                )
            return {
                'available': True,
                'device': self.device,
                'reading': self.reading,
                'fresh': fresh,
                'error': self.error,
                'message': self.message,
                'servos': servos,
                'reference': None
                if reference is None
                else {
                    'id': reference['id'],
                    'created_at': reference['created_at'],
                    'directions_verified': all(m['direction_verified'] for m in reference['motors']),
                },
                'limits': None
                if limits is None
                else {'created_at': limits.get('created_at'), 'matches_reference': matches},
                # The shoulder corners recorded, and the clearance saved with the limits for this reference.
                'corners': [dict(c) for c in self.corners],
                'clearance': None
                if not (matches and limits.get('clearance'))
                else {
                    'created_at': limits['clearance'].get('created_at'),
                    'corners': [dict(c) for c in limits['clearance']['corners']],
                    'margin_rad': limits['clearance']['margin_rad'],
                },
                'rows': cards,
                'pending': len(self.recorder.confirmed) if self.recorder else 0,
                'files': {kind: str(path) for kind, path in self.paths.items() if kind in ('reference', 'limits')},
            }

    def close(self):
        self.stop()
