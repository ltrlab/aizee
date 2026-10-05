"""The guarded receiver on Minerva's physical CAN for one arm, or both, for Helm's live teleop.

Runs on the robot computer, in the teleop loop's process, from the active
release. Starting it:
  1. The calibration tool's stopped reader scans the arm (each arm) with every
     motor disabled: host, adapter serial, services, identity, firmware and
     fault-free Reset, and each motor still and within tolerance of its reference
     position. Encoders cannot prove the physical pose; the operator confirms it too.
  2. The arm's CAN link (each arm's) comes up through the sudo-permitted helper.
  3. guarded_controller runs in teleop mode on that bus (both buses: one receiver
     for both arms, under a motion collision bound). It repeats the
     identity and reference checks and arms the motors' command-loss watchdogs
     before opening its socket; its first enable spends that permit. Until then
     it keeps auditing the watchdogs, which fails if a motor reads over
     0.2 rad/s three times in a row (one such reading is drive noise): the arm
     must hang still until Idle.
The receiver faults on any reading outside its joint limits, even disarmed, so
the pre-scan also refuses a joint at or near a limit. Closing stops the service,
which disables every motor on its bus on the way out, saves its last status as
evidence, then takes the CAN link down. The service dies with this process
(parent death signal). Nothing here writes a zero, a parameter or flash.
"""
import json
import math
from pathlib import Path
import signal
import subprocess
import time

ENDPOINT = 'tcp://127.0.0.1:5590'
HOST_ID = 0xAA  # The stopped reader's host id too.
FEEDBACK_MARGIN_RAD = .002  # Closer to a limit than this, 16-bit feedback steps (~0.0004 rad) may cross it.
STARTUP_HINT = ('every motor must answer promptly, hang still (not over 0.2 rad/s three readings in a row), '
                'stay in Reset without a fault and within tolerance of its reference. Let the arm settle, then '
                'start again.')


def service_config(controller, device, reference, motor_models, *, bus=None, buses=None, tolerance_rad,
                   period_us=2500, maximum_lateness_us=10000, endpoint=ENDPOINT, start_pose=None):
    """The teleop-mode service configuration for `controller`'s joints, all on `bus` (one arm), or on `buses`
    (both arms: the receiver needs a motion collision bound for two). The arm starts in its reference pose, or at
    `start_pose` ({motor: raw rad}): the pose an operator confirmed, to return it from."""
    from .commissioning import wire_ranges
    buses = list(buses) if buses is not None else [bus]
    roster = {motor['name']: motor for motor in reference['motors']}
    names = [joint['name'] for joint in controller['joints']]
    if not buses or len(set(buses)) != len(buses) or any(roster[name]['bus'] not in buses for name in names):
        raise ValueError(f'A live receiver drives the arms of its buses: every joint must be on {" or ".join(map(str, buses))}')
    if len(buses) > 1 and (controller.get('collision') or {}).get('bound') != 'motion':
        raise ValueError('A live receiver drives both arms only under a motion collision bound')
    pinned = {b['interface']: b['usb_serial'] for b in device['buses']}
    for name in buses:
        if not pinned.get(name):
            raise ValueError(f'{name} is not a pinned bus of {device["asset_id"]}')
    return {'mode': 'teleop', 'endpoint': endpoint, 'period_us': period_us,
            'maximum_lateness_us': maximum_lateness_us, 'host_id': HOST_ID,
            'buses': [{'interface': name, 'usb_serial': pinned[name]} for name in buses],
            'conflicting_services': list(device['conflicting_services']),
            'motors': [{'name': name, 'bus': buses.index(roster[name]['bus']), 'id': roster[name]['id'],
                        'scale': 1., 'zero': 0., 'ranges': wire_ranges(motor_models[name])} for name in names],
            'controller': controller,
            'watchdog_identities': [{'uid': list(bytes.fromhex(roster[name]['uid'])),
                                     'firmware': [int(part) for part in roster[name]['firmware'].split('.')]}
                                    for name in names],
            'reference_positions': [{'name': name, 'raw_position_rad': (roster[name]['raw_reference_rad']
                                                                        if start_pose is None else start_pose[name]),
                                     'tolerance_rad': tolerance_rad} for name in names]}


def reference_offsets(report, config):
    """[(motor, offset_rad)] for scanned motors outside their reference tolerance."""
    references = {row['name']: row for row in config['reference_positions']}
    scanned = {row['name']: row['position_rad'] for row in report.get('motors', [])}
    if set(scanned) != set(references):
        raise RuntimeError('The stopped scan did not cover every motor of the arm')
    return [(name, scanned[name] - row['raw_position_rad']) for name, row in references.items()
            if abs(scanned[name] - row['raw_position_rad']) > row['tolerance_rad']]


TURN_RAD = 2 * math.pi
TURN_ADVICE = ('A drive reports its angle within one turn when it powers up (0…2π unless its power-on range '
               'is set to −π…π), so a joint resting just below its zero at power-up reads a whole turn high. '
               'Nothing moved and nothing is re-zeroed: turn the joint to its reference by hand, then power-cycle '
               'the motors.')


def whole_turns(value, lower, upper, *, tolerance=.05):
    """How many whole turns (+1 or -1) a reading outside lower…upper sits from a reading inside it, else 0:
    a drive that powered up a turn away (see TURN_ADVICE), not a moved or re-zeroed joint. A reading inside
    the range is 0 even where the range spans more than a turn."""
    if lower - tolerance <= value <= upper + tolerance:
        return 0
    for turns in (1, -1):
        if lower - tolerance <= value - turns * TURN_RAD <= upper + tolerance:
            return turns
    return 0


def turned(name, value, turns, labels=None):
    """'Left J1 reads +6.18 rad, −0.11 rad plus a whole turn'."""
    label = (labels or {}).get(name, name)
    return f"{label} reads {value:+.2f} rad, {value - turns * TURN_RAD:+.2f} rad {'plus' if turns > 0 else 'minus'} a whole turn"


def outside_limits(positions, controller, *, margin=0.):
    """[(motor, position, lower, upper)] for positions outside the receiver's joint limits inset by `margin`."""
    joints = {joint['name']: joint for joint in controller['joints']}
    return [(name, value, joints[name]['lower'], joints[name]['upper']) for name, value in positions.items()
            if name in joints and not joints[name]['lower'] + margin <= value <= joints[name]['upper'] - margin]


def fault_detail(result, controller, labels=None):
    """What in a receiver response explains its fault, joint by joint ('' when nothing in it does).

    Diagnostic only, and called from fault handlers: it never raises."""
    try:
        return _fault_detail(result, controller, labels or {})
    except Exception:
        return ''


def _fault_detail(result, controller, labels):
    # The receiver's own words first: which motor failed which check, with the value.
    failure = (result or {}).get('hardware_failure')
    details = [f'the receiver reports {failure}'] if failure else []
    for joint, row in zip(controller['joints'], (result or {}).get('feedback') or []):
        label = labels.get(joint['name'], joint['name'])
        position, velocity, effort, temperature = (row.get(key) for key in
                                                   ('position', 'velocity', 'effort', 'temperature'))
        if row.get('fault'):
            details.append(f'{label} reports a drive fault')
        if position is not None and not joint['lower'] <= position <= joint['upper']:
            details.append(f"{label} at {position:+.4f} rad, outside its limits "
                           f"{joint['lower']:+.4f} to {joint['upper']:+.4f}")
        if velocity is not None and abs(velocity) > joint['max_velocity']:
            details.append(f"{label} moving at {velocity:+.2f} rad/s (limit {joint['max_velocity']:g})")
        if effort is not None and abs(effort) > joint['max_effort']:
            details.append(f"{label} effort {effort:+.1f} Nm (limit {joint['max_effort']:g})")
        if temperature is not None and temperature > joint['max_temperature']:
            details.append(f"{label} at {temperature:.0f} °C (limit {joint['max_temperature']:g})")
    if (result or {}).get('watchdog') == 'failed' and not failure:
        details.append('the watchdog audit failed: ' + STARTUP_HINT.split('. ')[0])
    return '; '.join(details)


def _die_with_parent():
    import ctypes
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class LiveReceiver:
    """One arm's guarded service (or both arms'), owned by this process. Exchange-compatible with the harness."""

    def __init__(self, config, *, device, reference, binary, directory, labels=None, capture=None,
                 command=subprocess.run, spawn=subprocess.Popen, transport_factory=None, clock=time.monotonic,
                 confirmed_pose=False):
        self.config, self.device, self.reference = config, device, reference
        # Its start pose is one the operator confirmed away from the reference (a return), not the reference.
        self.confirmed_pose = confirmed_pose
        self.buses = [b['interface'] for b in config['buses']]
        self.label = '-'.join(self.buses)  # Its evidence files: live-<label>.json, .log, -last.json.
        self.binary, self.directory = Path(binary), Path(directory)
        self.labels = dict(labels or {})
        self.capture, self.command, self.spawn = capture, command, spawn
        self.transport_factory, self.clock = transport_factory, clock
        self.process = self.transport = self.log = self.scan = self.exit_code = None
        self.links_up = []  # The buses this brought up, to take down again.

    @property
    def last(self):
        return self.transport.last if self.transport is not None else None

    def start(self, timeout_s=10.):
        try:
            self.scan = self._prescan()
            self._link(up=True)
            self._spawn()
            self._await_disarmed(timeout_s)
        except BaseException:
            self.close()
            raise
        return self

    def exchange(self, request):
        return self.transport.exchange(request)

    def _prescan(self):
        from .joint_capture import run_capture
        names = {motor['name'] for motor in self.config['motors']}
        device = dict(self.device, buses=[b for b in self.device['buses'] if b['interface'] in self.buses])
        reference = dict(self.reference, motors=[m for m in self.reference['motors'] if m['name'] in names])
        report = (self.capture or run_capture)(device, reference)
        if not report.get('success'):
            raise RuntimeError(f"The stopped check of the arm failed: {report.get('error', 'no report')}")
        scanned = {row['name']: row['position_rad'] for row in report['motors']}
        wrapped = [(name, value, whole_turns(value, lower, upper))
                   for name, value, lower, upper in outside_limits(scanned, self.config['controller'])]
        if any(turns for _, _, turns in wrapped):
            raise ValueError('; '.join(turned(name, value, turns, self.labels) for name, value, turns in wrapped
                                       if turns) + '. ' + TURN_ADVICE)
        offsets = reference_offsets(report, self.config)
        if offsets and self.confirmed_pose:
            tolerance = self.config['reference_positions'][0]['tolerance_rad']
            raise ValueError('The arm moved since you confirmed its pose: ' + ', '.join(
                f'{self.labels.get(name, name)} is {offset:+.2f} rad from it' for name, offset in offsets)
                + f' (tolerance {tolerance:g} rad). Confirm it again from where it is now.')
        if offsets:
            tolerance = self.config['reference_positions'][0]['tolerance_rad']
            raise ValueError('Not in the reference pose: ' + ', '.join(
                f'{self.labels.get(name, name)} is {offset:+.2f} rad away' for name, offset in offsets)
                + f' (tolerance {tolerance:g} rad). Let the arm hang straight down with the gripper closed, '
                  'then start again.')
        outside = outside_limits(scanned, self.config['controller'], margin=FEEDBACK_MARGIN_RAD)
        if outside:
            raise ValueError("At or next to the receiver's joint limits, which would fault it at once: " + ', '.join(
                f'{self.labels.get(name, name)} reads {value:+.4f} rad (limits {lower:+.4f} to {upper:+.4f})'
                for name, value, lower, upper in outside) + '. Re-sweep that joint in the Limits tab.')
        return report

    def _link(self, *, up):
        """Bring every bus up (each one recorded as it comes up), or each one brought up down again."""
        for bus in (self.buses if up else list(self.links_up)):
            command = (['sudo', '-n', self.device['can_up_helper'], bus] if up
                       else ['sudo', '-n', '/usr/sbin/ip', 'link', 'set', bus, 'down'])
            result = self.command(command, capture_output=True, text=True, timeout=10)
            if result.returncode:
                raise RuntimeError(f"Could not bring {bus} {'up' if up else 'down'}: "
                                   f"{(result.stderr or result.stdout).strip()}")
            if up:
                self.links_up.append(bus)
            else:
                self.links_up.remove(bus)

    def _spawn(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / f'live-{self.label}.json'
        path.write_text(json.dumps(self.config, indent=1, allow_nan=False) + '\n', encoding='utf-8')
        self.log = (self.directory / f'live-{self.label}.log').open('ab')
        self.process = self.spawn([str(self.binary), str(path), '--allow-powered-teleop', '--confirm-reference-pose'],
                                  stdout=self.log, stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)

    def _log_tail(self):
        try:
            return (self.directory / f'live-{self.label}.log').read_text(errors='replace').strip()[-400:]
        except OSError:
            return ''

    def _await_disarmed(self, timeout_s):
        if self.transport_factory is None:
            from .guarded_transport import GuardedZmqTransport
            self.transport_factory = GuardedZmqTransport
        self.transport = self.transport_factory(self.config['endpoint'], timeout_ms=200)
        deadline = self.clock() + timeout_s
        while True:
            if self.process.poll() is not None:
                tail = self._log_tail() or 'no output'
                hint = f' · {STARTUP_HINT}' if 'startup' in tail else ''
                raise RuntimeError(f'The live receiver stopped while starting: {tail}{hint}')
            try:
                result = self.transport.request({'type': 'status'})
            except TimeoutError:
                result = None
            if result is not None:
                status, feedback = result['status'], result.get('feedback') or []
                if status['state'] == 'faulted':
                    detail = fault_detail(result, self.config['controller'], self.labels)
                    raise RuntimeError(f"The live receiver faulted while starting: {status['reason']}"
                                       + (f' · {detail}' if detail else f' · {STARTUP_HINT}'))
                if (status['state'] == 'disarmed' and len(feedback) == len(self.config['motors'])
                        and all(row['sequence'] > 0 for row in feedback)):
                    return result
            if self.clock() >= deadline:
                raise TimeoutError('The live receiver did not report a disarmed arm with complete feedback in time')
            time.sleep(.05)

    def _save_last(self):
        """The receiver's last status, feedback and counters, for a later look at how the session ended."""
        if self.transport.last is None:
            return
        try:
            path = self.directory / f'live-{self.label}-last.json'
            path.write_text(json.dumps(self.transport.last, indent=1, default=repr) + '\n', encoding='utf-8')
        except (OSError, TypeError, ValueError):
            pass  # Evidence only: never stands in the way of stopping the service.

    def close(self):
        """Stop the service (it disables the arm's motors) and take the link down; safe to repeat."""
        errors = []
        if self.transport is not None:
            self._save_last()
            self.transport.close()
            self.transport = None
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=3)
                    errors.append('the live receiver had to be killed')
            self.exit_code = self.process.returncode
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        while self.links_up:
            try:
                self._link(up=False)
            except Exception as exc:
                errors.append(str(exc))
                break
        if errors:
            raise RuntimeError('; '.join(errors))
