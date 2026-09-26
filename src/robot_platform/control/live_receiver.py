"""The guarded receiver on Minerva's physical CAN for one arm, for Helm's live teleop.

Runs on the robot computer, in the teleop loop's process, from the active
release. Starting it:
  1. The calibration tool's stopped reader scans the arm with every motor
     disabled: host, adapter serial, services, identity, firmware and fault-free
     Reset, and each motor still and within tolerance of its reference position.
     Encoders cannot prove the physical pose; the operator confirms it too.
  2. The arm's CAN link comes up through the sudo-permitted helper.
  3. guarded_controller runs in teleop mode on that one bus. It repeats the
     identity and reference checks and arms the motors' command-loss watchdogs
     before opening its socket; its first enable spends that permit.
Closing stops the service, which disables every motor on its bus on the way
out, then takes the CAN link down. The service dies with this process (parent
death signal). Nothing here writes a zero, a parameter or flash.
"""
import json
from pathlib import Path
import signal
import subprocess
import time

ENDPOINT = 'tcp://127.0.0.1:5590'
HOST_ID = 0xAA  # The stopped reader's host id too.


def service_config(controller, device, reference, motor_models, *, bus, tolerance_rad, period_us=2500,
                   maximum_lateness_us=10000, endpoint=ENDPOINT):
    """The teleop-mode service configuration for `controller`'s joints, all on `bus`."""
    from .commissioning import wire_ranges
    roster = {motor['name']: motor for motor in reference['motors']}
    names = [joint['name'] for joint in controller['joints']]
    if any(roster[name]['bus'] != bus for name in names):
        raise ValueError(f'A live receiver drives one arm: every joint must be on {bus}')
    serial = next((b['usb_serial'] for b in device['buses'] if b['interface'] == bus), None)
    if not serial:
        raise ValueError(f'{bus} is not a pinned bus of {device["asset_id"]}')
    return {'mode': 'teleop', 'endpoint': endpoint, 'period_us': period_us,
            'maximum_lateness_us': maximum_lateness_us, 'host_id': HOST_ID,
            'buses': [{'interface': bus, 'usb_serial': serial}],
            'conflicting_services': list(device['conflicting_services']),
            'motors': [{'name': name, 'bus': 0, 'id': roster[name]['id'], 'scale': 1., 'zero': 0.,
                        'ranges': wire_ranges(motor_models[name])} for name in names],
            'controller': controller,
            'watchdog_identities': [{'uid': list(bytes.fromhex(roster[name]['uid'])),
                                     'firmware': [int(part) for part in roster[name]['firmware'].split('.')]}
                                    for name in names],
            'reference_positions': [{'name': name, 'raw_position_rad': roster[name]['raw_reference_rad'],
                                     'tolerance_rad': tolerance_rad} for name in names]}


def reference_offsets(report, config):
    """[(motor, offset_rad)] for scanned motors outside their reference tolerance."""
    references = {row['name']: row for row in config['reference_positions']}
    scanned = {row['name']: row['position_rad'] for row in report.get('motors', [])}
    if set(scanned) != set(references):
        raise RuntimeError('The stopped scan did not cover every motor of the arm')
    return [(name, scanned[name] - row['raw_position_rad']) for name, row in references.items()
            if abs(scanned[name] - row['raw_position_rad']) > row['tolerance_rad']]


def _die_with_parent():
    import ctypes
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class LiveReceiver:
    """One arm's guarded service, owned by this process. Exchange-compatible with the harness."""

    def __init__(self, config, *, device, reference, binary, directory, labels=None, capture=None,
                 command=subprocess.run, spawn=subprocess.Popen, transport_factory=None, clock=time.monotonic):
        self.config, self.device, self.reference = config, device, reference
        self.bus = config['buses'][0]['interface']
        self.binary, self.directory = Path(binary), Path(directory)
        self.labels = dict(labels or {})
        self.capture, self.command, self.spawn = capture, command, spawn
        self.transport_factory, self.clock = transport_factory, clock
        self.process = self.transport = self.log = self.scan = self.exit_code = None
        self.can_up = False

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
        device = dict(self.device, buses=[b for b in self.device['buses'] if b['interface'] == self.bus])
        reference = dict(self.reference, motors=[m for m in self.reference['motors'] if m['name'] in names])
        report = (self.capture or run_capture)(device, reference)
        if not report.get('success'):
            raise RuntimeError(f"The stopped check of the arm failed: {report.get('error', 'no report')}")
        offsets = reference_offsets(report, self.config)
        if offsets:
            tolerance = self.config['reference_positions'][0]['tolerance_rad']
            raise ValueError('Not in the reference pose: ' + ', '.join(
                f'{self.labels.get(name, name)} is {offset:+.2f} rad away' for name, offset in offsets)
                + f' (tolerance {tolerance:g} rad). Let the arm hang straight down with the gripper closed, '
                  'then start again.')
        return report

    def _link(self, *, up):
        command = (['sudo', '-n', self.device['can_up_helper'], self.bus] if up
                   else ['sudo', '-n', '/usr/sbin/ip', 'link', 'set', self.bus, 'down'])
        result = self.command(command, capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(f"Could not bring {self.bus} {'up' if up else 'down'}: "
                               f"{(result.stderr or result.stdout).strip()}")
        self.can_up = up

    def _spawn(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / f'live-{self.bus}.json'
        path.write_text(json.dumps(self.config, indent=1, allow_nan=False) + '\n', encoding='utf-8')
        self.log = (self.directory / f'live-{self.bus}.log').open('ab')
        self.process = self.spawn([str(self.binary), str(path), '--allow-powered-teleop', '--confirm-reference-pose'],
                                  stdout=self.log, stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)

    def _log_tail(self):
        try:
            return (self.directory / f'live-{self.bus}.log').read_text(errors='replace').strip()[-400:]
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
                raise RuntimeError(f'The live receiver stopped while starting: {self._log_tail() or "no output"}')
            try:
                result = self.transport.request({'type': 'status'})
            except TimeoutError:
                result = None
            if result is not None:
                status, feedback = result['status'], result.get('feedback') or []
                if status['state'] == 'faulted':
                    raise RuntimeError(f"The live receiver faulted while starting: {status['reason']}")
                if (status['state'] == 'disarmed' and len(feedback) == len(self.config['motors'])
                        and all(row['sequence'] > 0 for row in feedback)):
                    return result
            if self.clock() >= deadline:
                raise TimeoutError('The live receiver did not report a disarmed arm with complete feedback in time')
            time.sleep(.05)

    def close(self):
        """Stop the service (it disables the arm's motors) and take the link down; safe to repeat."""
        errors = []
        if self.transport is not None:
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
        if self.can_up:
            try:
                self._link(up=False)
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
