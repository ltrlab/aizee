"""Helm's Gripper zero: set one gripper's motor zero at its fully closed jaws, from the operator's confirmation.

After a gear skip the closed jaws no longer read zero, so the gripper's swept range is off by the skip. The
operator closes the jaws by hand (the gripper zero: inner jaw faces together), confirms in Helm, and this runs
the release's stopped maintenance on the robot (ops.probe zero_gripper): every drive verified, this one
zeroed and saved, nothing enabled. The server stops the live reader first and refuses while anything else holds
the motors. Persistence on the drive is confirmed by a power cycle, as for any re-zero.
"""
import json
import shlex
import threading
import time

from robot_platform.ops.probe import GRIPPERS

TIMEOUT_S = 120.


class GripperZero:
    """`address()` names the robot Helm uses; `open_ssh(address=...)` opens the pinned SSH client."""

    def __init__(self, address, *, open_ssh=None, remote_root=None, clock=time.time):
        from robot_platform.ops.check import CURRENT
        self.address, self.open_ssh, self.clock = address, open_ssh, clock
        self.remote_root = remote_root or CURRENT
        self.lock = threading.Lock()
        self.running = None  # The gripper being zeroed.
        self.last = None

    def zero(self, motor, *, confirm=False):
        """Zero `motor` (blocking: call it off the event loop). Returns the robot's report."""
        if motor not in GRIPPERS:
            raise ValueError('Choose the left or right gripper')
        if confirm is not True:
            raise ValueError('Close the gripper fully by hand, then confirm the zero')
        with self.lock:
            if self.running:
                raise ValueError('A gripper zero is already running')
            self.running = motor
        try:
            report = self._run(motor)
        except Exception as exc:
            report = {'motor': motor, 'success': False, 'error': str(exc) or type(exc).__name__}
        report['finished_at'] = self.clock()
        with self.lock:
            self.running, self.last = None, report
        return report

    def _run(self, motor):
        from robot_platform.ops.ssh import DEFAULT_PROFILE, connect, run
        client = (self.open_ssh or connect)(DEFAULT_PROFILE, address=self.address())
        try:
            command = (f'cd {shlex.quote(self.remote_root)} && PYTHONPATH=src python3 -u -m robot_platform.ops.probe '
                       f'zero_gripper {shlex.quote(motor)}')
            status, out, err = run(client, command, timeout=TIMEOUT_S)
        finally:
            client.close()
        if 'No module named robot_platform.ops.probe' in err or 'zero_gripper MOTOR' in out:
            raise RuntimeError('The deployed release predates Gripper zero: deploy it first')
        try:
            report = json.loads(out.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise RuntimeError(f'The robot did not answer: {err.strip()[-300:] or f"exit {status}"}') from None
        if not report.get('ok'):
            raise RuntimeError(report.get('error') or 'The gripper zero failed on the robot')
        report.pop('ok', None)
        return report

    def snapshot(self):
        with self.lock:
            return {'running': self.running, 'last': dict(self.last) if self.last else None}
