"""Explicit connect/disconnect of the stopped-only live reader for Helm.

Same finite remote reader as the calibration tool: host, adapter, service and
CAN-down checks, full motor identity/firmware/Reset preflight, then stop and
position reads only; CAN links go down on exit. A healthy 15-minute expiry
renews after verified cleanup.

When motors stop answering (a loose plug, a power cycle), the reader shuts down
cleanly and Helm waits, naming the silent motors, then retries until they answer
or the operator disconnects. Every attempt repeats the full identity preflight;
a different UID/firmware, unconfirmed CAN cleanup or a host/adapter problem
stops instead. Nothing here can enable a motor.
"""
import threading
import uuid

from robot_platform.control.joint_calibration import preview_cleanup_verified, save_json

RETRYABLE = frozenset({'incomplete_scans', 'motors_unresponsive'})


def cleanup_confirmed(report, reference):
    """Both CAN links verified down, with no enable or write requested."""
    after = report.get('after', {})
    return (report.get('asset_id') == reference['asset_id'] and report.get('enable_requested') is False
            and report.get('parameter_writes_requested') is False and not report.get('cleanup_errors')
            and set(after) == {m['bus'] for m in reference['motors']}
            and all('UP' not in row.get('flags', ['UP']) for row in after.values()))


def silent_motors(report, reference):
    """Motors that stopped answering after a clean shutdown; None if not a dropout."""
    if (report.get('success') is not False or report.get('error_code') not in RETRYABLE
            or not cleanup_confirmed(report, reference)):
        return None
    return list(report.get('unresponsive') or []) or ['a motor (not identified)']


class LiveLink:
    def __init__(self, session, open_client, *, retry_delays=(2., 3., 5.)):
        if not retry_delays or any(delay <= 0 for delay in retry_delays):
            raise ValueError('Positive retry delays are required')
        self.session, self.open_client = session, open_client
        self.retry_delays = tuple(retry_delays)
        self._guard = threading.Lock()
        self._thread, self._stop = None, threading.Event()
        self.reconnections = 0
        self.last_report = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def connect(self):
        with self._guard:
            if self.running:
                raise ValueError('Already connected or connecting')
            self._stop = threading.Event()
            self.reconnections = 0
            address = getattr(self.session, 'address', self.session.device['address'])
            self._publish('connecting', f"Connecting to {address}: "
                          'checking host, CAN adapters, services and all motor identities.')
            self._thread = threading.Thread(target=self._run, args=(self._stop,), daemon=True, name='helm-live-link')
            self._thread.start()

    def disconnect(self, timeout=10.):
        """Blocks until the reader confirms cleanup; call off the event loop."""
        with self._guard:
            thread, stop = self._thread, self._stop
        if thread is None:
            return
        stop.set()
        thread.join(timeout)
        if thread.is_alive():
            raise RuntimeError('The reader has not confirmed cleanup yet; check that CAN links are down')

    def _publish(self, state, message):
        self.session.link_state(state, message, reconnections=self.reconnections)

    def _run(self, stop):
        client = None
        try:
            client = self.open_client()
            waits = 0
            while True:
                received = [0]

                def receive(frame):
                    received[0] += 1
                    self.session.on_frame(frame)
                try:
                    final = client.stream(receive, stop)
                except Exception as exc:
                    raise RuntimeError(f'{exc}. Cleanup was not confirmed; the remote reader brings CAN down '
                                       'when its heartbeat stops') from exc
                self.last_report = final
                save_json(self.session.evidence / f'link-{uuid.uuid4().hex[:8]}.json', final)
                if received[0]:
                    waits = 0
                if final.get('success'):
                    if not preview_cleanup_verified(final, self.session.reference):
                        raise RuntimeError('Stopped-reader identity and CAN-down cleanup were not confirmed')
                    if final.get('completion_reason') == 'duration_limit' and not stop.is_set():
                        self._publish('reconnecting', 'Renewing the 15-minute stopped reader; last pose held.')
                        continue
                    self._publish('disconnected', 'Disconnected. CAN links are down; motors were never enabled.')
                    return
                silent = silent_motors(final, self.session.reference)
                if silent is None or stop.is_set():
                    raise RuntimeError(final.get('error', 'Stopped reader failed'))
                self.reconnections += 1
                delay = self.retry_delays[min(waits, len(self.retry_delays) - 1)]
                waits += 1
                names = ', '.join(silent)
                self._publish('waiting', f'Not answering: {names}. Check its plug and power; power-cycling is '
                              f'fine. Retrying in {delay:g} s (try {waits}). Sweep progress is kept.')
                if stop.wait(delay):
                    self._publish('disconnected', 'Disconnected. CAN links are down.')
                    return
                self._publish('waiting', f'Checking again for {names} (try {waits})…')
        except Exception as exc:
            self._publish('fault', f'Reader stopped: {exc}')
        finally:
            if client is not None:
                client.close()
