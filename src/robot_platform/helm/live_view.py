"""Helm's live view of the arms, kept running whenever Minerva is online.

The stopped reader (helm.link) reads every drive with its torque off; nothing in
it can enable a motor. LiveView starts it whenever Minerva is reached over the
tether and nothing else holds the motors (a teleop session, a servo sweep or
return, Minerva Check), stops it before one of those starts, and starts it again
once it ends. When Minerva is offline the reader's connection fails; it tries
again every OFFLINE_RETRY_S. Disconnect in Helm's header turns the live view
off, Connect turns it on again.
"""
import threading
import time

POLL_S = 1.
OFFLINE_RETRY_S = 10.
# After the reader stops for a session, it stays off this long for the session to take the motors. A session
# holding them keeps it off; one that never started lets it start again.
HANDOFF_S = 5.


class LiveView:
    """`owner()` names what holds the motors (or None). Call tick() to act now; start() polls it."""

    def __init__(self, session, link, owner, *, clock=time.monotonic, poll_s=POLL_S, retry_s=OFFLINE_RETRY_S,
                 handoff_s=HANDOFF_S):
        self.session, self.link, self.owner = session, link, owner
        self.clock, self.poll_s, self.retry_s, self.handoff_s = clock, poll_s, retry_s, handoff_s
        self.enabled = True
        self._lock = threading.Lock()
        self._quiet_until = 0.
        self._failed = False
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name='helm-live-view')
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)

    def _run(self):
        while not self._stop.wait(self.poll_s):
            try:
                self.tick()
            except Exception:  # The reader reports its own failures; the view keeps trying.
                pass

    def tick(self):
        """Start the reader when it should run and doesn't. True when it was started."""
        with self._lock:
            now = self.clock()
            if not self.enabled or self.link.running or now < self._quiet_until:
                return False
            if getattr(self.session, 'connection', 'tether') != 'tether' or self.owner() is not None:
                return False
            if self.session.feed.get('state') == 'fault' and not self._failed:
                # Its last try failed (Minerva offline, or a fault it names): try again in a while.
                self._failed, self._quiet_until = True, now + self.retry_s
                return False
            self._failed = False
            self.link.connect()
            return True

    def yield_motors(self):
        """Stop the reader for a session that needs the motors (blocks until its CAN links are down)."""
        with self._lock:
            self._quiet_until = self.clock() + self.handoff_s
        self.link.disconnect()

    def set_enabled(self, enabled):
        with self._lock:
            self.enabled, self._quiet_until, self._failed = bool(enabled), 0., False

    def snapshot(self):
        return {'enabled': self.enabled, 'owner': self.owner()}
