"""Minerva's UPS in Helm: the robot service's battery state (robot_platform.ops.ups) for the header's battery pill
and the on-battery banner, and Helm's part in a low-battery shutdown.

A watcher keeps one command open on the robot, over the pinned SSH connection, that prints the service's state
file every 2 s with its age by the robot's own clock (this PC's clock is never compared with the robot's). When
the robot announces a shutdown, Helm stops a recording, then the session and any sweep through their own stop
paths: the loop lowers an enabled arm before the receiver closes, well inside the service's grace period. Until
the power comes back, starting anything that moves the arms is refused.
"""
import json
import shlex
import threading
import time

from robot_platform.ops.ups import STATE

STALE_S = 10.   # A state the service wrote longer ago than this (robot clock): the service is not running.
SILENT_S = 8.   # No line from the robot for this long (PC clock): the link is down; reconnect.

# Runs on the robot (python3 -u -c WATCH <state path>): the state and its age every 2 s, off the control CPUs.
WATCH = r'''
import json, os, sys, time
path = sys.argv[1]
try:
    os.nice(10)
except Exception:
    pass
try:  # The background CPUs when the split is installed (robot_platform.ops.cpu_partition).
    os.sched_setaffinity(0, json.load(open('/etc/minerva/cpu-partition.json'))['background'])
except Exception:
    pass
while True:
    record = {'missing': False}
    try:
        with open(path, encoding='utf-8') as handle:
            state = json.load(handle)
        now = time.time()
        record['state'] = state
        updated = state.get('updated_at')
        record['age_s'] = round(now - updated, 1) if isinstance(updated, (int, float)) else None
        shutdown = state.get('shutdown')
        if isinstance(shutdown, dict) and isinstance(shutdown.get('poweroff_by'), (int, float)):
            record['poweroff_in_s'] = max(0, round(shutdown['poweroff_by'] - now))
    except FileNotFoundError:
        record['missing'] = True
    except Exception as exc:
        record['error'] = str(exc) or type(exc).__name__
    print(json.dumps(record), flush=True)
    time.sleep(2)
'''


def robot_ups(profile, *, open_ssh=None, timeout=5., path=STATE):
    """open(address) -> a line stream of the robot's UPS state, over the pinned SSH connection."""
    def open_connection(address):
        from robot_platform.ops.ssh import connect, open_command
        client = (open_ssh or connect)(profile, address=address, timeout=timeout)
        try:
            _stdin, out, _err = open_command(client, 'python3 -u -c ' + shlex.quote(WATCH) + ' ' + shlex.quote(path),
                                             timeout=timeout)
            out.channel.settimeout(SILENT_S)  # A line comes every 2 s; silence means the link is down.
        except BaseException:
            client.close()
            raise

        class Connection:
            def readline(self):
                line = out.readline()
                return line.decode() if isinstance(line, bytes) else line

            def close(self):
                out.channel.close()
                client.close()
        return Connection()
    return open_connection


class UpsWatch:
    """The robot's latest UPS state for Helm. `transport(address)` opens a line stream (robot_ups); `address()`
    is where Minerva is reached now (tether or Wi-Fi). `on_shutdown(record)` runs once per announced shutdown."""

    def __init__(self, transport, address, *, on_shutdown=None, retry_s=10., clock=time.monotonic):
        self.transport, self.address, self.on_shutdown = transport, address, on_shutdown
        self.retry_s, self.clock = retry_s, clock
        self._lock = threading.Lock()
        self._record = self._heard_at = self._error = self._connection = None
        self._good = None           # The latest record with a fresh state: shown as "last seen" when offline.
        self._handled = None        # The announced shutdown already acted on (its start time).
        self._closed = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name='helm-ups')
            self._thread.start()

    def close(self):
        self._closed.set()
        with self._lock:
            connection = self._connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(5)

    def _run(self):
        while not self._closed.is_set():
            where = self.address()
            try:
                connection = self.transport(where)
            except Exception as exc:
                self._fail(exc)
                self._closed.wait(self.retry_s)
                continue
            with self._lock:
                self._connection = connection
            try:
                while not self._closed.is_set():
                    line = connection.readline()
                    if not line:
                        raise EOFError("Minerva's UPS reading ended")
                    self.take(json.loads(line))
                    if self.address() != where:
                        break  # Helm switched between the tether and Wi-Fi: read over the new route.
            except Exception as exc:
                if not self._closed.is_set():
                    self._fail(exc)
            finally:
                with self._lock:
                    self._connection = None
                try:
                    connection.close()
                except Exception:
                    pass
            if not self._closed.is_set() and self.address() == where:
                self._closed.wait(self.retry_s)

    def _fail(self, exc):
        with self._lock:
            self._error = str(exc) or type(exc).__name__

    def take(self, record):
        """One line from the robot."""
        if not isinstance(record, dict):
            return
        state = record.get('state') if isinstance(record.get('state'), dict) else None
        fresh = state is not None and isinstance(record.get('age_s'), (int, float)) and record['age_s'] <= STALE_S
        with self._lock:
            self._record, self._heard_at, self._error = record, self.clock(), None
            if fresh:
                self._good = (record, self._heard_at)
        shutdown = state.get('shutdown') if fresh else None
        if isinstance(shutdown, dict) and shutdown.get('since') != self._handled:
            self._handled = shutdown.get('since')
            if self.on_shutdown is not None:
                threading.Thread(target=self.on_shutdown, args=(record,), daemon=True,
                                 name='helm-ups-shutdown').start()

    def snapshot(self):
        """What the pill and banner show: status ok, missing (no service), stale, unreachable or starting."""
        with self._lock:
            record, heard_at, error, good = self._record, self._heard_at, self._error, self._good
        now = self.clock()
        heard = heard_at is not None and now - heard_at <= SILENT_S
        result = {'status': 'starting', 'state': None, 'age_s': None, 'poweroff_in_s': None, 'error': error,
                  'last': None}
        if not heard:
            if error is not None or heard_at is not None:
                result['status'] = 'unreachable'
            if good is not None:
                result['last'] = dict(good[0].get('state') or {}, seen_s_ago=round(now - good[1]))
            return result
        result['age_s'] = record.get('age_s')
        if record.get('missing'):
            result['status'] = 'missing'
        elif record.get('state') is None or record.get('age_s') is None or record['age_s'] > STALE_S:
            result.update(status='stale', error=record.get('error') or error)
        else:
            result.update(status='ok', state=record['state'], poweroff_in_s=record.get('poweroff_in_s'))
        return result

    def refusal(self):
        """Why nothing that moves the arms may start now: a fresh, announced UPS shutdown; else None."""
        view = self.snapshot()
        state = view['state'] or {}
        if view['status'] == 'ok' and (state.get('shutdown') or (state.get('source') == 'battery'
                                                                  and state.get('level') == 'critical')):
            return ("Minerva's UPS battery is critical and the Jetson is shutting down. Plug in the UPS charger; "
                    'starts are allowed again once the shutdown is cancelled.')
        return None
