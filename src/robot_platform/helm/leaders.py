"""Helm's Leaders workspace: sweep each leader channel's full range and save it.

Read-only: the leader boards are only polled and their servos stay torque-free.
While connected the workspace owns the leader USB ports, so it excludes a teleop
session. Saved ranges are what teleop scales onto the robot's joint ranges, from
the leader zero (the pose that maps onto the robot's reference pose). The zero
can be set here or from a running teleop session, at any time; it saves at once.
"""
from copy import deepcopy
import json
from pathlib import Path
import threading
import time
import uuid

from robot_platform.control.joint_calibration import save_json
from robot_platform.teleop.leader_ranges import (LeaderRangeRecorder, channels, load_ranges, range_table,
                                                 reference_table, with_reference, wrap)


class LeaderWorkspace:
    poll_s = .02

    def __init__(self, teleop_profile, *, labels=None, evidence=None, open_leaders=None, clock=time.perf_counter):
        path = Path(teleop_profile).resolve()
        teleop = json.loads(path.read_text(encoding='utf-8'))
        self.profile_path = (path.parent / teleop['leader_profile']).resolve()
        self.profile = json.loads(self.profile_path.read_text(encoding='utf-8'))
        self.path = (path.parent / teleop['leader_ranges']).resolve()
        self.drives = {(route['leader'], route['channel']): motor for motor, route in teleop['routes'].items()}
        self.labels = dict(labels or {})
        self.evidence = Path(evidence) if evidence else None
        self.open_leaders = open_leaders or self._open_leaders
        self.clock = clock
        self.lock = threading.RLock()
        self.saved, self.saved_error = load_ranges(self.path, self.profile)
        self.recorder = LeaderRangeRecorder(self.profile, saved=self._saved_spans())
        self.state = 'disconnected'
        self.message = 'Leaders not connected. Connect them to sweep their ranges (read-only).'
        self.received = None
        self.skipped = 0
        self._thread, self._stop = None, threading.Event()

    def _saved_spans(self):
        table = range_table(self.saved, self.profile)
        return {key: table[leader][index] for key, leader, _name, index, _label in channels(self.profile)
                if table[leader][index] is not None}

    def _open_leaders(self, profile):
        from robot_platform.teleop.leader_bridge import LatestLeaderInputs
        from robot_platform.teleop.openrb import OpenRBInputs
        leaders = LatestLeaderInputs(OpenRBInputs(profile), rate_hz=100.)
        leaders.connect()
        return leaders

    # -- link ----------------------------------------------------------------

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def connect(self):
        with self.lock:
            if self.running:
                raise ValueError('The leaders are already connected')
            self._stop = threading.Event()
            self.state, self.message = 'connecting', 'Opening the leader boards…'
            self._thread = threading.Thread(target=self._run, args=(self._stop,), daemon=True, name='helm-leaders')
            self._thread.start()

    def disconnect(self, timeout=5.):
        with self.lock:
            thread, stop = self._thread, self._stop
        if thread is None:
            return
        stop.set()
        thread.join(timeout)

    def _run(self, stop):
        from robot_platform.teleop.openrb import LeaderSampleSkipped
        try:
            leaders = self.open_leaders(self.profile)
        except Exception as exc:
            with self.lock:
                self.state = 'fault'
                self.message = (f'Could not open the leader arms ({exc}). Close the leader viewer or any other '
                                'program using their USB ports, then connect again.')
            return
        with self.lock:
            self.recorder.restart()
            self.state, self.message = 'reading', 'Reading the leaders.'
        try:
            while not stop.wait(self.poll_s):
                try:
                    samples = leaders.poll()
                except LeaderSampleSkipped:
                    with self.lock:
                        self.skipped += 1
                    continue
                with self.lock:
                    now = self.clock()
                    self.recorder.observe(samples, now=now)
                    self.received = now
        except Exception as exc:
            with self.lock:
                self.state, self.message = 'fault', f'Leader reading stopped: {exc}. Connect again.'
        finally:
            leaders.close()
            with self.lock:
                if self.state != 'fault':
                    self.state, self.message = 'disconnected', 'Leaders disconnected.'

    def fresh(self, limit=.5):
        with self.lock:
            return self.state == 'reading' and self.received is not None and self.clock() - self.received <= limit

    # -- operator actions ------------------------------------------------------

    def act(self, action, key):
        with self.lock:
            if action == 'confirm':
                if not self.fresh():
                    raise ValueError('Confirm while the leaders are connected, so the sweep reflects live readings')
                self.recorder.confirm(key)
            elif action == 'unconfirm':
                self.recorder.unconfirm(key)
            elif action == 'reset':
                self.recorder.reset(key)
            else:
                raise ValueError('Unknown leader range action')

    def capture_reference(self):
        """Set the leader zero to the leaders' pose now, and save it."""
        with self.lock:
            readings = self.recorder.readings()
            if not self.fresh() or readings is None:
                raise ValueError('Set the zero while the leaders are connected and reading')
            captured = {}
            for key, leader, name, _index, _label in channels(self.profile):
                captured.setdefault(leader, {})[name] = readings[key]
            return self.store_reference(captured)

    def store_reference(self, captured):
        """Save a leader zero {leader: {channel: reading_rad}} (from this tab or a teleop session)."""
        with self.lock:
            existing = self.saved if self.saved_error is None else None
            document = with_reference(existing, self.profile, captured,
                                      session=self.evidence.name if self.evidence else None)
            self._write(document)
            return document['reference']

    def _write(self, document):
        if self.evidence is not None:
            self.evidence.mkdir(parents=True, exist_ok=True)
            if self.path.is_file() and self.saved is None:
                # Keep a copy of anything replaced; the evidence folder is local.
                save_json(self.evidence / ('replaced-' + self.path.name),
                          json.loads(self.path.read_text(encoding='utf-8')))
        save_json(self.path, document)
        if self.evidence is not None:
            save_json(self.evidence / ('saved-' + uuid.uuid4().hex[:8] + '-' + self.path.name), document)
        self.saved, self.saved_error = document, None

    def save(self):
        with self.lock:
            existing = self.saved if self.saved_error is None else None
            session = self.evidence.name if self.evidence else None
            document = self.recorder.document(self.profile, existing=existing, session=session)
            self._write(document)
            self.recorder.saved = self._saved_spans()
            self.recorder.confirmed.clear()
            return {'path': str(self.path), 'channels': sum(len(rows) for rows in document['leaders'].values())}

    def missing(self, routes):
        """Routed leader channels without a saved range, as operator-readable names."""
        with self.lock:
            table = range_table(self.saved, self.profile)
        joints = {name: item['joints'] for name, item in self.profile['leaders'].items()}
        result = []
        for route in routes.values():
            leader, index = route['leader'], route['channel'] - 1
            if leader not in table or not 0 <= index < len(table[leader]) or table[leader][index] is None:
                items = joints.get(leader, [])
                label = items[index].get('label', '') if 0 <= index < len(items) else ''
                result.append(f"{leader} {label.lower() or 'channel ' + str(route['channel'])}")
        if routes and reference_table(self.saved, self.profile) is None:
            result.append('the leader zero')
        return result

    # -- presentation ------------------------------------------------------------

    def snapshot(self):
        with self.lock:
            captured = ((self.saved or {}).get('reference') or {}).get('leaders', {})
            rows = []
            for row in self.recorder.rows():
                motor = self.drives.get((row['leader'], row['channel']))
                angle = captured.get(row['leader'], {}).get(row['name'])
                marker = None  # The zero in the current reading's turn, for the bar.
                if angle is not None and row['current'] is not None:
                    marker = row['current'] + wrap(angle - row['current'])
                rows.append({**row, 'drives': motor, 'drives_label': self.labels.get(motor, motor),
                             'reference': marker})
            saved_reference = (self.saved or {}).get('reference')
            saved = (self.saved or {}).get('leaders', {})
            age = None if self.received is None else self.clock() - self.received
            return {'state': self.state, 'message': self.message, 'fresh': self.fresh(), 'age_seconds': age,
                    'skipped': self.skipped, 'profile': self.profile['id'], 'path': str(self.path),
                    'titles': {name: item.get('title', name) for name, item in self.profile['leaders'].items()},
                    'saved_at': (self.saved or {}).get('updated_at'), 'saved_error': self.saved_error,
                    'saved_channels': sum(len(entries) for entries in saved.values()),
                    'total_channels': len(rows), 'rows': deepcopy(rows), 'pending': len(self.recorder.confirmed),
                    'reference': {'captured_at': (saved_reference or {}).get('captured_at')}}
