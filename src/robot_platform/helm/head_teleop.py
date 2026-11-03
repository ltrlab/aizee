"""The head in a teleop session: its own receiver beside the arms', driven through the same lifecycle (Idle, Enable,
Engage, Return, Disable) as the arms, one session for both.

Its target is the operator's headset (helm.xr: yaw turns neck_z, pitch turns neck_y, one to one from the headset's
zero) while a pose is fresh. Without one it holds its last target for a moment, then goes back to its zero pose, so
with no headset at all the head holds still at zero. Every target goes through the arms' lifecycle classes (the
direct law: speed caps, alignment, braking near the ends) and the head receiver's own limits. With its shoulder
clearance saved (neck.clearance), each target keeps clear of the shoulder too, and steps there so its path does.

The head cannot stay at its reference: limp, it droops to where it rests (the user, 2026-10-30). A session starts
there (its receiver checks the head is still where it was read resting, neck.head.read_rest), Engage raises it
slowly, and Return (X), or the session's end, takes it back to its reference, then down to where it rested, before
its torque goes: it is never dropped from straight ahead, and never crosses the shoulder corner on the way.
"""
from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

from robot_platform.neck import clearance

PERIOD_S = .04  # The head's commands at 25 Hz (its profile's loop_hz); its receiver ramps each over 40 ms.
# The head cycles on the arms' ticks, on a grid of PERIOD_S slots: a tick this early still takes its slot (the ticks
# meet the grid only to float precision), and a late one does not push the next slot. Each command takes the offer of
# the reply before it, so two consecutive gaps must stay under the direct policy's command lifetime (100 ms).
DUE_SLACK_S = PERIOD_S / 4
CYCLE_BUDGET_S = .035
POSE_FRESH_S = .2  # A headset pose this recent steers the head;
RETURN_AFTER_S = 1.  # without one the head holds its last target this long, then returns to its zero pose.
# With a shoulder clearance, the head's targets step at this share of its speeds: steps the lifecycle takes whole.
SPEED_SHARE = .9
# The head follows at most this share of its profile's speed, catching up included: its receiver trips at 1.25
# times that speed (control.approach OVERSPEED_HEADROOM), and coming down at 1.0 rad/s its tilt read 1.30 (2026-11-02).
FOLLOW_SHARE = .7


class HeadTeleop:
    """`bundle`: the head's profile path, reference, limits and servo calibration (helm.tune's head bundle shape).
    Openers are injectable for tests, as the arms' are."""

    def __init__(self, bundle, *, live, clock=time.perf_counter, open_receiver=None, open_harness=None,
                 open_feed=None):
        self.bundle, self.live, self.clock = deepcopy(bundle), live, clock
        self.open_receiver = open_receiver or self._open_receiver
        self.open_harness = open_harness or self._open_harness
        self.open_feed = open_feed or self._open_feed
        self.feed = None  # The head's measured pose for the headset's world-locked 3D view (neck.head_pose).
        self.derived = self.session = self.receiver = self.model = None
        self.fault = None
        self._closers = []
        self._pose = None  # (yaw, pitch, received at) from the headset, on this loop's clock.
        self._poses = 0
        self._held = None  # The last target followed, held while the headset is briefly quiet.
        self._due = None  # The next slot on the head's grid (helm clock); None before the first cycle.
        self._sequence = 0
        self.clearance = None
        self._stepped, self._aligned, self.limited = None, False, False  # Stepping clear of the shoulder (_clear).
        self.rest = None  # Where the head rested when the session began ({motor: raw rad}): Return ends there.
        self.returning = None  # Return's stage: 'centre' (to the reference), then 'rest'; None otherwise.
        self._disable_next = False

    # -- resources -------------------------------------------------------------------------

    def _open_receiver(self, config, reference):
        from pathlib import Path
        from robot_platform.neck.head import HeadReceiver, read_rest, service_config
        from robot_platform.ops.release import ROOT
        self.rest = read_rest(self.profile, self.bundle['limits'])
        service = service_config(self.profile, reference, self.derived, mode='teleop_direct', start=self.rest)
        evidence = Path.home() / 'minerva' / 'evidence' / time.strftime('live-head-%Y%m%dT%H%M%SZ', time.gmtime())
        return HeadReceiver(service, binary=ROOT / 'bin' / 'head_controller', directory=evidence,
                            labels=self.labels).start()

    def _open_feed(self, camera_pose):
        """The head's pose feed on its local PUB (neck.head_pose.ENDPOINT), or None where the port is taken: the
        headset's 3D view then follows the eyes, as without a session."""
        import zmq
        from robot_platform.neck.head_pose import ENDPOINT, HeadPoseFeed
        socket = zmq.Context.instance().socket(zmq.PUB)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDHWM, 50)
        try:
            socket.bind(ENDPOINT)
        except zmq.ZMQError:
            socket.close(0)
            return None
        self._closers.append(lambda: socket.close(0))
        return HeadPoseFeed(camera_pose, self.axes, self.names, socket)

    def _open_harness(self, config, initial):
        from robot_platform.simulation.control_harness import NativeControlHarness
        harness = NativeControlHarness(config, initial)
        harness.step(0)
        return harness

    def build(self):
        from robot_platform.control.guarded import GuardedControllerClient
        from robot_platform.models import ModelProfile, UrdfModel
        from robot_platform.neck import head
        from robot_platform.ops.release import ROOT
        from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
        from robot_platform.teleop.lifecycle import TeleopLifecycle
        path = Path(self.bundle['teleop_profile'])
        self.profile = head.load_profile(path if path.is_absolute() else ROOT / path)  # Release-relative on the robot.
        reference = self.bundle['head_reference']
        self.labels = {motor['name']: motor['label'] for motor in self.profile['motors']}
        self.model = UrdfModel(ModelProfile.load(ROOT / self.profile['model_profile']))
        self.derived = derived = head.derive(self.profile, reference, self.bundle['limits'], self.model,
                                             servo=self.bundle.get('servo'))
        config = derived['config']
        self.usable = {row['motor']: row['usable'] for row in derived['report']}
        # The shoulder clearance (neck.clearance), when one is saved for this reference: its targets then step.
        self.clearance = clearance.from_limits(self.bundle['limits'], reference)
        by_joint = {m['joint']: m for m in reference['motors']}
        self.pan, self.tilt = by_joint[clearance.PAN]['name'], by_joint[clearance.TILT]['name']
        self.centre = {m['name']: float(m['raw_reference_rad']) for m in reference['motors']}
        if self.live:
            receiver = self.open_receiver(config, reference)

            def exchange(request):
                reply = receiver.exchange(request)
                self._publish_pose(receiver.last)  # Every reply: each carries only the rows since the one before.
                return reply
        else:
            # No head to read: the virtual one rests where the real one droops, limp, tilted down to the end of its
            # range on the shoulder's down side (its saved clearance says which way that is), so Return shows the
            # real path, never ending straight ahead, where the real head would fall (the user, 2026-10-31). A
            # rehearsal may say where (`rest_pose`); with no clearance saved yet, it rests at its zero.
            joints = [joint['name'] for joint in config['joints']]
            self.rest = dict(zip(joints, derived['initial']))
            if self.clearance is not None:
                low, high = self.usable[self.tilt]
                self.rest[self.tilt] = low if self.clearance.down < 0 else high
            self.rest.update(self.bundle.get('rest_pose') or {})
            receiver = self.open_harness(config, [self.rest[name] for name in joints])
            epoch = self.clock()

            def exchange(request):
                now_ms = max(receiver.now_ms, int((self.clock() - epoch) * 1000))
                if now_ms > receiver.now_ms:
                    receiver.step(now_ms)
                return receiver.step(now_ms, request)['reply']
        self._closers.append(receiver.close)
        self.receiver = receiver
        names = tuple(joint['name'] for joint in config['joints'])
        controller = GuardedControllerClient(exchange, group=config['group'], identity=config['identity'],
                                             joint_names=names)
        # Each cycle reads the reply of the cycle before (DirectLifecycleSession), so its feedback and target ages
        # are bounded by the head's period, not the arms' 10 ms one.
        # Its commands keep under FOLLOW_SHARE of the head's speed; its receiver keeps its own cap.
        rows = [dict(row, max_velocity=FOLLOW_SHARE * row['max_velocity']) for row in derived['lifecycle']]
        lifecycle = TeleopLifecycle(rows, clock=self.clock, max_step_s=2 * PERIOD_S,
                                    feedback_max_age_s=1.5 * PERIOD_S, target_max_age_s=2.5 * PERIOD_S,
                                    allow_repeated_source=True)
        # A cycle following nothing (Idle, Enable, Hold) takes the receiver, renews it or takes a fresh offer at
        # the head service's 10 ms ticks before its command: it has that command's lifetime, which the receiver
        # bounds it by, as the arms' holds have their verdict's. Following cycles keep the cycle budget.
        self.session = DirectLifecycleSession(controller, config, lifecycle, lambda: receiver.last,
                                              period_s=PERIOD_S, cycle_budget_s=CYCLE_BUDGET_S,
                                              hold_budget_s=config['direct']['command_ms'] / 1000, clock=self.clock)
        self.names = names
        self.zero = {row['name']: row['zero_position'] for row in derived['lifecycle']}
        # The headset's yaw and pitch are the CAD angles of neck_z and neck_y; each motor maps from its CAD joint.
        self.axes = {row['actuator']: (joint, row['scale'], row['offset'])
                     for joint, row in derived['joint_map'].items()}
        self.speeds = {row['name']: {'align': row['align_speed'], 'follow': row['max_velocity'],
                                     'return': row['shutdown_speed']} for row in rows}
        self.tolerance = {row['name']: row['tolerance'] for row in derived['lifecycle']}
        # Return ends where the head rested, inside its usable range and clear of the shoulder: the last of a droop
        # past them happens limp.
        rest = dict(self.zero, **(self.rest or {}))
        rest = {name: min(max(rest[name], self.usable[name][0]), self.usable[name][1]) for name in names}
        if self.clearance is not None:
            pan, tilt = self.clearance.clamp(rest[self.pan] - self.centre[self.pan],
                                             rest[self.tilt] - self.centre[self.tilt])
            rest.update({self.pan: self.centre[self.pan] + pan, self.tilt: self.centre[self.tilt] + tilt})
        self.rest_target = rest
        if self.live and self.profile.get('camera'):
            self.feed = self.open_feed(self._camera_pose(reference))

    def _camera_pose(self, reference):
        """The head camera's pose for the feed: through the mount measured on the table's tags (the head's camera
        file, helm.head_alignment) when it was made through this reference, else the CAD one; with the digital twin's
        correction of the pan zero (vision.twin, helm.twin) when one holds for that alignment and reference."""
        from robot_platform.neck.head_pose import CameraPose, aligned_mount
        from robot_platform.vision.twin import head_offsets
        mount, saved = None, self.bundle.get('camera')
        self.camera_mount = 'cad: no head camera alignment saved'
        self.camera_offsets, self.camera_twin = {}, 'none: no correction of the head applied'
        if saved is not None:
            try:
                mount = aligned_mount(saved, self.profile['camera']['link'], reference)
                self.camera_mount = 'aligned'
            except ValueError as exc:
                self.camera_mount = f'cad: {exc}'
        correction = self.bundle.get('twin')
        if correction is not None and correction.get('head') is not None:
            try:
                if mount is None:  # The pan was compared through the aligned mount, never the CAD one.
                    raise ValueError(f'the camera is on its CAD mount ({self.camera_mount[5:]})')
                self.camera_offsets = head_offsets(correction, reference, saved)
                self.camera_twin = 'corrected'
            except ValueError as exc:
                self.camera_twin = f'none: {exc}'
        return CameraPose(self.model, self.profile['camera'], mount, self.camera_offsets)

    def _publish_pose(self, last):
        """A reply's rows on the head's pose feed. The feed is for the headset's view only: a failure there ends
        the feed (the view then follows the eyes), never the head."""
        if self.feed is None:
            return
        try:
            self.feed.publish(last, time.time())
        except Exception:
            self.feed = None

    # -- the headset -------------------------------------------------------------------------

    def pose(self, yaw, pitch, *, now=None):
        """The headset's latest yaw and pitch (radians, from its zero), as it arrived in this loop."""
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            raise ValueError('A headset pose is finite yaw and pitch')
        self._pose = (float(yaw), float(pitch), self.clock() if now is None else now)
        self._poses += 1

    def _following(self, now):
        """The positions the head should take now ({motor: raw rad}) and where they come from."""
        if self.returning is not None:
            return dict(self.zero if self.returning == 'centre' else self.rest_target), 'returning'
        if self._pose is not None and now - self._pose[2] <= POSE_FRESH_S:
            angles = {'neck_z': self._pose[0], 'neck_y': self._pose[1]}
            positions = {}
            for motor, (joint, scale, offset) in self.axes.items():
                low, high = self.usable[motor]
                positions[motor] = min(max((angles[joint] - offset) / scale, low), high)
            self._held = positions
            return positions, 'headset'
        if self._held is not None and self._pose is not None and now - self._pose[2] <= RETURN_AFTER_S:
            return self._held, 'holding'
        self._held = None
        return dict(self.zero), 'zero'

    def _clear(self, desired):
        """Toward `desired` in steps the lifecycle takes whole, each clear of the shoulder: the lifecycle moves each
        joint at its own speed, so one long move toward a clear pose could cut through the corner below it. Aligning
        (ENGAGING), the head holds where it is; following, it goes at the alignment speed until it first reaches its
        target, then at the head's speed; returning, at its shutdown speed. Without a clearance saved, `desired` as
        it is, except returning: those steps keep the two stages a straight line each."""
        self.limited = False
        floor = self.clearance or (clearance.unbounded() if self.returning is not None else None)
        if floor is None:
            return desired
        measured = {row['name']: row['position'] for row in (self.session.last_feedback or [])}
        if self.state not in ('ENGAGING', 'TELEOP') or self._stepped is None:
            # From where the head is held (the lifecycle's hold), else where it reads, inside its usable range (the
            # lowest and highest commands its receiver takes): never where its weight sags it. Held at the bottom of
            # tilt, the head read 0.022 rad below its hold; aligning from that reading, its command stepped down after
            # it, under the receiver's floor, and Engage faulted (2026-10-31, "invalid_command").
            held = self.session.lifecycle.target if self.state == 'HOLD' else None
            start = dict(zip(self.names, held)) if held is not None else measured
            self._stepped = {name: min(max(start.get(name, self.zero[name]), self.usable[name][0]),
                                       self.usable[name][1]) for name in self.names}
            self._aligned = False
        if self.state != 'TELEOP':
            return dict(self._stepped)
        pan, tilt, centre = self.pan, self.tilt, self.centre
        goal = (desired[pan] - centre[pan], desired[tilt] - centre[tilt])
        cleared = floor.clamp(*goal)
        self.limited = cleared != goal
        pace = 'return' if self.returning is not None else 'follow' if self._aligned else 'align'
        limit = tuple(SPEED_SHARE * self.speeds[name][pace] * PERIOD_S for name in (pan, tilt))
        step = floor.step((self._stepped[pan] - centre[pan], self._stepped[tilt] - centre[tilt]), cleared, limit)
        self._stepped = {pan: centre[pan] + step[0], tilt: centre[tilt] + step[1]}
        self._aligned = self._aligned or max(abs(a - b) for a, b in zip(step, cleared)) < 1e-6
        return dict(self._stepped)

    def _target(self, now):
        positions, self.source = self._following(now)
        positions = self._clear(positions)
        self._sequence += 1
        return [{'name': name, 'position': positions[name], 'sampled_at': now, 'sequence': self._sequence}
                for name in self.names]

    # -- the cycle -----------------------------------------------------------------------------

    @property
    def state(self):
        return self.session.lifecycle.state if self.session is not None else None

    @property
    def due(self):
        """When the head's next cycle is due (on its clock), or None before its first."""
        return self._due

    def cycle(self, action, started):
        """One head cycle with the arms' action (None between keys), at most every PERIOD_S unless a key came.
        Returns the cycle's record, or None when it was not due. A failure faults the head (raised to the caller,
        which holds the arms)."""
        if self.fault or self.session is None:
            return None
        if self._disable_next:  # Back where it rested: its torque goes.
            self._disable_next, action = False, 'disable'
        if action == 'shutdown':
            # Return: limp already (Idle), it rests where it is; powered, it goes back to its reference, then down
            # to where it rested, following (from Hold it engages first, where it is), and is disabled there.
            if self.state == 'IDLE':
                action = 'disable'
            elif self.state in ('HOLD', 'ENGAGING', 'TELEOP') and self.returning is None:
                self.returning = 'centre'
                action = 'engage' if self.state == 'HOLD' else None
            else:
                action = None
        elif self.returning is not None and action != 'disable':
            action = None  # Returning: only Disable (H) acts.
        if action == 'disable':
            self.returning = None
        allowed = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ('ENGAGING', 'TELEOP')}
        if action in allowed and self.state not in allowed[action]:
            action = None  # The head follows the arms' lifecycle; a transition it is not in is skipped.
        if action is None and self._due is not None and started < self._due - DUE_SLACK_S:
            return None
        # On the grid, the next slot follows this one; a key between slots, or a cycle half a period late (a
        # stall), starts the grid again from now.
        on_grid = action is None and self._due is not None and started - self._due < PERIOD_S / 2
        self._due = (self._due if on_grid else started) + PERIOD_S
        try:
            target = self._target(started)
            # Only Engage and following use it; any other cycle follows nothing (the session's hold budget).
            following = action == 'engage' or (action is None and self.state in ('ENGAGING', 'TELEOP'))
            record = self.session.cycle(action=action, target=target if following else None, started_at=started)
            self._advance_return()
            return record
        except Exception as exc:
            status = getattr(exc, 'status', None) or self.session.controller.status or {}
            reason = status.get('reason')
            self.fault = f"Head receiver {status.get('state')}: {reason} · {exc}" if reason not in (None, 'ok') \
                else f'Head: {exc}'
            self._keep_fault(exc, status)
            raise

    def _keep_fault(self, exc, status):
        """The failing cycle, beside the live head receiver's evidence (fault-<UTC>.json): what was asked of the
        receiver, what it read and why the cycle failed. A command the receiver refuses leaves no trace there, and
        "invalid_command" alone does not say which joint or limit (2026-10-31)."""
        directory = getattr(self.receiver, 'directory', None)
        if directory is None:
            return
        try:
            stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
            record = {'fault': self.fault, 'error': f'{type(exc).__name__}: {exc}', 'status': status,
                      'cycle': self.session.last_cycle, 'returning': self.returning, 'stepped': self._stepped,
                      'rest_target': self.rest_target, 'limited': self.limited}
            (Path(directory) / f'fault-{stamp}.json').write_text(
                json.dumps(record, indent=1, default=repr) + '\n', encoding='utf-8')
        except (OSError, TypeError, ValueError):
            pass  # Evidence only: never stands in the way of the fault itself.

    def _advance_return(self):
        """Return's next stage once the head has reached this one's goal (its steps there, and its reading)."""
        if self.returning is None or self.state != 'TELEOP' or self._stepped is None:
            return
        goal = self.zero if self.returning == 'centre' else self.rest_target
        measured = {row['name']: row['position'] for row in (self.session.last_feedback or [])}
        if all(name in measured and abs(self._stepped[name] - goal[name]) < 1e-6
               and abs(measured[name] - goal[name]) <= self.tolerance[name] for name in self.names):
            if self.returning == 'centre':
                self.returning = 'rest'
            else:
                self._disable_next = True

    def status(self):
        now = self.clock()
        pose = None if self._pose is None else {'yaw': self._pose[0], 'pitch': self._pose[1],
                                                'age_s': now - self._pose[2], 'count': self._poses}
        last = (self.receiver.last or {}) if self.receiver is not None else {}
        positions = ({row['name']: row['position'] for row in (self.session.last_feedback or [])}
                     if self.session is not None else {})
        # The neck's CAD angles (neck_z, neck_y), for the 3D view: each motor's measured position through its axis.
        joints = {joint: positions[motor] * scale + offset for motor, (joint, scale, offset)
                  in getattr(self, 'axes', {}).items() if motor in positions}
        return {'state': self.state, 'fault': self.fault, 'source': getattr(self, 'source', None), 'headset': pose,
                'positions': positions, 'joints': joints, 'motors_enabled': last.get('hardware_enabled'),
                'returning': self.returning, 'camera_mount': getattr(self, 'camera_mount', None),
                # The neck's offsets ({joint: rad}) the camera's published poses take (helm.maps places a capture by
                # them), and why; None without a camera feed.
                'camera_offsets': getattr(self, 'camera_offsets', None),
                'camera_twin': getattr(self, 'camera_twin', None),
                # Whether the shoulder clearance holds the head above where the headset looks, this cycle.
                'clearance': None if self.clearance is None else {'limited': self.limited}}

    def close(self):
        if self.session is not None and self.session.owned:
            try:
                self.session.controller.disable()
            except Exception:
                pass
        failures = []
        for close in reversed(self._closers):
            try:
                close()
            except Exception as exc:
                failures.append(str(exc) or type(exc).__name__)
        self._closers = []
        if failures:
            raise RuntimeError('Closing the head: ' + '; '.join(failures))


class HeadLoop:
    """A live head's cycles on a thread of their own, beside the arms' loop (helm.teleop). Each exchange with the
    head's service waits for its next 10 ms tick, so a head cycle (two of them) took 22 ms of the arms' 10 ms loop
    every 40 ms: the arms' commands went out 10 and 30 ms apart, each 30 ms long, and the one Enable sent expired
    before the next arrived (2026-10-31, "command_expired · wrong_owner"). The arms' actions reach the head in order
    (`post`); between them it cycles on its own 40 ms grid. A fault ends the thread (`head.fault` says why; the arms'
    loop holds the arms when it sees it), and `stop` ends it before the head closes.

    The thread first moves off the arms' CPUs (`placement`, control.scheduling): on them, beside the head, the arms'
    receiver started its ticks up to 9.2 ms late and an arm command expired (2026-10-31, 15:15)."""

    JOIN_S = 2.  # Several of the head's 200 ms exchange timeouts: a cycle stuck past this is not coming back.

    def __init__(self, head, *, clock=time.perf_counter, placement=None):
        from robot_platform.control.scheduling import prefer_background_thread
        self.head, self.clock = head, clock
        self._place = placement or prefer_background_thread
        self.placement = None  # The hints that took on the thread, once it started.
        self._actions = deque()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stopping = False
        self._thread = threading.Thread(target=self._run, daemon=True, name='helm-head')

    def start(self):
        self._thread.start()
        return self

    def post(self, action):
        """One of the arms' actions for the head (None is none), taken at its next cycle after those before it."""
        if action is None:
            return
        with self._lock:
            if not self._actions or self._actions[-1] != action:  # A faulted arms' loop repeats its action each tick.
                self._actions.append(action)
        self._wake.set()

    @property
    def pending(self):
        """Whether actions wait for the head (a landing session waits for them too)."""
        with self._lock:
            return bool(self._actions)

    def _run(self):
        try:
            self.placement = self._place()
        except Exception:
            self.placement = []  # A hint, not a condition: the head cycles where it is.
        head = self.head
        while not self._stopping and not head.fault:
            self._wake.clear()  # Before taking an action: one posted from here on wakes the wait below.
            with self._lock:
                action = self._actions.popleft() if self._actions else None
            try:
                head.cycle(action, self.clock())
            except Exception:
                return  # head.fault says why.
            if not self.pending:
                wait = PERIOD_S if head.due is None else head.due - self.clock()
                if wait > 0:
                    self._wake.wait(wait)

    def stop(self):
        self._stopping = True
        self._wake.set()
        self._thread.join(self.JOIN_S)
        if self._thread.is_alive():
            raise RuntimeError("The head's cycles did not stop")


STATUS_S = PERIOD_S  # The head's process reports its status this often, as often as it cycles.
START_S = 30.  # Building a live head (its service checks the servos and that the head rests where it was read).


def serve_head(bundle, live, conn, open_receiver=None, open_harness=None, verified=None):
    """The head in a process of its own (HeadProcess's child). It moves itself, and so its receiver's service, off
    the arms' CPUs, builds the head, and cycles it on its own grid (HeadLoop), steered over `conn` by the arms' loop:
    ('action', action, number), ('pose', yaw, pitch) and ('close',). It answers ('built', scheduling) or ('fault',
    reason) once, then ('status', head status, loop word) every cycle, and ('closed', error) as it ends. A lost
    channel (the arms' process gone) closes the head as ('close',) would. `verified`: the release's mesh digests the
    arms' process registered (models.urdf.verified_assets): on the robot, with no CAD files, the head model loads by
    them, as there (a head process without them failed to build in its rehearsal on Minerva, 2026-10-31)."""
    import os
    from robot_platform.control.scheduling import prefer_background
    from robot_platform.models.urdf import register_verified_assets
    if verified:
        register_verified_assets(verified)
    scheduling = prefer_background(os.getpid())
    head = HeadTeleop(bundle, live=live, open_receiver=open_receiver, open_harness=open_harness)
    try:
        head.build()
    except Exception as exc:
        try:
            head.close()
        except Exception:
            pass
        conn.send(('fault', str(exc) or type(exc).__name__))
        return
    loop = HeadLoop(head, placement=lambda: scheduling).start()
    conn.send(('built', scheduling))
    received, sent_at, error = 0, None, None
    try:
        while True:
            if conn.poll(STATUS_S):
                message = conn.recv()
                if message[0] == 'close':
                    break
                if message[0] == 'action':
                    loop.post(message[1])
                    received = message[2]
                elif message[0] == 'pose':
                    try:
                        head.pose(message[1], message[2])
                    except ValueError:
                        pass  # Checked on the arms' side already.
            now = time.perf_counter()
            if sent_at is None or now - sent_at >= STATUS_S:
                conn.send(('status', head.status(), {'fault': head.fault, 'state': head.state,
                                                     'returning': head.returning, 'pending': loop.pending,
                                                     'received': received}))
                sent_at = now
    except (EOFError, OSError):
        pass
    finally:
        try:
            loop.stop()
        except Exception as exc:
            error = str(exc)
        try:
            head.close()
        except Exception as exc:
            error = error or str(exc)
        try:
            conn.send(('closed', error))
        except (OSError, ValueError):
            pass


class HeadProcess:
    """A live head in a process of its own (serve_head), beside the arms' loop (helm.teleop), which talks to it as
    to HeadTeleop and HeadLoop together. On a thread in the arms' process the head shared their interpreter, and the
    arms' slow cycles kept time with its 40 ms cycle (phase-locked, R 0.7, in two full VR sessions): one of their
    commands expired (2026-10-31, live-20261031T195313Z, "command_expired · wrong_owner"). Nothing here waits for the
    head: actions and headset poses go to a sender thread, the head's status comes back on a reader thread, and the
    arms' side reads the last of it. `open_receiver` and `open_harness` go to the child (module-level callables)."""

    JOIN_S = 10.  # The head's close (its torque off at its rest, its service stopped) takes a few seconds at most.

    def __init__(self, bundle, *, live=True, open_receiver=None, open_harness=None, start_s=START_S):
        self.bundle, self.live = bundle, live
        self.open_receiver, self.open_harness, self.start_s = open_receiver, open_harness, start_s
        self.session = None  # True once built: the arms' side asks as it asks HeadTeleop.
        self.scheduling = []
        self._fault = None
        self._status, self._word = None, {}
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._actions = deque()
        self._pose = None
        self._numbered = 0
        self._closing, self._closed = False, None
        self._process = self._conn = None
        self._threads = []

    @classmethod
    def instead_of(cls, head):
        """The process for an unbuilt HeadTeleop, with its openers where they are not its own (a test's harness)."""
        own = lambda opener: None if getattr(opener, '__self__', None) is head else opener  # noqa: E731
        return cls(head.bundle, live=head.live, open_receiver=own(head.open_receiver),
                   open_harness=own(head.open_harness))

    # -- HeadTeleop's face -----------------------------------------------------------------------

    @property
    def fault(self):
        with self._lock:
            return self._fault or self._word.get('fault')

    @fault.setter
    def fault(self, value):
        with self._lock:
            self._fault = value

    @property
    def state(self):
        with self._lock:
            return self._word.get('state')

    @property
    def returning(self):
        with self._lock:
            return self._word.get('returning')

    def status(self):
        with self._lock:
            status = dict(self._status) if self._status is not None else {'state': None}
            if self._fault:
                status['fault'] = self._fault
            return status

    def pose(self, yaw, pitch, *, now=None):
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            raise ValueError('A headset pose is finite yaw and pitch')
        with self._wake:
            self._pose = (float(yaw), float(pitch))  # Only the newest goes.
            self._wake.notify()

    # -- HeadLoop's face -------------------------------------------------------------------------

    def post(self, action):
        if action is None:
            return
        with self._wake:
            if not self._actions or self._actions[-1][0] != action:  # A faulted arms' loop repeats its action.
                self._numbered += 1
                self._actions.append((action, self._numbered))
            self._wake.notify()

    @property
    def pending(self):
        """Whether actions wait for the head: here, on their way, or in its loop (a landing session waits too)."""
        with self._lock:
            waiting = bool(self._actions) or self._word.get('received', 0) < self._numbered
            return waiting or bool(self._word.get('pending'))

    def stop(self):
        """Its cycles end with close(), in its own process."""

    # -- the process -----------------------------------------------------------------------------

    def build(self):
        import multiprocessing
        from robot_platform.models.urdf import verified_assets
        context = multiprocessing.get_context('spawn')  # Never a fork of the arms' process, its threads and locks.
        conn, child = context.Pipe()
        self._process = context.Process(target=serve_head, name='helm-head', daemon=True,
                                        args=(self.bundle, self.live, child, self.open_receiver, self.open_harness,
                                              verified_assets()))
        self._process.start()
        child.close()
        self._conn = conn
        if not conn.poll(self.start_s):
            raise RuntimeError(f'the head process did not answer within {self.start_s:g} s')
        try:
            message = conn.recv()
        except EOFError:
            self._process.join(2.)
            raise RuntimeError(f'the head process ended before it answered (exit code {self._process.exitcode})') \
                from None
        if message[0] == 'fault':
            raise RuntimeError(message[1])
        self.scheduling, self.session = message[1], True
        self._threads = [threading.Thread(target=self._send, daemon=True, name='helm-head-send'),
                         threading.Thread(target=self._read, daemon=True, name='helm-head-read')]
        for thread in self._threads:
            thread.start()

    def _send(self):
        while True:
            with self._wake:
                self._wake.wait_for(lambda: self._actions or self._pose is not None or self._closing)
                actions, pose, closing = list(self._actions), self._pose, self._closing
                self._actions.clear()
                self._pose = None
            try:
                for action, number in actions:
                    self._conn.send(('action', action, number))
                if pose is not None:
                    self._conn.send(('pose', *pose))
                if closing:
                    self._conn.send(('close',))
                    return
            except (OSError, EOFError, ValueError):
                return

    def _read(self):
        try:
            while True:
                message = self._conn.recv()
                with self._lock:
                    if message[0] == 'status':
                        self._status, self._word = message[1], message[2]
                    elif message[0] == 'closed':
                        self._closed = message[1] or ''
                        return
        except (EOFError, OSError):
            with self._lock:
                if not self._closing and not self._fault:
                    self._fault = 'the head process ended unexpectedly'

    def close(self):
        if self._process is None:
            return
        with self._wake:
            self._closing = True
            self._wake.notify_all()
        if self.session is not None:
            for thread in self._threads:
                thread.join(self.JOIN_S)
        self._process.join(self.JOIN_S)
        stuck = self._process.is_alive()
        if stuck:
            self._process.terminate()
            self._process.join(2.)
        self._conn.close()
        if stuck:
            raise RuntimeError('the head process did not close in time: stopped')
        if self._closed:
            raise RuntimeError(self._closed)
