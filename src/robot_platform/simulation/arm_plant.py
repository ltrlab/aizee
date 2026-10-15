"""A simulated arm behind the actual Rust receiver, to rehearse a live session offline.

`PlantReceiver` stands in for the live receiver (its exchange, last, transport hook and
close) while an `ArmPlant` moves the joints. Every 2.5 ms receiver tick the plant
integrates each joint under the drive's MIT law (kp (reference - q) + kd (velocity - v)
+ effort, from the receiver's commands) against inertia, gravity and Coulomb friction
with stiction; the receiver reads the result back with the drives' speed noise and
16-bit position steps, and its results carry the ticks since the last one as the live
recorder's rows. The receiver reads its configuration from JSON, as the live one does.

This rehearses the receiver's checks against realistic feedback. It is not a model of
Minerva's dynamics, and no evidence about the physical arm.
"""
from __future__ import annotations

import math
from pathlib import Path
import random
import tempfile
from types import SimpleNamespace

from .control_harness import NativeControlHarness

TICK_S = .0025  # The live receiver's period.
SUBSTEPS = 5
POSITION_STEP_RAD = 8 * math.pi / 65535  # RobStride feedback: 16 bits over +-4 pi.


class ArmPlant:
    """Independent joints (their gravity may depend on the whole pose), in the receiver's joint order.

    joints: [{'name', 'inertia', 'friction', 'static' (default: friction), 'viscous' (Nm per rad/s, default 0:
    the gearbox's damping), 'q' (default 0)}].
    gravity: {name: callable(pose) -> the torque that holds the joint against gravity}.
    """

    def __init__(self, joints, *, gravity=None, speed_noise=.08, position_noise=0., seed=1, disturbance=None):
        self.specs = [dict(joint) for joint in joints]
        self.names = [joint['name'] for joint in joints]
        self.q = [float(joint.get('q', 0.)) for joint in joints]
        self.v = [0.] * len(joints)
        self.torque = [0.] * len(joints)
        self.gravity = dict(gravity or {})
        self.speed_noise, self.position_noise = speed_noise, position_noise
        self.random = random.Random(seed)
        self.t = 0.  # Simulated time (s).
        # callable(plant) -> {name: Nm}: an outside push (a bump, a hand), added to the drive's torque.
        self.disturbance = disturbance
        self.quiet = set()  # Drives that have stopped answering: the receiver sees no new reading from them.
        # Drives that rebooted while driven: limp, and (their readings withheld by the adapter) quiet for good.
        self.dropped = set()

    def pose(self):
        return dict(zip(self.names, self.q))

    def holding(self):
        pose = self.pose()
        return [self.gravity[name](pose) if name in self.gravity else 0. for name in self.names]

    def step(self, commands, dt):
        """Integrate `dt` s under `commands` (the receiver's, per joint), or limp when None."""
        h = dt / SUBSTEPS
        for _ in range(SUBSTEPS):
            holding = self.holding()
            pushed = self.disturbance(self) if self.disturbance else {}
            for i, spec in enumerate(self.specs):
                q, v = self.q[i], self.v[i]
                if commands is None or spec['name'] in self.dropped:
                    torque = 0.
                else:
                    c = commands[i]
                    torque = c['kp'] * (c['position'] - q) + c['kd'] * (c['velocity'] - v) + c['effort']
                net = torque - holding[i] + pushed.get(spec['name'], 0.) - spec.get('viscous', 0.) * v
                friction, static = spec['friction'], spec.get('static', spec['friction'])
                if abs(v) < 1e-4 and abs(net) <= static:
                    v = 0.  # Sticks, then slides.
                else:
                    v += (net - friction * math.copysign(1, v if abs(v) >= 1e-4 else net)) / spec['inertia'] * h
                    q += v * h
                self.q[i], self.v[i], self.torque[i] = q, v, torque
            self.t += h

    def measured(self):
        return [{'position': round((q + self.random.gauss(0., self.position_noise)) / POSITION_STEP_RAD)
                 * POSITION_STEP_RAD,
                 'velocity': v + self.random.gauss(0., self.speed_noise), 'effort': torque,
                 'quiet': name in self.quiet or name in self.dropped}
                for name, q, v, torque in zip(self.names, self.q, self.v, self.torque)]


class PlantReceiver:
    """The live receiver's interface over Rust ticking against an ArmPlant or MujocoPlant.

    `clock` is the caller's clock (s); the receiver's time is milliseconds since construction.
    As the live service does, it takes each request at its next tick (`service_latency`), so a
    loop's requests span ticks and each result brings the rows since the last.
    """

    def __init__(self, config, plant, *, clock, directory=None, binary=None, service_latency=True):
        if [joint['name'] for joint in config['joints']] != plant.names:
            raise ValueError('Plant joint order must match receiver configuration')
        self.config, self.plant, self.clock = config, plant, clock
        self.service_latency = service_latency
        self.directory = Path(directory) if directory else Path(tempfile.mkdtemp(prefix='plant-receiver-'))
        self.directory.mkdir(parents=True, exist_ok=True)
        positions = [q for q in plant.q]
        self.harness = NativeControlHarness(config, positions, binary=binary,
                                            setup_json=self.directory / 'setup.json')
        self.epoch = clock()
        self.ticks = 0
        self.rows = []
        self.last = self.harness.step(0, feedback=plant.measured())
        self.transport = SimpleNamespace(on_result=None)
        self.closed = False

    def _now_ms(self):
        return (self.clock() - self.epoch) * 1000

    def _row(self, result, t_ms):
        """A live recorder row (servo_sweep.recorder_columns): measured and commanded position per joint, then each
        drive's commanded velocity, effort and gains."""
        row = [t_ms]
        commands = result['status']['joints']
        for feedback, command in zip(result['feedback'], commands):
            row += [feedback['position'], feedback['velocity'], feedback['effort'], command['position']]
        for command in commands:
            row += [command['velocity'], command['effort'], command['kp'], command['kd']]
        return row

    def _tick(self):
        enabled = self.last.get('hardware_enabled')
        self.plant.step(self.last['status']['joints'] if enabled else None, TICK_S)
        self.ticks += 1
        t_ms = self.ticks * TICK_S * 1000
        # Whole milliseconds of the tick's own time: 1606 ticks are 4015 ms, not int(4014.9999999999995).
        self.last = dict(self.harness.step(int(round(t_ms, 6)), feedback=self.plant.measured()),
                         hardware_dropped=self._dropped())
        self.rows.append(self._row(self.last, t_ms))

    def advance(self, now_ms=None):
        """Tick the receiver and the plant up to the caller's clock (`now_ms`, or one reading of it)."""
        now_ms = self._now_ms() if now_ms is None else now_ms
        while (self.ticks + 1) * TICK_S * 1000 <= now_ms:
            self._tick()

    def exchange(self, request):
        # One reading of the caller's clock. A real clock runs on while the plant ticks: read again afterwards, a
        # slow tick put the request past the next scheduled tick, which then stepped the receiver back
        # (clock_regression in Helm's simulated Minerva, 2026-10-14).
        now_ms = self._now_ms()
        self.advance(now_ms)
        if self.service_latency:
            self._tick()  # The live service takes a request at its next tick.
        now_ms = max(self.harness.now_ms, int(now_ms))
        result = self.harness.step(now_ms, request, feedback=self.plant.measured())
        result = dict(result, recent=self.rows, recent_dropped=0, hardware_dropped=self._dropped())
        self.rows = []
        self.last = result
        if self.transport.on_result:
            self.transport.on_result(result)
        return result['reply']

    def _dropped(self):
        """The live adapter's note on the first drive that left Run mode while driven."""
        name = next((name for name in self.plant.names if name in self.plant.dropped), None)
        return f'{name}: left Run mode while driven (it rebooted, or its own watchdog disabled it)' if name else None

    def close(self):
        self.closed = True
        self.harness.close()
