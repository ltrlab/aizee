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

    joints: [{'name', 'inertia', 'friction', 'static' (default: friction), 'q' (default 0)}].
    gravity: {name: callable(pose) -> the torque that holds the joint against gravity}.
    """

    def __init__(self, joints, *, gravity=None, speed_noise=.08, position_noise=0., seed=1):
        self.specs = [dict(joint) for joint in joints]
        self.names = [joint['name'] for joint in joints]
        self.q = [float(joint.get('q', 0.)) for joint in joints]
        self.v = [0.] * len(joints)
        self.torque = [0.] * len(joints)
        self.gravity = dict(gravity or {})
        self.speed_noise, self.position_noise = speed_noise, position_noise
        self.random = random.Random(seed)

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
            for i, spec in enumerate(self.specs):
                q, v = self.q[i], self.v[i]
                if commands is None:
                    torque = 0.
                else:
                    c = commands[i]
                    torque = c['kp'] * (c['position'] - q) + c['kd'] * (c['velocity'] - v) + c['effort']
                net = torque - holding[i]
                friction, static = spec['friction'], spec.get('static', spec['friction'])
                if abs(v) < 1e-4 and abs(net) <= static:
                    v = 0.  # Sticks, then slides.
                else:
                    v += (net - friction * math.copysign(1, v if abs(v) >= 1e-4 else net)) / spec['inertia'] * h
                    q += v * h
                self.q[i], self.v[i], self.torque[i] = q, v, torque

    def measured(self):
        return [{'position': round((q + self.random.gauss(0., self.position_noise)) / POSITION_STEP_RAD)
                 * POSITION_STEP_RAD,
                 'velocity': v + self.random.gauss(0., self.speed_noise), 'effort': torque}
                for q, v, torque in zip(self.q, self.v, self.torque)]


class PlantReceiver:
    """The live receiver's interface over the Rust receiver ticking against an `ArmPlant`.

    `clock` is the caller's clock (s); the receiver's time is milliseconds since construction.
    """

    def __init__(self, config, plant, *, clock, directory=None, binary=None):
        self.config, self.plant, self.clock = config, plant, clock
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
        row = [t_ms]
        for feedback, command in zip(result['feedback'], result['status']['joints']):
            row += [feedback['position'], feedback['velocity'], feedback['effort'], command['position']]
        return row

    def advance(self):
        """Tick the receiver and the plant up to the caller's clock."""
        while (self.ticks + 1) * TICK_S * 1000 <= self._now_ms():
            enabled = self.last.get('hardware_enabled')
            self.plant.step(self.last['status']['joints'] if enabled else None, TICK_S)
            self.ticks += 1
            t_ms = self.ticks * TICK_S * 1000
            self.last = self.harness.step(int(t_ms), feedback=self.plant.measured())
            self.rows.append(self._row(self.last, t_ms))

    def exchange(self, request):
        self.advance()
        now_ms = max(self.harness.now_ms, int(self._now_ms()))
        result = self.harness.step(now_ms, request, feedback=self.plant.measured())
        result = dict(result, recent=self.rows, recent_dropped=0)
        self.rows = []
        self.last = result
        if self.transport.on_result:
            self.transport.on_result(result)
        return result['reply']

    def close(self):
        self.closed = True
        self.harness.close()
