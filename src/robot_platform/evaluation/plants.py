"""What a policy's commands drive in simulation. No USB, CAN, SSH or physical motors.

`armplant` is the Helm `--sim` stack: the receiver Helm derives for the real arms (the actual
native control harness), the lifecycle and direct session TeleopCore runs, and the simulated
Minerva (minerva_sim.open_simulation: ArmPlant joints with the Tune sweeps' and the actuator
file's fitted values). A policy's command replaces the leader target; everything after it is the
code Helm runs. A stepped clock makes runs deterministic and faster than real time; it measures
no wall-clock timing and charges the policy no inference time.

`ideal` follows each command exactly: no receiver, no tracking error, no sag. It reproduces the
conditions the frozen contact fixture was validated under, to check the harness itself. It is
never evidence about the robot.

Both report the 14 receiver joints in receiver motor coordinates, the same coordinates the task
capture records as qpos/qcmd/actions.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from robot_platform.simulation.arm_plant import TICK_S

ROOT = Path(__file__).resolve().parents[3]
DEVICES = ROOT / 'config/devices'
PLANTS = ('armplant', 'ideal')
# The documented signed directions the receiver/capture contract uses
# (tests/contract/test_task_receiver_capture.py); a Helm direction session can replace them.
REVERSED_MOTORS = frozenset({'right_arm_j1', 'right_arm_j3', 'right_arm_j4'})


class PlantFault(RuntimeError):
    """The receiver stopped, or the lifecycle could not follow: the trial aborts and the stack is rebuilt."""


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def documented_directions(steps):
    return {step['joint']: dict(motor=step['motor'], offset_rad=0., direction_verified=True,
                                sign=-1 if step['motor'] in REVERSED_MOTORS else 1) for step in steps}


def session_directions(path, steps):
    """A Helm direction session's confirmed mapping (helm.session: session.json's preview_mapping)."""
    state = json.loads(Path(path).read_text(encoding='utf-8'))
    mapping = state.get('preview_mapping') if isinstance(state, dict) else None
    if (not isinstance(mapping, dict) or set(mapping) != {s['joint'] for s in steps}
            or any(r.get('sign') not in (-1, 1) or not math.isfinite(r.get('offset_rad', math.nan))
                   for r in mapping.values())):
        raise ValueError(f'{path} has no complete direction mapping (preview_mapping) for this calibration plan')
    return mapping


def derive_stack(*, devices=DEVICES, calibration='minerva-joint-calibration.json', profile='minerva-teleop-direct.json',
                 limits='minerva-production-01-joint-limits.json', servo='minerva-production-01-servo.json',
                 directions=None, actuators=True):
    """The receiver Helm would derive for these saved files (helm.teleop.TeleopCore.build), and their provenance."""
    from robot_platform.control.guarded_config import derive_receiver, fit_motor_ranges
    from robot_platform.control.joint_calibration import load_plan, validate_preview_transmissions
    from robot_platform.control.servo_calibration import load_servo
    from robot_platform.helm.teleop import load_profile
    from robot_platform.simulation.minerva_sim import actuators_path
    devices = Path(devices)
    paths = {key: devices / name for key, name in
             (('calibration', calibration), ('profile', profile), ('limits', limits), ('servo', servo))}
    plan, _, device, reference, model, _pose, bindings = load_plan(paths['calibration'])
    loaded = load_profile(paths['profile'])
    receiver = loaded['receiver']
    if receiver.get('collision_mode') != 'disabled':
        # Enforced two-arm checks need the motion envelope compiler and collision backend; not wired here.
        raise ValueError('The evaluation runner drives collision-disabled direct profiles only '
                         f"({paths['profile'].name} is {receiver.get('collision_mode', 'enforced')})")
    steps = plan['steps']
    transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), steps, reference, model)
    if directions is None:
        mapping, direction_source = documented_directions(steps), 'documented test directions (REVERSED_MOTORS)'
    else:
        mapping, direction_source = session_directions(directions, steps), str(Path(directions).resolve())
    saved_servo = load_servo(paths['servo'])
    derived = derive_receiver(reference=reference, steps=steps, directions=mapping, transmissions=transmissions,
                              model=model, limits=json.loads(paths['limits'].read_text(encoding='utf-8')),
                              profile=loaded, backend=None, servo=saved_servo, collision_bound=None)
    fit_motor_ranges(derived['config']['joints'], {})
    actuator_file = None
    if actuators:
        actuator_file = actuators_path(paths['servo'], device['asset_id']) if actuators is True else Path(actuators)
    names = tuple(joint['name'] for joint in derived['config']['joints'])
    provenance = {'asset_id': device['asset_id'], 'receiver_identity': derived['config']['identity'],
                  'profile': loaded.get('id'), 'directions': direction_source,
                  'files': {key: {'path': str(path), 'sha256': _sha(path)} for key, path in paths.items()},
                  'joint_names': list(names)}
    return {'derived': derived, 'model': model, 'reference': reference, 'directions': mapping,
            'servo': saved_servo, 'profile': loaded, 'names': names, 'actuators': actuator_file,
            'provenance': provenance}


class _Stack:
    """Shared joint bookkeeping: names, limits, the motor->model map, the period."""

    def __init__(self, stack):
        from robot_platform.helm.teleop import loop_period_s
        self.stack = stack
        derived = stack['derived']
        self.names = stack['names']
        self.joint_map = deepcopy(derived['joint_map'])
        rows = {row['name']: row for row in derived['lifecycle']}
        # The lifecycle refuses a target outside [lower, upper]; commands are clipped into it first.
        self.lower = np.array([rows[n]['lower'] for n in self.names])
        self.upper = np.array([rows[n]['upper'] for n in self.names])
        self.home = np.array(derived['initial'], dtype=float)
        self.period_s = loop_period_s(stack['profile'])
        self.time_s = 0.

    def clip(self, command):
        return np.minimum(self.upper, np.maximum(self.lower, command))


class IdealPlant(_Stack):
    kind = 'ideal'

    def __init__(self, stack, **_):
        super().__init__(stack)
        self.q = self.home.copy()
        self._velocity = np.zeros(len(self.names))

    def stage(self, q, timeout_s=None):
        self.q = self.clip(np.asarray(q, dtype=float))
        self._velocity[:] = 0.
        return {'ok': True, 'duration_s': 0., 'error': np.zeros(len(self.names)).tolist()}

    def scene_pose(self):
        return self.q.copy()

    def step(self, command):
        command = self.clip(np.asarray(command, dtype=float))
        self._velocity = (command - self.q) / self.period_s
        self.q = command
        self.time_s += self.period_s
        return self.q.copy(), self.period_s

    def observation(self):
        nan = np.full(len(self.names), np.nan)
        return {'q': self.q.copy(), 'qcmd': self.q.copy(), 'dq': self._velocity.copy(), 'effort': nan}

    def describe(self):
        return {'plant': 'ideal', 'note': 'commands become joint positions exactly; harness check only, '
                                          'never evidence about the robot', **self.stack['provenance']}

    def close(self):
        pass


class _SteppedClock:
    def __init__(self, start=100.):
        self.now = start

    def __call__(self):
        return self.now


class ArmPlant(_Stack):
    """The receiver, lifecycle and simulated joints TeleopCore runs under `helm --sim`, driven by a policy."""
    kind = 'armplant'
    HOLD_CYCLES = 20

    def __init__(self, stack, *, directory=None, seed=1, scene_pose='plant'):
        from robot_platform.control.guarded import GuardedControllerClient
        from robot_platform.helm.teleop import CYCLE_BUDGET_S, source_misses
        from robot_platform.simulation.minerva_sim import open_simulation
        from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
        from robot_platform.teleop.lifecycle import TeleopLifecycle
        super().__init__(stack)
        if scene_pose not in ('plant', 'measured'):
            raise ValueError("scene_pose is 'plant' (the simulated joints) or 'measured' (receiver feedback rows)")
        self.scene_pose_source = scene_pose
        derived = stack['derived']
        self.clock = _SteppedClock()
        self.directory = Path(directory) if directory else Path(tempfile.mkdtemp(prefix='minerva-eval-'))
        self.receiver = open_simulation(derived, model=stack['model'], directions=stack['directions'],
                                        motors=stack['reference']['motors'], servo=stack['servo'],
                                        directory=self.directory, clock=self.clock, viewer=False, seed=seed,
                                        actuators=stack['actuators'])
        self.plant = self.receiver.plant
        if list(self.plant.names) != list(self.names):
            raise ValueError('Simulated joints and receiver joints differ in order')
        # The joints' true positions after each 2.5 ms receiver tick: what the arm physically does. The policy
        # sees the receiver's feedback (16-bit steps and reading noise) instead, as on the robot.
        self._ticks = []
        self._coupled = None
        self._grippers = {name: i for i, name in enumerate(self.names) if name.endswith('_gripper')}
        step = self.plant.step

        def recorded(commands, dt):
            start = list(self.plant.q)
            step(commands, dt)
            if self._coupled is not None:
                # Coupled: the scene's driven grippers, not ArmPlant, decide where the gripper joints went.
                held = {name: None if commands is None or name in self.plant.dropped else commands[i]
                        for name, i in self._grippers.items()}
                for name, (q, v, torque) in self._coupled.step_driven(start, self.plant.q, held, dt).items():
                    i = self._grippers[name]
                    self.plant.q[i], self.plant.v[i], self.plant.torque[i] = q, v, torque
            self._ticks.append(tuple(self.plant.q))
        self.plant.step = recorded
        config = derived['config']
        self.controller = GuardedControllerClient(self.receiver.exchange, group=config['group'],
                                                  identity=config['identity'], joint_names=self.names)
        self.lifecycle = TeleopLifecycle(derived['lifecycle'], clock=self.clock,
                                         max_source_misses=source_misses(self.period_s), trial=None,
                                         allow_repeated_source=True)
        self.session = DirectLifecycleSession(self.controller, config, self.lifecycle, lambda: self.receiver.last,
                                              period_s=self.period_s, cycle_budget_s=CYCLE_BUDGET_S, clock=self.clock)
        self._sequence = 0
        self._row = None
        self._pose = np.array(self.plant.q)
        try:
            # Idle, then Enable and hold, as an operator starts a session.
            self.cycle('idle')
            for _ in range(5):
                self.cycle()
            self.cycle('enable')
            for _ in range(self.HOLD_CYCLES):
                self.cycle()
        except BaseException:
            self.receiver.close()
            raise

    def _target(self, q):
        self._sequence += 1
        now = self.clock()
        return [dict(name=name, position=float(value), sampled_at=now, sequence=self._sequence)
                for name, value in zip(self.names, q)]

    def cycle(self, action=None, command=None):
        """One control period: a lifecycle cycle through the receiver, then the clock moves on."""
        self._ticks.clear()
        try:
            self.session.cycle(action=action, target=None if command is None else self._target(command),
                               started_at=self.clock())
        except Exception as exc:
            status = getattr(exc, 'status', None) or self.controller.status or {}
            reason = status.get('reason')
            raise PlantFault(f"receiver {status.get('state')}: {reason} · {exc}" if reason not in (None, 'ok')
                             else f'{type(exc).__name__}: {exc}') from exc
        recent = (self.receiver.last or {}).get('recent') or []
        if recent:
            self._row = recent[-1]
        ticks = len(self._ticks)
        if ticks:
            if self.scene_pose_source == 'plant':
                self._pose = np.array(self._ticks[-1])
            elif recent:
                self._pose = np.array([recent[-1][1 + 4 * i] for i in range(len(self.names))])
        self.clock.now += self.period_s
        self.time_s += ticks * TICK_S
        return self._pose.copy(), ticks * TICK_S

    def stage(self, q, timeout_s=30.):
        """Engage onto `q` the way an operator's Engage aligns, until the lifecycle says it has settled there."""
        q = self.clip(np.asarray(q, dtype=float))
        if self.lifecycle.state in ('ENGAGING', 'TELEOP'):
            self.cycle('disengage')
        self.cycle('engage', q)
        elapsed = 0.
        while self.lifecycle.state != 'TELEOP':
            if self.lifecycle.state != 'ENGAGING':
                raise PlantFault(f'staging left ENGAGING for {self.lifecycle.state}: {self.lifecycle.reason}')
            if elapsed > timeout_s:
                return {'ok': False, 'duration_s': elapsed,
                        'error': (np.array(self.plant.q) - q).tolist(),
                        'detail': f'did not settle on the start pose within {timeout_s:.0f} s'}
            self.cycle(None, q)
            elapsed += self.period_s
        return {'ok': True, 'duration_s': elapsed, 'error': (np.array(self.plant.q) - q).tolist()}

    def scene_pose(self):
        return self._pose.copy()

    def gripper_drives(self):
        """What a scene's driven grippers need (MinervaTaskScene(grippers=...)): each gripper joint's ArmPlant values
        and the receiver's torque limit for it."""
        rows = {row['name']: row for row in self.stack['derived']['config']['joints']}
        nominal = self.receiver.nominal or {}
        drives = {}
        for name in self._grippers:
            spec = next(s for s in self.plant.specs if s['name'] == name)
            guessed = nominal.get(name) or []
            drives[name] = dict(inertia=spec['inertia'], friction=spec['friction'], viscous=spec.get('viscous', 0.),
                                max_effort=float(rows[name]['max_effort']),
                                source=('ArmPlant joint values' + (f" (nominal, unmeasured: {', '.join(guessed)})"
                                                                   if guessed else '')
                                        + '; torque limit: the receiver configuration\'s max_effort'))
        return drives

    def couple(self, scene):
        """From the next tick on, the scene's driven grippers move the gripper joints: the receiver reads them
        stalled on whatever the jaws hold. The scene's drives must already sit where the plant's grippers are."""
        if not getattr(scene, 'driven', False):
            raise ValueError('coupling needs a scene with driven grippers')
        state = scene.gripper_state()
        for name, i in self._grippers.items():
            q, v, _ = state[name]
            if abs(q - self.plant.q[i]) > 1e-9:
                raise ValueError(f'{name}: the scene drive is at {q:.6f}, the plant at {self.plant.q[i]:.6f}')
            self.plant.v[i] = v
        self._coupled = scene

    def decouple(self):
        self._coupled = None

    @property
    def coupled(self):
        return self._coupled is not None

    def step(self, command):
        if self.lifecycle.state != 'TELEOP':
            raise PlantFault(f'lifecycle is {self.lifecycle.state}: {self.lifecycle.reason}')
        return self.cycle(None, self.clip(np.asarray(command, dtype=float)))

    def observation(self):
        row, n = self._row, len(self.names)
        if row is None:
            raise PlantFault('no receiver feedback yet')
        return {'q': np.array([row[1 + 4 * i] for i in range(n)]), 'dq': np.array([row[2 + 4 * i] for i in range(n)]),
                'effort': np.array([row[3 + 4 * i] for i in range(n)]),
                'qcmd': np.array([row[4 + 4 * i] for i in range(n)])}

    def describe(self):
        derived = self.stack['derived']
        return {'plant': 'armplant', 'scene_pose': self.scene_pose_source,
                'receiver': 'native control harness (simulation.control_harness) via minerva_sim.open_simulation',
                'loop_hz': 1 / self.period_s, 'receiver_tick_s': TICK_S,
                'control_law': sorted({row['control_law'] for row in derived['report']}),
                'actuators': self.receiver.actuators, 'nominal_joint_values': self.receiver.nominal,
                'clock': 'stepped simulation clock; no wall-clock timing or policy inference latency',
                **self.stack['provenance']}

    def close(self):
        try:
            try:
                self.session.disable()
            except Exception:
                pass
        finally:
            self.receiver.close()


def open_plant(kind, stack, **options):
    if kind == 'armplant':
        return ArmPlant(stack, **options)
    if kind == 'ideal':
        return IdealPlant(stack)
    raise ValueError(f'plant is one of {PLANTS}')
