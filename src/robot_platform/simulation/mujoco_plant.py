"""MuJoCo physics behind PlantReceiver; never connects to physical devices.

The receiver still owns watchdogs, limits and authority. Here its motor-coordinate
MIT commands become generalized forces. Feedback noise is applied after physics.
This is an ideal torque source, not a validated model of a RobStride drive.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import random

import numpy as np

from .arm_plant import POSITION_STEP_RAD


@dataclass(frozen=True)
class JointBinding:
    """q_model = offset + scale*q_motor; power implies force_model = torque_motor/scale.

    A hinge scale is rad/rad; a slide scale is m/rad. Negative scales reverse the
    coordinate. Both need measured values for a physical model. Joint names are
    explicitly mapped in receiver order; XML joint order need not match it.
    """
    name: str
    joint: str
    max_effort: float
    scale: float = 1.0
    offset: float = 0.0


class MujocoPlant:
    def __init__(self, model_path, bindings, *, positions=None, timestep=.0005,
                 speed_noise=0., position_noise=0., position_step=POSITION_STEP_RAD,
                 seed=1, disturbance=None):
        import mujoco

        self._mj = mujoco
        self.model_path = Path(model_path).resolve()
        self.bindings = tuple(bindings)
        if not self.bindings or len({b.name for b in self.bindings}) != len(self.bindings):
            raise ValueError('Bindings need unique receiver names')
        if len({b.joint for b in self.bindings}) != len(self.bindings):
            raise ValueError('A model joint may only have one motor binding')
        for b in self.bindings:
            if (not b.name or not b.joint or not all(map(math.isfinite, (b.scale, b.offset, b.max_effort)))
                    or b.scale == 0 or b.max_effort <= 0):
                raise ValueError('Bindings require finite nonzero scale and positive effort limit')
        if not math.isfinite(timestep) or timestep <= 0:
            raise ValueError('Physics timestep must be positive and finite')
        if any(not math.isfinite(x) or x < 0 for x in (speed_noise, position_noise, position_step)):
            raise ValueError('Feedback noise and quantization must be finite and nonnegative')
        if self.model_path.suffix.lower() == '.urdf':
            from robot_platform.models.physics_audit import audit_urdf
            audit = audit_urdf(self.model_path)
            if audit['errors']:
                raise ValueError('URDF physics audit failed: ' + '; '.join(audit['errors']))
            if audit['mimic_joints']:
                raise ValueError('URDF mimic joints require an explicit, tested MJCF coupling')
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        if self.model.nu:
            raise ValueError('Use an actuator-free model: the adapter applies the receiver MIT effort law')
        # Moving bodies with exporter placeholders must not masquerade as physics.
        for body in range(1, self.model.nbody):
            if self.model.body_dofnum[body] and self.model.body_mass[body] <= 1e-6:
                raise ValueError('Moving body has missing/placeholder mass; run the URDF physics audit')
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, b.joint) for b in self.bindings]
        if any(i < 0 for i in ids):
            raise ValueError('Binding references a missing MuJoCo joint')
        # NumPy scalars and pybind enums do not compare equally in every wheel.
        motor_joint_types = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        if any(int(self.model.jnt_type[i]) not in motor_joint_types for i in ids):
            raise ValueError('Motor bindings support hinge and slide joints only')
        self._qadr = np.array([self.model.jnt_qposadr[i] for i in ids])
        self._vadr = np.array([self.model.jnt_dofadr[i] for i in ids])
        self._scale = np.array([b.scale for b in self.bindings])
        self._offset = np.array([b.offset for b in self.bindings])
        self._limit = np.array([b.max_effort for b in self.bindings])
        self.names = [b.name for b in self.bindings]
        if positions is not None:
            q = np.asarray(positions, dtype=float)
            if q.shape != (len(self.names),) or not np.isfinite(q).all():
                raise ValueError('Initial positions must be finite and in receiver order')
            self.data.qpos[self._qadr] = self._offset + self._scale * q
        self.torque = np.zeros(len(self.names))
        self.speed_noise, self.position_noise, self.position_step = speed_noise, position_noise, position_step
        self.random = random.Random(seed)
        self.disturbance = disturbance
        self.quiet, self.dropped = set(), set()
        mujoco.mj_forward(self.model, self.data)

    @property
    def q(self):
        return (self.data.qpos[self._qadr] - self._offset) / self._scale

    @property
    def v(self):
        return self.data.qvel[self._vadr] / self._scale

    @property
    def t(self):
        return float(self.data.time)

    def pose(self):
        return dict(zip(self.names, map(float, self.q)))

    def metadata(self):
        return {'backend': 'mujoco', 'version': self._mj.__version__,
                'root_xml_sha256': hashlib.sha256(self.model_path.read_bytes()).hexdigest(),
                'timestep_s': float(self.model.opt.timestep),
                'bindings': [asdict(b) for b in self.bindings],
                'physical_validation': 'unvalidated',
                'effort_model': 'ideal_clamped_motor_torque',
                'feedback_effort': 'applied_motor_command_not_a_torque_sensor'}

    def step(self, commands, dt):
        h = float(self.model.opt.timestep)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Step duration must be positive and finite')
        steps = round(dt / h)
        if steps < 1 or not math.isclose(steps * h, dt, rel_tol=1e-10, abs_tol=1e-12):
            raise ValueError('Receiver step must be an integer multiple of the physics timestep')
        values = None
        if commands is not None:
            if len(commands) != len(self.names):
                raise ValueError('Commands must match receiver joint order and count')
            values = np.array([[c[key] for key in ('position', 'velocity', 'effort', 'kp', 'kd')]
                               for c in commands], dtype=float)
            if not np.isfinite(values).all() or (values[:, 3:] < 0).any():
                raise ValueError('Commands need finite values and nonnegative gains')
        warnings = self.data.warning.number.copy()
        for _ in range(steps):
            torque = np.zeros(len(self.names)) if values is None else (
                values[:, 3] * (values[:, 0] - self.q) +
                values[:, 4] * (values[:, 1] - self.v) + values[:, 2])
            torque = np.clip(torque, -self._limit, self._limit)
            for i, name in enumerate(self.names):
                if name in self.dropped:
                    torque[i] = 0.
            self.torque = torque
            pushed = self.disturbance(self) if self.disturbance else {}
            if set(pushed) - set(self.names) or any(not math.isfinite(v) for v in pushed.values()):
                raise ValueError('Disturbance needs known motor names and finite effort')
            external = np.array([pushed.get(name, 0.) for name in self.names])
            self.data.qfrc_applied[:] = 0.
            self.data.qfrc_applied[self._vadr] = (torque + external) / self._scale
            before = self.t
            self._mj.mj_step(self.model, self.data)
            if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                    or not math.isclose(self.t, before + h, rel_tol=1e-9, abs_tol=1e-12)
                    or np.any(self.data.warning.number != warnings)):
                raise RuntimeError('MuJoCo reported an unstable/invalid step; preserve the failing model and input')

    def measured(self):
        readings = []
        for name, q, v, torque in zip(self.names, self.q, self.v, self.torque):
            position = q + self.random.gauss(0., self.position_noise)
            if self.position_step:
                position = round(position / self.position_step) * self.position_step
            readings.append({'position': float(position),
                             'velocity': float(v + self.random.gauss(0., self.speed_noise)),
                             'effort': float(torque),
                             'quiet': name in self.quiet or name in self.dropped})
        return readings
