"""A simulated Minerva behind the actual Rust receiver, for Helm teleop with the leader arms.

Helm derives the receiver (limits, gains, feed-forward, lifecycle) exactly as it does for the
real arms; only what the receiver drives differs. Each joint is an ArmPlant joint with the
inertia, friction and gravity its Tune sweep measured. Where a sweep measured none, the joint
takes the rough value in NOMINAL (the rehearsals' values), and `nominal` lists it. A MuJoCo
window draws the arms from those joints, mapped into the model through Helm's confirmed
directions and the grippers' preview transmissions.

A motor in the simulator's actuator file (`actuators_path`, written by characterization.sim2real
from real teleop) takes the values fitted there instead: inertia, friction, damping, its drive's
command delay and a scale on its gravity. The file binds each to the motor it was fitted on.

This is the receiver and operator loop on measured joints, not a physics scene: there are no
objects or contacts, and the joints do not load one another beyond the measured gravity.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import threading
import time

from robot_platform.control.gravity import gravity_effort, gravity_model
from .arm_plant import TICK_S, ArmPlant, PlantReceiver

ROOT = Path(__file__).resolve().parents[3]
SCENE = ROOT / 'urdf/minerva/mujoco/scene.xml'
# Inertia (kg m2), friction and static friction (Nm) where no sweep measured them: rough, not
# measured (tests/contract/test_rehearsal.py SPECS).
NOMINAL = {'j1': (.3, .33, .45), 'j2': (.25, .67, .9), 'j3': (.08, .41, .55), 'j4': (.01, .19, .25),
           'j5': (.004, .15, .2), 'j6': (.003, .11, .15), 'gripper': (.002, .1, .15)}
POSITION_NOISE_RAD = .0006  # Reading noise before the drives' 16-bit steps, as the rehearsals use.
# A sweep accepts gains whose steps overshoot at most 10%: a damping ratio of 0.6 or more. Where
# the gains' own damping falls short of it, the gearbox makes up the rest (the rehearsals' rule).
DAMPING_RATIO = .6
VIEW_HZ = 30.
VIEW_GROUPS = (0, 2)  # The floor and the source visuals.
ACTUATORS_KIND = 'sim_actuator_calibration'  # The simulator's actuator file (characterization.sim2real).
FITTED = ('inertia', 'friction', 'static', 'viscous')


def actuators_path(profile_path, asset_id):
    """The simulator's actuator file: beside the servo calibration, in `simulation/`, which robot releases leave out
    (ops.release ships config/devices/*.json only)."""
    return Path(profile_path).resolve().parent / 'simulation' / f'{asset_id}-actuators.json'


def _fitted(entry):
    """A motor's fitted values from the actuator file, or why they cannot drive the plant."""
    values = {key: entry.get(key) for key in FITTED + ('gravity_scale', 'delay_ticks')}
    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values.values()):
        return None, 'a value is missing or not a finite number'
    if not (values['inertia'] > 0 and 0 <= values['friction'] <= values['static'] and values['viscous'] >= 0
            and values['gravity_scale'] > 0):
        return None, 'a value is out of range'
    if not 0 <= values['delay_ticks'] <= 1:
        return None, 'a command delay over one tick is not modelled'
    fitted = {key: float(values[key]) for key in FITTED + ('gravity_scale',)}
    return dict(fitted, delay=float(values['delay_ticks']) * TICK_S), None


def calibrated_actuators(path, servo, names, terms):
    """{motor: values fitted on real responses} from the simulator's actuator file at `path`, and what was used.

    A motor of `names` takes the file's values only where the file was fitted on the motor the servo calibration
    binds there now: the same asset, UID and firmware. Any other keeps its current values, and the record says why.
    A motor's gravity scale applies only to the gravity terms it was fitted against (`terms`, control.gravity);
    with other terms (a new gravity sweep, other directions) its gravity is unscaled."""
    path = Path(path)
    record = {'path': str(path), 'used': []}
    if not path.is_file():
        return {}, dict(record, reason='no file')
    raw = path.read_bytes()
    record['sha256'] = hashlib.sha256(raw).hexdigest()
    try:
        document = json.loads(raw.decode('utf-8'))
    except ValueError as exc:
        return {}, dict(record, reason=f'not JSON: {exc}')
    if not isinstance(document, dict) or document.get('schema') != 1 or document.get('kind') != ACTUATORS_KIND:
        return {}, dict(record, reason='not a simulator actuator file')
    asset = (servo or {}).get('asset_id')
    if document.get('asset_id') != asset:
        return {}, dict(record, reason=f"fitted on {document.get('asset_id')}, not {asset}")
    motors = document.get('motors') if isinstance(document.get('motors'), dict) else {}
    bound = (servo or {}).get('joints') or {}
    calibrated, refused, unscaled = {}, {}, {}
    for name in names:
        entry = motors.get(name)
        if not isinstance(entry, dict):
            continue
        fitted_on = (entry.get('uid'), entry.get('firmware'))
        binds = ((bound.get(name) or {}).get('uid'), (bound.get(name) or {}).get('firmware'))
        if fitted_on != binds:
            refused[name] = (f'fitted on UID {fitted_on[0]} firmware {fitted_on[1]}; the servo calibration binds UID '
                             f'{binds[0]} firmware {binds[1]}')
            continue
        fitted, problem = _fitted(entry)
        if problem:
            refused[name] = problem
            continue
        here = [[float(coefficient), dict(angle)] for coefficient, angle in terms.get(name, ())]
        if fitted['gravity_scale'] != 1 and here != entry.get('gravity_terms'):
            unscaled[name] = 'its gravity terms differ from those the scale was fitted against'
            fitted['gravity_scale'] = 1.
        calibrated[name] = fitted
    record['used'] = list(calibrated)
    if refused:
        record['refused'] = refused
    if unscaled:
        record['gravity_unscaled'] = unscaled
    return calibrated, record


def joint_specs(names, servo, calibrated=None):
    """ArmPlant joints for the receiver's `names`, and {name: [the fields taken from NOMINAL]}. A joint in
    `calibrated` (calibrated_actuators) takes its values fitted on real responses instead."""
    specs, nominal = [], {}
    for name in names:
        fitted = (calibrated or {}).get(name)
        if fitted:
            specs.append(dict(name=name, **{key: fitted[key] for key in FITTED},
                              **({'delay': fitted['delay']} if fitted['delay'] else {})))
            continue
        inertia, friction, static = NOMINAL[name.split('_')[-1]]
        saved = ((servo or {}).get('joints') or {}).get(name) or {}
        torque = (saved.get('measured') or {}).get('torque') or {}
        missing = []
        if (torque.get('inertia') or 0.) > 0.:
            inertia = float(torque['inertia'])
        else:
            missing.append('inertia')
        # Left J1's drive reports no torque, so its sweep fitted a friction of ~1e-19.
        if (torque.get('friction') or 0.) > 1e-6:
            friction = float(torque['friction'])
            static = max(static, 4 / 3 * friction)
        else:
            missing += ['friction', 'static']
        viscous = 0.
        if saved.get('hold_kp') and saved.get('hold_kd') is not None:
            viscous = max(0., 2 * DAMPING_RATIO * math.sqrt(saved['hold_kp'] * inertia) - saved['hold_kd'])
        specs.append(dict(name=name, inertia=inertia, friction=friction, static=static, viscous=viscous))
        if missing:
            nominal[name] = missing
    return specs, nominal


class ModelPose:
    """Model joint positions from receiver (motor) positions: q_model = scale * q_motor + offset, as the
    receiver's joint map gives it, then each mimic jaw from its equality (q = a0 + a1 * q_source)."""

    def __init__(self, model, joint_map, names):
        import mujoco
        self._mujoco, self.model = mujoco, model
        index = {name: i for i, name in enumerate(names)}
        self.rows = []
        for joint, row in joint_map.items():
            if row['actuator'] not in index:
                continue  # A motor outside this receiver (the other arm) stays at the model's zero.
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0:
                raise ValueError(f'The simulation model has no joint {joint}')
            self.rows.append((int(model.jnt_qposadr[jid]), index[row['actuator']], float(row['scale']),
                              float(row['offset'])))
        self.mimic = [(int(model.jnt_qposadr[model.eq_obj1id[e]]), int(model.jnt_qposadr[model.eq_obj2id[e]]),
                       float(model.eq_data[e][0]), float(model.eq_data[e][1]))
                      for e in range(model.neq) if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT]

    def apply(self, data, q):
        for address, i, scale, offset in self.rows:
            data.qpos[address] = offset + scale * q[i]
        for follower, source, a0, a1 in self.mimic:
            data.qpos[follower] = a0 + a1 * data.qpos[source]
        self._mujoco.mj_kinematics(self.model, data)


class ModelView:
    """A MuJoCo window showing the plant's joints. Its own thread draws at VIEW_HZ; `show` only stores
    the latest positions, so the control loop never waits on the window. Closing the window ends the
    view, not the session."""

    def __init__(self, scene, joint_map, names, *, rate_hz=VIEW_HZ):
        import mujoco
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.pose = ModelPose(self.model, joint_map, names)
        self.period_s = 1 / rate_hz
        self._latest = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='minerva-sim-view')
        self._thread.start()

    def show(self, q):
        self._latest = tuple(q)

    def _run(self):
        from mujoco import viewer
        with viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as window:
            with window.lock():
                for group in range(len(window.opt.geomgroup)):
                    window.opt.geomgroup[group] = int(group in VIEW_GROUPS)
            while window.is_running() and not self._stop.is_set():
                q = self._latest
                if q is not None:
                    with window.lock():
                        self.pose.apply(self.data, q)
                window.sync()
                time.sleep(self.period_s)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)


class SimulatedReceiver(PlantReceiver):
    """The live receiver's interface over the simulated arms, plus the window that shows them."""

    def __init__(self, config, plant, *, clock, directory, view=None, nominal=None, actuators=None):
        super().__init__(config, plant, clock=clock, directory=directory)
        self.view, self.nominal = view, dict(nominal or {})
        self.task_view = view if hasattr(view, 'command') else None
        self.actuators = actuators  # What the actuator file gave (calibrated_actuators), when one was named.
        if view is not None:
            view.show(plant.q)

    def exchange(self, request):
        reply = super().exchange(request)
        if self.task_view is not None:
            recent = (self.last or {}).get('recent') or []
            if recent:
                row = recent[-1]
                self.task_view.show([row[1 + 4*i] for i in range(len(self.plant.names))],
                                    qcmd=[row[4 + 4*i] for i in range(len(self.plant.names))],
                                    source_time=self.epoch + row[0] / 1000)
        elif self.view is not None:
            self.view.show(self.plant.q)
        return reply

    def close(self):
        try:
            super().close()
        finally:
            if self.view is not None:
                self.view.close()


def _holding(terms, scale):
    if scale == 1:
        return lambda pose: gravity_effort(terms, pose)
    return lambda pose: scale * gravity_effort(terms, pose)


def open_simulation(derived, *, model, directions, motors, servo, directory, clock, scene=SCENE, viewer=True,
                    seed=1, actuators=None, task_scene=False, visual_mode='collision_proxy'):
    """A SimulatedReceiver for a derived receiver (guarded_config.derive_receiver), resting at its initial pose.

    `model` is the URDF model the receiver was derived against, `directions` Helm's confirmed mapping and
    `motors` the reference roster, for the gravity each joint's sweep fitted (control.gravity). `actuators` names
    the simulator's actuator file (actuators_path), if any: the motors it matches take its fitted values."""
    config = derived['config']
    names = [joint['name'] for joint in config['joints']]
    terms = gravity_model(model, directions, [m for m in motors if m['name'] in names], (servo or {}).get('gravity'))
    calibrated, record = calibrated_actuators(actuators, servo, names, terms) if actuators is not None else ({}, None)
    specs, nominal = joint_specs(names, servo, calibrated)
    for spec, q in zip(specs, derived['initial']):
        spec['q'] = float(q)
    plant = ArmPlant(specs, gravity={n: _holding(terms[n], calibrated.get(n, {}).get('gravity_scale', 1.))
                                     for n in names if n in terms},
                     position_noise=POSITION_NOISE_RAD, seed=seed)
    if task_scene:
        from .task_scene import TaskSceneView
        view = TaskSceneView(scene, derived['joint_map'], names, directory=Path(directory) / 'episodes',
                             metadata={'actuators': record, 'nominal': nominal,
                                       'receiver_identity': config['identity']},
                             seed=seed, viewer=viewer, visual_mode=visual_mode)
    else:
        view = ModelView(scene, derived['joint_map'], names) if viewer else None
    return SimulatedReceiver(config, plant, clock=clock, directory=directory, view=view, nominal=nominal,
                             actuators=record)
