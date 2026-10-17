"""Run N trials of a policy against MinervaTaskScene and report them.

A policy is any callable `policy(observation) -> chunk`: a (H, 14) array of joint commands in
receiver motor coordinates (the task capture's qcmd/action space, receiver joint order), row k
meant for k * action_dt seconds after the observation. Optional attributes:

- `name`: shown in the report (default: the class or function name);
- `action_dt`: seconds between chunk rows (default: the control period);
- `execute_steps`: rows executed before the policy is asked again (default: the whole chunk);
- `camera_roles`: cameras rendered into the observation, e.g. ('right_wrist', 'head');
- `start_pose(context)`: where the trial starts, in motor coordinates (default: the receiver's
  initial pose, the arms hanging);
- `reset(context)`: called before each trial;
- `describe()`: a dict recorded in the report.

The observation holds `t` (s since the scene reset), `q`, `qcmd`, `dq`, `effort` (receiver
feedback, motor coordinates), `joint_names` and `images` when requested. The context passed to
`reset`/`start_pose` also holds the cube start: privileged simulation state that only scripted
baselines may use.

Each trial: stage the arms onto the start pose (Helm's Engage alignment), reset the scene there
(the arms are clear of the table: a start pose that is not ends the trial as a forbidden
penetration), place the cube at the trial's grid pose, then run the policy until the scene
reports success, a drop, a forbidden penetration or the cube settled out of sequence, or the
timeout. Commands outside the receiver's joint limits are clipped (and counted), as the leader
mapping clips its targets.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from robot_platform.simulation.task_scene import MAX_SOURCE_GAP_S, PHYSICS_DT, MinervaTaskScene
from robot_platform.simulation.workcell import resolve as resolve_workcell

from . import report as reports, trials as trial_records
from .grid import GridSpec, cube_poses
from .plants import PlantFault, derive_stack, open_plant

MAX_SCENE_STEP_S = .1  # MinervaTaskScene.update's largest dt.
GRIPPERS = ('kinematic', 'driven')


@dataclass
class RunConfig:
    plant: str = 'armplant'
    timeout_s: float = 30.
    staging_timeout_s: float = 30.
    grid: GridSpec = field(default_factory=GridSpec)
    trials: tuple | None = None  # Indices into the grid's poses; None runs them all.
    plant_seed: int = 1
    scene_pose: str = 'plant'
    camera_size: tuple = (160, 120)
    visual_mode: str = 'collision_proxy'
    workcell: object = None  # simulation.workcell: None or 'synthetic', a Workcell, or a workcell file's path.
    # 'kinematic': the grippers are posed like the arms. 'driven': the armplant's grippers move in the scene under
    # their drives, stall on the cube, and the receiver reads that back (MinervaTaskScene.step_driven).
    grippers: str = 'kinematic'
    # Disturbances, each drawn per trial from disturbance_seed and the trial's index:
    latency_s: float = 0.  # A chunk takes effect this long after its observation; one inference at a time.
    actuator_spread: float = 0.  # Each armplant joint's inertia, friction and damping x U[1-s, 1+s], unknown to the receiver.
    appearance: dict | None = None  # MinervaTaskScene.randomize_appearance bounds: lights, colours, camera places.
    disturbance_seed: int = 0

    def __post_init__(self):
        if self.grippers not in GRIPPERS:
            raise ValueError(f'grippers is one of {GRIPPERS}')
        if self.grippers == 'driven' and self.plant != 'armplant':
            raise ValueError('driven grippers need the armplant plant: the ideal plant sends no drive commands')
        if not (math.isfinite(self.latency_s) and 0 <= self.latency_s <= 1):
            raise ValueError('latency_s is in [0, 1] s')
        if not 0 <= self.actuator_spread < 1 or (self.actuator_spread and self.plant != 'armplant'):
            raise ValueError('actuator_spread is in [0, 1) and needs the armplant plant')

    def describe(self):
        return {'plant': self.plant, 'timeout_s': self.timeout_s, 'staging_timeout_s': self.staging_timeout_s,
                'grid': self.grid.describe(), 'trials': None if self.trials is None else list(self.trials),
                'plant_seed': self.plant_seed, 'scene_pose': self.scene_pose,
                'camera_size': list(self.camera_size), 'visual_mode': self.visual_mode,
                'workcell': resolve_workcell(self.workcell).name, 'grippers': self.grippers,
                'latency_s': self.latency_s, 'actuator_spread': self.actuator_spread,
                'appearance': self.appearance, 'disturbance_seed': self.disturbance_seed}


@dataclass
class TrialContext:
    index: int
    cube_start: dict  # Privileged: scripted baselines only.
    joint_names: tuple
    joint_map: dict
    lower: np.ndarray
    upper: np.ndarray
    home: np.ndarray
    period_s: float
    plant: str
    workcell: object = None  # The scene's Workcell (simulation.workcell): the table, bucket and cube it judges.
    lead: np.ndarray | None = None  # How far following lets a command lead each joint (rad), the lifecycle's max_lead.


class _PolicyError(Exception):
    """The policy's reset or start pose failed: the trial aborts, the run goes on."""


def policy_name(policy):
    return getattr(policy, 'name', None) or getattr(policy, '__name__', None) or type(policy).__name__


def _chunk(raw, joints):
    chunk = np.asarray(raw, dtype=float)
    if chunk.ndim == 1:
        chunk = chunk[None, :]
    if chunk.ndim != 2 or chunk.shape[1] != joints or chunk.shape[0] < 1 or not np.all(np.isfinite(chunk)):
        raise ValueError(f'a policy returns a finite (H, {joints}) command chunk, got shape {chunk.shape}')
    return chunk


def _command(chunk, tau, action_dt):
    """Row k is meant for k * action_dt after the chunk's observation: linear between rows, the last held."""
    f = max(0., tau / action_dt)
    k = int(math.floor(f + 1e-9))
    if k >= len(chunk) - 1:
        return chunk[-1].copy()
    w = f - k
    return chunk[k] * (1 - w) + chunk[k + 1] * w


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)])


def git_identity(root=None):
    root = Path(root or Path(__file__).resolve().parents[3])
    try:
        commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True,
                                timeout=10).stdout.strip()
        # Untracked source counts: an uncommitted policy or harness file is not reproducible from the commit.
        changed = subprocess.run(['git', 'status', '--porcelain'], cwd=root, capture_output=True, text=True,
                                 timeout=10).stdout.splitlines()
        return {'commit': commit or None, 'uncommitted_paths': len(changed)}
    except (OSError, subprocess.SubprocessError):
        return {'commit': None, 'uncommitted_paths': None}


class TrialRunner:
    def __init__(self, config=None, *, stack=None, directory=None, progress=None):
        self.config = config or RunConfig()
        self.stack = stack or derive_stack()
        self.directory = Path(directory) if directory else None
        self.progress = progress or (lambda message: None)
        self.plant = None
        self.scene = None
        self._plant_builds = 0
        self._description = None

    # -- resources ---------------------------------------------------------

    def _open_plant(self):
        if self.plant is None:
            options = {}
            if self.config.plant == 'armplant':
                options = dict(seed=self.config.plant_seed + self._plant_builds, scene_pose=self.config.scene_pose,
                               directory=None if self.directory is None else
                               self.directory / f'receiver-{self._plant_builds}')
            self.plant = open_plant(self.config.plant, self.stack, **options)
            self._plant_builds += 1
            self._description = self.plant.describe()
        return self.plant

    def _drop_plant(self):
        if self.plant is not None:
            try:
                self.plant.close()
            finally:
                self.plant = None

    def _open_scene(self):
        if self.scene is None:
            width, height = self.config.camera_size
            drives = self._open_plant().gripper_drives() if self.config.grippers == 'driven' else None
            self.scene = MinervaTaskScene(self.stack['derived']['joint_map'], self.stack['names'], width=width,
                                          height=height, visual_mode=self.config.visual_mode,
                                          workcell=self.config.workcell, grippers=drives)
        return self.scene

    def close(self):
        try:
            self._drop_plant()
        finally:
            if self.scene is not None:
                self.scene.close()
                self.scene = None

    # -- one trial ---------------------------------------------------------

    def _advance_scene(self, q, dt):
        """One scene update: the scene interpolates the robot linearly from its last pose over `dt`, auditing
        every substep for forbidden penetration. A control period (at most a few receiver ticks) is far
        below the scene's 0.1 s bound."""
        if not 0 < dt <= MAX_SCENE_STEP_S:
            raise ValueError(f'scene step {dt} s outside (0, {MAX_SCENE_STEP_S}] s')
        return self.scene.update(q, round(dt / PHYSICS_DT) * PHYSICS_DT)

    def _place_cube(self, pose):
        """Put the cube at the trial's start, 2 mm above the table as the scene's own start; None, or why not."""
        scene, mj = self.scene, self.scene.mj
        address = scene.cube_qadr
        workcell = scene.workcell
        scene.data.qpos[address:address + 3] = (pose['x'], pose['y'], workcell.table_top + workcell.cube_half + .002)
        scene.data.qpos[address + 3:address + 7] = _quat_yaw(pose['yaw'])
        scene.data.qvel[:] = 0.
        mj.mj_forward(scene.model, scene.data)
        for i in range(scene.data.ncon):
            contact = scene.data.contact[i]
            if scene.cube_geom in (contact.geom1, contact.geom2) and contact.dist < 0:
                other = int(contact.geom2 if contact.geom1 == scene.cube_geom else contact.geom1)
                name = mj.mj_id2name(scene.model, mj.mjtObj.mjOBJ_GEOM, other)
                return f'cube placed inside {name} ({-contact.dist * 1000:.1f} mm) at the start pose'
        return None

    def run_trial(self, policy, pose, index, recorder=None):
        """One trial's record. A `recorder` (generate.EpisodeRecorder) starts once the scene is reset with the cube in
        place, samples after every control cycle, and is finished with the record."""
        record = self._run_trial(policy, pose, index, recorder)
        if recorder is not None:
            recorder.finish(record)
        return record

    def _run_trial(self, policy, pose, index, recorder):
        config, name = self.config, policy_name(policy)
        plant, scene = self._open_plant(), self._open_scene()
        context = TrialContext(index=index, cube_start=dict(pose), joint_names=tuple(plant.names),
                               joint_map=plant.joint_map, lower=plant.lower.copy(), upper=plant.upper.copy(),
                               home=plant.home.copy(), period_s=plant.period_s, plant=config.plant,
                               workcell=scene.workcell, lead=plant.lead.copy())
        record = {'schema': trial_records.SCHEMA, 'trial': index, 'source': 'simulation', 'policy': name,
                  'cube_start': {k: pose[k] for k in ('x', 'y', 'yaw', 'cell')}, 'seed': config.grid.seed}
        aborted = timed_out = False
        state, staged, ready = None, None, False
        clipped = calls = 0
        tracking = np.zeros(len(plant.names))
        started = time.perf_counter()
        rng = np.random.default_rng([config.disturbance_seed, index])
        record['disturbances'] = {'latency_s': config.latency_s, 'actuator_factors': self._vary_actuators(plant, rng)}
        try:
            try:
                if hasattr(policy, 'reset'):
                    policy.reset(context)
                start = policy.start_pose(context) if hasattr(policy, 'start_pose') else None
                start = plant.home if start is None else np.asarray(start, dtype=float)
                if start.shape != (len(plant.names),) or not np.all(np.isfinite(start)):
                    raise ValueError(f'start_pose must be {len(plant.names)} finite motor coordinates')
            except Exception as exc:
                raise _PolicyError(exc) from exc
            staged = plant.stage(start, config.staging_timeout_s)
            if not staged['ok']:
                return self._excluded(record, f"staging: {staged.get('detail')}", staged)
            # The object scene skips staging: jump its robot to the staged pose, then reset there. Reset
            # restores the cube and discards anything that jump did.
            try:
                self._advance_scene(plant.scene_pose(), .05)
            except RuntimeError:
                pass
            scene.reset(seed=config.grid.seed * 1000 + index)
            ready = True
            if config.appearance:
                record['disturbances']['appearance'] = scene.randomize_appearance(rng, **config.appearance)
            problem = self._place_cube(pose)
            if problem:
                return self._excluded(record, problem, staged)
            if scene.driven:
                # The reset placed the scene's gripper drives where the plant's grippers are; from here on the
                # scene moves them, tick by tick, inside plant.step.
                plant.couple(scene)
            if recorder is not None:
                recorder.start(scene, plant, context, policy)
            action_dt = float(getattr(policy, 'action_dt', None) or plant.period_s)
            execute = getattr(policy, 'execute_steps', None)
            roles = tuple(getattr(policy, 'camera_roles', None) or ())
            chunk, chunk_t, horizon = None, 0., 0
            pending = None  # One inference in flight: (chunk, observed at, takes effect at).
            held = np.asarray(plant.observation()['qcmd'], dtype=float)  # Until the first chunk takes effect.
            t = 0.
            while True:
                if pending is None and (chunk is None or t - chunk_t >= horizon * action_dt - 1e-9):
                    observation = dict(plant.observation(), t=t, joint_names=plant.names)
                    if roles:
                        observation['images'] = scene.render(roles)[0]
                    try:
                        pending = (_chunk(policy(observation), len(plant.names)), t, t + config.latency_s)
                    except Exception as exc:
                        aborted = f'policy error: {type(exc).__name__}: {exc}'
                        break
                    calls += 1
                if pending is not None and t >= pending[2] - 1e-9:
                    # Rows are meant for times after the observation: those already past are skipped.
                    chunk, chunk_t, _ = pending
                    pending = None
                    horizon = min(len(chunk), int(execute)) if execute else len(chunk)
                command = held.copy() if chunk is None else _command(chunk, t - chunk_t, action_dt)
                bounded = plant.clip(command)
                clipped += int(np.any(bounded != command))
                q, dt = plant.step(bounded)
                tracking = np.maximum(tracking, np.abs(q - bounded))
                if getattr(plant, 'coupled', False):
                    state = scene.snapshot()  # The plant's ticks already stepped the scene.
                elif dt > 0:
                    state = self._advance_scene(q, dt)
                if recorder is not None:
                    recorder.sample(scene, plant)
                t = float(scene.data.time)
                if state and (state['forbidden_contacts'] or state['dropped'] or state['bucket_settled']):
                    break
                if t >= config.timeout_s - 1e-9:
                    timed_out = True
                    break
        except _PolicyError as exc:
            aborted = f'policy error: {type(exc.__cause__).__name__}: {exc.__cause__}'
        except PlantFault as exc:
            aborted = str(exc)
            self._drop_plant()
        except RuntimeError as exc:
            # MuJoCo numerical warning: the scene quarantines the episode.
            aborted = f'scene: {exc}'
        finally:
            if self.plant is not None and getattr(self.plant, 'coupled', False):
                self.plant.decouple()
            if self.plant is not None:
                self._restore_actuators(self.plant)
            scene.restore_appearance()
        # Events belong to this trial only once its reset happened.
        state = scene.snapshot() if ready else {'events': [], 'scene_time': 0., 'run_id': None}
        stages, outcome, eligible, detail = trial_records.outcome_from_events(state['events'], timed_out=timed_out,
                                                                              aborted=aborted)
        if outcome is None:
            outcome, detail = 'source_gap_or_abort', 'trial ended without a terminal event'
        record.update(outcome=outcome, eligible=eligible, stages=stages, detail=detail,
                      duration_s=float(state['scene_time']), events=state['events'], run_id=state['run_id'],
                      policy_calls=calls, commands_clipped_cycles=clipped,
                      max_abs_pose_minus_command_rad=dict(zip(plant.names, np.round(tracking, 5).tolist())),
                      staging=None if staged is None else {'duration_s': staged['duration_s'],
                                                          'max_abs_error_rad': float(np.max(np.abs(staged['error'])))},
                      wall_s=time.perf_counter() - started)
        return record

    def _vary_actuators(self, plant, rng):
        """Scale each armplant joint's inertia, friction (and stiction) and damping by its own U[1-s, 1+s]; the
        receiver keeps the values it was derived with. {joint: [inertia, friction, damping factors]}, or None."""
        spread = self.config.actuator_spread
        if not spread:
            return None
        specs = plant.plant.specs
        if not hasattr(plant, '_nominal_specs'):
            plant._nominal_specs = [dict(spec) for spec in specs]
        factors = {}
        for spec, nominal in zip(specs, plant._nominal_specs):
            f = rng.uniform(1 - spread, 1 + spread, size=3)
            spec['inertia'] = nominal['inertia'] * f[0]
            spec['friction'] = nominal['friction'] * f[1]
            spec['static'] = nominal.get('static', nominal['friction']) * f[1]
            spec['viscous'] = nominal.get('viscous', 0.) * f[2]
            factors[spec['name']] = f.round(4).tolist()
        return factors

    @staticmethod
    def _restore_actuators(plant):
        for spec, nominal in zip(getattr(getattr(plant, 'plant', None), 'specs', ()), getattr(plant, '_nominal_specs', ())):
            spec.clear()
            spec.update(nominal)

    def _excluded(self, record, detail, staged):
        record.update(outcome='invalid_start', eligible=True, stages=dict.fromkeys(trial_records.STAGES),
                      detail=detail, duration_s=0.,
                      events=[], staging=None if staged is None else {
                          'duration_s': staged['duration_s'],
                          'max_abs_error_rad': float(np.max(np.abs(staged['error'])))})
        return record

    # -- a run -------------------------------------------------------------

    def run(self, policy):
        poses = cube_poses(self.config.grid)
        selected = poses if self.config.trials is None else [poses[i] for i in self.config.trials]
        records = []
        for pose in selected:
            record = self.run_trial(policy, pose, pose['index'])
            self.progress(f"trial {record['trial']}: {record['outcome']}"
                          + (f" ({record['detail']})" if record.get('detail') else ''))
            records.append(record)
        return records

    def meta(self, policy):
        capabilities = self._open_scene().capabilities
        description = self._description if self._description is not None else self._open_plant().describe()
        return {'policy': policy_name(policy),
                'protocol': dict(self.config.describe(), cube_poses=cube_poses(self.config.grid),
                                 max_source_gap_s=MAX_SOURCE_GAP_S,
                                 stop_conditions='success, dropped cube, forbidden penetration, bucket out of '
                                                 'sequence, timeout, abort'),
                'provenance': dict(description, plant_builds=self._plant_builds, git=git_identity(),
                                   policy=getattr(policy, 'describe', lambda: {})(),
                                   scene={key: capabilities[key] for key in
                                          ('scene_sha256', 'contact_geometry_sha256', 'mujoco_version',
                                           'visual_mode', 'task_parameters', 'workcell', 'workcell_placeholders',
                                           'contact_parameters_measured', 'limitations')})}


def evaluate(policy, config=None, out=None, *, stack=None, progress=None):
    """Run every configured trial and write trials.jsonl, report.json and report.md under `out`."""
    out = Path(out) if out else None
    runner = TrialRunner(config or RunConfig(), stack=stack, directory=None if out is None else out / 'receiver', progress=progress)
    try:
        records = runner.run(policy)
        meta = runner.meta(policy)
    finally:
        runner.close()
    report = reports.build_report(records, meta)
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
        trial_records.write_jsonl(out / 'trials.jsonl', records)
        reports.write(report, out)
    return report
