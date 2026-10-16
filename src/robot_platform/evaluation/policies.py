"""Reference policies that validate the harness. None of them is a learned or capable policy.

- fixture_replay: the frozen offline-IK contact trajectory (tests/fixtures/minerva_task_contact_path.json)
  replayed open loop from its own prepositioned start. It reaches the task only where the cube sits
  where the fixture expects it, and only as far as the plant tracks it.
- hold_still: holds the receiver's initial (hanging) pose. Must score 0, every trial a timeout.
- penetrating: starts at the fixture's start and drives the left gripper down through the table.
  Must be ineligible (forbidden penetration).
"""
from __future__ import annotations

import hashlib
import importlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/minerva_task_contact_path.json'


def fixture_model_coordinates(fixture):
    """{fixture joint name: (model joint, model coordinate per fixture unit)}: the fixture's own explicit map
    (arm joints are their model joints; a gripper value times the fixture-only scale is its slide)."""
    scale = fixture['gripper_test_scale_m_per_motor_rad']
    return {name: (name, 1.) if 'arm' in name else (f"{name.split('_')[0]}_gripper_finger_a_slide", scale)
            for name in fixture['joint_names']}


def fixture_trajectory(joint_names, joint_map, fixture=None):
    """The fixture's waypoints interpolated exactly as its regression replays them, as (rows, motor coordinates in
    `joint_names` order) through the receiver's motor->model `joint_map`, and the row period (s)."""
    fixture = fixture if isinstance(fixture, dict) else json.loads(Path(fixture or FIXTURE).read_text(encoding='utf-8'))
    model = fixture_model_coordinates(fixture)
    by_motor = {row['actuator']: (joint, row) for joint, row in joint_map.items()}
    if set(joint_names) != set(fixture['joint_names']):
        raise ValueError('The fixture and the receiver name different joints')
    columns = []
    for name in joint_names:
        joint, row = by_motor[name]
        if joint != model[name][0]:
            raise ValueError(f'{name} drives {joint}, but the fixture moves {model[name][0]}')
        columns.append((fixture['joint_names'].index(name), model[name][1], float(row['scale']), float(row['offset'])))

    def motor(values):
        return np.array([(values[i] * unit - offset) / scale for i, unit, scale, offset in columns])
    waypoints = fixture['waypoints']
    q = motor(waypoints[0]['q'])
    rows = [q]
    for point in waypoints[1:]:
        target = motor(point['q'])
        rows += [q + (target - q) * (k + 1) / point['steps'] for k in range(point['steps'])]
        q = target
    return np.array(rows), float(fixture['step_s'])


class FixtureReplayPolicy:
    name = 'fixture_replay'

    def __init__(self, fixture=FIXTURE):
        self.fixture = Path(fixture)
        self.rows = None

    def reset(self, context):
        self.rows, self.action_dt = fixture_trajectory(context.joint_names, context.joint_map, self.fixture)

    def start_pose(self, context):
        return self.rows[0]

    def __call__(self, observation):
        # Open loop: the rest of the trajectory from now; past its end, its last row held.
        index = min(int(round(observation['t'] / self.action_dt)), len(self.rows) - 1)
        return self.rows[index:]

    def describe(self):
        return {'kind': 'open-loop replay', 'fixture': str(self.fixture),
                'sha256': hashlib.sha256(self.fixture.read_bytes()).hexdigest(),
                'rows': None if self.rows is None else len(self.rows),
                'note': 'frozen offline IK; prepositioned start; no feedback'}


class HoldStillPolicy:
    name = 'hold_still'

    def __init__(self, pose=None):
        self.pose = None if pose is None else np.asarray(pose, dtype=float)
        self.held = None

    def reset(self, context):
        self.held = context.home.copy() if self.pose is None else self.pose.copy()

    def start_pose(self, context):
        return self.held

    def __call__(self, observation):
        return self.held[None, :]

    def describe(self):
        return {'kind': 'hold the start pose', 'pose': 'receiver initial pose (arms hanging)' if self.pose is None
                else self.pose.tolist()}


class PenetratingPolicy:
    """From the fixture's prepositioned start (gripper open just above the table), lower the left shoulder pitch
    (left_arm_j2) by `depth_rad` over `duration_s` and hold: the jaw pads go down into the table."""
    name = 'penetrating'
    action_dt = .05

    def __init__(self, depth_rad=.3, duration_s=1., joint='left_arm_j2', fixture=FIXTURE):
        self.depth_rad, self.duration_s, self.joint, self.fixture = depth_rad, duration_s, joint, Path(fixture)
        self.rows = None

    def reset(self, context):
        rows, _ = fixture_trajectory(context.joint_names, context.joint_map, self.fixture)
        start = rows[0]
        end = start.copy()
        end[list(context.joint_names).index(self.joint)] -= self.depth_rad
        steps = max(1, round(self.duration_s / self.action_dt))
        self.rows = np.array([start + (end - start) * k / steps for k in range(steps + 1)])

    def start_pose(self, context):
        return self.rows[0]

    def __call__(self, observation):
        index = min(int(round(observation['t'] / self.action_dt)), len(self.rows) - 1)
        return self.rows[index:]

    def describe(self):
        return {'kind': 'deliberate table penetration (negative control)', 'joint': self.joint,
                'depth_rad': self.depth_rad, 'duration_s': self.duration_s}


REFERENCE = {'fixture_replay': FixtureReplayPolicy, 'hold_still': HoldStillPolicy, 'penetrating': PenetratingPolicy}


def load_policy(spec):
    """A reference policy by name, or 'package.module:factory' called with no arguments."""
    if spec in REFERENCE:
        return REFERENCE[spec]()
    module, sep, attribute = spec.partition(':')
    if not sep:
        raise ValueError(f"policy is one of {sorted(REFERENCE)} or 'module:factory'")
    return getattr(importlib.import_module(module), attribute)()
