"""Fixed, seeded cube start poses over the reachable table area.

The default area is a rectangle inside what the left gripper reaches with the frozen fixture's
grasp orientation (fingers down, jaws along a cube face), within the receiver's joint limits and
without robot/table/bucket penetration: an IK sweep over the table (evaluation.reach) found it,
and tests/contract/test_eval_reference_policies.py re-checks every default pose. Its centre cell
is the scene's own cube start (task_scene.CUBE_START). These are synthetic task settings, not a
measured real workcell; on the robot, place the cube at the same table coordinates. A measured
workcell (simulation.workcell) brings its own cube region and start: `GridSpec.for_workcell`.

Each cell's pose is its centre plus a seeded offset of at most `jitter` of half a cell, so a seed
fixes the poses exactly and a different seed samples the same cells differently.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

from robot_platform.simulation.task_scene import CUBE_START

DEFAULT_X = (.10, .26)
DEFAULT_Y = (-.42, -.26)


@dataclass(frozen=True)
class GridSpec:
    x: tuple = DEFAULT_X  # Table coordinates (m), the task scene's world frame.
    y: tuple = DEFAULT_Y
    nx: int = 3
    ny: int = 3
    jitter: float = .5  # Fraction of half a cell.
    yaw: float = 0.  # Cube yaw drawn uniformly from [-yaw, yaw] (rad); 0 keeps the faces axis-aligned.
    seed: int = 0
    nominal: bool = True  # Trial 0 at the scene's own cube start, unjittered.
    start: tuple | None = None  # That start's (x, y); None: the synthetic scene's CUBE_START.

    @classmethod
    def for_workcell(cls, workcell, **options):
        """The grid over a workcell's cube region, its trial 0 at the workcell's cube start."""
        (x, y) = workcell.cube_region
        return cls(x=tuple(x), y=tuple(y), start=tuple(workcell.cube_start[:2]), **options)

    def __post_init__(self):
        if (len(self.x) != 2 or len(self.y) != 2 or not self.x[0] < self.x[1] or not self.y[0] < self.y[1]
                or not all(math.isfinite(v) for v in (*self.x, *self.y))):
            raise ValueError('grid x and y are increasing (low, high) table coordinates')
        if self.start is not None and (len(self.start) != 2 or not all(math.isfinite(v) for v in self.start)):
            raise ValueError('grid start is a finite (x, y)')
        if type(self.nx) is not int or type(self.ny) is not int or self.nx < 1 or self.ny < 1:
            raise ValueError('grid counts must be positive integers')
        if not 0 <= self.jitter <= 1 or not 0 <= self.yaw <= math.pi / 4 or type(self.seed) is not int:
            raise ValueError('jitter is in [0, 1], yaw in [0, pi/4] and seed an integer')

    def describe(self):
        return dict(asdict(self), x=list(self.x), y=list(self.y),
                    start=None if self.start is None else list(self.start))


def cube_poses(spec=GridSpec()):
    """[{'index', 'cell', 'x', 'y', 'yaw'}] in trial order: the nominal pose, then row-major cells."""
    rng = np.random.default_rng(spec.seed)
    poses = []
    if spec.nominal:
        x, y = CUBE_START[:2] if spec.start is None else spec.start
        poses.append({'cell': 'nominal', 'x': float(x), 'y': float(y), 'yaw': 0.})
    width, depth = (spec.x[1] - spec.x[0]) / spec.nx, (spec.y[1] - spec.y[0]) / spec.ny
    for j in range(spec.ny):
        for i in range(spec.nx):
            dx, dy, dyaw = rng.uniform(-1, 1, size=3)  # Drawn for every cell, so the sequence never shifts.
            poses.append({'cell': [i, j],
                          'x': float(spec.x[0] + (i + .5) * width + dx * spec.jitter * width / 2),
                          'y': float(spec.y[0] + (j + .5) * depth + dy * spec.jitter * depth / 2),
                          'yaw': float(dyaw * spec.yaw)})
    return [dict(pose, index=k) for k, pose in enumerate(poses)]


def parse_counts(text):
    """'3x3' -> (3, 3)."""
    nx, sep, ny = text.lower().partition('x')
    if not sep:
        raise ValueError('grid size is NXxNY, e.g. 3x3')
    return int(nx), int(ny)
