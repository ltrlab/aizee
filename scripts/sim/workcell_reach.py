"""Can each gripper take the cube on a workcell's table, straight down or tilted? Kinematic probe, no motion.

    .venv-dev/Scripts/python.exe scripts/sim/workcell_reach.py [--workcell config/devices/simulation/<asset>-workcell.json]

For each probe point it tries a top grasp (pads centred on the cube, fingers straight down) and grasps tilted
20-80 degrees from vertical, leaning away from the arm's shoulder with the closing axis level, from random
starting poses within the receiver's joint limits. Kinematic only: penetration and approach paths are not
checked. See docs/simulation/SIM_WORKCELL.md.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from robot_platform.evaluation.plants import _Stack, derive_stack
from robot_platform.evaluation.reach import GraspIK
from robot_platform.simulation.task_scene import MinervaTaskScene
from robot_platform.simulation.workcell import resolve

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
POINTS = {'right': ((-.25, -.30), (-.20, -.40), (-.10, -.25)), 'left': ((.20, -.30), (.15, -.40), (.05, -.25))}
TILTS = (20, 35, 50, 65, 80)


def tilted(ik, side, x, y, tilt, seed, iterations=300):
    """Damped least squares for a grasp tilted `tilt` rad from straight down, leaning away from the shoulder."""
    workcell = ik.scene.workcell
    joints = ik.arm(side)
    shoulder = ik.scene.fk_data.xanchor[ik.scene.mj.mj_name2id(ik.scene.fk_model, ik.scene.mj.mjtObj.mjOBJ_JOINT,
                                                                 f'{side}_arm_j1')][:2].copy()
    radial = np.array([x, y]) - shoulder
    radial /= np.linalg.norm(radial)
    finger_goal = np.array([radial[0] * np.sin(tilt), radial[1] * np.sin(tilt), -np.cos(tilt)])
    target = np.array([x, y, workcell.table_top + workcell.cube_half])

    def residual(q):
        middle, rotation = ik._frame(side, q)
        # The pad frame's z axis points from the fingertips toward the wrist (up when hanging).
        return np.concatenate([middle - target, -rotation[:, 2] - finger_goal, [rotation[2, 0]]])
    q = seed.copy()
    weights = np.diag([1, 1, 1, .2, .2, .2, .2])
    r = residual(q)
    for _ in range(iterations):
        if np.linalg.norm(r[:3]) < 5e-4 and np.abs(r[3:]).max() < 2e-2:
            return True, q
        jacobian = np.zeros((7, 6))
        for column, joint in enumerate(joints):
            moved = q.copy()
            moved[joint] += 1e-5
            jacobian[:, column] = (residual(moved) - r) / 1e-5
        step = np.linalg.solve(jacobian.T @ weights @ jacobian + 1e-2 * np.eye(6), -jacobian.T @ weights @ r)
        q[joints] = np.clip(q[joints] + np.clip(step, -.2, .2), ik.lower[joints], ik.upper[joints])
        r = residual(q)
    return False, q


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--workcell', default=str(DEFAULT), help="a workcell file or 'synthetic'")
    parser.add_argument('--seeds', type=int, default=12, help='random starting poses per grasp (default 12)')
    args = parser.parse_args(argv)
    stack = derive_stack()
    limits = _Stack(stack)
    workcell = resolve(args.workcell)
    scene = MinervaTaskScene(stack['derived']['joint_map'], stack['names'], width=64, height=48, workcell=workcell)
    ik = GraspIK(scene, limits.lower, limits.upper)
    scene.pose.apply(scene.fk_data, limits.home)
    rng = np.random.default_rng(0)
    print(f'{workcell.name}: table top z {workcell.table_top:.3f} m, cube edge {workcell.cube_edge * 1000:.1f} mm')
    for side, points in POINTS.items():
        joints = ik.arm(side)
        for x, y in points:
            seeds = []
            for _ in range(args.seeds):
                seed = limits.home.copy()
                seed[joints] = rng.uniform(limits.lower[joints], limits.upper[joints])
                seeds.append(seed)
            top = [ik.solve(side, x, y, 0., seed=seed, iterations=400) for seed in seeds]
            best = min(top, key=lambda r: r['position_error_m'] + r['axis_error'])
            at_limit = [ik.names[j] for j in joints
                        if min(abs(best['q'][j] - ik.lower[j]), abs(best['q'][j] - ik.upper[j])) < 1e-3]
            line = [f"top {'Y' if any(r['ok'] for r in top) else '-'}"
                    + ('' if any(r['ok'] for r in top) else
                       f" (best {best['position_error_m'] * 1000:.0f} mm, at limit: {', '.join(at_limit) or 'none'})")]
            for tilt in TILTS:
                ok = any(tilted(ik, side, x, y, np.radians(tilt), seed)[0] for seed in seeds)
                line.append(f"{tilt}:{'Y' if ok else '-'}")
            print(f'{side:5s} ({x:+.2f}, {y:+.2f})  ' + '  '.join(line), flush=True)
    scene.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
