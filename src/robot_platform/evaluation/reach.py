"""Can a gripper take the cube at a table pose? Kinematic IK with the frozen fixture's grasp geometry.

The goal frame comes from the task scene's own contact geometry: the two terminal jaw pads of one
gripper centred on the cube, their finger axis vertical, and their closing axis along a cube
face. Joints move only within the receiver's limits (motor coordinates). This is kinematic reach
for a top grasp; contact, dynamics and approach paths are not checked here (the scene audits
penetration of the posed robot separately).
"""
from __future__ import annotations

import numpy as np

from robot_platform.simulation.task_scene import JAW_BODIES

POSITION_TOLERANCE_M = 5e-4
AXIS_TOLERANCE = 1e-2  # Sine of the remaining axis misalignment, about 0.6 degrees.


class GraspIK:
    def __init__(self, scene, lower, upper):
        """`scene`: a MinervaTaskScene (its FK model, joint map and pad geometry); `lower`/`upper`: receiver
        limits in its joint order."""
        mj = scene.mj
        self.scene, self.names = scene, tuple(scene.names)
        self.lower, self.upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        self.pads = {}
        for side, bodies in JAW_BODIES.items():
            rows = []
            for geom in range(scene.model.ngeom):
                name = mj.mj_id2name(scene.model, mj.mjtObj.mjOBJ_GEOM, geom) or ''
                body = mj.mj_id2name(scene.model, mj.mjtObj.mjOBJ_BODY, int(scene.model.geom_bodyid[geom]))
                if name.endswith('/terminal_pad') and body in bodies:
                    rotation = np.zeros(9)
                    mj.mju_quat2Mat(rotation, scene.model.geom_quat[geom])
                    # Each robot body keeps its FK frame as a mocap body, so the pad's local frame carries over.
                    rows.append((mj.mj_name2id(scene.fk_model, mj.mjtObj.mjOBJ_BODY, body),
                                 scene.model.geom_pos[geom].copy(), rotation.reshape(3, 3)))
            if len(rows) != 2:
                raise ValueError(f'{side} gripper needs two terminal pads, found {len(rows)}')
            self.pads[side] = rows

    def arm(self, side):
        return [self.names.index(f'{side}_arm_j{k}') for k in range(1, 7)]

    def _frame(self, side, q):
        self.scene.pose.apply(self.scene.fk_data, q)
        data = self.scene.fk_data
        centres, rotations = [], []
        for body, position, rotation in self.pads[side]:
            frame = data.xmat[body].reshape(3, 3)
            centres.append(data.xpos[body] + frame @ position)
            rotations.append(frame @ rotation)
        return (centres[0] + centres[1]) / 2, rotations[0]

    def _residual(self, side, q, target, yaw):
        middle, rotation = self._frame(side, q)
        finger, closing = rotation[:, 2], rotation[:, 0]
        faces = np.array([[np.cos(yaw), np.sin(yaw), 0.], [-np.sin(yaw), np.cos(yaw), 0.]])
        along = 0 if abs(closing @ faces[0]) >= abs(closing @ faces[1]) else 1
        # Closing along one face normal: no component along the other.
        return np.concatenate([middle - target, finger[:2], [closing @ faces[1 - along]]])

    def solve(self, side, x, y, yaw=0., *, seed, iterations=150):
        """{'ok', 'q' (all joints, motor coordinates), 'position_error_m', 'axis_error'} for a top grasp of a cube
        resting at (x, y). `seed`: a full joint vector to start from (it also fixes the other joints)."""
        q = np.array(seed, dtype=float)
        joints = self.arm(side)
        workcell = self.scene.workcell
        target = np.array([x, y, workcell.table_top + workcell.cube_half])
        weights = np.diag([1., 1., 1., .1, .1, .1])
        residual = self._residual(side, q, target, yaw)
        for _ in range(iterations):
            if np.linalg.norm(residual[:3]) < POSITION_TOLERANCE_M and np.abs(residual[3:]).max() < AXIS_TOLERANCE:
                break
            jacobian = np.zeros((6, 6))
            for column, joint in enumerate(joints):
                moved = q.copy()
                moved[joint] += 1e-5
                jacobian[:, column] = (self._residual(side, moved, target, yaw) - residual) / 1e-5
            step = np.linalg.solve(jacobian.T @ weights @ jacobian + 1e-2 * np.eye(6), -jacobian.T @ weights @ residual)
            q[joints] = np.clip(q[joints] + np.clip(step, -.2, .2), self.lower[joints], self.upper[joints])
            residual = self._residual(side, q, target, yaw)
        position, axis = float(np.linalg.norm(residual[:3])), float(np.abs(residual[3:]).max())
        return {'ok': position < POSITION_TOLERANCE_M and axis < AXIS_TOLERANCE, 'q': q,
                'position_error_m': position, 'axis_error': axis}
