"""A scripted, closed-loop demonstration of the cube handoff: pick with the left gripper, hand over, drop in the bucket.

It plans from privileged simulation state (the cube's start pose and the workcell) and then holds each grasp pose
on the receiver's feedback. It exists to generate demonstrations and to show the task can be done through the
simulated receiver; it is never a learned policy. See docs/development/SIM_DEMONSTRATOR.md.

The grasp orientations are the ones the arms reach on the measured bench (docs/development/SIM_WORKCELL.md): the
fingers roughly level, never straight down. The left takes the cube with its fingers pointing forward and tilted
down, and lifts it. At the handoff, the left's fingers point forward-right and the right's forward-left, both jaws
closing horizontally on perpendicular pairs of faces. The right lifts the cube, carries it over the bucket with its
fingers forward and 20 degrees up, and opening its jaws drops it.

Every move is a straight line for the pads while the gripper turns evenly (solved every CART_STEP from the last
solution), so the paths are predictable near the table and the bucket; `plan_clearance` measures them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import zlib

import numpy as np

# Geometry of the plan (metres, degrees). Directions are (azimuth from +x, elevation): 270 points forward (-y).
READY = {'left': (.15, -.30), 'right': (-.10, -.20)}  # Pad midpoints at the start, READY_HEIGHT above the table,
READY_FINGERS = {'left': (270., 0.), 'right': (315., 0.)}  # clear of the bucket, fingers the way each arm reaches.
# Which side of the fingers the wrist housing hangs: 'narrow' (45 mm) wherever the gripper works near the table,
# 'wide' (63 mm) elsewhere. The narrow side down puts both wrist rolls near the ends of their range, so the left
# keeps it from its start through the lift and rolls over on its way to the handoff; the right never needs it.
READY_HOUSING = {'left': 'narrow', 'right': 'wide'}
READY_HEIGHT = .12
# Near the table the arms reach a thin set of poses, and IK from random joint seeds rarely lands in it. A keyframe
# solved from scratch starts from sampled poses whose pads are already near the target (SAMPLES random poses per arm,
# kept when their pads lie in the task's region).
SAMPLES, SEEDS_TRIED, SOLUTIONS_KEPT = 100_000, 16, 8
CART_STEP, TURN_STEP, JOINT_STEP = .01, .05, .02  # m and rad between path points.
# The wrist housing reaches 45 mm to one side of the fingers (63 mm to the other) from 82 mm behind the pads, so on
# the table the fingers tilt 20 degrees down, the 45 mm side toward the table (level_closing), and level out as they
# lift.
PICK_FINGER = (270., -20.)
LIFT_FINGER = (270., 0.)
# Where the default pick is out of reach, the grasp may turn up to 20 degrees off straight ahead (the jaws then close
# across the cube's faces at that angle and square it up) and tilt only 10 degrees down (the housing still clears).
PICK_OPTIONS = ((0., -20.), (0., -10.), (-10., -20.), (10., -20.), (-20., -15.), (20., -15.), (-10., -10.),
                (10., -10.))
APPROACH = .06  # Pre-grasp stand-off behind the cube along the fingers ...
APPROACH_RISE = .02  # ... and above it.
# The left first comes over the pre-grasp, fingers 10 degrees down (the steeper tilt only reaches near the table),
# and tilts the rest of the way as it descends.
ABOVE_PREGRASP, ABOVE_FINGER = .06, (270., -10.)
LIFT = .08
HANDOFF = (0., -.30, .12)  # Cube centre at the handoff: x, y on the table, z above its top.
HANDOFF_FINGERS = {'left': (225., 0.), 'right': (315., 0.)}
# The left rolls its wrist housing over on its way to the handoff, and the arm cannot keep the pads on a straight line
# while it does: the move interpolates the joints, which can swing the pads down into the table (14 mm, picking a cube
# turned 40 degrees at the far edge of its region). Where that move would not clear, the left first rises HANDOFF_CLIMB
# straight up and rolls over from there.
HANDOFF_CLIMB = .10
# The right takes the cube with its jaws closing vertically: its J5 a quarter turn (TAKE_ROLL) from the level take.
# Level jaws on perpendicular faces cross the two grippers' inner fingers behind the cube's corner (6.5 mm with the
# fingers' paddles as collision); turned, the grippers clear each other by 5 mm holding the cube and by 25 mm as the
# right comes in. J5's axis sits off the fingers' centre, so the cube meets the turned pads about 9 mm from HANDOFF.
TAKE_ROLL = math.pi / 2
CLEARANCE = .001  # m: a plan is used only if every move keeps the robot this far from the table, the bucket and itself.
REACH = .05  # m: pairs whose bounds are farther apart than this are left out of the clearance check.
# The left backs off along its fingers after letting go, level (rising, its fingertips met the right's upper jaw),
# then returns to where it started, out of the right's way as it lifts the cube.
RETREAT = .08
# The right lifts the cube 10 cm at the handoff, turning toward the bucket, before carrying it over: lower, its
# wrist housing sweeps the bucket's wall.
CARRY_RISE, CARRY_FINGER = .10, (290., 10.)
DROP_FINGER = (270., 20.)
DROP_CLEARANCE = .12  # Pad midpoint above the bucket's rim when the right lets go, over its centre.
DROP_OFFSET = (0., 0.)
# The bucket is long front to back and narrow across: the right comes to DROP_LEAD in front of the drop point at
# its height, then slides straight forward over the near wall, rather than crossing a side wall.
DROP_LEAD = .12
DROP_HOUSING = 'narrow'  # Its wide side would hang into the bucket's walls.
# The ways the right may bring the cube over the bucket and let it go, tried in order until a plan clears: the
# fingers' direction at the drop, the lead (x, y) from which the pads come to the drop point at its height, and the
# pads' height above the rim when they open. The first is the drop above.
DROPS = ({'name': 'from the front', 'finger': DROP_FINGER, 'lead': (0., DROP_LEAD), 'clearance': DROP_CLEARANCE},)
OPEN_MARGIN = .02  # Jaw clearance each side of the cube when open.
SQUEEZE_RAD = 1.  # Command this far past touching the cube.
SPEED = .5  # rad/s, the fastest arm joint in transit.
SLOW = .15  # rad/s on approaches and lifts.
SETTLE_TOLERANCE = .004  # rad: a pose is held once every joint of the moving arm is this close ...
SETTLE_TIMEOUT = 1.5  # ... or after this long.
CORRECTION_GAIN, CORRECTION_LIMIT = .1, .03  # Integral correction toward a held pose (fraction per cycle, rad).
# Position-only following lets each command lead its joint by a bounded lead, so a joint's top speed is about
# (kp x lead - friction) / (kd + kp x 10 ms): 0.23-0.35 rad/s for left J1 and J2 and right J2 and J3 on the direct
# profile, under the 0.75 rad/s peak of a transit move. Carrying the cube over the bucket, right J1 fell 0.5 rad
# behind its command, off the planned path. A move's clock runs at full rate while every arm joint trails its last
# planned command by less than PACE_LAG of the lead following allows it (TrialContext.lead), slows as one trails
# further, and stops at PACE_STOP of it: the arms keep to the planned path and arrive later. The soft wrists may
# lead 0.087 rad and J1 0.023, so one bound in radians would hold the wrists back. Without a lead, PACE_LEAD rad.
PACE_LAG, PACE_STOP, PACE_LEAD = .5, 1., .03
# A gripper step ends once that gripper's reading has moved less than GRIP_STILL over the last GRIP_SETTLE (stalled
# on the cube, or open), after at least GRIP_MIN and at most GRIP_WAIT. A fixed wait let the left open before the
# right had both jaws on the cube: the right needs more than 0.6 s to close from open at the receiver's gripper
# gains. Position, not the speed reading, which carries the drives' noise.
GRIP_MIN, GRIP_SETTLE, GRIP_WAIT, GRIP_STILL = .3, .2, 3., .01  # s, s, s, rad


def direction(azimuth, elevation):
    a, e = math.radians(azimuth), math.radians(elevation)
    return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


def level_closing(finger, housing='narrow'):
    """The horizontal axis perpendicular to the fingers: the jaws close along it, so opening drops what they hold.
    Taken as the signed jaw axis, it puts the wrist housing's narrow (45 mm) side below the fingers; with
    housing='wide', its wide (63 mm) side."""
    closing = np.cross(finger, [0., 0., 1.])
    return closing / np.linalg.norm(closing) * (1. if housing == 'narrow' else -1.)


def _pose(position, finger, housing='wide'):
    """A gripper pose for _move: pads at `position`, fingers along `finger`, jaws closing level with the wrist
    housing's `housing` side down (level_closing)."""
    return np.asarray(position, dtype=float), finger, level_closing(finger, housing)


def faces(yaw):
    """A cube turned `yaw` degrees offers the jaws two pairs of side faces a quarter turn apart: their yaws, the pair
    facing nearest straight ahead first. By that pair alone, a cube turned 40 degrees could not be picked at 10 of
    20 places in its region."""
    first = (yaw + 45.) % 90. - 45.
    return first, first - 90. if first > 0 else first + 90.


def _frame(closing, finger):
    """The rotation whose columns are the jaw axis, the third axis and the finger direction."""
    return np.column_stack([closing, np.cross(finger, closing), finger])


@dataclass
class Segment:
    """One step of the plan. kind 'move': follow `path` (joint vectors) to `target` over `duration`. 'hold': stay
    on `target`, correcting toward it, until the arm settles (or timeout). 'grip': hold while the grippers move."""
    kind: str
    target: np.ndarray
    duration: float = 0.
    arm: str | None = None
    label: str = ''
    path: list | None = field(default=None, repr=False)
    slow: bool = False
    cartesian: bool = True  # False: the straight-line path failed, so the move interpolates its joints.


class PlanningError(ValueError):
    """A keyframe the arms cannot reach from this cube start."""


class HandoffDemonstrator:
    name = 'handoff_demonstrator'
    execute_steps = 1  # Asked every control cycle: the commands close the loop on the receiver's feedback.

    def __init__(self, *, seed=0, restarts=8):
        self.seed, self.restarts = seed, restarts
        self._scene = self._ik = None
        self._key = self._samples_key = self._planned = None
        self.drop, self.notes = None, []

    # -- tools -----------------------------------------------------------------

    def _tools(self, context):
        """An IK model of the same robot and workcell as the trial (rebuilt whenever the workcell differs in any
        value, so each sampled layout plans against its own table, bucket and cube), and each arm's sampled poses
        (kept while the robot, its limits and the table's height stay the same)."""
        from scipy.spatial import cKDTree
        from robot_platform.simulation.task_scene import MinervaTaskScene
        from robot_platform.simulation.workcell import resolve
        from .reach import GraspIK
        workcell = resolve(context.workcell)
        key = (tuple(context.joint_names), json.dumps(workcell.describe(), sort_keys=True))
        if self._key != key:
            if self._scene is not None:
                self._scene.close()
            self._scene = MinervaTaskScene(context.joint_map, context.joint_names, width=64, height=48,
                                           workcell=workcell)
            self._ik = GraspIK(self._scene, context.lower, context.upper)
            self._key = key
        top = self._scene.workcell.table_top
        samples_key = (tuple(context.joint_names), top, self.seed, self._ik.lower.tobytes(), self._ik.upper.tobytes(),
                       np.asarray(context.home, dtype=float).tobytes())
        if self._samples_key != samples_key:
            rng = np.random.default_rng(self.seed)
            self._samples = {}
            for side in ('left', 'right'):
                joints = self._ik.arm(side)
                poses = np.tile(np.asarray(context.home, dtype=float), (SAMPLES, 1))
                poses[:, joints] = rng.uniform(self._ik.lower[joints], self._ik.upper[joints], size=(SAMPLES, 6))
                frames = np.array([np.concatenate(self._ik.frame(side, q)) for q in poses])
                keep = (frames[:, 2] > top - .05) & (frames[:, 2] < top + .45) & (frames[:, 1] < .1)
                self._samples[side] = (poses[keep], frames[keep], cKDTree(frames[keep, :3]))
            self._samples_key = samples_key
        return self._scene, self._ik

    def _solve(self, side, position, finger, closing, near, label):
        """The reachable joint solution for this gripper pose nearest `near` (so the arm moves as little as it can)."""
        rng = np.random.default_rng([self.seed, zlib.crc32(label.encode())])  # The same restarts every run.
        ik, joints = self._ik, self._ik.arm(side)
        poses, frames, tree = self._samples[side]
        finger_unit = np.asarray(finger, dtype=float) / np.linalg.norm(finger)
        _, nearest = tree.query(position, k=min(len(poses), 8 * SEEDS_TRIED))
        nearest = sorted(np.atleast_1d(nearest), key=lambda i: -(frames[i, 3:6] @ finger_unit))[:SEEDS_TRIED]
        seeds = [near.copy()]
        for i in nearest:
            seed = near.copy()
            seed[joints] = poses[i, joints]
            seeds.append(seed)
        for _ in range(self.restarts):
            seed = near.copy()
            seed[joints] = rng.uniform(ik.lower[joints], ik.upper[joints])
            seeds.append(seed)
        found = []
        for seed in seeds:
            result = ik.solve_pose(side, position, finger, closing, seed=seed, iterations=300,
                                   signed=closing is not None)
            if result['ok']:
                found.append(result['q'])
                if len(found) >= SOLUTIONS_KEPT:
                    break
        if not found:
            raise PlanningError(f'{label}: the {side} gripper cannot reach {np.round(position, 3).tolist()} '
                                f'with its fingers along {np.round(finger, 2).tolist()}')
        best = min(found, key=lambda q: np.abs(q[joints] - near[joints]).max())
        q = near.copy()
        q[joints] = best[joints]
        return q

    def _straight(self, side, q, position, finger, closing):
        """Joint vectors moving `side`'s pads in a straight line from where they are at `q` to `position` while the
        gripper turns evenly to `finger`/`closing`, each solved from the last; None where the arm cannot follow."""
        from scipy.spatial.transform import Rotation, Slerp
        ik = self._ik
        start, pointing, jaws = ik.frame(side, q)
        end = np.asarray(position, dtype=float)
        turn = Slerp([0., 1.], Rotation.from_matrix([_frame(jaws, pointing), _frame(closing, finger)]))
        angle = (turn([1.])[0] * turn([0.])[0].inv()).magnitude()
        count = max(1, math.ceil(max(np.linalg.norm(end - start) / CART_STEP, angle / TURN_STEP)))
        path = [q.copy()]
        for k in range(1, count + 1):
            s = k / count
            frame = turn([s])[0].as_matrix()
            result = ik.solve_pose(side, start + (end - start) * s, frame[:, 2], frame[:, 0], seed=path[-1],
                                   iterations=100, signed=True)
            if not result['ok']:
                return None
            path.append(result['q'])
        return path

    # -- planning --------------------------------------------------------------

    def _gripper(self, context, side, gap):
        """The motor coordinate that opens `side`'s jaws `gap` apart (each jaw slides gap / 2), from the joint map."""
        row = context.joint_map[f'{side}_gripper_finger_a_slide']
        slide = math.copysign(gap / 2, row['scale'])
        return (slide - row['offset']) / row['scale']

    def _move(self, q, poses, label, *, slow=False):
        """A move of one or both arms to their gripper poses ({side: (position, finger, closing)}), or to that arm's
        joints in a full joint vector."""
        paths, cartesian = {}, True
        for side, target in poses.items():
            path = None
            if isinstance(target, np.ndarray):
                end = q.copy()
                end[self.arms[side]] = target[self.arms[side]]
            else:
                path = self._straight(side, q, *target)
                if path is None:
                    # The arm cannot follow the line: solve the end pose afresh.
                    end = self._solve(side, *target, q, label)
            if path is None:
                # Interpolate the joints, in steps small enough that the clearance check sees the whole sweep.
                cartesian = False
                steps = max(1, math.ceil(float(np.abs(end - q).max()) / JOINT_STEP))
                path = [q + (end - q) * k / steps for k in range(steps + 1)]
            paths[side] = path
        count = max(len(path) for path in paths.values())
        merged = []
        for k in range(count):
            point = q.copy()
            for side, path in paths.items():
                f = k / (count - 1) * (len(path) - 1) if count > 1 else len(path) - 1
                i = min(int(f), len(path) - 2) if len(path) > 1 else 0
                w = f - i if len(path) > 1 else 0.
                joints = self.arms[side]
                point[joints] = path[i][joints] * (1 - w) + path[min(i + 1, len(path) - 1)][joints] * w
            merged.append(point)
        arm = next(iter(poses)) if len(poses) == 1 else None
        return Segment('move', merged[-1].copy(), arm=arm, label=label, path=merged, slow=slow, cartesian=cartesian)

    def reset(self, context):
        scene, ik = self._tools(context)
        workcell = context.workcell or scene.workcell
        names = list(context.joint_names)
        self.names, self.period = names, context.period_s
        lead = getattr(context, 'lead', None)
        self.lead = np.full(len(names), PACE_LEAD) if lead is None else np.asarray(lead, dtype=float)
        index = {name: i for i, name in enumerate(names)}
        self.arms = {side: [index[f'{side}_arm_j{k}'] for k in range(1, 7)] for side in ('left', 'right')}
        grip = {side: index[f'{side}_gripper'] for side in ('left', 'right')}
        edge, top = workcell.cube_edge, workcell.table_top
        opened = {side: self._gripper(context, side, edge + 2 * OPEN_MARGIN) for side in grip}
        touching = {side: self._gripper(context, side, edge) for side in grip}
        # Past touching, away from open: the jaws stop on the cube and the drive squeezes with kp * SQUEEZE_RAD.
        closed = {side: touching[side] + math.copysign(SQUEEZE_RAD, touching[side] - opened[side]) for side in grip}
        self._stops = [(grip[side], touching[side], opened[side]) for side in grip]
        yaw = math.degrees(context.cube_start.get('yaw', 0.))
        cube = np.array([context.cube_start['x'], context.cube_start['y'], top + workcell.cube_half])

        # The same trial planned again (the generator plans a drawn layout before it runs it): the same plan.
        key = (self._key, float(context.cube_start['x']), float(context.cube_start['y']),
               float(context.cube_start.get('yaw', 0.)), np.asarray(context.home, dtype=float).tobytes(),
               self.lead.tobytes(), ik.lower.tobytes(), ik.upper.tobytes(), float(context.period_s))
        if key == self._planned:
            self._timeline(self.frames)
            self.bias = np.zeros(len(names))
            return None
        self._planned = None
        q = np.array(context.home, dtype=float)
        for side in grip:
            q[grip[side]] = opened[side]
        for side in ('left', 'right'):
            x, y = READY[side]
            finger = direction(*READY_FINGERS[side])
            q = self._solve(side, [x, y, top + READY_HEIGHT], finger, level_closing(finger, READY_HOUSING[side]), q,
                            f'ready {side}')
        # The same start pose a turn apart on a wrist roll is a different branch for every move after it: plan
        # from each and, of the plans that clear, keep the one with the most straight-line moves, then the shortest.
        # Each branch plans the pick and the handoff once, then the drop each way DROPS lists, in order, until a
        # plan clears.
        # Why each branch and drop failed, for the PlanningError (a generator counts and records its rejections).
        self.notes = notes = []
        heads = []
        for ready in self._branches(q):
            try:
                heads.append((ready, *self._to_handoff(ready, cube, yaw, top, grip, opened, closed)))
            except PlanningError as exc:
                notes.append(f'pick and handoff: {exc}')
        planned = False
        for drop in DROPS:
            candidates = []
            for ready, head, after, point in heads:
                try:
                    frames = head + self._drop(after, point, top, workcell, grip, opened, drop)
                except PlanningError as exc:
                    notes.append(f"drop {drop['name']}: {exc}")
                    continue
                planned = True
                fallbacks = sum(1 for frame in frames if frame.kind == 'move' and not frame.cartesian)
                self._timeline(frames)
                candidates.append(((fallbacks, sum(frame.duration for frame in frames)), ready, frames))
            # The best-scored plan whose every move clears the table, the bucket and the other arm.
            for _, ready, frames in sorted(candidates, key=lambda candidate: candidate[0]):
                self._timeline(frames)
                label, distance, robot, other = min(self.plan_clearance(), key=lambda move: move[1])
                if distance > CLEARANCE:
                    self.ready, self.drop = ready, drop
                    self.bias = np.zeros(len(names))
                    self._planned = key
                    return None
                notes.append(f"drop {drop['name']}: {label}: {robot} comes {distance * 1000:.1f} mm from {other}")
        reasons = '; '.join(dict.fromkeys(notes))
        if not planned:
            raise PlanningError(f'no plan reaches the cube at {np.round(cube, 3).tolist()} ({reasons})')
        raise PlanningError(f'no plan clears the table, the bucket and the other arm for the cube at '
                            f'{np.round(cube, 3).tolist()} ({reasons})')

    def _branches(self, q):
        """`q` and its equals a turn apart on each joint whose range spans more than a turn, within the range."""
        ik = self._ik
        variants = [q.copy()]
        for j in self.arms['left'] + self.arms['right']:
            if ik.upper[j] - ik.lower[j] <= 2 * math.pi:
                continue
            more = []
            for variant in variants:
                for turn in (-2 * math.pi, 2 * math.pi):
                    if ik.lower[j] <= variant[j] + turn <= ik.upper[j]:
                        shifted = variant.copy()
                        shifted[j] += turn
                        more.append(shifted)
            variants += more
        return variants

    def _to_handoff(self, start, cube, yaw, top, grip, opened, closed):
        """The demonstration's segments from the start pose `start` until the left has let go of the cube at the
        handoff and returned to its start; with the joints there and the handoff's planned point."""
        q = start.copy()
        pose = _pose
        frames = []

        def add(segment):
            nonlocal q
            frames.append(segment)
            q = segment.target.copy()

        def hold(arm, label):
            frames.append(Segment('hold', q.copy(), arm=arm, label=label))

        def set_gripper(side, value, label, arm=None):
            q[grip[side]] = value
            frames.append(Segment('grip', q.copy(), arm=arm, label=label, duration=GRIP_WAIT))

        for turn, tilt in [(face + turn, tilt) for face in faces(yaw) for turn, tilt in PICK_OPTIONS]:
            azimuth = PICK_FINGER[0] + turn
            pick, above_cube = direction(azimuth, tilt), direction(azimuth, ABOVE_FINGER[1])
            pregrasp = cube - APPROACH * pick + [0., 0., APPROACH_RISE]
            try:
                approach = [self._move(q, {'left': pose(pregrasp + [0., 0., ABOVE_PREGRASP], above_cube, 'narrow')},
                                       'left above the cube')]
                approach.append(self._move(approach[-1].target, {'left': pose(pregrasp, pick, 'narrow')},
                                           'left pre-grasp', slow=True))
                approach.append(self._move(approach[-1].target, {'left': pose(cube, pick, 'narrow')}, 'left grasp',
                                           slow=True))
                break
            except PlanningError:
                continue
        else:
            raise PlanningError(f'no pick reaches the cube at {np.round(cube, 3).tolist()}')
        for segment in approach:
            add(segment)
        lift = direction(azimuth - PICK_FINGER[0] + LIFT_FINGER[0], LIFT_FINGER[1])
        hold('left', 'left on the cube')
        set_gripper('left', closed['left'], 'left closes', 'left')
        add(self._move(q, {'left': pose(cube + [0., 0., LIFT], lift, 'narrow')}, 'left lifts', slow=True))
        handoff = np.array([HANDOFF[0], HANDOFF[1], top + HANDOFF[2]])
        fingers = {side: direction(*HANDOFF_FINGERS[side]) for side in ('left', 'right')}
        # The right's level take at HANDOFF, then its J5 a quarter turn, the way its range allows that leaves the
        # grip point higher (J5's offset moves it 6 mm up one way and down the other); the cube meets the turned pads.
        level = self._solve('right', handoff, fingers['right'], level_closing(fingers['right'], 'wide'), q,
                            'right level take')
        j5 = self.names.index('right_arm_j5')
        turns = []
        for roll in (TAKE_ROLL, -TAKE_ROLL):
            if self._ik.lower[j5] <= level[j5] + roll <= self._ik.upper[j5]:
                turned = level.copy()
                turned[j5] += roll
                turns.append((self._ik.frame('right', turned)[0][2], roll, turned))
        if not turns:
            raise PlanningError('the right wrist cannot turn a quarter turn to take the cube')
        _, roll, turned = max(turns, key=lambda turn: turn[0])
        planned = handoff
        handoff, take_finger, take_closing = self._ik.frame('right', turned)
        meet = {'left': pose(handoff, fingers['left']),
                'right': pose(handoff - APPROACH * fingers['right'], fingers['right'])}
        move = self._move(q, meet, 'to the handoff')
        if self._clearance(move.path)[0] <= CLEARANCE:
            add(self._move(q, {'left': pose(cube + [0., 0., LIFT + HANDOFF_CLIMB], lift, 'narrow')}, 'left climbs'))
            move = self._move(q, meet, 'to the handoff')
        if self._clearance(move.path)[0] <= CLEARANCE:
            # Moving together, one arm sweeps through the other (from the far corner, the left's wrist through the
            # right's fingertips): the left brings the cube to the handoff first, then the right comes to wait.
            add(self._move(q, {'left': meet['left']}, 'left to the handoff'))
            move = self._move(q, {'right': meet['right']}, 'right to the handoff')
        add(move)
        hold('left', 'left holds the cube out')
        # The right turns its jaws vertical where it waits, clear of the cube, then comes in along its fingers. The
        # turn starts from where it is (a joint path from its start to the turned pose went 7 mm into the bucket).
        waiting = q.copy()
        if not self._ik.lower[j5] <= waiting[j5] + roll <= self._ik.upper[j5]:
            raise PlanningError('the right wrist cannot turn a quarter turn where it waits')
        waiting[j5] += roll
        waiting = self._solve('right', handoff - APPROACH * take_finger, take_finger, take_closing, waiting,
                              'right turns')
        add(self._move(q, {'right': waiting}, 'right turns'))
        add(self._move(q, {'right': (handoff, take_finger, take_closing)}, 'right takes', slow=True))
        hold('right', 'right on the cube')
        set_gripper('right', closed['right'], 'right closes', 'right')
        set_gripper('left', opened['left'], 'left lets go', 'left')
        add(self._move(q, {'left': pose(handoff - RETREAT * fingers['left'], fingers['left'])}, 'left retreats',
                       slow=True))
        add(self._move(q, {'left': start}, 'left clears'))
        return frames, q, planned

    def _drop(self, q, planned, top, workcell, grip, opened, drop):
        """The segments that carry the cube from the handoff over the bucket and let it go, one of DROPS's ways."""
        q = q.copy()
        pose = _pose
        frames = []

        def add(segment):
            nonlocal q
            frames.append(segment)
            q = segment.target.copy()

        carry = direction(*CARRY_FINGER)
        # The right rolls over to the drop's housing side here, clear of the bucket, not on the way to it.
        # In free space, at transit speed: this move includes the half-turn roll.
        # To the planned lift point: above the turned grip point, the carry's orientation is out of the right's reach.
        add(self._move(q, {'right': pose(planned + [0., 0., CARRY_RISE], carry, DROP_HOUSING)}, 'right lifts'))
        finger = direction(*drop['finger'])
        above = [workcell.bucket_center[0] + DROP_OFFSET[0], workcell.bucket_center[1] + DROP_OFFSET[1],
                 top + workcell.bucket_height + drop['clearance']]
        add(self._move(q, {'right': pose(np.asarray(above) + [*drop['lead'], 0.], finger, DROP_HOUSING)},
                       'right carries'))
        add(self._move(q, {'right': pose(above, finger, DROP_HOUSING)}, 'right over the bucket', slow=True))
        frames.append(Segment('hold', q.copy(), arm='right', label='right holds over the bucket'))
        q[grip['right']] = opened['right']
        frames.append(Segment('grip', q.copy(), arm='right', label='right lets go', duration=GRIP_WAIT))
        frames.append(Segment('grip', q.copy(), label='done', duration=5.))
        return frames

    def _timeline(self, frames):
        """Durations from joint travel along each path: SPEED in transit, SLOW on approaches and lifts."""
        arms = self.arms['left'] + self.arms['right']
        for frame in frames:
            if frame.kind == 'move':
                steps = [float(np.abs(b[arms] - a[arms]).max()) for a, b in zip(frame.path, frame.path[1:])]
                frame.arc = np.concatenate([[0.], np.cumsum(steps)])
                frame.duration = max(.3, frame.arc[-1] / (SLOW if frame.slow else SPEED))
            elif frame.kind == 'hold':
                frame.duration = SETTLE_TIMEOUT
        self.frames = frames
        self.current, self.started, self.grip_trace = 0, 0., []
        self.paced, self.ticked, self.planned = 0., 0., None  # The move's clock, the last call's time and command.
        self.log = []

    def start_pose(self, context):
        return self.ready

    def _stopped(self, q):
        """`q` with each gripper that is commanded past the cube posed where its jaws stop on it."""
        stopped = q
        for index, touching, opened in self._stops:
            if (q[index] - touching) * (touching - opened) > 0:
                stopped = stopped.copy() if stopped is q else stopped
                stopped[index] = touching
        return stopped

    def _clearance(self, path):
        """(closest distance in m along `path` from the robot to the table or bucket, or from one arm to the other;
        the two geoms), at each point, with the grippers where they stop on the cube. Pairs whose bounds are farther
        apart than REACH are left out (they clear by more)."""
        scene, mj = self._scene, self._scene.mj
        worst = (1., None, None)
        for q in path:
            scene._pose(self._stopped(q))
            mj.mj_kinematics(scene.model, scene.data)
            for a, b, _ in scene._overlapping(REACH):
                d = mj.mj_geomDistance(scene.model, scene.data, a, b, REACH, None)
                if d < worst[0]:
                    worst = (d, a, b)
        return worst

    def plan_clearance(self):
        """[(label, closest distance in m along the move from the robot to the table or bucket or from one arm to
        the other, the two geoms)] for every move of the current plan, at each path point."""
        mj = self._scene.mj
        name = lambda g: None if g is None else mj.mj_id2name(self._scene.model, mj.mjtObj.mjOBJ_GEOM, g)
        result = []
        for frame in self.frames:
            if frame.kind == 'move':
                distance, robot, environment = self._clearance(frame.path)
                result.append((frame.label, distance, name(robot), name(environment)))
        return result

    # -- execution -----------------------------------------------------------

    def __call__(self, observation):
        t, q = float(observation['t']), np.asarray(observation['q'], dtype=float)
        arms = self.arms['left'] + self.arms['right']
        frame = self.frames[self.current]
        if frame.kind == 'move':
            trail = 0. if self.planned is None else np.abs(self.planned[arms] - q[arms]) / self.lead[arms]
            behind = float(np.max(trail))
            self.paced += (t - self.ticked) * min(1., max(0., (PACE_STOP - behind) / (PACE_STOP - PACE_LAG)))
            elapsed = self.paced
        else:
            elapsed = t - self.started
        self.ticked = t
        done = elapsed >= frame.duration - 1e-9
        if frame.kind == 'grip' and frame.arm is not None and not done:
            self.grip_trace.append((t, float(q[self.names.index(f'{frame.arm}_gripper')])))
            recent = [value for at, value in self.grip_trace if at >= t - GRIP_SETTLE - 1e-9]
            done = (elapsed >= GRIP_MIN and self.grip_trace[0][0] <= t - GRIP_SETTLE + 1e-9
                    and max(recent) - min(recent) < GRIP_STILL)
        if frame.kind == 'hold' and not done:
            joints = self.arms[frame.arm]
            done = float(np.abs(frame.target[joints] - q[joints]).max()) < SETTLE_TOLERANCE
        if done and self.current < len(self.frames) - 1:
            self.log.append((frame.label, round(t, 3)))
            self.current += 1
            self.started, self.grip_trace, self.paced = t, [], 0.
            frame, elapsed = self.frames[self.current], 0.
        if frame.kind == 'move':
            s = min(1., elapsed / frame.duration)
            s = s * s * (3 - 2 * s)  # Smooth start and stop.
            arc = s * frame.arc[-1]
            k = min(int(np.searchsorted(frame.arc, arc, side='right')) - 1, len(frame.path) - 2)
            span = frame.arc[k + 1] - frame.arc[k]
            w = 0. if span <= 0 else (arc - frame.arc[k]) / span
            command = frame.path[k] * (1 - w) + frame.path[k + 1] * w
        else:
            command = frame.target.copy()
            if frame.kind == 'hold':
                # Close the loop on the receiver's feedback: lean the held arm toward its pose.
                joints = self.arms[frame.arm]
                self.bias[joints] = np.clip(self.bias[joints] + CORRECTION_GAIN * (frame.target[joints] - q[joints]),
                                            -CORRECTION_LIMIT, CORRECTION_LIMIT)
        self.planned = command.copy()
        command[arms] += self.bias[arms]
        return command[None, :]

    def describe(self):
        return {'kind': 'scripted closed-loop handoff demonstration (privileged cube start and workcell)',
                'handoff_m': list(HANDOFF), 'pick_finger_deg': list(PICK_FINGER),
                'handoff_fingers_deg': {k: list(v) for k, v in HANDOFF_FINGERS.items()},
                'drop_finger_deg': list(DROP_FINGER), 'speed_rad_s': SPEED, 'approach_rad_s': SLOW,
                'squeeze_rad': SQUEEZE_RAD, 'cartesian_step_m': CART_STEP,
                'drops': [drop['name'] for drop in DROPS],
                'planned_drop': None if self.drop is None else json.loads(json.dumps(self.drop))}

    def close(self):
        if self._scene is not None:
            self._scene.close()
            self._scene = None


def handoff_demonstrator():
    return HandoffDemonstrator()
