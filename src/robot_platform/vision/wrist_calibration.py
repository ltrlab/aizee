"""The wrist cameras' calibration from still poses of the arms looking at the table's board.

A still is one camera's view of some of the board's tags while its arm is held still: the tags' corner pixels and the
arm's joint positions. The unknowns:
- each camera's intrinsics (fx, fy, cx, cy) and distortion (k1, k2, p1, p2, and k3 when asked), unless held at a
  camera's own published calibration (the head camera's, vision.head_alignment);
- each camera's mount X = T_link_camera, the camera in the link it is fixed to;
- the board's placement Y = T_base_board, shared by both arms;
- each tag's offset (dx, dy, dyaw) from its nominal place, with the board file's sigmas as priors;
- optionally, offsets of the arm joints' zeros (all but the last joint of each chain, whose offset the mount absorbs),
  with a prior of `joint_sigma_deg`. Unmodelled zero errors of 0.3 degrees already cost about 2 px and bias a mount by
  millimetres (rehearsed on synthetic stills), so a real arm should be fitted both ways and compared.
A board corner p is seen at project(camera, X^-1 F(q)^-1 Y p), where F(q) = T_base_link is the URDF chain.

Stages:
1. Per camera, a planar calibration (Zhang): the focal length from the stills' homographies, then the intrinsics and
   the board's pose in every still refined together, from starting focal lengths until one fits.
2. Per camera, X and Y in closed form from those poses (Shah 2013, robot-world-hand-eye), translation by least
   squares.
3. Everything refined together on reprojection error with a soft-L1 loss, both cameras sharing Y and the tag offsets.

Fits use MINPACK's Levenberg-Marquardt on soft-L1-transformed residuals with a grouped forward-difference Jacobian:
multithreaded BLAS made scipy's trust-region solver ten times slower on problems this small.

The report adds each camera's error per still and over the image, the parameters' uncertainty from the Jacobian, a
cross-validated error (each fold's stills predicted from a fit of the other stills, intrinsics, tags and joints held),
and each camera's own fit of the board's placement: their difference tells how well the two arms' kinematics agree.
"""
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import least_squares

from .geometry import (chordal_mean, homography, invert, invert_each, matrix_rpy, normalize, plane_pose, pose_matrix,
                       pose_vector, rotations, rotvec)

INTRINSICS = ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3')
MIN_STILLS = 5
FIELD_OF_VIEW_STARTS_DEG = (60., 80., 100., 120.)
# A planar fit this good has found the camera; further starting focal lengths only cost time.
ACCEPT_PLANAR_PX = 2.


@dataclass
class Still:
    camera: str
    joints: dict  # URDF joint name -> position (radians, metres for slides)
    corners: dict  # tag id -> (4, 2) pixels, OpenCV's corner order
    name: str = ''


@dataclass
class CameraSpec:
    name: str
    chain: object  # vision.chain.Chain from the base to the link the camera is fixed to
    size: tuple  # (width, height) pixels
    focal_px: float = None  # an optional starting focal length
    meta: dict = field(default_factory=dict)


class CalibrationError(ValueError):
    pass


# ----------------------------------------------------------------------------------------------------- shared helpers

def _project_each(cameras, points):
    """Pixels of camera-frame points (P, 3), each with its own nine camera numbers (P, 9)."""
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = cameras.T
    z = np.maximum(points[:, 2], 1e-3)
    x, y = points[:, 0] / z, points[:, 1] / z
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([fx * xd + cx, fy * yd + cy], axis=1)


def _tag_corners(board, tags, offsets):
    """(T, 4, 3) corners of `tags` with (T, 3) offsets (dx, dy, dyaw)."""
    nominal = np.array([board.tags[t] for t in tags], float)
    x, y = nominal[:, 0] + offsets[:, 0], nominal[:, 1] + offsets[:, 1]
    yaw = nominal[:, 2] + offsets[:, 2]
    h = board.square / 2
    local = np.array([(-h, h), (h, h), (h, -h), (-h, -h)])
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    corners = np.zeros((len(tags), 4, 3))
    corners[:, :, 0] = x[:, None] + c * local[:, 0] - s * local[:, 1]
    corners[:, :, 1] = y[:, None] + s * local[:, 0] + c * local[:, 1]
    return corners


class _Observations:
    """Every observed corner, flattened: its still, camera, tag and corner index, and its pixel."""

    def __init__(self, stills, cameras, tags):
        cameras, tag_index = list(cameras), {t: i for i, t in enumerate(tags)}
        still, camera, tag, corner, pixels = [], [], [], [], []
        for s, item in enumerate(stills):
            for t, quad in sorted(item.corners.items()):
                if t not in tag_index:
                    continue
                quad = np.asarray(quad, float).reshape(4, 2)
                for k in range(4):
                    still.append(s)
                    camera.append(cameras.index(item.camera))
                    tag.append(tag_index[t])
                    corner.append(k)
                    pixels.append(quad[k])
        self.still, self.camera = np.array(still, int), np.array(camera, int)
        self.tag, self.corner = np.array(tag, int), np.array(corner, int)
        self.pixels = np.array(pixels, float).reshape(-1, 2)
        self.count = len(self.still)


def _grouped_jacobian(fun, x, rows_of, groups):
    """Forward-difference Jacobian, perturbing each group of columns (with disjoint rows) at once."""
    f0 = fun(x)
    jac = np.zeros((len(f0), len(x)))
    step = np.sqrt(np.finfo(float).eps) * np.maximum(1., np.abs(x))
    for group in groups:
        dx = np.zeros_like(x)
        dx[group] = step[group]
        diff = fun(x + dx) - f0
        for column in group:
            rows = rows_of[column]
            jac[rows, column] = diff[rows] / step[column]
    return jac


def _groups(rows_of, residuals):
    """Columns packed greedily into groups whose row sets are disjoint."""
    groups, used = [], []
    for column, rows in enumerate(rows_of):
        for group, taken in zip(groups, used):
            if not taken[rows].any():
                group.append(column)
                taken[rows] = True
                break
        else:
            taken = np.zeros(residuals, bool)
            taken[rows] = True
            groups.append([column])
            used.append(taken)
    return groups


def _soft_l1(residuals, scale):
    """Residuals whose squares sum to the soft-L1 cost, scale^2 * 2 (sqrt(1 + (r/scale)^2) - 1): quadratic within
    `scale` of zero, linear beyond, so an outlying corner pulls with a bounded force."""
    u = residuals / scale
    return np.sign(u) * scale * np.sqrt(2 * (np.sqrt(1 + u * u) - 1))


@dataclass
class _Fit:
    x: np.ndarray
    fun: np.ndarray  # Plain residuals at x.
    cost: float  # Half the sum of squared soft-L1 residuals.
    nfev: int
    status: int
    message: str


def _solve(fun, x0, rows_of, *, f_scale=1., max_nfev=400):
    """Levenberg-Marquardt (MINPACK, with its own QR) on soft-L1 residuals and the grouped Jacobian."""
    robust = lambda x: _soft_l1(fun(x), f_scale)  # noqa: E731
    groups = _groups(rows_of, len(fun(x0)))
    result = least_squares(robust, x0, jac=lambda x: _grouped_jacobian(robust, x, rows_of, groups), method='lm',
                           x_scale='jac', max_nfev=max_nfev)
    return _Fit(result.x, fun(result.x), float(result.cost), int(result.nfev), int(result.status), str(result.message))


def _rms(values):
    values = np.asarray(values, float).reshape(-1, 2)
    return float(np.sqrt((values ** 2).sum(axis=1).mean())) if len(values) else None


# ----------------------------------------------------------------------------------------------------- stage 1: planar

def _plane_points(still, board):
    plane, pixels = [], []
    for tag, quad in sorted(still.corners.items()):
        if tag in board.tags:
            plane.append(board.corners(tag)[:, :2])
            pixels.append(np.asarray(quad, float).reshape(4, 2))
    if not plane:
        return np.zeros((0, 2)), np.zeros((0, 2))
    return np.concatenate(plane), np.concatenate(pixels)


def _focal_from_homographies(stills, board, size):
    """Focal lengths from each still's homography, the principal point at the image centre and square pixels:
    Zhang's two orthogonality constraints, each solved for f; the median of the plausible ones."""
    cx, cy = (size[0] - 1) / 2, (size[1] - 1) / 2
    estimates = []
    for still in stills:
        plane, pixels = _plane_points(still, board)
        if len(plane) < 8:  # One tag's homography says little about the focal length.
            continue
        h = homography(plane, pixels - (cx, cy))
        h = h / np.linalg.norm(h[:, 0])
        (a1, b1, c1), (a2, b2, c2) = h[:, 0], h[:, 1]
        # Each constraint reads (1/f^2) * denominator = numerator.
        for numerator, denominator in ((-c1 * c2, a1 * a2 + b1 * b2), (c2 * c2 - c1 * c1, a1 * a1 + b1 * b1
                                                                         - a2 * a2 - b2 * b2)):
            if abs(numerator) > 1e-12 and denominator / numerator > 0:
                f = math.sqrt(denominator / numerator)
                if .2 * size[0] < f < 5 * size[0]:
                    estimates.append(f)
    return float(np.median(estimates)) if estimates else None


def _initial_pose(still, board, camera):
    plane, pixels = _plane_points(still, board)
    return plane_pose(homography(plane, normalize(camera, pixels)), plane)


def _planar(board, stills, size, f0, fit_k3, loss_px, held=None):
    """Intrinsics and per-still board poses (with tag offsets) for one camera from starting focal length f0; with
    `held` (nine numbers), the poses alone, the intrinsics held at them."""
    tags = sorted(board.tags)
    obs = _Observations(stills, [stills[0].camera], tags)
    n_still, n_tag = len(stills), len(tags)
    n_intr = 0 if held is not None else 9 if fit_k3 else 8
    camera0 = (np.array(held, float) if held is not None else
               np.array([f0, f0, (size[0] - 1) / 2, (size[1] - 1) / 2, 0., 0., 0., 0., 0.]))
    poses = np.array([pose_vector(_initial_pose(s, board, camera0)) for s in stills])
    x0 = np.concatenate([camera0[:n_intr], poses.ravel(), np.zeros(3 * n_tag)])
    sigmas = np.array([board.sigma_m, board.sigma_m, board.sigma_yaw])

    def unpack(x):
        camera = camera0.copy()
        camera[:n_intr] = x[:n_intr]
        pose = x[n_intr:n_intr + 6 * n_still].reshape(n_still, 6)
        offsets = x[n_intr + 6 * n_still:].reshape(n_tag, 3)
        return camera, pose, offsets

    def residuals(x):
        camera, pose, offsets = unpack(x)
        corners = _tag_corners(board, tags, offsets)[obs.tag, obs.corner]
        r = rotations(pose[:, :3])[obs.still]
        points = np.einsum('pij,pj->pi', r, corners) + pose[obs.still, 3:]
        pixels = _project_each(np.broadcast_to(camera, (obs.count, 9)), points)
        return np.concatenate([(pixels - obs.pixels).ravel(), (offsets / sigmas).ravel()])

    pixel_rows = np.arange(2 * obs.count).reshape(-1, 2)
    rows_of = [pixel_rows.ravel()] * n_intr
    for s in range(n_still):
        rows_of.extend([pixel_rows[obs.still == s].ravel()] * 6)
    for t in range(n_tag):
        rows = pixel_rows[obs.tag == t].ravel()
        rows_of.extend(np.r_[rows, 2 * obs.count + 3 * t + k] for k in range(3))
    fit = _solve(residuals, x0, rows_of, f_scale=loss_px)
    camera, pose, offsets = unpack(fit.x)
    return dict(camera=camera, poses=[pose_matrix(p) for p in pose], offsets=offsets, cost=fit.cost,
                residuals=fit.fun[:2 * obs.count])


# --------------------------------------------------------------------------------------------- stage 2: closed form

def robot_world_hand_eye(links, views):
    """X = T_link_camera and Y = T_base_board from F_i X C_i = Y, with F_i = T_base_link and C_i = T_camera_board
    (Shah 2013). The rotations are the null vector of the stacked Kronecker system; the translations then follow by
    linear least squares. Returns (X, Y, rotation condition): the condition is the ratio of the two smallest singular
    values, near 0 for a well-posed set and near 1 when the stills barely turn the camera."""
    if len(links) != len(views) or len(links) < 3:
        raise CalibrationError('Hand-eye needs at least three stills')
    a = np.vstack([np.hstack([np.kron(c[:3, :3].T, f[:3, :3]), -np.eye(9)]) for f, c in zip(links, views)])
    _, singular, vt = np.linalg.svd(a)
    v = vt[-1]
    rx, ry = v[:9].reshape(3, 3, order='F'), v[9:].reshape(3, 3, order='F')
    det = np.linalg.det(rx)
    if abs(det) < 1e-12:
        raise CalibrationError('Hand-eye rotation is degenerate: vary the wrist tilt and roll between stills')
    scale = math.copysign(abs(det) ** (-1 / 3), det)
    rx, ry = chordal_mean([scale * rx]), chordal_mean([scale * ry])
    m = np.vstack([np.hstack([f[:3, :3], -np.eye(3)]) for f in links])
    b = np.concatenate([-f[:3, 3] - f[:3, :3] @ rx @ c[:3, 3] for f, c in zip(links, views)])
    t = np.linalg.lstsq(m, b, rcond=None)[0]
    x, y = np.eye(4), np.eye(4)
    x[:3, :3], x[:3, 3], y[:3, :3], y[:3, 3] = rx, t[:3], ry, t[3:]
    return x, y, float(singular[-1] / singular[-2])


# --------------------------------------------------------------------------------------------- stage 3: joint fit

@dataclass
class _State:
    intrinsics: dict  # camera -> nine numbers
    mounts: dict  # camera -> X
    placement: np.ndarray  # Y
    tags: np.ndarray  # (T, 3) offsets
    joints: dict  # camera -> offsets of its fitted joints


class _Model:
    """Stage 3's layout. Each camera's mount and the board placement are always fitted; `free` adds 'intrinsics',
    'tags' and 'joints' (the joints named per camera in `joints`). What is not fitted comes from the held state."""

    def __init__(self, board, specs, stills, *, fit_k3, free, joints, joint_sigma):
        self.board, self.stills = board, stills
        self.cameras = [name for name in specs if any(s.camera == name for s in stills)]
        self.tags = sorted(board.tags)
        self.obs = _Observations(stills, self.cameras, self.tags)
        self.n_intr = (9 if fit_k3 else 8) if 'intrinsics' in free else 0
        self.free_tags, self.free_joints = 'tags' in free, 'joints' in free
        self.joints = {name: tuple(joints.get(name, ())) for name in self.cameras}
        self.joint_sigma = joint_sigma
        self.board_sigmas = np.array([board.sigma_m, board.sigma_m, board.sigma_yaw])
        self.camera_of = np.array([self.cameras.index(s.camera) for s in stills], int)
        self.chains = {name: specs[name].chain for name in self.cameras}
        self.index = {name: np.flatnonzero(self.camera_of == c) for c, name in enumerate(self.cameras)}
        self.positions = {name: self.chains[name].positions([stills[i].joints for i in self.index[name]])
                          for name in self.cameras}
        self.slots = {name: [self.chains[name].joints.index(j) for j in self.joints[name]] for name in self.cameras}
        self.fixed_inverse = None

    def _links_inverse(self, state):
        result = np.zeros((len(self.stills), 4, 4))
        for name in self.cameras:
            offsets = np.zeros(len(self.chains[name].joints))
            offsets[self.slots[name]] = state.joints.get(name, np.zeros(len(self.slots[name])))
            result[self.index[name]] = invert_each(self.chains[name].poses(self.positions[name], offsets))
        return result

    def pack(self, state):
        parts = []
        for name in self.cameras:
            parts += [state.intrinsics[name][:self.n_intr], pose_vector(state.mounts[name])]
            if self.free_joints:
                parts.append(np.asarray(state.joints.get(name, np.zeros(len(self.joints[name]))), float))
        parts.append(pose_vector(state.placement))
        if self.free_tags:
            parts.append(np.asarray(state.tags, float).ravel())
        return np.concatenate(parts)

    def unpack(self, x, held):
        i, intrinsics, mounts, joints = 0, {}, {}, {}
        for name in self.cameras:
            camera = np.array(held.intrinsics[name], float)
            camera[:self.n_intr] = x[i:i + self.n_intr]
            intrinsics[name] = camera
            i += self.n_intr
            mounts[name] = pose_matrix(x[i:i + 6])
            i += 6
            if self.free_joints:
                joints[name] = x[i:i + len(self.joints[name])]
                i += len(self.joints[name])
            else:
                joints[name] = np.asarray(held.joints.get(name, np.zeros(len(self.joints[name]))), float)
        placement = pose_matrix(x[i:i + 6])
        i += 6
        tags = x[i:i + 3 * len(self.tags)].reshape(-1, 3) if self.free_tags else np.asarray(held.tags, float)
        return _State(intrinsics, mounts, placement, tags, joints)

    def predict(self, state):
        obs = self.obs
        if self.free_joints or self.fixed_inverse is None:
            links_inverse = self._links_inverse(state)
            if not self.free_joints:
                self.fixed_inverse = links_inverse
        else:
            links_inverse = self.fixed_inverse
        corners = _tag_corners(self.board, self.tags, state.tags)[obs.tag, obs.corner]
        mount_inverse = np.array([invert(state.mounts[name]) for name in self.cameras])
        views = mount_inverse[self.camera_of] @ links_inverse @ state.placement  # T_camera_board per still
        v = views[obs.still]
        points = np.einsum('pij,pj->pi', v[:, :3, :3], corners) + v[:, :3, 3]
        per_point = np.array([state.intrinsics[name] for name in self.cameras])[obs.camera]
        return _project_each(per_point, points)

    def residual_function(self, held):
        def residuals(x):
            state = self.unpack(x, held)
            parts = [(self.predict(state) - self.obs.pixels).ravel()]
            if self.free_tags:
                parts.append((state.tags / self.board_sigmas).ravel())
            if self.free_joints:
                parts += [state.joints[name] / self.joint_sigma for name in self.cameras]
            return np.concatenate(parts)
        return residuals

    def rows_of(self):
        obs = self.obs
        pixel_rows = np.arange(2 * obs.count).reshape(-1, 2)
        prior = 2 * obs.count + (3 * len(self.tags) if self.free_tags else 0)
        joint_prior = {}
        for name in self.cameras:
            joint_prior[name] = prior + np.arange(len(self.joints[name]))
            prior += len(self.joints[name])
        rows_of = []
        for c, name in enumerate(self.cameras):
            rows = pixel_rows[obs.camera == c].ravel()
            rows_of.extend([rows] * (self.n_intr + 6))
            if self.free_joints:
                rows_of.extend(np.r_[rows, joint_prior[name][k]] for k in range(len(self.joints[name])))
        rows_of.extend([pixel_rows.ravel()] * 6)
        if self.free_tags:
            for t in range(len(self.tags)):
                rows = pixel_rows[obs.tag == t].ravel()
                rows_of.extend(np.r_[rows, 2 * obs.count + 3 * t + k] for k in range(3))
        return rows_of

    def fit(self, held, loss_px):
        fit = _solve(self.residual_function(held), self.pack(held), self.rows_of(), f_scale=loss_px)
        return fit, self.unpack(fit.x, held)


# ----------------------------------------------------------------------------------------------------- the calibration

def _check(board, cameras, stills):
    names = [c.name for c in cameras]
    if len(set(names)) != len(names) or not names:
        raise CalibrationError('Name each camera once')
    usable = {name: [] for name in names}
    for still in stills:
        if still.camera not in usable:
            raise CalibrationError(f'Still {still.name or "?"} is from an unknown camera {still.camera!r}')
        if any(t in board.tags for t in still.corners):
            usable[still.camera].append(still)
    for name, items in usable.items():
        if len(items) < MIN_STILLS:
            raise CalibrationError(f'{name}: {len(items)} stills see the board; capture at least {MIN_STILLS}, '
                                   'better 15-25, from varied distances and angles')
    return usable


def _focal_starts(spec, stills, board):
    starts = [spec.focal_px] if spec.focal_px else []
    estimate = _focal_from_homographies(stills, board, spec.size)
    if estimate:
        starts.append(estimate)
    starts += [spec.size[0] / 2 / math.tan(math.radians(fov) / 2) for fov in FIELD_OF_VIEW_STARTS_DEG]
    return starts


def _fitted_joints(cameras, joint_offsets):
    """{camera: joint names} whose zero offsets are fitted: True means all but each chain's last moving joint."""
    if not joint_offsets:
        return {}
    if joint_offsets is True:
        return {c.name: c.chain.joints[:-1] for c in cameras}
    result = {}
    for c in cameras:
        names = tuple(joint_offsets.get(c.name, ()))
        unknown = [j for j in names if j not in c.chain.joints]
        if unknown:
            raise CalibrationError(f'{c.name}: no joints {unknown} on its chain')
        if c.chain.joints and c.chain.joints[-1] in names:
            raise CalibrationError(f'{c.name}: the last joint\'s offset is the mount\'s; leave it out')
        result[c.name] = names
    return result


def _held_intrinsics(intrinsics, names):
    """{camera: nine finite numbers} for every camera, or None; held for some cameras but not others is refused."""
    if not intrinsics:
        return None
    if set(intrinsics) != set(names):
        raise CalibrationError("Hold every camera's intrinsics or none")
    held = {}
    for name in names:
        camera = np.asarray(intrinsics[name], float)
        if camera.shape != (9,) or not np.isfinite(camera).all() or min(camera[:2]) <= 0:
            raise CalibrationError(f'{name}: held intrinsics are nine finite numbers (fx, fy, cx, cy, k1, k2, p1, p2, '
                                   'k3), the focal lengths positive')
        held[name] = camera
    return held


def calibrate(board, cameras, stills, *, fit_k3=False, joint_offsets=None, joint_sigma_deg=1., loss_px=1., folds=5,
              intrinsics=None):
    """Calibrate the cameras (CameraSpec) from stills (Still). `joint_offsets` is None, True (each chain's joints but
    the last) or {camera: joint names}. `intrinsics` ({camera: nine numbers} for every camera, or None) holds each
    camera's intrinsics as given, a camera's own published calibration, instead of fitting them. Returns a JSON-ready
    report (see the module docstring)."""
    usable = _check(board, cameras, stills)
    specs = {c.name: c for c in cameras}
    names = [c.name for c in cameras]
    stills = [s for name in names for s in usable[name]]
    joints = _fitted_joints(cameras, joint_offsets)
    joint_sigma = math.radians(joint_sigma_deg)
    held_intrinsics = _held_intrinsics(intrinsics, names)
    intrinsics, mounts, placements, planar = {}, {}, [], {}
    for name in names:
        items = usable[name]
        starts = [] if held_intrinsics else _focal_starts(specs[name], items, board)
        best = _planar(board, items, specs[name].size, None, fit_k3, loss_px,
                       held=held_intrinsics[name]) if held_intrinsics else None
        for f0 in starts:  # The likeliest first; stop at the first that fits.
            fit = _planar(board, items, specs[name].size, f0, fit_k3, loss_px)
            if best is None or fit['cost'] < best['cost']:
                best = fit
            if _rms(fit['residuals']) < ACCEPT_PLANAR_PX:
                break
        intrinsics[name] = best['camera']
        links = [specs[name].chain.pose(s.joints) for s in items]
        mount, placement, condition = robot_world_hand_eye(links, best['poses'])
        mounts[name] = mount
        placements.append(placement)
        planar[name] = dict(rms_px=round(_rms(best['residuals']), 3), focal_starts=[round(f, 1) for f in starts],
                            rotation_condition=round(condition, 4))
    start = np.eye(4)
    start[:3, :3] = chordal_mean([p[:3, :3] for p in placements])
    start[:3, 3] = np.mean([p[:3, 3] for p in placements], axis=0)
    held = _State(intrinsics, mounts, start, np.zeros((len(board.tags), 3)),
                  {name: np.zeros(len(joints.get(name, ()))) for name in names})
    free = {'tags'} | (set() if held_intrinsics else {'intrinsics'}) | ({'joints'} if joints else set())
    model = _Model(board, specs, stills, fit_k3=fit_k3, free=free, joints=joints, joint_sigma=joint_sigma)
    fit, state = model.fit(held, loss_px)
    report = _report(model, specs, fit, state, planar)
    report['options'] = dict(fit_k3=fit_k3, joint_offsets={k: list(v) for k, v in joints.items()},
                             joint_sigma_deg=joint_sigma_deg, loss_px=loss_px,
                             intrinsics='held' if held_intrinsics else 'fitted')
    report['cross_validation'] = _cross_validate(board, specs, stills, state, fit_k3, folds, loss_px, joints,
                                                 joint_sigma)
    report['per_camera_placement'] = _per_camera_placement(board, specs, stills, state, fit_k3, loss_px, joints,
                                                           joint_sigma)
    return report


# ------------------------------------------------------------------------------------------------------- the report

def _pose_report(t, sigma=None):
    roll, pitch, yaw = matrix_rpy(t[:3, :3])
    result = dict(matrix=np.round(t, 7).tolist(), xyz_m=np.round(t[:3, 3], 5).tolist(),
                  rpy_deg=[round(math.degrees(a), 3) for a in (roll, pitch, yaw)],
                  rotvec=np.round(rotvec(t[:3, :3]), 6).tolist())
    if sigma is not None:
        result['sigma_xyz_mm'] = [round(float(v) * 1e3, 2) for v in sigma[3:]]
        result['sigma_rotation_deg'] = round(math.degrees(float(np.linalg.norm(sigma[:3]))), 3)
    return result


def _sigmas(model, fit, held, rms_px):
    """One-sigma parameter uncertainty: pixel rows weighted by the fit's own per-axis error, priors as given."""
    residuals = model.residual_function(held)
    rows_of = model.rows_of()
    jac = _grouped_jacobian(residuals, fit.x, rows_of, _groups(rows_of, len(fit.fun)))
    scale = np.ones(len(fit.fun))
    scale[:2 * model.obs.count] = 1 / max(rms_px / math.sqrt(2), 1e-6)
    weighted = jac * scale[:, None]
    return np.sqrt(np.maximum(np.diag(np.linalg.pinv(weighted.T @ weighted)), 0.))


def _report(model, specs, fit, state, planar):
    obs, board = model.obs, model.board
    pixel = fit.fun[:2 * obs.count].reshape(-1, 2)
    error = np.sqrt((pixel ** 2).sum(axis=1))
    rms_all = float(np.sqrt((error ** 2).mean()))
    sigma = _sigmas(model, fit, state, rms_all)
    report = dict(cameras={}, board={}, fit=dict(cost=fit.cost, evaluations=fit.nfev, status=fit.status,
                                                  message=fit.message, corners=int(obs.count),
                                                  stills=len(model.stills), rms_px=round(rms_all, 3)))
    i = 0
    for c, name in enumerate(model.cameras):
        n, k = model.n_intr, len(model.joints[name]) if model.free_joints else 0
        sigma_intr, sigma_mount, sigma_joints = sigma[i:i + n], sigma[i + n:i + n + 6], sigma[i + n + 6:i + n + 6 + k]
        i += n + 6 + k
        mine = obs.camera == c
        stills = []
        for s, still in enumerate(model.stills):
            if still.camera != name:
                continue
            e = error[obs.still == s]
            stills.append(dict(name=still.name, tags=sorted(int(t) for t in still.corners if t in board.tags),
                               rms_px=round(float(np.sqrt((e ** 2).mean())), 3), max_px=round(float(e.max()), 3)))
        typical = float(np.median([row['rms_px'] for row in stills]))
        for row in stills:
            row['outlier'] = row['rms_px'] > max(3 * typical, 2.)
        size, camera = specs[name].size, state.intrinsics[name]
        report['cameras'][name] = dict(
            size=list(size), link=specs[name].chain.tip,
            intrinsics={key: round(float(v), 5) for key, v in zip(INTRINSICS, camera)},
            sigma_intrinsics={key: round(float(v), 5) for key, v in zip(INTRINSICS, sigma_intr)},
            camera_matrix=[[float(camera[0]), 0., float(camera[2])], [0., float(camera[1]), float(camera[3])],
                           [0., 0., 1.]],
            distortion=[float(v) for v in camera[4:]],
            field_of_view_deg=_field_of_view(camera, size),
            mount=_pose_report(state.mounts[name], sigma_mount),
            joint_offsets={j: dict(deg=round(math.degrees(float(v)), 3),
                                   sigma_deg=round(math.degrees(float(sigma_joints[q])), 3) if k else None)
                           for q, (j, v) in enumerate(zip(model.joints[name], state.joints.get(name, ())))},
            rms_px=round(float(np.sqrt((error[mine] ** 2).mean())), 3),
            p95_px=round(float(np.percentile(error[mine], 95)), 3), corners=int(mine.sum()), stills=stills,
            coverage=_coverage(obs.pixels[mine], size),
            by_radius=_by_radius(obs.pixels[mine], error[mine], camera, size), planar=planar[name])
    report['board'] = dict(placement=_pose_report(state.placement, sigma[i:i + 6]), tag_offsets={
        str(t): dict(dx_mm=round(float(o[0]) * 1e3, 2), dy_mm=round(float(o[1]) * 1e3, 2),
                     dyaw_deg=round(math.degrees(float(o[2])), 3)) for t, o in zip(model.tags, state.tags)},
        seen={str(t): int((obs.tag == k).sum() // 4) for k, t in enumerate(model.tags)})
    return report


def _field_of_view(camera, size):
    return dict(horizontal=round(math.degrees(2 * math.atan(size[0] / 2 / camera[0])), 2),
                vertical=round(math.degrees(2 * math.atan(size[1] / 2 / camera[1])), 2))


def _coverage(pixels, size, columns=4, rows=3):
    """Corners seen in each cell of a columns x rows grid over the image (row-major from the top-left)."""
    grid = np.zeros((rows, columns), int)
    for u, v in pixels:
        c, r = int(u / size[0] * columns), int(v / size[1] * rows)
        if 0 <= c < columns and 0 <= r < rows:
            grid[r, c] += 1
    return dict(grid=grid.tolist(), cells_seen=int((grid > 0).sum()), cells=columns * rows)


def _by_radius(pixels, error, camera, size, bins=4):
    """RMS error by distance from the principal point, in fractions of the half-diagonal: a distortion model that
    fits the centre but not the edges shows here."""
    radius = np.hypot(pixels[:, 0] - camera[2], pixels[:, 1] - camera[3]) / math.hypot(size[0] / 2, size[1] / 2)
    rows = []
    for k in range(bins):
        sel = (radius >= k / bins) & (radius < (k + 1) / bins) if k < bins - 1 else radius >= k / bins
        if sel.any():
            rows.append(dict(radius=[round(k / bins, 2), round((k + 1) / bins, 2)], corners=int(sel.sum()),
                             rms_px=round(float(np.sqrt((error[sel] ** 2).mean())), 3)))
    return rows


def _held_model(board, specs, stills, fit_k3, joints, joint_sigma):
    """A model fitting only mounts and placement; intrinsics, tag and joint offsets held."""
    return _Model(board, specs, stills, fit_k3=fit_k3, free=set(), joints=joints, joint_sigma=joint_sigma)


def _cross_validate(board, specs, stills, state, fit_k3, folds, loss_px, joints, joint_sigma):
    """Each fold's stills predicted from mounts and placement fitted to the other stills (intrinsics, tag and joint
    offsets held at the full fit's): the error for poses the fit has not seen."""
    names = [name for name in specs if any(s.camera == name for s in stills)]
    order = {name: [i for i, s in enumerate(stills) if s.camera == name] for name in names}
    if min(len(v) for v in order.values()) < folds + MIN_STILLS - 1:
        return dict(folds=0, note='Too few stills for cross-validation')
    errors = {name: [] for name in names}
    for k in range(folds):
        held_out = sorted(i for name in names for j, i in enumerate(order[name]) if j % folds == k)
        train = [stills[i] for i in range(len(stills)) if i not in held_out]
        _, fold = _held_model(board, specs, train, fit_k3, joints, joint_sigma).fit(state, loss_px)
        test = _held_model(board, specs, [stills[i] for i in held_out], fit_k3, joints, joint_sigma)
        e = np.sqrt(((test.predict(fold) - test.obs.pixels) ** 2).sum(axis=1))
        for c, name in enumerate(test.cameras):
            errors[name].extend(e[test.obs.camera == c].tolist())
    return dict(folds=folds, cameras={name: dict(rms_px=round(float(np.sqrt(np.mean(np.square(v)))), 3),
                                                 p95_px=round(float(np.percentile(v, 95)), 3))
                                      for name, v in errors.items() if v})


def _per_camera_placement(board, specs, stills, state, fit_k3, loss_px, joints, joint_sigma):
    """Each camera's own fit of its mount and the board placement (intrinsics, tag and joint offsets held): the
    difference between the cameras' placements is how far the two arms' kinematics disagree about the table."""
    names = [name for name in specs if any(s.camera == name for s in stills)]
    if len(names) < 2:
        return {}
    result, own = {}, {}
    for name in names:
        mine = [s for s in stills if s.camera == name]
        _, fitted = _held_model(board, specs, mine, fit_k3, joints, joint_sigma).fit(state, loss_px)
        own[name] = fitted.placement
        result[name] = _pose_report(fitted.placement)
    a, b = own[names[0]], own[names[1]]
    result['difference'] = dict(between=names[:2], translation_mm=round(float(np.linalg.norm(
        a[:3, 3] - b[:3, 3])) * 1e3, 2), rotation_deg=round(math.degrees(float(np.linalg.norm(
            rotvec((invert(a) @ b)[:3, :3])))), 3))
    return result
