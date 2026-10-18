#!/usr/bin/env python3
"""Find the fiducial tags in real camera frames, and compare them with the task scene's head camera.

    python scripts/sim/fiducial_camera_check.py <frames dir> [--workcell FILE] [--intrinsics FILE]

<frames dir> holds frames as the cameras publish them: head.jpg, left_wrist.jpg and right_wrist.jpg (any missing one
is skipped). In each it detects the workcell's fiducial family (ArUco DICT_4X4_50) with sub-pixel corners. A wanted ID
not found is looked for again in the frame upscaled twice, then with loosened detector settings; a tag found that last
way is kept only if its cells read back as that ID with at most 4 cells wrong and at least 4 fewer than any other ID,
and is marked weak. Every found code is read back cell by cell (the decode check), and each frame gets an annotated
copy, <role>.annotated.png. For the head camera it then:

- projects every workcell tag through the scene's head camera, with its pixel intrinsics when the workcell has them,
  at the frame's size, and reports each found tag's pixel offset: the proof that sim and real land on each other,
  drawn in head.sim-vs-real.png;
- fits, from the flat tags alone, the focal length (or takes the intrinsics), the camera's pose over the table plane
  and where each tag lies on it, knowing only that each is the workcell's square on one plane, and compares that
  layout with the workcell's (the reference tag, --reference, at its place);
- solves PnP from the flat tags at their workcell places, in the frame they are given in (the measured rectangle, or
  the table frame), and reports its residual, the camera in that frame and the base frame, and how it differs from
  the scene's head camera. When it fits (within --fit-px) it also gives the solution's spread for assumed 1-sigma
  input errors (--sigma-*), solves each tag on a bucket wall from its own square through that camera against the
  workcell's place for it, and proposes, without writing it, a workcell head camera in that frame with its source.

The intrinsics are --intrinsics, else the workcell head camera's own when they are for the frame's size, else a focal
length fitted to the tags (an estimate, not a calibration). Base-frame values carry the placement of the tags' frame
relative to the robot. Writes detections.json and head_pose.json into the frames dir.

--intrinsics is JSON: {"head": {"width", "height", "fx", "fy", "cx", "cy", "dist": [k1, k2, p1, p2, k3], "label",
"source"}} for the frame as published (the head camera node turns its image half a turn before publishing); label
names the calibration in the proposal's source. Needs OpenCV 4.7 or newer and numpy, which the development
environment does not include.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from robot_platform.simulation.fiducials import CODES, cells, corners, grid, marker_axes  # noqa: E402
from robot_platform.simulation.workcell import head_pinhole, load_workcell  # noqa: E402

ROLES = ('head', 'left_wrist', 'right_wrist')
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
EDGE_MARGIN_PX = 2.
WEAK_MAX_ERRORS, WEAK_MARGIN = 4, 4  # Cells, for a tag only the loosened detector finds.
LOOSENED = dict(adaptiveThreshWinSizeMax=53, adaptiveThreshWinSizeStep=4, polygonalApproxAccuracyRate=.06,
                perspectiveRemovePixelPerCell=8, perspectiveRemoveIgnoredMarginPerCell=.2,
                maxErroneousBitsInBorderRate=.5, errorCorrectionRate=1., minMarkerPerimeterRate=.01)
FOCAL_STARTS = (450., 550., 650., 750., 900.)


def read_errors(gray, quad, family, tag_id, px=24):
    """Cells that differ between the code table and the image, sampled at each cell's centre after unwarping the
    found square (Otsu threshold), border included."""
    n = grid(family)
    size = n * px
    target = np.array([[0, 0], [size, 0], [size, size], [0, size]], np.float32)
    patch = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(quad.astype(np.float32), target), (size, size))
    _, binary = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = px // 4
    seen = [[int(binary[r * px + m:(r + 1) * px - m, c * px + m:(c + 1) * px - m].mean() > 127) for c in range(n)]
            for r in range(n)]
    expected = cells(family, tag_id)
    return sum(a != b for row_a, row_b in zip(seen, expected) for a, b in zip(row_a, row_b))


def detect(gray, family, wanted):
    """({id: 4 x 2 corners in OpenCV's order}, {id: how it was found}, other IDs, duplicated IDs) for a frame."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))
    tags, how, others, duplicates = {}, {}, set(), set()
    for loosened in (False, True):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        for key, value in (LOOSENED if loosened else {}).items():
            setattr(params, key, value)
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        for scale in (1, 2):
            if set(tags) == set(wanted):
                break
            image = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            found, ids, _ = detector.detectMarkers(image)
            seen = set()
            for quad, tag_id in zip(found, [] if ids is None else ids.ravel().tolist()):
                # Pixel centres on integers: a pixel u of the upscaled frame is (u + .5) / scale - .5 here.
                quad = (quad.reshape(4, 2).astype(float) + .5) / scale - .5
                if loosened:
                    # Loosened settings invent codes: keep a wanted ID only if its cells clearly read back as it.
                    if tag_id not in wanted or tag_id in tags:
                        continue
                    errors = {k: read_errors(gray, quad, family, k) for k in CODES[family]}
                    rest = min(v for k, v in errors.items() if k != tag_id)
                    if errors[tag_id] <= WEAK_MAX_ERRORS and rest - errors[tag_id] >= WEAK_MARGIN:
                        tags[tag_id], how[tag_id] = quad, f'loosened x{scale} (weak)'
                    continue
                if tag_id not in wanted:
                    others.add(tag_id)
                elif tag_id in seen:
                    duplicates.add(tag_id)
                elif tag_id not in tags:
                    tags[tag_id], how[tag_id] = quad, f'x{scale}'
                seen.add(tag_id)
    return tags, how, sorted(others), sorted(duplicates)


def side_px(quad):
    return float(np.mean(np.linalg.norm(quad - np.roll(quad, -1, axis=0), axis=1)))


def scene_camera(camera, width, height):
    """(K, R, t) of the workcell's head camera as the scene renders it at width x height: OpenCV's x = R @ base + t,
    pixel centres on integers. With pixel intrinsics, scaled to that size; otherwise from fovy (head_pinhole)."""
    x, y = np.array(camera['xyaxes'][:3], float), np.array(camera['xyaxes'][3:], float)
    x /= np.linalg.norm(x)
    y -= x * (x @ y)
    y /= np.linalg.norm(y)
    rotation = np.stack([x, -y, -np.cross(x, y)])
    fx, fy, cx, cy = head_pinhole(camera, width, height)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    return K, rotation, -rotation @ np.array(camera['pos'], float)


def project(points, K, R, t):
    """Pixels of points (n x 3), and whether each lies in front of the camera."""
    camera = np.asarray(points, float) @ R.T + t
    in_front = camera[:, 2] > 1e-6
    pixels = (camera @ K.T)[:, :2] / np.where(in_front, camera[:, 2], np.nan)[:, None]
    return pixels, in_front


def coverage(pixels, in_front, width, height):
    inside = in_front & (pixels[:, 0] >= -.5 + EDGE_MARGIN_PX) & (pixels[:, 0] <= width - .5 - EDGE_MARGIN_PX) & \
        (pixels[:, 1] >= -.5 + EDGE_MARGIN_PX) & (pixels[:, 1] <= height - .5 - EDGE_MARGIN_PX)
    return 'in view' if inside.all() else 'partly in view' if inside.any() else 'out of view'


def angles(axes):
    """(down, left, roll) in degrees of a camera whose OpenCV axes (rows: right, down, forward) are given in a frame
    with z up and y away from the robot: how far it looks below level, how far it turns toward the robot's left (+x
    of the table frame is the robot's right), and its roll."""
    right, _, forward = axes
    return (math.degrees(math.asin(-forward[2])), math.degrees(math.atan2(-forward[0], forward[1])),
            math.degrees(math.atan2(right[2], math.hypot(right[0], right[1]))))


class Plane:
    """The frame the flat tags are placed in: the workcell's measured rectangle when it has one, else the table frame;
    x along the near side, y away from the robot, z up, its origin on the table top."""

    def __init__(self, workcell):
        rectangle = workcell.fiducials.get('rectangle')
        if rectangle:
            self.name, origin, turn = 'rectangle', rectangle['origin'], math.pi + rectangle['yaw']
        else:
            (cx, cy), (sx, sy) = workcell.table_center, workcell.table_size
            self.name, origin, turn = 'table', (cx + sx / 2, cy + sy / 2), math.pi
        self.origin = np.array([origin[0], origin[1], workcell.table_top])
        c, s = math.cos(turn), math.sin(turn)
        self.axes = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])  # Rows: this frame's axes in the base frame.

    def point(self, base):
        return self.axes @ (np.asarray(base, float) - self.origin)

    def to_base(self, point):
        return self.origin + self.axes.T @ np.asarray(point, float)


def fit_focal(objects, images, width, height, principal=None):
    """One focal length (fx = fy) fitted to a single view of the planar tags at known places, with the principal point
    fixed (at the centre unless given) and no distortion. Returns K, distortion, the fit's RMS in pixels and the focal
    length's standard deviation from the pixel residuals alone."""
    cx, cy = principal if principal is not None else ((width - 1) / 2, (height - 1) / 2)
    guess = np.array([[600., 0, cx], [0, 600., cy], [0, 0, 1]])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    rms, K, _, _, _, deviations, _, _ = cv2.calibrateCameraExtended(
        [objects.astype(np.float32)], [images.astype(np.float32)], (width, height), guess, np.zeros(5), flags=flags)
    return K, np.zeros(5), float(rms), float(deviations.ravel()[1])  # With the aspect fixed, fy is the free one.


def solve(objects, pixels, K, dist):
    """The table frame's pose in the camera from the planar tag corners: IPPE, then Levenberg-Marquardt."""
    ok, rvec, tvec = cv2.solvePnP(objects, pixels, K, dist, flags=cv2.SOLVEPNP_IPPE)
    rvec, tvec = cv2.solvePnPRefineLM(objects, pixels, K, dist, rvec, tvec)
    return bool(ok), rvec, tvec


def free_layout(quads, square, size, reference, K=None, dist=None):
    """Fit from the tags alone: the focal length (unless K is given), the camera's pose over the tags' plane, and
    every tag's place and turn on it, knowing only that each tag is a `square` on that one plane. The plane's frame
    is the reference tag's marker frame (x right, y toward its code's top, z out of the tag). Levenberg-Marquardt on
    the corner pixels from several starting focal lengths. None with fewer than three tags or no reference."""
    if reference not in quads or len(quads) < 3:
        return None
    width, height = size
    others = sorted(i for i in quads if i != reference)
    h = square / 2
    local = np.array([[-h, h], [h, h], [h, -h], [-h, -h]])
    observed = np.concatenate([quads[reference]] + [quads[i] for i in others])
    if K is not None:
        # Fixed intrinsics: work on the ideal pinhole pixels of the given camera.
        observed = cv2.undistortPoints(observed.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
    observed = observed.ravel()
    cx, cy = (K[0, 2], K[1, 2]) if K is not None else ((width - 1) / 2, (height - 1) / 2)
    free_f = K is None
    start = 1 if free_f else 0

    def unpack(params):
        fx, fy = (params[0], params[0]) if free_f else (K[0, 0], K[1, 1])
        return fx, fy, params[start:start + 3], params[start + 3:start + 6], params[start + 6:]

    def plane_points(layout):
        points = [local]
        for k in range(len(others)):
            x, y, turn = layout[3 * k:3 * k + 3]
            cos, sin = math.cos(turn), math.sin(turn)
            points.append(local @ np.array([[cos, sin], [-sin, cos]]) + (x, y))
        points = np.concatenate(points)
        return np.column_stack([points, np.zeros(len(points))])

    def predict(params):
        fx, fy, rvec, tvec, layout = unpack(params)
        camera = plane_points(layout) @ cv2.Rodrigues(rvec)[0].T + tvec
        return np.column_stack([fx * camera[:, 0] / camera[:, 2] + cx, fy * camera[:, 1] / camera[:, 2] + cy]).ravel()

    def initial(f0):
        guess = K if K is not None else np.array([[f0, 0, cx], [0, f0, cy], [0, 0, 1]])
        poses = {}
        for i, quad in quads.items():
            pixels = observed.reshape(-1, 2)[4 * ([reference] + others).index(i):][:4]
            _, rvecs, tvecs, errors = cv2.solvePnPGeneric(np.column_stack([local, np.zeros(4)]), pixels, guess, None,
                                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            best = int(np.argmin(np.ravel(errors)))
            poses[i] = (cv2.Rodrigues(rvecs[best])[0], tvecs[best].ravel())
        R0, t0 = poses[reference]
        params = ([f0] if free_f else []) + [*cv2.Rodrigues(R0)[0].ravel(), *t0]
        for i in others:
            R, t = poses[i]
            relative = R0.T @ (t - t0)
            params += [relative[0], relative[1], math.atan2((R0.T @ R)[1, 0], (R0.T @ R)[0, 0])]
        return np.array(params, float)

    def refine(params):
        damping = 1e-3
        for _ in range(300):
            residual = predict(params) - observed
            J = np.empty((len(residual), len(params)))
            for j in range(len(params)):
                step = 1e-7 * max(1., abs(params[j]))
                moved = params.copy()
                moved[j] += step
                J[:, j] = (predict(moved) - observed - residual) / step
            A, g = J.T @ J, J.T @ residual
            while damping < 1e12:
                delta = np.linalg.solve(A + damping * np.diag(np.diag(A)), -g)
                if np.sum((predict(params + delta) - observed) ** 2) < np.sum(residual ** 2):
                    params, damping = params + delta, damping / 3
                    break
                damping *= 4
            else:
                break
            if np.abs(delta).max() < 1e-10:
                break
        return params, J, predict(params) - observed

    best = None
    for f0 in (FOCAL_STARTS if free_f else (None,)):
        params, J, residual = refine(initial(f0))
        if best is None or residual @ residual < best[2] @ best[2]:
            best = (params, J, residual)
    params, J, residual = best
    dof = len(observed) - len(params)
    covariance = (residual @ residual / max(dof, 1)) * np.linalg.pinv(J.T @ J)
    std = np.sqrt(np.clip(np.diag(covariance), 0, None))
    fx, fy, rvec, tvec, layout = unpack(params)
    R = cv2.Rodrigues(rvec)[0]
    centre = -R.T @ tvec
    samples = np.random.default_rng(0).multivariate_normal(params, covariance, 2000)
    spread = np.array([-cv2.Rodrigues(s[start:start + 3])[0].T @ s[start + 3:start + 6] for s in samples]).std(axis=0)
    places = {reference: (0., 0., 0.)}
    places.update({i: (layout[3 * k], layout[3 * k + 1], math.degrees(layout[3 * k + 2])) for k, i in enumerate(others)})
    return {'reference': reference, 'tags': sorted(quads), 'focal_fitted': free_f,
            'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'f_std_px': float(std[0]) if free_f else 0.,
            'fovy_deg': math.degrees(2 * math.atan(height / 2 / fy)),
            'rms_px': float(math.sqrt(residual @ residual / (len(observed) / 2))), 'dof': int(dof),
            'camera_m': centre.tolist(), 'camera_std_m': spread.tolist(), 'axes': R.tolist(),
            'layout': {str(i): [float(v) for v in places[i]] for i in sorted(places)},
            'layout_std_m': {str(i): [float(std[start + 6 + 3 * k]), float(std[start + 7 + 3 * k])]
                             for k, i in enumerate(others)}}


def wall_pose(quad, square, K, dist, R, t):
    """A tag's own pose from its square alone (IPPE, the solution with the smaller error), in the frame that (R, t)
    maps into the camera: (centre, marker axes as rows, error px, the other solution's error px)."""
    h = square / 2
    local = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    _, rvecs, tvecs, errors = cv2.solvePnPGeneric(local, quad, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    order = np.argsort(np.ravel(errors))
    best = int(order[0])
    marker = cv2.Rodrigues(rvecs[best])[0]
    return (R.T @ (tvecs[best].ravel() - t), (R.T @ marker).T, float(np.ravel(errors)[best]),
            float(np.ravel(errors)[order[1]]) if len(order) > 1 else None)


def nominal_spread(ids, centres, square, pixels, K, dist, size, estimated, nominal, sigma, trials, seed):
    """How far the nominal-place PnP moves when its inputs move by their assumed 1-sigma: each detected corner (by the
    solution's own reprojection RMS), each tag's centre per axis, the printed scale (one for every tag), and, for a
    fitted focal length, the principal point it fixes. Positions in the plane frame."""
    rng = np.random.default_rng(seed)
    width, height = size
    focal, places, turns = [], [], []
    for _ in range(trials):
        scale = 1 + rng.normal(0, sigma['scale'])
        objects = np.concatenate([np.array(corners(square * scale, centres[i][0] + rng.normal(0, sigma['tag_m']),
                                                   centres[i][1] + rng.normal(0, sigma['tag_m']), 0., 0.))
                                  for i in ids])
        seen = pixels + rng.normal(0, sigma['pixel_px'], pixels.shape)
        trial_K = K
        if estimated:
            principal = ((width - 1) / 2 + rng.normal(0, sigma['principal_px']),
                         (height - 1) / 2 + rng.normal(0, sigma['principal_px']))
            trial_K = fit_focal(objects, seen, width, height, principal)[0]
        _, rvec, tvec = solve(objects, seen, trial_K, dist)
        rotation = cv2.Rodrigues(rvec)[0]
        places.append((-rotation.T @ tvec).ravel())
        turn = rotation @ nominal.T
        turns.append(math.degrees(math.acos(max(-1., min(1., (np.trace(turn) - 1) / 2)))))
        focal.append(trial_K[1, 1])
    focal, places = np.array(focal), np.array(places)
    return {'trials': trials, 'seed': seed, 'assumed_1sigma': sigma,
            'fy_std_px': round(float(focal.std()), 1),
            'fovy_std_deg': round(float(np.std([math.degrees(2 * math.atan(height / 2 / f)) for f in focal])), 2),
            'position_std_m': places.std(axis=0).round(4).tolist(),
            'rotation_rms_deg': round(float(np.sqrt(np.mean(np.square(turns)))), 2)}


def camera_report(centre, axes, plane, workcell, R_sim, fovy):
    """A camera at `centre` with OpenCV axes `axes` (rows: right, down, forward), both in the plane frame: the plane
    and base frames, MuJoCo xyaxes in both, its look angles, and how it differs from the scene's head camera."""
    base_centre, base_axes = plane.to_base(centre), axes @ plane.axes
    down, left, roll = angles(axes)
    turn = base_axes @ R_sim.T
    position = base_centre - np.array(workcell.head_camera['pos'])
    return {plane.name: {'position_m': np.round(centre, 4).tolist(),
                         'mujoco_xyaxes': np.round([*axes[0], *(-axes[1])], 5).tolist()},
            'base': {'position_m': np.round(base_centre, 4).tolist(),
                     'mujoco_xyaxes': np.round([*base_axes[0], *(-base_axes[1])], 5).tolist()},
            'looks_down_deg': round(down, 2), 'turned_left_deg': round(left, 2), 'roll_deg': round(roll, 2),
            'versus_scene_camera': {
                'position_difference_m': position.round(4).tolist(),
                'distance_m': round(float(np.linalg.norm(position)), 4),
                'rotation_deg': round(math.degrees(math.acos(max(-1., min(1., (np.trace(turn) - 1) / 2)))), 2),
                'fovy_deg': [round(fovy, 3), workcell.head_camera['fovy']]}}


def draw_quad(image, quad, color, label=None, thickness=1):
    points = np.round(np.asarray(quad)).astype(int).reshape(-1, 1, 2)
    cv2.polylines(image, [points], True, color, thickness, cv2.LINE_AA)
    cv2.circle(image, tuple(points[0, 0]), 3, color, -1, cv2.LINE_AA)  # The code's top-left corner.
    if label:
        corner = np.asarray(quad).min(axis=0)
        cv2.putText(image, label, (int(corner[0]), max(10, int(corner[1]) - 4)), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1,
                    cv2.LINE_AA)


def drawable(quad, width, height):
    quad = np.asarray(quad, float)
    return np.all(np.isfinite(quad)) and np.abs(quad).max() < 4 * max(width, height)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('frames', type=Path)
    parser.add_argument('--workcell', type=Path, default=WORKCELL)
    parser.add_argument('--intrinsics', type=Path,
                        help="the head camera's intrinsics JSON (default: the workcell head camera's own, when it has "
                             'them for the frame size)')
    parser.add_argument('--reference', type=int, default=5,
                        help='the tag taken at its nominal place (default 5; the lowest found ID when it is missing)')
    parser.add_argument('--fit-px', type=float, default=2.,
                        help='reprojection RMS within which the tags fit their nominal places (default 2 px)')
    parser.add_argument('--sigma-tag', type=float, default=.005,
                        help='assumed 1-sigma error of each nominal tag centre per axis, m (default .005)')
    parser.add_argument('--sigma-scale', type=float, default=.005,
                        help="assumed 1-sigma error of the printed square's scale, a fraction (default .005)")
    parser.add_argument('--sigma-principal', type=float, default=10.,
                        help='assumed 1-sigma error of the centred principal point when the focal length is fitted, '
                             'px (default 10)')
    parser.add_argument('--trials', type=int, default=300, help='perturbed solutions for the spread (default 300)')
    parser.add_argument('--seed', type=int, default=20261017)
    args = parser.parse_args()
    workcell = load_workcell(args.workcell)
    f = workcell.fiducials
    family, square = f['family'], f['square']
    plane = Plane(workcell)
    tags = {tag[0]: tag for tag in f['tags']}
    flat = {i for i, tag in tags.items() if tag[5] == 0}
    nominal = {i: plane.point(tags[i][1:4])[:2] for i in flat}  # Plane frame.
    base_corners = {i: np.array(corners(square, *tag[1:])) for i, tag in tags.items()}
    plane_corners = {i: np.array(corners(square, *nominal[i], 0., 0.)) for i in flat}
    given = json.loads(args.intrinsics.read_text(encoding='utf-8'))['head'] if args.intrinsics else None

    detections, frames = {}, {}
    for role in ROLES:
        path = args.frames / f'{role}.jpg'
        if not path.exists():
            continue
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, how, others, duplicates = detect(gray, family, set(tags))
        frames[role] = (image, found, how)
        detections[role] = {
            'frame': path.name, 'size': [image.shape[1], image.shape[0]], 'ids': sorted(found),
            'missing_ids': sorted(set(tags) - set(found)), 'other_ids': others, 'duplicate_ids': duplicates,
            'tags': {str(i): {'corners_px': found[i].round(2).tolist(), 'side_px': round(side_px(found[i]), 1),
                              'px_per_cell': round(side_px(found[i]) / grid(family), 2), 'found_by': how[i],
                              'decode_cell_errors': read_errors(gray, found[i], family, i)} for i in sorted(found)}}

    head = None
    if 'head' in frames:
        image, found, how = frames['head']
        height, width = image.shape[:2]
        head = {'frame_size': [width, height], 'tags_found': sorted(found), 'plane_frame': plane.name,
                'caveat': 'Poses are in the frame the flat tags are given in; their base-frame values carry that '
                          "frame's placement relative to the robot, a placeholder until measured. A focal length "
                          'fitted here is an estimate from the tags, not a calibration.'}
        # The scene's head camera: the calibrated sim, when the workcell has the camera's intrinsics.
        K_sim, R_sim, t_sim = scene_camera(workcell.head_camera, width, height)
        sim = {}
        for i, quad in base_corners.items():
            pixels, in_front = project(quad, K_sim, R_sim, t_sim)
            row = {'corners_px': np.round(pixels, 2).tolist(), 'view': coverage(pixels, in_front, width, height)}
            if i in found:
                offset = found[i] - pixels
                row.update(offset_px_mean=np.round(offset.mean(axis=0), 2).tolist(),
                           offset_px_rms=round(float(np.sqrt((offset ** 2).sum(axis=1).mean())), 2),
                           offset_px_max=round(float(np.linalg.norm(offset, axis=1).max()), 2))
            sim[str(i)] = row
        head['scene_camera'] = {'camera': workcell.head_camera, 'K': K_sim.round(3).tolist(), 'tags': sim}

        K_given = dist_given = None
        label = None
        if given:
            if (given['width'], given['height']) != (width, height):
                raise SystemExit(f"head intrinsics are for {given['width']}x{given['height']}, the frame is "
                                 f'{width}x{height}')
            K_given = np.array([[given['fx'], 0, given['cx']], [0, given['fy'], given['cy']], [0, 0, 1]], float)
            dist_given, label = np.array(given.get('dist', [0] * 5), float), given.get('label', 'given intrinsics')
        elif workcell.head_camera.get('intrinsics') and \
                tuple(workcell.head_camera['intrinsics']['resolution']) == (width, height):
            K_given, dist_given, label = K_sim, np.zeros(5), "the workcell head camera's intrinsics"
        if K_given is not None:
            head['intrinsics'] = {'fx': K_given[0, 0], 'fy': K_given[1, 1], 'cx': K_given[0, 2], 'cy': K_given[1, 2],
                                  'dist': dist_given.tolist(), 'from': label}

        # The fit from the flat tags alone: focal length estimated, and with the given intrinsics when there are some.
        strong = {i: q for i, q in found.items() if 'weak' not in how[i] and i in flat}
        weak = {i: q for i, q in found.items() if 'weak' in how[i] and i in flat}
        reference = args.reference if args.reference in strong else min(strong, default=None)
        fits = {}
        for name, K_fit, dist_fit in (('focal_estimated', None, None), ('given_intrinsics', K_given, dist_given)):
            if name == 'given_intrinsics' and K_given is None:
                continue
            fit = free_layout(strong, square, (width, height), reference, K_fit, dist_fit)
            if fit and weak:
                # Weak tags join for their places only, not for the focal length.
                fixed = np.array([[fit['fx'], 0, fit['cx']], [0, fit['fy'], fit['cy']], [0, 0, 1]])
                fit_all = free_layout({**strong, **weak}, square, (width, height), reference, fixed,
                                      dist_fit if dist_fit is not None else np.zeros(5))
                for i in weak:
                    fit['layout'][str(i)] = fit_all['layout'][str(i)]
                    fit['layout_std_m'][str(i)] = fit_all['layout_std_m'][str(i)]
            if not fit:
                continue
            anchor = nominal[fit['reference']]
            axes = np.array(fit['axes'])
            centre = np.array([anchor[0] + fit['camera_m'][0], anchor[1] + fit['camera_m'][1], fit['camera_m'][2]])
            fit['anchored'] = dict(camera_report(centre, axes, plane, workcell, R_sim, fit['fovy_deg']),
                                   assumes=f"tag {fit['reference']} at its nominal place, square to the {plane.name}")
            fit['laid_versus_nominal_mm'] = {
                key: {'laid': np.round((anchor + (x, y)) * 1000, 1).tolist(),
                      'nominal': np.round(nominal[int(key)] * 1000, 1).tolist(),
                      'nominal_minus_laid': np.round((nominal[int(key)] - anchor - (x, y)) * 1000, 1).tolist(),
                      'turn_deg': round(turn, 1)} for key, (x, y, turn) in fit['layout'].items()}
            fits[name] = fit
        head['from_flat_tags_alone'] = fits

        # PnP from the flat tags at their nominal places, and the wall tags through that camera.
        used = sorted(i for i in found if i in flat)
        if len(used) >= 2:
            objects = np.concatenate([plane_corners[i] for i in used])
            pixels = np.concatenate([found[i] for i in used])
            if K_given is not None:
                K, dist, estimated = K_given, dist_given, False
            else:
                K, dist, fit_rms, focal_std = fit_focal(objects, pixels, width, height)
                estimated = True
                head['estimated_intrinsics'] = {
                    'fx': K[0, 0], 'fy': K[1, 1], 'cx': K[0, 2], 'cy': K[1, 2], 'fit_rms_px': fit_rms,
                    'fy_std_px_from_residuals': focal_std,
                    'source': 'estimated from the tags at their nominal places, not a calibration'}
            ok, rvec, tvec = solve(objects, pixels, K, dist)
            errors = np.linalg.norm(cv2.projectPoints(objects, rvec, tvec, K, dist)[0].reshape(-1, 2) - pixels, axis=1)
            R, t = cv2.Rodrigues(rvec)[0], tvec.ravel()
            fovy = math.degrees(2 * math.atan(height / 2 / K[1, 1]))
            rms = float(np.sqrt((errors ** 2).mean()))
            pose = dict(camera_report(-R.T @ t, R, plane, workcell, R_sim, fovy), tags=used, solved=ok,
                        fits=rms <= args.fit_px, reprojection_rms_px=round(rms, 3),
                        reprojection_max_px=round(float(errors.max()), 3),
                        per_tag_rms_px={str(i): round(float(np.sqrt((errors[4 * k:4 * k + 4] ** 2).mean())), 2)
                                        for k, i in enumerate(used)})
            if pose['fits']:
                sigma = {'pixel_px': round(rms, 3), 'tag_m': args.sigma_tag, 'scale': args.sigma_scale,
                         **({'principal_px': args.sigma_principal} if estimated else {})}
                pose['uncertainty'] = nominal_spread(used, nominal, square, pixels, K, dist, (width, height), estimated,
                                                     R, sigma, args.trials, args.seed)
                walls = {}
                for i in sorted(set(found) - flat):
                    centre, axes, error, other = wall_pose(found[i], square, K, dist, R, t)
                    expected = plane.point(tags[i][1:4])
                    expected_normal = plane.axes @ np.array(marker_axes(*tags[i][4:])[2])
                    walls[str(i)] = {
                        'centre_m': np.round(centre, 4).tolist(), 'normal': np.round(axes[2], 4).tolist(),
                        'up': np.round(axes[1], 4).tolist(), 'own_reprojection_px': round(error, 3),
                        'other_solution_px': None if other is None else round(other, 3),
                        'workcell_centre_m': np.round(expected, 4).tolist(),
                        'workcell_minus_camera_mm': np.round((expected - centre) * 1000, 1).tolist(),
                        'normal_angle_deg': round(math.degrees(math.acos(max(-1., min(1., float(
                            axes[2] @ expected_normal))))), 1)}
                pose['wall_tags'] = walls
            head['nominal_places_pnp'] = pose
            if pose['fits']:
                where = f'pose from the flat tags {used} at their places in the {plane.name}'
                source = (f'{label}; {where}' if not estimated else
                          f'{where}; focal length fitted to the tags, not the camera calibration')
                source += f": {pose['reprojection_rms_px']} px RMS"
                camera = {'frame': plane.name,
                          'pos': {'value': np.round(pose[plane.name]['position_m'], 4).tolist(), 'source': source},
                          'xyaxes': {'value': np.round(pose[plane.name]['mujoco_xyaxes'], 5).tolist(),
                                     'source': source},
                          'fovy': {'value': round(fovy, 4), 'source': source}}
                if not estimated:
                    camera['intrinsics'] = {
                        'resolution': [width, height], 'focalpixel': [K[0, 0], K[1, 1]],
                        'principalpixel': [(width - 1) / 2 - K[0, 2], (height - 1) / 2 - K[1, 2]], 'source': label}
                head['proposed_workcell_head_camera'] = camera

    # Annotated copies: what was found; and for the head, the scene's camera over it.
    for role, (image, found, how) in frames.items():
        canvas = image.copy()
        for i, quad in found.items():
            draw_quad(canvas, quad, (0, 200, 0), f'{i}{" weak" if "weak" in how[i] else ""}', 2)
        cv2.imwrite(str(args.frames / f'{role}.annotated.png'), canvas)
    if head:
        image, found, how = frames['head']
        height, width = image.shape[:2]
        canvas = image.copy()
        for i, quad in found.items():
            draw_quad(canvas, quad, (0, 200, 0), None, 2)
        for i, row in head['scene_camera']['tags'].items():
            if drawable(row['corners_px'], width, height):
                text = f"sim {i}" + (f" {row['offset_px_rms']:.1f}px" if 'offset_px_rms' in row else '')
                draw_quad(canvas, row['corners_px'], (255, 0, 255), text, 1)
        cv2.putText(canvas, 'green: tags found; magenta: the workcell head camera (sim) projecting the workcell tags',
                    (6, height - 8), cv2.FONT_HERSHEY_SIMPLEX, .38, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(args.frames / 'head.sim-vs-real.png'), canvas)

    (args.frames / 'detections.json').write_text(json.dumps(detections, indent=1), encoding='utf-8')
    if head:
        (args.frames / 'head_pose.json').write_text(json.dumps(head, indent=1, default=float), encoding='utf-8')
    for role, row in detections.items():
        print(f"{role}: IDs {row['ids']} (missing {row['missing_ids']}); decode cell errors "
              f"{ {i: t['decode_cell_errors'] for i, t in row['tags'].items()} }; px per cell "
              f"{ {i: t['px_per_cell'] for i, t in row['tags'].items()} }; found by "
              f"{ {i: t['found_by'] for i, t in row['tags'].items()} }"
              + (f"; other IDs {row['other_ids']}" if row['other_ids'] else '')
              + (f"; duplicated {row['duplicate_ids']}" if row['duplicate_ids'] else ''))
    if head:
        print('sim camera over the frame (px RMS per found tag): ' + ', '.join(
            f"{i} {row['offset_px_rms']}" for i, row in head['scene_camera']['tags'].items() if 'offset_px_rms' in row))
        for name, fit in head['from_flat_tags_alone'].items():
            a = fit['anchored']
            print(f"head from the flat tags alone ({name}): fx {fit['fx']:.1f}"
                  + (f" +- {fit['f_std_px']:.1f}" if fit['focal_fitted'] else '')
                  + f" px, fovy {fit['fovy_deg']:.2f}, RMS {fit['rms_px']:.2f} px; in the {plane.name} "
                  f"{a[plane.name]['position_m']}; looks down {a['looks_down_deg']}, left {a['turned_left_deg']}, "
                  f"roll {a['roll_deg']} deg")
        p = head.get('nominal_places_pnp')
        if p:
            print(f"head PnP at the nominal places ({p['tags']}): RMS {p['reprojection_rms_px']} px "
                  f"({'fits' if p['fits'] else 'does not fit'}); in the {plane.name} {p[plane.name]['position_m']}; "
                  f"vs scene camera {p['versus_scene_camera']}")
            for i, w in p.get('wall_tags', {}).items():
                print(f"  wall tag {i}: centre {np.round(np.array(w['centre_m']) * 1000, 1).tolist()} mm, normal "
                      f"{w['normal']}; workcell minus camera {w['workcell_minus_camera_mm']} mm, normals "
                      f"{w['normal_angle_deg']} deg apart")
        if 'proposed_workcell_head_camera' in head:
            print('proposed workcell head camera (not written): ' + json.dumps(head['proposed_workcell_head_camera']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
