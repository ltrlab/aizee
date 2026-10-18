#!/usr/bin/env python3
"""Find the table's fiducial tags in real camera frames, and compare them with the task scene's head camera.

    python scripts/sim/fiducial_camera_check.py <frames dir> [--workcell FILE] [--intrinsics FILE]

<frames dir> holds frames as the cameras publish them: head.jpg, left_wrist.jpg and right_wrist.jpg (any missing one
is skipped). In each it detects the workcell's fiducial family (ArUco DICT_4X4_50) with sub-pixel corners. A wanted ID
not found is looked for again in the frame upscaled twice, then with loosened detector settings; a tag found that last
way is kept only if its cells read back as that ID with at most 4 cells wrong and at least 4 fewer than any other ID,
and is marked weak. Every found code is read back cell by cell (the decode check), and each frame gets an annotated
copy, <role>.annotated.png. For the head camera it then:

- projects the workcell's tags through the scene's head camera (its pos, xyaxes and fovy, at the frame's size) and
  reports each found tag's pixel offset from that projection (red in the annotated frame);
- fits, from the found tags alone, one focal length (principal point at the centre, no distortion; fixed instead when
  --intrinsics gives the camera's), the camera's pose over the table plane and where each tag lies on it, knowing only
  that every tag is the workcell's square on one plane. It does not use the tags' nominal places, so it holds when
  they are wrong. Taking the reference tag (--reference, default 5) at its nominal place, it gives the camera in the
  table and base frames, how far each found tag is from its nominal place, and draws the nominal places through that
  camera (cyan): where to move a tag to;
- solves PnP from the found tags at their nominal places, with the same intrinsics, and reports how well that fits.
  When it fits (RMS within --fit-px), it also reports where each tag falls in the frame and the solution's spread when
  the corners, tag centres, printed scale and principal point move by assumed 1-sigma errors (--sigma-*);
- proposes, without writing it, a workcell head camera (pos, xyaxes, fovy) with its source: from the nominal-place
  PnP when it fits, otherwise from the fit above anchored at the reference tag.

The result is only as good as the tags' places, the printed square size and the table's placement in the workcell
file, until they are measured. A focal length fitted here is an estimate from the tags, not a calibration. Writes
detections.json and head_pose.json into the frames dir.

--intrinsics is JSON: {"head": {"width", "height", "fx", "fy", "cx", "cy", "dist": [k1, k2, p1, p2, k3], "source"}}
for the frame as published (the head camera node turns its image half a turn before publishing). Needs OpenCV 4.7 or
newer and numpy, which the development environment does not include.
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
from robot_platform.simulation.fiducials import CODES, cells, corners, grid  # noqa: E402
from robot_platform.simulation.workcell import load_workcell  # noqa: E402

ROLES = ('head', 'left_wrist', 'right_wrist')
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
HALF_TURN = np.diag([-1., -1., 1.])  # The table frame's axes in the base frame.
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


def mujoco_camera(camera, width, height):
    """(K, R, t) of a MuJoCo camera at width x height: OpenCV's x = R @ base + t, pixel centres on integers."""
    x, y = np.array(camera['xyaxes'][:3], float), np.array(camera['xyaxes'][3:], float)
    x /= np.linalg.norm(x)
    y -= x * (x @ y)
    y /= np.linalg.norm(y)
    f = height / 2 / math.tan(math.radians(camera['fovy']) / 2)
    rotation = np.stack([x, -y, -np.cross(x, y)])
    return (np.array([[f, 0, (width - 1) / 2], [0, f, (height - 1) / 2], [0, 0, 1]]), rotation,
            -rotation @ np.array(camera['pos'], float))


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


def pose_in_frames(centre, axes, c, d, top):
    """A camera at `centre` with OpenCV axes `axes` (rows), both in the table frame: table frame, base frame, MuJoCo
    xyaxes, and its look angles."""
    base_axes = axes @ HALF_TURN.T
    down, left, roll = angles(axes)
    return {'table': {'position_m': np.round(centre, 4).tolist()},
            'base': {'position_m': np.round([c - centre[0], -d - centre[1], top + centre[2]], 4).tolist(),
                     'mujoco_xyaxes': np.round([*base_axes[0], *(-base_axes[1])], 5).tolist()},
            'looks_down_deg': round(down, 2), 'turned_left_deg': round(left, 2), 'roll_deg': round(roll, 2),
            '_rotation_base': base_axes}


def versus(pose, rotation_base, fovy, workcell, R_sim):
    """How a solved camera differs from the scene's head camera."""
    turn = rotation_base @ R_sim.T
    position = np.array(pose['base']['position_m']) - np.array(workcell.head_camera['pos'])
    return {'position_difference_m': position.round(4).tolist(), 'distance_m': round(float(np.linalg.norm(position)), 4),
            'rotation_deg': round(math.degrees(math.acos(max(-1., min(1., (np.trace(turn) - 1) / 2)))), 2),
            'optical_axis_deg': round(math.degrees(math.acos(max(-1., min(1., float(rotation_base[2] @ R_sim[2]))))), 2),
            'fovy_deg': [round(fovy, 2), workcell.head_camera['fovy']]}


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


def nominal_spread(ids, centres, square, pixels, K, dist, size, estimated, c, d, top, nominal, sigma, trials, seed):
    """How far the nominal-place PnP moves when its inputs move by their assumed 1-sigma: each detected corner (by the
    solution's own reprojection RMS), each tag's centre per axis, the printed scale (one for every tag), and, for a
    fitted focal length, the principal point it fixes."""
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
        centre = (-rotation.T @ tvec).ravel()
        places.append((c - centre[0], -d - centre[1], top + centre[2]))
        turn = (rotation @ HALF_TURN.T) @ nominal.T
        turns.append(math.degrees(math.acos(max(-1., min(1., (np.trace(turn) - 1) / 2)))))
        focal.append(trial_K[1, 1])
    focal, places = np.array(focal), np.array(places)
    return {'trials': trials, 'seed': seed, 'assumed_1sigma': sigma,
            'fy_std_px': round(float(focal.std()), 1),
            'fovy_std_deg': round(float(np.std([math.degrees(2 * math.atan(height / 2 / f)) for f in focal])), 2),
            'position_std_m': places.std(axis=0).round(4).tolist(),
            'rotation_rms_deg': round(float(np.sqrt(np.mean(np.square(turns)))), 2)}


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
    parser.add_argument('--intrinsics', type=Path)
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
    top = workcell.table_top
    c = workcell.table_center[0] + workcell.table_size[0] / 2
    d = -(workcell.table_center[1] + workcell.table_size[1] / 2)
    tags = {tag[0]: tag for tag in f['tags']}
    nominal = {i: (c - x, -d - y) for i, (_, x, y, _) in tags.items()}  # Table frame.
    base_corners = {i: np.array(corners(square, x, y, top, yaw)) for i, (_, x, y, yaw) in tags.items()}
    # Every marker frame is parallel to the table frame: corners at the table-frame centre, yaw 0, on the top (z 0).
    table_corners = {i: np.array(corners(square, *nominal[i], 0., 0.)) for i in tags}
    intrinsics = json.loads(args.intrinsics.read_text(encoding='utf-8')) if args.intrinsics else {}

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

    head, plan = None, {}
    if 'head' in frames:
        image, found, how = frames['head']
        height, width = image.shape[:2]
        head = {'frame_size': [width, height], 'tags_found': sorted(found),
                'caveat': 'Tag places are the nominal layout and the square the printed design size; the table '
                          'placement (c, d) is a workcell placeholder. Until they are measured the poses here are only '
                          'as good as they are. A fitted focal length is an estimate from the tags, not a calibration.'}
        K_sim, R_sim, t_sim = mujoco_camera(workcell.head_camera, width, height)
        sim = {}
        for i, quad in base_corners.items():
            pixels, in_front = project(quad, K_sim, R_sim, t_sim)
            row = {'corners_px': np.round(pixels, 1).tolist(), 'view': coverage(pixels, in_front, width, height)}
            if i in found:
                offset = found[i] - pixels
                row.update(offset_px_mean=np.round(offset.mean(axis=0), 1).tolist(),
                           offset_px_rms=round(float(np.sqrt((offset ** 2).sum(axis=1).mean())), 1))
            sim[str(i)] = row
        head['scene_camera'] = {'camera': workcell.head_camera, 'K': K_sim.round(2).tolist(), 'tags': sim}

        given = intrinsics.get('head')
        K_given = dist_given = None
        if given:
            if (given['width'], given['height']) != (width, height):
                raise SystemExit(f"head intrinsics are for {given['width']}x{given['height']}, the frame is "
                                 f'{width}x{height}')
            K_given = np.array([[given['fx'], 0, given['cx']], [0, given['fy'], given['cy']], [0, 0, 1]], float)
            dist_given = np.array(given.get('dist', [0] * 5), float)
            head['intrinsics_given'] = given

        # The fit from the tags alone: focal length estimated, and with the given intrinsics when there are some.
        strong = {i: q for i, q in found.items() if 'weak' not in how[i]}
        reference = args.reference if args.reference in strong else min(strong, default=None)
        fits = {}
        for name, K_fit, dist_fit in (('focal_estimated', None, None), ('given_intrinsics', K_given, dist_given)):
            if name == 'given_intrinsics' and K_given is None:
                continue
            fit = free_layout(strong, square, (width, height), reference, K_fit, dist_fit)
            if fit and len(strong) < len(found):
                # Weak tags join for their places only, not for the focal length.
                fixed = np.array([[fit['fx'], 0, fit['cx']], [0, fit['fy'], fit['cy']], [0, 0, 1]])
                fit_all = free_layout(found, square, (width, height), reference, fixed,
                                      dist_fit if dist_fit is not None else np.zeros(5))
                for i in set(found) - set(strong):
                    fit['layout'][str(i)] = fit_all['layout'][str(i)]
                    fit['layout_std_m'][str(i)] = fit_all['layout_std_m'][str(i)]
            if fit:
                fits[name] = fit
        for name, fit in fits.items():
            reference = nominal[fit['reference']]
            axes = np.array(fit['axes'])
            centre = np.array([reference[0] + fit['camera_m'][0], reference[1] + fit['camera_m'][1], fit['camera_m'][2]])
            pose = pose_in_frames(centre, axes, c, d, top)
            rotation_base = pose.pop('_rotation_base')
            fit['anchored'] = dict(pose, versus_scene_camera=versus(pose, rotation_base, fit['fovy_deg'], workcell, R_sim),
                                   assumes=f"tag {fit['reference']} at its nominal place, square to the table")
            fit['moves_mm'] = {}
            for key, (x, y, turn) in fit['layout'].items():
                laid = (reference[0] + x, reference[1] + y)
                fit['moves_mm'][key] = {'laid_table_mm': [round(laid[0] * 1000, 1), round(laid[1] * 1000, 1)],
                                        'nominal_table_mm': [round(nominal[int(key)][0] * 1000, 1),
                                                             round(nominal[int(key)][1] * 1000, 1)],
                                        'move_by_mm': [round((nominal[int(key)][0] - laid[0]) * 1000, 1),
                                                       round((nominal[int(key)][1] - laid[1]) * 1000, 1)],
                                        'turn_deg': round(turn, 1)}
            fit['_camera'] = (centre, axes)
        chosen = fits.get('given_intrinsics') or fits.get('focal_estimated')
        if chosen:
            # The nominal places through the fitted camera: where each tag should be, and what the camera would see.
            centre, axes = chosen.pop('_camera')
            K_fit = np.array([[chosen['fx'], 0, chosen['cx']], [0, chosen['fy'], chosen['cy']], [0, 0, 1]])
            for i in sorted(tags):
                pixels, in_front = project(table_corners[i], K_fit, axes, -axes @ centre)
                plan[str(i)] = {'corners_px': np.round(pixels, 1).tolist(), 'view': coverage(pixels, in_front, width, height),
                                'found': i in found}
            chosen['nominal_places_through_this_camera'] = plan
        for fit in fits.values():
            fit.pop('_camera', None)
        head['from_tags_alone'] = fits

        # PnP from the tags at their nominal places.
        if len(found) >= 2:
            objects = np.concatenate([table_corners[i] for i in sorted(found)])
            pixels = np.concatenate([found[i] for i in sorted(found)])
            if K_given is not None:
                K, dist, estimated = K_given, dist_given, False
                head['nominal_intrinsics'] = dict(given, estimated=False)
            else:
                K, dist, fit_rms, focal_std = fit_focal(objects, pixels, width, height)
                estimated = True
                head['nominal_intrinsics'] = {
                    'fx': K[0, 0], 'fy': K[1, 1], 'cx': K[0, 2], 'cy': K[1, 2], 'estimated': True, 'fit_rms_px': fit_rms,
                    'fy_std_px_from_residuals': focal_std,
                    'source': 'estimated from the table tags at their nominal positions, not a calibration'}
            ok, rvec, tvec = solve(objects, pixels, K, dist)
            errors = np.linalg.norm(cv2.projectPoints(objects, rvec, tvec, K, dist)[0].reshape(-1, 2) - pixels, axis=1)
            rotation = cv2.Rodrigues(rvec)[0]
            pose = pose_in_frames((-rotation.T @ tvec).ravel(), rotation, c, d, top)
            rotation_base = pose.pop('_rotation_base')
            fovy = math.degrees(2 * math.atan(height / 2 / K[1, 1]))
            rms = float(np.sqrt((errors ** 2).mean()))
            pose.update(solved=ok, fits=rms <= args.fit_px, reprojection_rms_px=round(rms, 3),
                        reprojection_max_px=round(float(errors.max()), 3), fovy_deg_from_fy=round(fovy, 2),
                        per_tag_rms_px={str(i): round(float(np.sqrt((errors[4 * k:4 * k + 4] ** 2).mean())), 2)
                                        for k, i in enumerate(sorted(found))},
                        versus_scene_camera=versus(pose, rotation_base, fovy, workcell, R_sim))
            if pose['fits']:
                sigma = {'pixel_px': round(rms, 3), 'tag_m': args.sigma_tag, 'scale': args.sigma_scale,
                         **({'principal_px': args.sigma_principal} if estimated else {})}
                pose['uncertainty'] = nominal_spread(sorted(found), nominal, square, pixels, K, dist, (width, height),
                                                     estimated, c, d, top, rotation_base, sigma, args.trials, args.seed)
                views = {}
                for i in sorted(tags):
                    tag_pixels = cv2.projectPoints(table_corners[i], rvec, tvec, K, dist)[0].reshape(-1, 2)
                    in_front = (table_corners[i] @ rotation.T + tvec.ravel())[:, 2] > 0
                    views[str(i)] = {'view': coverage(tag_pixels, in_front, width, height), 'found': i in found}
                pose['tags_by_this_pose'] = views
            head['nominal_places_pnp'] = pose
            if pose['fits']:
                source, proposal, proposal_fovy = 'estimated from table tags at nominal positions', pose, fovy
                if estimated:
                    source += '; focal length fitted to the tags, not the D435 calibration'
            elif chosen:
                source = (f"estimated from table tags; tag {chosen['reference']} taken at its nominal place, the "
                          'others where they lie')
                if chosen['focal_fitted']:
                    source += '; focal length fitted to the tags, not the D435 calibration'
                proposal, proposal_fovy = chosen['anchored'], chosen['fovy_deg']
            else:
                proposal = None
            if proposal:
                source += '; table placement c, d not measured'
                head['proposed_workcell_head_camera'] = {
                    'pos': {'value': np.round(proposal['base']['position_m'], 3).tolist(), 'source': source},
                    'xyaxes': {'value': np.round(proposal['base']['mujoco_xyaxes'], 4).tolist(), 'source': source},
                    'fovy': {'value': round(proposal_fovy, 1), 'source': source}}

    # Annotated copies.
    for role, (image, found, how) in frames.items():
        canvas = image.copy()
        height, width = image.shape[:2]
        if role == 'head' and head:
            for i, row in head['scene_camera']['tags'].items():
                if drawable(row['corners_px'], width, height):
                    draw_quad(canvas, row['corners_px'], (0, 0, 255), f'sim {i}')
            for i, row in plan.items():
                if drawable(row['corners_px'], width, height):
                    draw_quad(canvas, row['corners_px'], (255, 200, 0), f'plan {i}')
        for i, quad in found.items():
            draw_quad(canvas, quad, (0, 200, 0), f'{i}{" weak" if "weak" in how[i] else ""}', 2)
        cv2.imwrite(str(args.frames / f'{role}.annotated.png'), canvas)

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
        for name, fit in head['from_tags_alone'].items():
            a = fit['anchored']
            print(f"head from the tags alone ({name}): fx {fit['fx']:.1f}"
                  + (f" +- {fit['f_std_px']:.1f}" if fit['focal_fitted'] else '')
                  + f" px, fovy {fit['fovy_deg']:.1f}, RMS {fit['rms_px']:.2f} px; camera from tag {fit['reference']} "
                  f"{np.round(np.array(fit['camera_m']) * 1000).tolist()} mm (std "
                  f"{np.round(np.array(fit['camera_std_m']) * 1000, 1).tolist()}); looks down {a['looks_down_deg']}, "
                  f"left {a['turned_left_deg']}, roll {a['roll_deg']} deg; base {a['base']['position_m']} "
                  f"vs scene camera {a['versus_scene_camera']}")
            print('  moves to the nominal places (mm, table frame): ' + ', '.join(
                f"{i} {m['move_by_mm']} turn {m['turn_deg']}" for i, m in fit['moves_mm'].items()))
        if plan:
            print('  nominal places in this view: ' + ', '.join(f"{i} {v['view']}{'' if v['found'] else ' (not found)'}"
                                                               for i, v in plan.items()))
        p = head.get('nominal_places_pnp')
        if p:
            print(f"head PnP at nominal places: RMS {p['reprojection_rms_px']} px ({'fits' if p['fits'] else 'does not fit'})"
                  + (f"; spread {p['uncertainty']}" if 'uncertainty' in p else ''))
        if 'proposed_workcell_head_camera' in head:
            print('proposed workcell head camera (not written): ' + json.dumps(head['proposed_workcell_head_camera']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
