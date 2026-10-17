#!/usr/bin/env python3
"""Find the table's fiducial tags in real camera frames, and compare them with the task scene's head camera.

    python scripts/sim/fiducial_camera_check.py <frames dir> [--workcell FILE] [--intrinsics FILE]

<frames dir> holds frames as the cameras publish them: head.jpg, left_wrist.jpg and right_wrist.jpg (any missing one
is skipped). In each it detects the workcell's fiducial family (ArUco DICT_4X4_50) with sub-pixel corners, looking
again in the frame upscaled twice for any wanted ID not found, keeps the workcell's IDs, reads every found code back
cell by cell against the code table (the decode check), and saves an annotated copy, <role>.annotated.png. For the
head camera it then:

- projects the workcell's tag corners through the scene's head camera (its pos, xyaxes and fovy, at the frame's size)
  and reports each found tag's pixel offset from that projection;
- solves PnP for the real camera from the found tags at their nominal table-frame places, with the head camera's
  intrinsics from --intrinsics. Without them it fits one focal length to the tags themselves (principal point at the
  centre, no distortion), and labels everything that rests on it an estimate;
- reports that pose in the table and base frames, its reprojection error, how it differs from the scene's head
  camera, and where each tag falls in the frame (in view, partly, or out of it);
- reports the solution's spread when each tag centre, the printed scale and (for a fitted focal length) the
  principal point move by assumed 1-sigma errors (--sigma-*), and proposes, without writing it, a workcell head
  camera (pos, xyaxes, fovy) with its source.

The result is only as good as the tags' nominal positions, the printed square size and the table's placement in the
workcell file, until they are measured. Writes detections.json and head_pose.json into the frames dir.

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
from robot_platform.simulation.fiducials import cells, corners, grid  # noqa: E402
from robot_platform.simulation.workcell import load_workcell  # noqa: E402

ROLES = ('head', 'left_wrist', 'right_wrist')
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
HALF_TURN = np.diag([-1., -1., 1.])  # The table frame's axes in the base frame.
EDGE_MARGIN_PX = 2.


def detect(gray, family, wanted):
    """{id: [4 x 2 corners in OpenCV's order], ...} for the wanted IDs, the scale each was found at, other IDs found,
    and duplicated IDs. A wanted ID not found in the frame is looked for again in the frame upscaled twice (small
    codes), its corners mapped back to the frame's pixels."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family)), params)
    tags, scales, others, duplicates = {}, {}, set(), set()
    for scale in (1, 2):
        image = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        found, ids, _ = detector.detectMarkers(image)
        seen = set()
        for quad, tag_id in zip(found, [] if ids is None else ids.ravel().tolist()):
            if tag_id not in wanted:
                others.add(tag_id)
            elif tag_id in seen:
                duplicates.add(tag_id)
            elif tag_id not in tags:
                # Pixel centres on integers: a pixel u of the upscaled frame is (u + .5) / scale - .5 here.
                tags[tag_id] = (quad.reshape(4, 2).astype(float) + .5) / scale - .5
                scales[tag_id] = scale
            seen.add(tag_id)
        if set(tags) == set(wanted):
            break
    return tags, scales, sorted(others), sorted(duplicates)


def read_back(gray, quad, family, tag_id, px=24):
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


def camera_pose(rvec, tvec, c, d, top):
    """The camera's place and axes from a table-frame PnP solution: table frame, base frame, MuJoCo xyaxes."""
    rotation = cv2.Rodrigues(rvec)[0]
    centre = (-rotation.T @ tvec).ravel()
    axes = rotation  # Rows: the camera's OpenCV x (right), y (down), z (forward) in the table frame.
    base_centre = np.array([c - centre[0], -d - centre[1], top + centre[2]])
    base_axes = axes @ HALF_TURN.T
    return {'table': {'position_m': centre.tolist(), 'x_right': axes[0].tolist(), 'y_down': axes[1].tolist(),
                      'z_forward': axes[2].tolist()},
            'base': {'position_m': base_centre.tolist(), 'x_right': base_axes[0].tolist(),
                     'y_down': base_axes[1].tolist(), 'z_forward': base_axes[2].tolist(),
                     'mujoco_xyaxes': [*base_axes[0].tolist(), *(-base_axes[1]).tolist()]},
            '_rotation_base': np.stack([base_axes[0], base_axes[1], base_axes[2]])}


def fit_focal(objects, images, width, height, principal=None):
    """One focal length (fx = fy) fitted to a single view of the planar tags, with the principal point fixed (at the
    centre unless given) and no distortion: an estimate, not the camera's calibration. Returns K, distortion, the
    fit's RMS in pixels and the focal length's standard deviation from the pixel residuals alone."""
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


def spread(ids, centres, square, pixels, K, dist, size, estimated, c, d, top, nominal, sigma, trials, seed):
    """How far the solution moves when its inputs move by their assumed 1-sigma: each detected corner (by the
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('frames', type=Path)
    parser.add_argument('--workcell', type=Path, default=WORKCELL)
    parser.add_argument('--intrinsics', type=Path)
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
    base_corners = {i: np.array(corners(square, x, y, top, yaw)) for i, (_, x, y, yaw) in tags.items()}
    # Every marker frame is parallel to the table frame: corners at the table-frame centre, yaw 0, on the top (z 0).
    table_corners = {i: np.array(corners(square, c - x, -d - y, 0., 0.)) for i, (_, x, y, _) in tags.items()}
    intrinsics = json.loads(args.intrinsics.read_text(encoding='utf-8')) if args.intrinsics else {}

    detections, frames = {}, {}
    for role in ROLES:
        path = args.frames / f'{role}.jpg'
        if not path.exists():
            continue
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, scales, others, duplicates = detect(gray, family, set(tags))
        frames[role] = (image, found)
        detections[role] = {
            'frame': path.name, 'size': [image.shape[1], image.shape[0]], 'ids': sorted(found),
            'missing_ids': sorted(set(tags) - set(found)), 'other_ids': others, 'duplicate_ids': duplicates,
            'tags': {str(i): {'corners_px': found[i].round(2).tolist(), 'side_px': round(side_px(found[i]), 1),
                              'px_per_cell': round(side_px(found[i]) / grid(family), 2), 'found_at_scale': scales[i],
                              'decode_cell_errors': read_back(gray, found[i], family, i)} for i in sorted(found)}}

    head = None
    if 'head' in frames:
        image, found = frames['head']
        height, width = image.shape[:2]
        head = {'frame_size': [width, height], 'tags_used': sorted(found),
                'caveat': 'Tag places are the nominal layout and the square the printed design size; the table '
                          'placement (c, d) is a workcell placeholder. Until they are measured this pose is only as '
                          'good as they are.'}
        # The scene's head camera.
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
        if len(found) >= 2:
            objects = np.concatenate([table_corners[i] for i in sorted(found)])
            pixels = np.concatenate([found[i] for i in sorted(found)])
            given = intrinsics.get('head')
            if given:
                if (given['width'], given['height']) != (width, height):
                    raise SystemExit(f"head intrinsics are for {given['width']}x{given['height']}, the frame is "
                                     f'{width}x{height}')
                K = np.array([[given['fx'], 0, given['cx']], [0, given['fy'], given['cy']], [0, 0, 1]], float)
                dist = np.array(given.get('dist', [0] * 5), float)
                head['intrinsics'] = dict(given, estimated=False)
            else:
                K, dist, fit_rms, focal_std = fit_focal(objects, pixels, width, height)
                head['intrinsics'] = {'fx': K[0, 0], 'fy': K[1, 1], 'cx': K[0, 2], 'cy': K[1, 2], 'dist': dist.tolist(),
                                      'estimated': True, 'fit_rms_px': fit_rms, 'fy_std_px_from_residuals': focal_std,
                                      'source': 'estimated from the table tags at their nominal positions, not a '
                                                'calibration: one focal length (fx = fy) fitted with the principal '
                                                'point at the centre and no distortion; the D435 calibration was '
                                                'not read'}
            ok, rvec, tvec = solve(objects, pixels, K, dist)
            reprojected = cv2.projectPoints(objects, rvec, tvec, K, dist)[0].reshape(-1, 2)
            errors = np.linalg.norm(reprojected - pixels, axis=1)
            pose = camera_pose(rvec, tvec, c, d, top)
            rotation_base = pose.pop('_rotation_base')
            per_tag = {str(i): round(float(np.sqrt((errors[4 * k:4 * k + 4] ** 2).mean())), 2)
                       for k, i in enumerate(sorted(found))}
            placeholder_axes = R_sim  # Rows: the scene camera's OpenCV axes in the base frame.
            turn = rotation_base @ placeholder_axes.T
            angle = math.degrees(math.acos(max(-1., min(1., (np.trace(turn) - 1) / 2))))
            optical = math.degrees(math.acos(max(-1., min(1., float(rotation_base[2] @ placeholder_axes[2])))))
            fovy = math.degrees(2 * math.atan(height / 2 / K[1, 1]))
            pose.update(solved=bool(ok), reprojection_rms_px=round(float(np.sqrt((errors ** 2).mean())), 3),
                        reprojection_max_px=round(float(errors.max()), 3), per_tag_rms_px=per_tag,
                        fovy_deg_from_fy=round(fovy, 2),
                        versus_scene_camera={
                            'position_difference_m': (np.array(pose['base']['position_m']) -
                                                      np.array(workcell.head_camera['pos'])).round(4).tolist(),
                            'distance_m': round(float(np.linalg.norm(np.array(pose['base']['position_m']) -
                                                                     np.array(workcell.head_camera['pos']))), 4),
                            'rotation_deg': round(angle, 2), 'optical_axis_deg': round(optical, 2),
                            'fovy_deg': [round(fovy, 2), workcell.head_camera['fovy']]})
            estimated = head['intrinsics']['estimated']
            sigma = {'pixel_px': round(float(np.sqrt((errors ** 2).mean())), 3), 'tag_m': args.sigma_tag,
                     'scale': args.sigma_scale, **({'principal_px': args.sigma_principal} if estimated else {})}
            centres = {i: (c - tags[i][1], -d - tags[i][2]) for i in found}
            pose['uncertainty'] = spread(sorted(found), centres, square, pixels, K, dist, (width, height), estimated,
                                         c, d, top, rotation_base, sigma, args.trials, args.seed)
            pose['uncertainty']['note'] = (
                'The spread of the solution when the unmeasured inputs move by these assumed 1-sigma values; the '
                'table placement (c, d) adds its own error to the base position one for one.')
            head['pnp'] = pose
            # A proposal for the workcell's head camera; this script never writes the workcell file.
            source = 'estimated from table tags at nominal positions; table placement c, d not measured'
            if estimated:
                source += '; focal length fitted to the tags, not the D435 calibration'
            head['proposed_workcell_head_camera'] = {
                'pos': {'value': np.round(pose['base']['position_m'], 3).tolist(), 'source': source},
                'xyaxes': {'value': np.round(pose['base']['mujoco_xyaxes'], 4).tolist(), 'source': source},
                'fovy': {'value': round(fovy, 1), 'source': source}}
            # Where every tag falls in the real frame, by the solved pose.
            views = {}
            for i in sorted(tags):
                tag_pixels = cv2.projectPoints(table_corners[i], rvec, tvec, K, dist)[0].reshape(-1, 2)
                in_front = (table_corners[i] @ cv2.Rodrigues(rvec)[0].T + tvec.ravel())[:, 2] > 0
                views[str(i)] = {'view': coverage(tag_pixels, in_front, width, height), 'found': i in found,
                                 'corners_px': tag_pixels.round(1).tolist(),
                                 'px_per_cell': round(side_px(tag_pixels) / grid(family), 2)}
            head['tags_by_solved_pose'] = views

    # Annotated copies.
    for role, (image, found) in frames.items():
        canvas = image.copy()
        if role == 'head' and head:
            for i, row in head['scene_camera']['tags'].items():
                quad = np.array(row['corners_px'])
                if np.all(np.isfinite(quad)) and np.abs(quad).max() < 1e5:
                    draw_quad(canvas, quad, (0, 0, 255), f'sim {i}')
            for i, row in head.get('tags_by_solved_pose', {}).items():
                if not row['found']:
                    draw_quad(canvas, row['corners_px'], (0, 215, 255), f'expected {i} ({row["view"]})')
        for i, quad in found.items():
            draw_quad(canvas, quad, (0, 200, 0), f'{i}', 2)
        cv2.imwrite(str(args.frames / f'{role}.annotated.png'), canvas)

    (args.frames / 'detections.json').write_text(json.dumps(detections, indent=1), encoding='utf-8')
    if head:
        (args.frames / 'head_pose.json').write_text(json.dumps(head, indent=1, default=float), encoding='utf-8')
    for role, row in detections.items():
        print(f"{role}: IDs {row['ids']} (missing {row['missing_ids']}); decode cell errors "
              f"{ {i: t['decode_cell_errors'] for i, t in row['tags'].items()} }; px per cell "
              f"{ {i: t['px_per_cell'] for i, t in row['tags'].items()} }"
              + (f"; found only upscaled {[i for i, t in row['tags'].items() if t['found_at_scale'] > 1]}"
                 if any(t['found_at_scale'] > 1 for t in row['tags'].values()) else '')
              + (f"; other IDs {row['other_ids']}" if row['other_ids'] else '')
              + (f"; duplicated {row['duplicate_ids']}" if row['duplicate_ids'] else ''))
    if head and 'pnp' in head:
        p = head['pnp']
        print(f"head PnP ({'estimated' if head['intrinsics']['estimated'] else 'given'} intrinsics, fx "
              f"{head['intrinsics']['fx']:.1f}): reprojection RMS {p['reprojection_rms_px']} px; base position "
              f"{np.round(p['base']['position_m'], 3).tolist()}; vs scene camera {p['versus_scene_camera']}")
        u = p['uncertainty']
        print(f"spread for {u['assumed_1sigma']}: fy {u['fy_std_px']} px, fovy {u['fovy_std_deg']} deg, position "
              f"{u['position_std_m']} m, rotation {u['rotation_rms_deg']} deg RMS")
        print('tags by solved pose: ' + ', '.join(f"{i} {v['view']}{'' if v['found'] else ' (not found)'}"
                                                   for i, v in head['tags_by_solved_pose'].items()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
