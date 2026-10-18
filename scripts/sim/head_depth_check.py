#!/usr/bin/env python3
"""Measure the table, the box and the blocks from the head RealSense's depth, in the tag rectangle's frame.

    python scripts/sim/head_depth_check.py <depth dir> [--workcell FILE] [--calibration FILE]

<depth dir> holds a head grab as grab_head_depth.py takes it and unpack_head_depth.py unpacks it: depth_median.npy (the
per-pixel median of up to 20 frames, mm, as the head node publishes it: turned half a turn), depth_count.npy (frames
with a valid value), depth_mad.npy, head.jpg and header.json (the node's header, with the depth intrinsics it
publishes: negative focal lengths and a mirrored principal point, which turn a published pixel back into the depth
sensor's own optical frame).

Each depth pixel goes into the depth sensor's frame, then into the colour sensor's by the "Depth" to "Color"
extrinsics in the calibration (rs-enumerate-devices -c), then into the published colour camera's frame (turned half a
turn about its optical axis): the frame the scene's head camera models. The tag rectangle's pose in that frame comes
from the colour image's tags (fiducial_camera_check, the workcell's intrinsics and tag places).

It fits the table plane (RANSAC, then least squares on its inliers) and compares it with the tags' plane: the camera's
height above each and the angle between them. Then it seats the rectangle frame on the depth's plane: the depth gives
the camera's height above the table and its tilt, the tags only where the camera stands over the plane and which way
it faces (the depth-anchored pose, whose camera it proposes for the workcell). In that frame it finds what stands on
the table near the rectangle by height above the depth plane, and fits the box (the largest object: outer footprint,
yaw, rim height, its front and back faces) and each block (edge, centre, yaw, height). Writes depth_check.json, the
registered depth (registered_depth.npy, metres, on the colour pixel grid) and depth_check.png, and prints a summary.
Needs numpy and OpenCV.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'scripts/sim'))
import fiducial_camera_check as check  # noqa: E402
from robot_platform.simulation.fiducials import corners  # noqa: E402
from robot_platform.simulation.workcell import head_pinhole, load_workcell  # noqa: E402

WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
CALIBRATION = ROOT / 'build/fiducials-check/intrinsics/d435-calibration-copy.txt'  # rs-enumerate-devices -c
MIN_FRAMES = 10  # A depth pixel counts with at least this many valid frames of the median.
PLANE_TOLERANCE_M = .004
OBJECT_HEIGHT_M = .012


def extrinsics(text, source, target):
    """(R, t) taking a point in `source`'s optical frame into `target`'s, as rs-enumerate-devices prints them (the
    matrix as it reads, row by row; target = R @ source + t)."""
    block = text[text.index(f'Extrinsic from "{source}"\t  To\t  "{target}"'):]
    numbers = [float(v) for v in re.findall(r'-?\d+\.?\d*(?:e-?\d+)?', block.split('Translation Vector:')[0])[0:9]]
    translation = [float(v) for v in re.findall(r'-?\d+\.?\d*(?:e-?\d+)?', block.split('Translation Vector:')[1])[:3]]
    return np.array(numbers).reshape(3, 3), np.array(translation)


def plane_fit(points, rng, iterations=400, tolerance=PLANE_TOLERANCE_M):
    """RANSAC then least squares: (unit normal, offset) with normal . p = offset, and the inlier mask."""
    best = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        if np.linalg.norm(normal) < 1e-9:
            continue
        normal /= np.linalg.norm(normal)
        inliers = np.abs(points @ normal - normal @ sample[0]) < tolerance
        if best is None or inliers.sum() > best.sum():
            best = inliers
    chosen = points[best]
    centre = chosen.mean(axis=0)
    normal = np.linalg.svd(chosen - centre, full_matrices=False)[2][2]
    inliers = np.abs((points - centre) @ normal) < tolerance
    centre = points[inliers].mean(axis=0)
    normal = np.linalg.svd(points[inliers] - centre, full_matrices=False)[2][2]
    return normal, float(normal @ centre), inliers


def anchored_pose(normal, offset, objects, pixels, K, R, t, iterations=30):
    """The rectangle frame's pose in the camera (x = R q + t) with its plane on the depth-fitted table plane
    (normal . x = offset, the normal up): the depth sets the camera's height and tilt, and the tags' corners (objects,
    in the rectangle frame, flat on it; pixels) only the three unknowns left, where the camera stands over the plane
    and its turn about the normal. Gauss-Newton from the tags' own pose (R, t). Returns (R, t) and each corner's
    error in pixels."""
    a = R[:, 0] - (R[:, 0] @ normal) * normal
    a /= np.linalg.norm(a)
    b = np.cross(normal, a)
    start = t - (normal @ t - offset) * normal  # The tags' origin, dropped onto the depth plane.

    def pose(params):
        x, y, turn = params
        e1 = math.cos(turn) * a + math.sin(turn) * b
        return np.column_stack([e1, np.cross(normal, e1), normal]), start + x * a + y * b

    def residual(params):
        rotation, translation = pose(params)
        projected = (objects @ rotation.T + translation) @ K.T
        return (projected[:, :2] / projected[:, 2:] - pixels).ravel()

    params = np.zeros(3)
    for _ in range(iterations):
        r = residual(params)
        jacobian = np.column_stack([(residual(params + step) - r) / 1e-7 for step in np.eye(3) * 1e-7])
        delta = np.linalg.lstsq(jacobian, -r, rcond=None)[0]
        params = params + delta
        if np.abs(delta).max() < 1e-10:
            break
    rotation, translation = pose(params)
    return rotation, translation, np.linalg.norm(residual(params).reshape(-1, 2), axis=1)


def footprint(xy):
    """The minimum-area rectangle around points (x, y): centre, (long, short) side lengths and the long side's angle
    from x, in radians."""
    (cx, cy), (w, h), angle = cv2.minAreaRect(xy.astype(np.float32))
    angle = math.radians(angle)
    if w < h:
        w, h, angle = h, w, angle + math.pi / 2
    return np.array([cx, cy]), (w, h), math.atan2(math.sin(angle), math.cos(angle))


def wall_lines(xy, rng, tolerance=.004, minimum=250, count=5):
    """Up to `count` straight runs of points in the table plane (walls seen edge-on from above), strongest first:
    RANSAC, then a least-squares line through the inliers. Each is its centre, unit direction, point count and length
    (1-99% of its points along it)."""
    remaining, lines = xy.copy(), []
    for _ in range(count):
        if len(remaining) < minimum:
            break
        best = None
        for _ in range(600):
            a, b = remaining[rng.choice(len(remaining), 2, replace=False)]
            d = b - a
            if np.linalg.norm(d) < .02:
                continue
            d /= np.linalg.norm(d)
            inliers = np.abs((remaining - a) @ np.array([-d[1], d[0]])) < tolerance
            if best is None or inliers.sum() > best.sum():
                best = inliers
        if best is None or best.sum() < minimum:
            break
        for _ in range(2):
            points = remaining[best]
            centre = points.mean(axis=0)
            direction = np.linalg.svd(points - centre, full_matrices=False)[2][0]
            best = np.abs((remaining - centre) @ np.array([-direction[1], direction[0]])) < tolerance
        points = remaining[best]
        centre = points.mean(axis=0)
        along = (points - centre) @ direction
        lines.append({'centre': centre, 'direction': direction, 'points': int(best.sum()),
                      'length': float(np.percentile(along, 99) - np.percentile(along, 1))})
        remaining = remaining[~best]
    return lines


def box_from_walls(lines, wall):
    """The box from its wall faces seen from above: the faces fall into two families at right angles. The family whose
    faces stand farther apart runs across the box's length (front and back walls); the other, its width. The camera
    sees the front and the near side from outside and the back and far side from inside, so each family's spread
    is its outer size less one wall (`wall`). Returns None without two faces in each family."""
    if len(lines) < 2:
        return None
    reference = lines[0]['direction']
    families = {0: [], 1: []}
    for line in lines:
        cos = abs(float(line['direction'] @ reference))
        if cos > math.cos(math.radians(15)):
            families[0].append(line)
        elif cos < math.sin(math.radians(15)):
            families[1].append(line)
    spans = {}
    for key, members in families.items():
        if len(members) < 2:
            continue
        normal = np.array([-members[0]['direction'][1], members[0]['direction'][0]])
        offsets = sorted(float(m['centre'] @ normal) for m in members)
        spans[key] = (normal, offsets[0], offsets[-1])
    if len(spans) < 2:
        return {'families': {k: len(v) for k, v in families.items()}}
    (n0, a0, b0), (n1, a1, b1) = spans[0], spans[1]
    if b1 - a1 > b0 - a0:
        (n0, a0, b0), (n1, a1, b1) = (n1, a1, b1), (n0, a0, b0)
    axis = n0 if n0[1] < 0 else -n0  # Along the length, toward the robot (-y).
    centre = n0 * (a0 + b0) / 2 + n1 * (a1 + b1) / 2
    return {'axis_toward_robot': axis, 'yaw_deg': math.degrees(math.atan2(axis[0], -axis[1])),
            'length_faces_apart_m': b0 - a0, 'width_faces_apart_m': b1 - a1,
            'outer_length_m': b0 - a0 + wall, 'outer_width_m': b1 - a1 + wall,
            'centre_m': centre, 'front_face_along_axis_m': max(a0, b0) if n0 @ axis > 0 else -min(a0, b0)}


def box_from_faces(points, rng, wall):
    """The box from its walls, given its steady wall points (x, y, height) in the rectangle frame.

    Seen from above, each wall is a straight run of points; a leaning wall is a band, so runs within 20 mm and 15
    degrees of each other are one face. The faces across the box's length are the strongest family; the camera sees
    the front one from outside and the back one from inside. Each gets a plane through its points (x, y, height),
    which gives its lean and where it stands at the table (height 0) and at mid-height. The front face also gives the
    box's outer width: its extent along itself (1-99%), since the camera sees all of it."""
    lines = wall_lines(points[:, :2], rng, minimum=200, count=8)
    if not lines:
        return None
    reference = lines[0]['direction']
    across = [line for line in lines if abs(float(line['direction'] @ reference)) > math.cos(math.radians(15))]
    normal = np.array([-reference[1], reference[0]])
    if normal[1] > 0:
        normal = -normal  # Toward the robot (-y): the box's axis.
    # Merge runs into faces by their offset along the axis.
    offsets = sorted((float(line['centre'] @ normal), line) for line in across)
    faces = []
    for offset, line in offsets:
        if faces and offset - faces[-1]['offsets'][-1] < .02:
            faces[-1]['offsets'].append(offset)
        else:
            faces.append({'offsets': [offset]})
    if len(faces) < 2:
        return {'faces_found': len(faces), 'runs': len(lines)}
    front_offset, back_offset = max(f['offsets'][-1] for f in faces), min(f['offsets'][0] for f in faces)
    fitted = {}
    for name, low, high in (('front', front_offset - .025, front_offset + .012),
                            ('back', back_offset - .012, back_offset + .025)):
        on = points[((points[:, :2] @ normal) > low) & ((points[:, :2] @ normal) < high)]
        centre = on.mean(axis=0)
        plane = np.linalg.svd(on - centre, full_matrices=False)[2][2]
        if plane[:2] @ normal < 0:
            plane = -plane  # Facing the robot.
        at = {}
        side = np.array([-normal[1], normal[0], 0.])
        middle = float(centre[:2] @ side[:2])
        for label, z in (('table', 0.), ('mid', .09), ('tags', .147)):
            # Where the face is along the axis at height z, at its own middle across: plane . p = plane . centre
            # with p = s n + middle side + z e_z.
            at[label] = float((plane @ centre - middle * (plane @ side) - plane[2] * z) / (plane[:2] @ normal))
        along = (on[:, :2] - centre[:2]) @ np.array([-normal[1], normal[0]])
        fitted[name] = {'points': int(len(on)), 'lean_deg': math.degrees(math.asin(np.clip(-plane[2], -1, 1))),
                        'along_axis_m': at, 'extent_m': float(np.percentile(along, 99) - np.percentile(along, 1)),
                        'centre': centre, 'top_m': float(np.percentile(on[:, 2], 98))}
    front, back = fitted['front'], fitted['back']
    middle_across = front['centre'][:2] @ np.array([-normal[1], normal[0]])
    length_mid = front['along_axis_m']['mid'] - back['along_axis_m']['mid']
    centre_along = (front['along_axis_m']['mid'] + back['along_axis_m']['mid'] - wall) / 2
    centre = centre_along * normal + middle_across * np.array([-normal[1], normal[0]])
    return {'axis_toward_robot': normal, 'yaw_deg': math.degrees(math.atan2(normal[0], -normal[1])),
            'front': front, 'back': back, 'runs': len(lines),
            'faces_apart_at_tags_m': front['along_axis_m']['tags'] - back['along_axis_m']['tags'],
            'faces_apart_mid_m': length_mid, 'outer_length_mid_m': length_mid + wall,
            'outer_width_m': front['extent_m'], 'centre_m': centre}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('grab', type=Path)
    parser.add_argument('--workcell', type=Path, default=WORKCELL)
    parser.add_argument('--calibration', type=Path, default=CALIBRATION)
    parser.add_argument('--seed', type=int, default=20261018)
    parser.add_argument('--wall', type=float, default=.004,
                        help="the box wall's thickness, added once to each span of its faces (default .004 m, the "
                             "workcell's placeholder)")
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)
    workcell = load_workcell(args.workcell)
    header = json.loads((args.grab / 'header.json').read_text(encoding='utf-8'))['header']
    depth = np.load(args.grab / 'depth_median.npy').astype(float) * header['depth']['scale']
    count = np.load(args.grab / 'depth_count.npy')
    mad = np.load(args.grab / 'depth_mad.npy').astype(float) * header['depth']['scale']
    intr = header['depth']['intrinsics']
    height, width = depth.shape
    valid = (count >= MIN_FRAMES) & (depth > 0)

    # Published depth pixels -> depth sensor frame -> colour sensor frame -> published colour camera frame.
    v, u = np.nonzero(valid)
    z = depth[v, u]
    sensor = np.column_stack([(u - intr['cx']) / intr['fx'] * z, (v - intr['cy']) / intr['fy'] * z, z])
    R_dc, t_dc = extrinsics(args.calibration.read_text(encoding='utf-8'), 'Depth', 'Color')
    points = (sensor @ R_dc.T + t_dc) * (-1., -1., 1.)
    camera = workcell.head_camera
    fx, fy, cx, cy = head_pinhole(camera)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    # The depth registered onto the colour image's pixels (nearest point wins).
    pixel = (points @ K.T)
    cu, cv_ = np.round(pixel[:, 0] / pixel[:, 2]).astype(int), np.round(pixel[:, 1] / pixel[:, 2]).astype(int)
    inside = (cu >= 0) & (cu < width) & (cv_ >= 0) & (cv_ < height)
    registered = np.full((height, width), np.inf)
    np.minimum.at(registered, (cv_[inside], cu[inside]), points[inside, 2])
    registered[~np.isfinite(registered)] = 0.
    np.save(args.grab / 'registered_depth.npy', registered)

    # The tag rectangle in this colour frame (the workcell's tag places and intrinsics).
    image = cv2.imread(str(args.grab / 'head.jpg'))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    f = workcell.fiducials
    plane = check.Plane(workcell)
    found, how, _, _ = check.detect(gray, f['family'], {tag[0] for tag in f['tags']})
    tags = {tag[0]: tag for tag in f['tags']}
    flat = [i for i in sorted(found) if tags[i][5] == 0]
    objects = np.concatenate([np.array(corners(f['square'], *plane.point(tags[i][1:4])[:2], 0., 0.)) for i in flat])
    pixels = np.concatenate([found[i] for i in flat])
    _, rvec, tvec = check.solve(objects, pixels, K, np.zeros(5))
    R, t = cv2.Rodrigues(rvec)[0], tvec.ravel()  # Rectangle frame into the camera frame.
    errors = np.linalg.norm(cv2.projectPoints(objects, rvec, tvec, K, np.zeros(5))[0].reshape(-1, 2) - pixels, axis=1)
    local = (points - t) @ R  # Every depth point in the rectangle frame, as the tags place it.

    # The table plane: points near the tags' plane around the rectangle, then RANSAC.
    near = (np.abs(local[:, 2]) < .03) & (local[:, 0] > -.10) & (local[:, 0] < .75) & (local[:, 1] > -.10) & \
        (local[:, 1] < .60)
    normal, offset, inliers = plane_fit(points[near], rng)
    if normal @ (R[:, 2]) < 0:
        normal, offset = -normal, -offset
    tag_normal = R[:, 2]
    report = {'grab': args.grab.name, 'frames': int(np.median(count[valid])), 'valid_share': round(float(valid.mean()), 3),
              'depth_mad_median_mm': round(float(np.median(mad[valid]) * 1000), 2),
              'tags': {'used': flat, 'rms_px': round(float(np.sqrt((errors ** 2).mean())), 3),
                       'camera_height_m': round(float(-(tag_normal @ t)), 4)}}
    camera_height_depth = -offset  # Signed height of the camera (origin) above the plane, the normal pointing up.
    down_depth = math.degrees(math.asin(-normal[2]))
    down_tags = math.degrees(math.asin(-tag_normal[2]))
    residual = points[near][inliers] @ normal - offset
    report['table'] = {
        'normal_in_camera': normal.round(5).tolist(), 'inliers': int(inliers.sum()),
        'fit_rms_mm': round(float(np.sqrt((residual ** 2).mean()) * 1000), 2),
        'camera_height_m': {'depth': round(camera_height_depth, 4), 'tags': round(float(-(tag_normal @ t)), 4),
                            'squares_only': None},
        'optical_axis_down_deg': {'depth': round(down_depth, 2), 'tags': round(down_tags, 2)},
        'planes_angle_deg': round(math.degrees(math.acos(min(1., float(normal @ tag_normal)))), 3),
    }
    # The tags without the taped rectangle: their squares alone (each 43.18 mm) set the scale.
    alone = check.free_layout({i: found[i] for i in flat if 'weak' not in how[i]}, f['square'], (width, height),
                              min(flat), K, np.zeros(5))
    if alone:
        report['table']['camera_height_m']['squares_only'] = round(float(alone['camera_m'][2]), 4)
    # The depth plane in the rectangle frame: its height under each taped corner.
    corner_heights = {}
    for i in (0, 1, 2, 3):
        x, y = plane.point(tags[i][1:4])[:2]
        # Height of the depth plane above the tags' plane at (x, y): solve n . (R p + t) = offset for p = (x, y, h).
        p0 = R @ np.array([x, y, 0.]) + t
        corner_heights[str(i)] = round(float((offset - normal @ p0) / (normal @ R[:, 2])) * 1000, 2)
    report['table']['depth_plane_above_tag_plane_mm'] = corner_heights

    # The rectangle frame seated on the depth plane; from here on, every measurement is in it.
    R_a, t_a, anchored_errors = anchored_pose(normal, offset, objects, pixels, K, R, t)
    _, R_sim, _ = check.scene_camera(workcell.head_camera, width, height)
    tag_camera, anchored_camera = -R.T @ t, -R_a.T @ t_a
    anchored = check.camera_report(anchored_camera, R_a, plane, workcell, R_sim,
                                   math.degrees(2 * math.atan(height / 2 / K[1, 1])))
    report['anchored'] = {
        'what': 'the rectangle frame on the depth-fitted table plane: the depth sets the camera height and tilt, the '
                f'tags {flat} its place over the plane and its turn',
        'rotation': R_a.round(6).tolist(), 'translation_m': t_a.round(5).tolist(),
        'tags_rms_px': round(float(np.sqrt((anchored_errors ** 2).mean())), 3),
        'tags_max_px': round(float(anchored_errors.max()), 3),
        'camera': anchored,
        'versus_tag_pose': {
            'camera_mm': np.round((anchored_camera - tag_camera) * 1000, 1).tolist(),
            'rotation_deg': round(math.degrees(math.acos(max(-1., min(1., (np.trace(R_a @ R.T) - 1) / 2)))), 3)}}
    source = (f'estimated from RealSense depth plus tags: head frame {args.grab.name}, the table plane fitted to its '
              f'depth ({report["table"]["fit_rms_mm"]} mm RMS) sets the camera height and tilt, the flat tags {flat} at '
              f'their places in the rectangle its place over the plane and its turn '
              f'({report["anchored"]["tags_rms_px"]} px RMS); with the D435 colour intrinsics')
    report['proposed_workcell_head_camera'] = {
        'frame': plane.name,
        'pos': {'value': anchored[plane.name]['position_m'], 'source': source},
        'xyaxes': {'value': anchored[plane.name]['mujoco_xyaxes'], 'source': source}}
    local = (points - t_a) @ R_a

    # How far the table top runs, as the depth sees it: the plane's inliers in the rectangle frame, and its far edge
    # (where the inliers end, per 50 mm of x) as a line.
    table = local[near][inliers]
    report['table']['visible_extent_rectangle_m'] = {
        'x': np.round(np.percentile(table[:, 0], [.5, 99.5]), 3).tolist(),
        'y': np.round(np.percentile(table[:, 1], [.5, 99.5]), 3).tolist()}
    # The far edge, where the inliers end toward the far side, per 50 mm of x left of the box (the box hides the table
    # behind it); the near and side edges are out of view.
    edge = []
    for x0 in np.arange(0., .35, .05):
        column = table[(table[:, 0] >= x0) & (table[:, 0] < x0 + .05)]
        if len(column) > 200:
            edge.append((x0 + .025, float(np.percentile(column[:, 1], 99))))
    if len(edge) >= 3:
        edge = np.array(edge)
        slope, intercept = np.polyfit(edge[:, 0], edge[:, 1], 1)
        report['table']['far_edge'] = {
            'y_at_x0_mm': round(intercept * 1000, 1), 'turn_from_rectangle_deg': round(math.degrees(math.atan(slope)), 2),
            'fit_rms_mm': round(float(np.sqrt(np.mean((np.polyval([slope, intercept], edge[:, 0]) - edge[:, 1]) ** 2)))
                                * 1000, 1), 'columns': len(edge)}

    # What stands on the table inside its visible extent near the rectangle: height above the depth plane.
    heights = points @ normal - offset
    # Inside the tag rectangle, short of its far side: a shelf behind the table overhangs it from about 30 mm in, about
    # 150 mm up.
    size = workcell.fiducials['rectangle']['size']
    region = (local[:, 0] > -.02) & (local[:, 0] < size[0] + .02) & (local[:, 1] > -.02) & (local[:, 1] < size[1] - .035)
    # Objects are connected on the table's own grid (5 mm cells of x, y in the rectangle frame), not in the image,
    # where a box's rim can touch what lies far behind it.
    keep = np.flatnonzero(region & (heights > OBJECT_HEIGHT_M) & (heights < .30))
    cell = .005
    gx = np.floor((local[keep, 0] + .10) / cell).astype(int)
    gy = np.floor((local[keep, 1] + .10) / cell).astype(int)
    occupancy = np.zeros((int(.80 / cell) + 2, int(1. / cell) + 2), np.uint8)
    np.add.at(occupancy, (gy, gx), 1)
    occupied = (occupancy >= 2).astype(np.uint8)  # At least two points in a cell: lone stray points do not join.
    occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count_components, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    point_label = labels[gy, gx]
    found_objects = []
    for k in range(1, count_components):
        members = keep[point_label == k]
        if len(members) >= 150:
            found_objects.append(members)
    found_objects.sort(key=len, reverse=True)
    report['objects'] = []
    for members in found_objects:
        h, xy = heights[members], local[members, :2]
        entry = {'depth_pixels': int(len(members))}
        # The top: the densest 2 mm of heights above 20 mm (flying pixels at edges stray above it).
        counts, bins = np.histogram(h[h > .02], bins=np.arange(.02, .30, .002))
        top = float(bins[int(np.argmax(counts))] + .001) if counts.sum() else float(np.max(h))
        if not report['objects'] and top > .08:
            # The box: the largest object. Its rim is where its heights end (99%: edges throw a few points higher).
            # Its walls, seen from above, are straight runs of steady points (flying pixels at its silhouette flicker
            # from frame to frame) between 30 mm up and 20 mm under the rim.
            rim = float(np.percentile(h, 99))
            steady = mad[v[members], u[members]] < .003
            selected = (h > .03) & (h < rim - .02) & steady
            box = box_from_faces(np.column_stack([xy[selected], h[selected]]), rng, args.wall)
            entry.update(kind='box', rim_mm=round(rim * 1000, 1), rim_p95_mm=round(float(np.percentile(h, 95)) * 1000, 1),
                         wall_points=int(selected.sum()))
            if box and 'yaw_deg' in box:
                mm = lambda value: round(value * 1000, 1)  # noqa: E731
                entry.update(
                    yaw_deg=round(box['yaw_deg'], 2), axis_toward_robot=np.round(box['axis_toward_robot'], 4).tolist(),
                    centre_rectangle_mm=np.round(box['centre_m'] * 1000, 1).tolist(),
                    outer_length_mid_mm=mm(box['outer_length_mid_m']), outer_width_mm=mm(box['outer_width_m']),
                    faces_apart_at_tags_mm=mm(box['faces_apart_at_tags_m']), faces_apart_mid_mm=mm(box['faces_apart_mid_m']),
                    faces={name: {'points': face['points'], 'lean_out_deg': round(face['lean_deg'], 2),
                                  'along_axis_mm': {k: mm(s) for k, s in face['along_axis_m'].items()},
                                  'extent_mm': mm(face['extent_m']), 'top_mm': mm(face['top_m'])}
                           for name, face in (('front_outer', box['front']), ('back_inner', box['back']))})
            elif box:
                entry['faces_found'] = box
            report['objects'].append(entry)
            continue
        if not .025 < top < .06:
            entry.update(kind='other', top_mm=round(top * 1000, 1),
                         centre_rectangle_mm=np.round(xy.mean(axis=0) * 1000, 1).tolist())
            report['objects'].append(entry)
            continue
        # A block: its top face is the steady points within 5 mm of its top; its extent along its own axes (2-98%).
        face = xy[(np.abs(h - top) < .005) & (mad[v[members], u[members]] < .003)]
        middle = face.mean(axis=0)
        axes = np.linalg.svd(face - middle, full_matrices=False)[2]
        extent = [float(np.percentile((face - middle) @ a, 98) - np.percentile((face - middle) @ a, 2)) for a in axes]
        turn = math.degrees(math.atan2(axes[0][1], axes[0][0]))
        turn = (turn + 45) % 90 - 45  # A square's yaw, within +-45 degrees.
        entry.update(kind='block', top_mm=round(top * 1000, 1), top_face_points=int(len(face)),
                     top_face_centre_rectangle_mm=np.round(middle * 1000, 1).tolist(),
                     top_face_extent_mm=[round(e * 1000, 1) for e in extent], yaw_deg=round(turn, 1))
        report['objects'].append(entry)
    (args.grab / 'depth_check.json').write_text(json.dumps(report, indent=1), encoding='utf-8')

    # A picture: heights above the table on the colour frame, the region's objects only.
    canvas = image.copy()
    sel = inside & region
    heat = np.zeros((height, width), float)
    np.maximum.at(heat, (cv_[sel], cu[sel]), np.clip(heights[sel], 0, .2))
    colour = cv2.applyColorMap((heat / .2 * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    blend = np.where((heat > OBJECT_HEIGHT_M)[..., None], (.55 * colour + .45 * canvas).astype(np.uint8), canvas)
    cv2.putText(blend, 'height above the depth-fitted table plane, 0-200 mm (turbo)', (6, height - 8),
                cv2.FONT_HERSHEY_SIMPLEX, .38, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(args.grab / 'depth_check.png'), blend)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
