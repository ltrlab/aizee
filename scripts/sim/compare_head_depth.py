#!/usr/bin/env python3
"""Compare the scene's rendered head depth with the real head camera's, pixel by pixel, over the table, box and blocks.

    python scripts/sim/compare_head_depth.py <depth grab dir> <sim render dir> [--workcell FILE] [--name NAME]

<depth grab dir> is a head_depth_check.py run (registered_depth.npy on the colour pixel grid, depth_check.json);
<sim render dir> is a render_head_depth.py run (sim_depth.npy, sim_render.json) at the same size, with the workcell it
used (--workcell; the default is the one recorded in sim_render.json). Pixels are classed from the real depth: table
(within 8 mm of the depth-fitted table plane, inside the tag rectangle), box (more than 15 mm up, within 20 mm of the
workcell box's footprint) and blocks (more than 15 mm up, within 40 mm of a block's centre). The rectangle frame is the
grab's depth-anchored one (depth_check.json), the frame its box and blocks were measured in.

For each class it reports sim minus real depth along the optical axis (median, mean, RMS and 90th percentile of its
size, in mm) over all its pixels, and again over its interior: pixels more than 3 px from a depth step of over 15 mm in
either image, where a pixel or two of misplaced outline does not count. For the box and each block it also compares
where they stand in the image: the sim's pixels of it (classed the same way from the sim depth) against the real
ones, as intersection over union and the shift of their centroid, over the pixels where both depths exist. Writes
depth_compare_<NAME>.json and depth_compare_<NAME>.png (real depth, sim depth, difference) into the sim render dir.
Needs numpy and OpenCV.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'scripts/sim'))
import fiducial_camera_check as check  # noqa: E402
from robot_platform.simulation.workcell import bucket_frame, head_pinhole, load_workcell  # noqa: E402

RANGE_M = (0.45, 1.05)  # The depth colour scale.
DIFF_MM = 30.0
STEP_M, STEP_REACH_PX = 0.015, 3  # A depth step, and how far from one a pixel stops counting as interior.


def colourise(depth):
    scaled = np.clip((depth - RANGE_M[0]) / (RANGE_M[1] - RANGE_M[0]), 0, 1)
    image = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    image[depth <= 0] = 0
    return image


def near_steps(depth):
    """Pixels within STEP_REACH_PX of a step of more than STEP_M between neighbours (small holes closed first)."""
    filled = cv2.morphologyEx(depth.astype(np.float32), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    step = np.zeros(depth.shape, bool)
    across, down = np.abs(np.diff(filled, axis=1)) > STEP_M, np.abs(np.diff(filled, axis=0)) > STEP_M
    step[:, 1:] |= across
    step[:, :-1] |= across
    step[1:, :] |= down
    step[:-1, :] |= down
    reach = 2 * STEP_REACH_PX + 1
    return cv2.dilate(step.astype(np.uint8), np.ones((reach, reach), np.uint8)) > 0


def stats(difference):
    if not len(difference):
        return {'pixels': 0}
    return {
        'pixels': int(len(difference)),
        'median_mm': round(float(np.median(difference)), 2),
        'mean_mm': round(float(difference.mean()), 2),
        'rms_mm': round(float(np.sqrt((difference**2).mean())), 2),
        'p90_abs_mm': round(float(np.percentile(np.abs(difference), 90)), 2),
    }


class Classes:
    """Points of a depth image, classed by height above the depth-fitted table and place in the rectangle frame."""

    def __init__(self, depth_at, u, v, K, report, workcell):
        z = depth_at
        points = np.column_stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z])
        normal = np.array(report['table']['normal_in_camera'])
        self.heights = points @ normal + report['table']['camera_height_m']['depth']
        R, t = np.array(report['anchored']['rotation']), np.array(report['anchored']['translation_m'])
        local = (points - t) @ R
        size = workcell.fiducials['rectangle']['size']
        self.inside = (local[:, 0] > 0) & (local[:, 0] < size[0]) & (local[:, 1] > 0) & (local[:, 1] < size[1] - 0.035)
        # The box's footprint, from the workcell, in its own axes.
        plane = check.Plane(workcell)
        to_base = plane.origin[:2] + local[:, :2] @ plane.axes[:2, :2]
        along_x, along_y = bucket_frame(workcell, to_base[:, 0], to_base[:, 1])
        half = (
            workcell.bucket_inner[0] / 2 + workcell.bucket_wall + 0.02,
            workcell.bucket_inner[1] / 2 + workcell.bucket_wall + 0.02,
        )
        self.in_box = (np.abs(along_x) < half[0]) & (np.abs(along_y) < half[1])
        blocks = [np.array(o['centre_rectangle_mm']) / 1000 for o in report['objects'] if o['kind'] == 'block']
        self.blocks = [
            (self.heights > 0.015) & (np.linalg.norm(local[:, :2] - centre, axis=1) < 0.04) & ~self.in_box
            for centre in blocks
        ]
        self.box = (self.heights > 0.015) & self.in_box
        self.table = self.inside & (np.abs(self.heights) < 0.008) & ~self.in_box


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('grab', type=Path)
    parser.add_argument('render', type=Path)
    parser.add_argument('--workcell', type=Path)
    parser.add_argument('--name', default='sim')
    args = parser.parse_args()
    meta = json.loads((args.render / 'sim_render.json').read_text(encoding='utf-8'))
    workcell = load_workcell(args.workcell or meta['workcell'])
    real = np.load(args.grab / 'registered_depth.npy')
    sim = np.load(args.render / 'sim_depth.npy').astype(float)
    report = json.loads((args.grab / 'depth_check.json').read_text(encoding='utf-8'))
    height, width = real.shape
    camera = workcell.head_camera
    if tuple(camera['intrinsics']['resolution']) != (width, height) or sim.shape != real.shape:
        raise SystemExit("the real and sim depth must be on the camera intrinsics' pixel grid")
    fx, fy, cx, cy = head_pinhole(camera)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    if not np.allclose(K, report['registered_K'], atol=1e-6):
        raise SystemExit(
            f'the sim camera {K.tolist()} is not the one the real depth was registered to {report["registered_K"]}'
        )

    # Every pixel with both depths, classed from the real depth and, for the outlines, from the sim's.
    v, u = np.nonzero((real > 0) & (sim > 0))
    z, simulated = real[v, u], sim[v, u]
    measured, rendered = Classes(z, u, v, K, report, workcell), Classes(simulated, u, v, K, report, workcell)
    interior = ~(near_steps(real) | near_steps(sim))[v, u]
    classes = {'table': measured.table, 'box': measured.box, 'blocks': np.any(measured.blocks, axis=0)}
    result = {
        'name': args.name,
        'grab': args.grab.name,
        'render': args.render.name,
        'workcell': str(args.workcell or meta['workcell']),
        'head_camera': {k: camera[k] for k in ('pos', 'xyaxes', 'fovy')},
        'classes': {},
        'outlines': {},
    }
    difference = (simulated - z) * 1000
    for name, members in classes.items():
        result['classes'][name] = {**stats(difference[members]), 'interior': stats(difference[members & interior])}
    pixels = np.column_stack([u, v]).astype(float)
    for name, real_mask, sim_mask in [('box', measured.box, rendered.box)] + [
        (f'block {k + 1}', a, b) for k, (a, b) in enumerate(zip(measured.blocks, rendered.blocks))
    ]:
        union = int((real_mask | sim_mask).sum())
        result['outlines'][name] = {
            'real_px': int(real_mask.sum()),
            'sim_px': int(sim_mask.sum()),
            'iou': round(float((real_mask & sim_mask).sum() / union), 3) if union else None,
            'centroid_sim_minus_real_px': (
                np.round(pixels[sim_mask].mean(axis=0) - pixels[real_mask].mean(axis=0), 2).tolist()
                if real_mask.any() and sim_mask.any()
                else None
            ),
        }
    (args.render / f'depth_compare_{args.name}.json').write_text(json.dumps(result, indent=1), encoding='utf-8')

    # Real, sim and their difference side by side; the classes' pixels outlined in the difference.
    diff = np.zeros((height, width))
    diff[v, u] = difference
    shown = np.clip(diff / DIFF_MM, -1, 1)
    diff_image = np.zeros((height, width, 3), np.uint8)
    diff_image[..., 2] = (np.clip(shown, 0, 1) * 255).astype(np.uint8)  # Sim farther: red.
    diff_image[..., 0] = (np.clip(-shown, 0, 1) * 255).astype(np.uint8)  # Sim nearer: blue.
    diff_image[..., 1] = ((1 - np.abs(shown)) * 90).astype(np.uint8)
    diff_image[(real <= 0) | (sim <= 0)] = 0
    for name, colour in (('table', (255, 255, 255)), ('box', (0, 200, 255)), ('blocks', (0, 255, 0))):
        mask = np.zeros((height, width), np.uint8)
        mask[v[classes[name]], u[classes[name]]] = 255
        contours, _ = cv2.findContours(
            cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(diff_image, contours, -1, colour, 1)
    panels = [colourise(real), colourise(sim), diff_image]
    titles = [
        'real depth (RealSense, registered to colour)',
        f'sim depth ({args.name})',
        f'sim - real, +-{DIFF_MM:.0f} mm (red: sim farther)',
    ]
    for panel, title in zip(panels, titles):
        cv2.putText(panel, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    lines = [
        f'{k}: median {c.get("median_mm")}, RMS {c.get("rms_mm")} mm; interior {c["interior"].get("median_mm")}, '
        f'{c["interior"].get("rms_mm")} mm'
        for k, c in result['classes'].items()
    ]
    for k, line in enumerate(lines):
        cv2.putText(
            panels[2], line, (8, height - 50 + 16 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA
        )
    cv2.imwrite(str(args.render / f'depth_compare_{args.name}.png'), np.hstack(panels))
    print(json.dumps({'classes': result['classes'], 'outlines': result['outlines']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
