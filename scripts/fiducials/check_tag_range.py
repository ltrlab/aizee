#!/usr/bin/env python3
"""Synthetic decode check for the table fiducial labels.

Lays a generated 62 mm label on a black table, renders it into a pinhole camera
view (supersampled, then area-averaged), adds lens blur, sensor noise and JPEG
compression, and runs OpenCV's default ArUco detector. A trial passes when the
detector returns the label's own ID and nothing else. Each trial sees the tag at
a given distance and tilt from the table normal, with a random in-plane
rotation and sub-pixel position.

This is a model, not a camera: no lens distortion, motion blur, glare or
exposure change. It ranks families and sizes and shows where decoding falls
off; it does not replace a check on the real head camera.

    python scripts/fiducials/check_tag_range.py [--trials 20] [--seed 20261017]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_tag_labels as labels  # noqa: E402

DISTANCES_M = (0.6, 0.9, 1.2, 1.5)
TILTS_DEG = (0, 45, 60)
FAMILIES = ('DICT_4X4_50', 'DICT_APRILTAG_25h9', 'DICT_APRILTAG_36h11')


def fx_for_hfov(width: int, hfov_deg: float) -> float:
    return width / 2 / math.tan(math.radians(hfov_deg) / 2)


# (description, width, height, fx, families compared)
VIEWS = (
    ('640x480, 69 deg spread over the row', 640, 480, fx_for_hfov(640, 69.0), FAMILIES),
    ('640x480, typical D435 4:3 colour crop', 640, 480, 616.0, FAMILIES[:1]),
    ('1920x1080, 69 deg', 1920, 1080, fx_for_hfov(1920, 69.0), FAMILIES[:1]),
)


def render_view(
    label: np.ndarray,
    geo: labels.Geometry,
    fx: float,
    width: int,
    height: int,
    distance: float,
    tilt_deg: float,
    rng: np.random.Generator,
) -> np.ndarray:
    ss = 4 if width <= 640 else 2
    tilt = math.radians(tilt_deg)
    camera = np.array([distance * math.sin(tilt), 0.0, distance * math.cos(tilt)])
    forward = -camera / distance
    right = np.cross([0.0, 1.0, 0.0], forward)
    right /= np.linalg.norm(right)
    rotation = np.stack([right, np.cross(forward, right), forward])  # world -> camera rows
    k = np.array([[fx * ss, 0, (width * ss - 1) / 2], [0, fx * ss, (height * ss - 1) / 2], [0, 0, 1]])
    plane_to_image = k @ np.column_stack([rotation[:, 0], rotation[:, 1], -rotation @ camera])

    metres_per_dot = labels.MM_PER_DOT / 1000
    # Shrink the 300 dpi label near the rendered scale first so the warp does not alias.
    shrink = min(1.0, 2.0 * fx * ss * metres_per_dot / distance)
    small = cv2.resize(label, None, fx=shrink, fy=shrink, interpolation=cv2.INTER_AREA) if shrink < 1 else label
    scale = metres_per_dot / shrink
    yaw = rng.uniform(0, 2 * math.pi)
    # Label rows run down the printed image, which is world -y seen from above; without the
    # flip the view would be a mirror image, which ArUco does not decode.
    to_plane = np.array(
        [
            [scale * math.cos(yaw), scale * math.sin(yaw), 0],
            [scale * math.sin(yaw), -scale * math.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    centre_px = (geo.quiet + geo.tag_dots / 2 - 0.5) * shrink
    offset = rng.uniform(-0.5, 0.5, 2) * distance / fx
    to_plane[:2, 2] = offset - (to_plane @ [centre_px, centre_px, 1])[:2]

    reflectance = 30 + 190 * (small.astype(np.float32) / 255)  # ink 30, paper 220
    big = cv2.warpPerspective(
        reflectance,
        plane_to_image @ to_plane,
        (width * ss, height * ss),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=20,
    )
    image = cv2.resize(big, (width, height), interpolation=cv2.INTER_AREA)
    image = cv2.GaussianBlur(image, (0, 0), 0.7)
    image = np.clip(image + rng.normal(0, 2.5, image.shape), 0, 255).astype(np.uint8)
    _, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE)


def decode_count(
    family: str, view: tuple, distance: float, tilt: float, trials: int, rng: np.random.Generator
) -> tuple[int, labels.Geometry]:
    _, width, height, fx, _ = view
    media = labels.MEDIA['62']
    geo = labels.choose_geometry(media, labels.dictionary(family).markerSize + 2, None)
    detector = cv2.aruco.ArucoDetector(labels.dictionary(family), cv2.aruco.DetectorParameters())
    hits = 0
    for trial in range(trials):
        tag_id = trial % 8
        label = np.asarray(labels.render_label(family, tag_id, media, geo).convert('L'))
        _, ids, _ = detector.detectMarkers(render_view(label, geo, fx, width, height, distance, tilt, rng))
        hits += ids is not None and ids.ravel().tolist() == [tag_id]
    return hits, geo


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--seed', type=int, default=20261017)
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)

    print(
        f'decoded / {args.trials} at {", ".join(f"{d} m" for d in DISTANCES_M)}; '
        f'tilt from the table normal; seed {args.seed}'
    )
    for view in VIEWS:
        name, width, height, fx, families = view
        print(f'\n{name} (fx {fx:.0f} px)')
        for family in families:
            for tilt in TILTS_DEG:
                counts = []
                for distance in DISTANCES_M:
                    hits, geo = decode_count(family, view, distance, tilt, args.trials, rng)
                    counts.append(f'{hits:3d}')
                cells_at = fx * geo.cell_mm / 1000 / 1.2
                print(
                    f'  {family:20s} {geo.tag_mm:4.1f} mm  tilt {tilt:2d}:{"".join(counts)}'
                    f'   ({cells_at:.1f} px per cell face-on at 1.2 m)'
                )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
