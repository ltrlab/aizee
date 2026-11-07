#!/usr/bin/env python3
"""Render fiducial tag labels for the Brother QL-800 at exactly 300 dpi.

Each label carries one ArUco or AprilTag marker drawn with a whole number of
printer dots per cell, a white quiet zone of at least one cell on every side,
the tag ID, family and black-square edge length, and an L-shaped scale whose
arms are 10 mm across and along the tape. Printed at 100% the black square has
exactly the stated size, so a ruler on the square or the scale shows whether
anything in the print path scaled it.

The media table holds the printable width in dots for each DK roll. The default
cell is the largest whole number of dots that leaves a one-cell quiet zone on
each side and 1 mm of spare width.

    python scripts/fiducials/make_tag_labels.py                 # 62 mm, DICT_4X4_50, IDs 0-7
    python scripts/fiducials/make_tag_labels.py --media 29 --ids 0-3

Writes build/fiducials/<media>-<family>-c<cell>/idNN.png (1-bit, 300 dpi) and
labels.pdf (one exact-size page per label), and checks that every label decodes
to its own ID with the stated corner geometry. Needs numpy, Pillow and
OpenCV 4.7 or newer (cv2.aruco).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
DPI = 300
MM_PER_DOT = 25.4 / DPI
SPARE_DOTS = 12  # 1 mm of printable width kept free for the tape's side-to-side play
EDGE_DOTS = 12  # text and scale stay 1 mm inside the printable edge
SCALE_DOTS = round(10 / MM_PER_DOT)  # 118 dots, 9.99 mm
SCALE_BAR_DOTS = 6
HEAD_CAMERA_HFOV_DEG = 69.0  # RealSense D435 colour, spread over the 640 px row (conservative)
HEAD_CAMERA_WIDTH_PX = 640
HEAD_CAMERA_DISTANCES_M = (0.6, 0.9, 1.2)


@dataclass(frozen=True)
class Media:
    rolls: str
    width_dots: int  # printable dots across the tape
    max_length_dots: int | None  # printable length of a die-cut label; None for continuous tape


# Printable area at 300 dpi per DK roll, from the QL-800 raster command reference
# (the same table brother_ql uses).
MEDIA = {
    '62': Media('DK-22205 or DK-22251 62 mm continuous', 696, None),
    '62x100': Media('DK-11202 62 x 100 mm die-cut', 696, 1109),
    '50': Media('DK-22223 50 mm continuous', 554, None),
    '38': Media('DK-22225 38 mm continuous', 413, None),
    '29': Media('DK-22210 29 mm continuous', 306, None),
    '29x90': Media('DK-11201 29 x 90 mm die-cut', 306, 991),
}

# OpenCV dictionary name -> label text. Fewer cells across give larger cells for a given width.
FAMILIES = {
    'DICT_4X4_50': 'ArUco DICT_4X4_50',
    'DICT_5X5_50': 'ArUco DICT_5X5_50',
    'DICT_6X6_50': 'ArUco DICT_6X6_50',
    'DICT_APRILTAG_16h5': 'AprilTag tag16h5',
    'DICT_APRILTAG_25h9': 'AprilTag tag25h9',
    'DICT_APRILTAG_36h11': 'AprilTag tag36h11',
}


@dataclass(frozen=True)
class Geometry:
    grid: int  # cells across the marker, black border included
    cell: int  # printer dots per cell
    quiet: int  # white dots between the black border and the label's left edge

    @property
    def tag_dots(self) -> int:
        return self.grid * self.cell

    @property
    def cell_mm(self) -> float:
        return self.cell * MM_PER_DOT

    @property
    def tag_mm(self) -> float:
        return self.tag_dots * MM_PER_DOT


def dictionary(family: str) -> cv2.aruco.Dictionary:
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))


def choose_geometry(media: Media, grid: int, cell: int | None) -> Geometry:
    largest = (media.width_dots - SPARE_DOTS) // (grid + 2)
    cell = largest if cell is None else cell
    if cell < 1 or (grid + 2) * cell > media.width_dots:
        raise SystemExit(
            f'{cell} dots per cell leaves no one-cell quiet zone across {media.width_dots} dots; '
            f'the default is {largest}')
    return Geometry(grid, cell, (media.width_dots - grid * cell) // 2)


def tag_image(family: str, tag_id: int, geo: Geometry) -> np.ndarray:
    """The marker with exactly geo.cell x geo.cell dots per cell (0 black, 255 white)."""
    cells = cv2.aruco.generateImageMarker(dictionary(family), tag_id, geo.grid, borderBits=1)
    return np.repeat(np.repeat(cells, geo.cell, axis=0), geo.cell, axis=1)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in (('arialbd.ttf', 'DejaVuSans-Bold.ttf') if bold else ('arial.ttf', 'DejaVuSans.ttf')):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


def fitted_font(text: str, size: int, width: int, bold: bool) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    while size > 12 and font(size, bold).getlength(text) > width:
        size -= 1
    chosen = font(size, bold)
    if chosen.getlength(text) > width:
        raise SystemExit(f'"{text}" does not fit across {width} dots')
    return chosen


def render_label(family: str, tag_id: int, media: Media, geo: Geometry) -> Image.Image:
    width = media.width_dots
    room = width - 2 * EDGE_DOTS
    title_size = max(18, min(40, round(width * 0.055)))
    body_size = max(16, min(30, round(width * 0.043)))
    lines = [(f'ID {tag_id}  {FAMILIES[family]}', title_size, True),
             (f'black square {geo.tag_mm:.2f} mm', body_size, False),
             ('text end toward robot', body_size, False)]
    fonts = [fitted_font(text, size, room, bold) for text, size, bold in lines]
    advances = [round(size * 1.3) for _, size, _ in lines]
    text_w = math.ceil(max(f.getlength(text) for f, (text, _, _) in zip(fonts, lines)))
    text_h = sum(advances)
    gap = 2 * EDGE_DOTS

    # The tag sits in a white square as wide as the printable area; text and scale follow
    # below its quiet zone so they never touch the one-cell border the detector needs.
    text_y = 2 * geo.quiet + geo.tag_dots
    if text_w + gap + SCALE_DOTS <= room:
        block_h = max(text_h, SCALE_DOTS)
        text_x = (width - (text_w + gap + SCALE_DOTS)) // 2
        scale_x, scale_y = text_x + text_w + gap, text_y + (block_h - SCALE_DOTS) // 2
        height = text_y + block_h + 2 * EDGE_DOTS
    else:
        text_x = (width - text_w) // 2
        scale_x, scale_y = (width - SCALE_DOTS) // 2, text_y + text_h + EDGE_DOTS
        height = scale_y + SCALE_DOTS + 2 * EDGE_DOTS
    if media.max_length_dots is not None and height > media.max_length_dots:
        raise SystemExit(f'label is {height} dots long; the die-cut label prints {media.max_length_dots}')

    canvas = Image.new('L', (width, height), 255)
    canvas.paste(Image.fromarray(tag_image(family, tag_id, geo)), (geo.quiet, geo.quiet))
    draw = ImageDraw.Draw(canvas)
    y = text_y
    for f, (text, _, _), advance in zip(fonts, lines, advances):
        draw.text((text_x, y), text, font=f, fill=0, anchor='lt')
        y += advance
    # L-shaped scale: both arms measure SCALE_DOTS from the shared outer corner (bottom left).
    bottom = scale_y + SCALE_DOTS
    draw.rectangle([scale_x, bottom - SCALE_BAR_DOTS, scale_x + SCALE_DOTS - 1, bottom - 1], fill=0)
    draw.rectangle([scale_x, scale_y, scale_x + SCALE_BAR_DOTS - 1, bottom - 1], fill=0)
    draw.text((scale_x + SCALE_BAR_DOTS + 8, scale_y), '10 mm', font=font(body_size), fill=0, anchor='lt')
    # Thermal printing is black or white; threshold the anti-aliased text instead of dithering it.
    return canvas.point(lambda v: 255 if v >= 128 else 0).convert('1', dither=Image.Dither.NONE)


def check_label(label: Image.Image, family: str, tag_id: int, geo: Geometry) -> None:
    pixels = np.asarray(label.convert('L'))
    x0 = y0 = geo.quiet
    x1, y1 = x0 + geo.tag_dots, y0 + geo.tag_dots
    if not np.array_equal(pixels[y0:y1, x0:x1], tag_image(family, tag_id, geo)):
        raise SystemExit(f'ID {tag_id}: the tag is not drawn dot for dot')
    q = geo.cell
    ring = pixels[y0 - q:y1 + q, x0 - q:x1 + q].copy()
    ring[q:-q, q:-q] = 255
    if ring.min() != 255:
        raise SystemExit(f'ID {tag_id}: the one-cell quiet zone is not white')
    detector = cv2.aruco.ArucoDetector(dictionary(family), cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(pixels)
    found = [] if ids is None else ids.ravel().tolist()
    if found != [tag_id]:
        raise SystemExit(f'ID {tag_id}: detector found {found}')
    # OpenCV puts pixel centres on integers, so the square's outer edges sit half a dot out.
    expected = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float) - 0.5
    error = np.abs(corners[0].reshape(4, 2) - expected).max()
    if error > 1.5:
        raise SystemExit(f'ID {tag_id}: corners are {error:.1f} dots from the drawn square')


def parse_ids(text: str) -> list[int]:
    ids: list[int] = []
    for part in text.split(','):
        low, _, high = part.strip().partition('-')
        ids.extend(range(int(low), int(high or low) + 1))
    return ids


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--media', choices=sorted(MEDIA), default='62', help='DK roll (default 62 mm continuous)')
    parser.add_argument('--family', choices=sorted(FAMILIES), default='DICT_4X4_50')
    parser.add_argument('--ids', default='0-7', help='tag IDs, for example 0-3,6 (default 0-7)')
    parser.add_argument('--cell-dots', type=int, help='printer dots per tag cell (default: largest that fits)')
    parser.add_argument('--out', type=Path, default=ROOT / 'build' / 'fiducials')
    args = parser.parse_args()

    media = MEDIA[args.media]
    vocabulary = dictionary(args.family)
    ids = parse_ids(args.ids)
    if not ids or min(ids) < 0 or max(ids) >= vocabulary.bytesList.shape[0]:
        raise SystemExit(f'{args.family} has IDs 0-{vocabulary.bytesList.shape[0] - 1}')
    geo = choose_geometry(media, vocabulary.markerSize + 2, args.cell_dots)
    out = args.out / f'{args.media}-{args.family}-c{geo.cell}'
    out.mkdir(parents=True, exist_ok=True)

    labels = []
    for tag_id in ids:
        label = render_label(args.family, tag_id, media, geo)
        check_label(label, args.family, tag_id, geo)
        label.save(out / f'id{tag_id:02d}.png', dpi=(DPI, DPI))
        labels.append(label)
    labels[0].save(out / 'labels.pdf', save_all=True, append_images=labels[1:], resolution=float(DPI))

    w, h = labels[0].size
    fx = HEAD_CAMERA_WIDTH_PX / 2 / math.tan(math.radians(HEAD_CAMERA_HFOV_DEG) / 2)
    per_cell = ', '.join(f'{fx * geo.cell_mm / 1000 / d:.1f} at {d} m' for d in HEAD_CAMERA_DISTANCES_M)
    print(f'media     {args.media}: {media.rolls}, {media.width_dots} printable dots across')
    print(f'family    {FAMILIES[args.family]}, {geo.grid} x {geo.grid} cells with the black border')
    print(f'cell      {geo.cell} dots = {geo.cell_mm:.3f} mm')
    print(f'square    {geo.tag_dots} dots = {geo.tag_mm:.2f} mm black edge, quiet zone {geo.quiet} dots '
          f'= {geo.quiet / geo.cell:.2f} cells at the sides')
    print(f'label     {w} x {h} dots = {w * MM_PER_DOT:.2f} x {h * MM_PER_DOT:.2f} mm')
    print(f'head cam  fx {fx:.0f} px ({HEAD_CAMERA_HFOV_DEG:g} deg over {HEAD_CAMERA_WIDTH_PX} px), '
          f'face-on px per cell: {per_cell}')
    print(f'wrote     {len(labels)} labels (IDs {", ".join(map(str, ids))}) to {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
