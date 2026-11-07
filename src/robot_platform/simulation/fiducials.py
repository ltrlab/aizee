"""The printed fiducial tags on the task table: their codes, and where their corners are.

The labels are made by `scripts/fiducials/make_tag_labels.py` on `claude/table-fiducial-labels` (see TABLE_FIDUCIALS.md
there): ArUco DICT_4X4_50, IDs 0-7. A workcell file places them (simulation.workcell) and the task scene draws them
(simulation.task_scene).

A marker's frame is OpenCV's: x to the right of the printed code, y toward its top, z out of the label, origin at the
centre of the black square. Corners run in OpenCV's order: top-left, top-right, bottom-right, bottom-left.

Imports only the standard library, so scripts outside the development environment can use it.
"""

from __future__ import annotations

import math

# Each marker's inner cells, row by row from the top of the code and left to right along each row (1 white, 0 black),
# inside a one-cell black border. Copied from OpenCV 4.13 (cv2.aruco.generateImageMarker); scripts/sim/
# check_fiducial_codes.py checks this table against OpenCV.
CODES = {
    'DICT_4X4_50': {
        0: ('1011', '0101', '0011', '0010'),
        1: ('0000', '1111', '1001', '1010'),
        2: ('0011', '0011', '0010', '1101'),
        3: ('1001', '1001', '0100', '0110'),
        4: ('0101', '0100', '1001', '1110'),
        5: ('0111', '1001', '1100', '1101'),
        6: ('1001', '1110', '0010', '1110'),
        7: ('1100', '0100', '1111', '0010'),
    },
}
BORDER = 1  # Black cells around the code.


def grid(family):
    """Cells across a marker of `family`, its black border included."""
    return len(next(iter(CODES[family].values()))) + 2 * BORDER


def cells(family, tag_id):
    """The marker's cells, border included, as rows of 0 (black) and 1 (white) from the top of the code."""
    code, n = CODES[family][tag_id], grid(family)
    rows = [[0] * n for _ in range(n)]
    for r, row in enumerate(code):
        for c, bit in enumerate(row):
            rows[r + BORDER][c + BORDER] = int(bit)
    return rows


def white_runs(family, tag_id):
    """(row, first column, last column) of each run of white cells along a row, border included in the indices."""
    runs = []
    for r, row in enumerate(cells(family, tag_id)):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c + 1 < len(row) and row[c + 1]:
                    c += 1
                runs.append((r, start, c))
            c += 1
    return runs


def cell_center(family, square, row, column):
    """A cell's centre in the marker frame: (x, y), metres, for a black square `square` across."""
    n = grid(family)
    size = square / n
    return ((column - (n - 1) / 2) * size, ((n - 1) / 2 - row) * size)


def marker_axes(yaw, tilt=0.0):
    """A tag's marker axes (x, y, z) in the frame it is placed in (z up): turned `yaw` about z, then tipped `tilt`
    about its own x axis. Flat on a table, tilt is 0 and z points up; on an upright wall with the top of the code up,
    tilt is a quarter turn and z points out of the wall."""
    c, s, ct, st = math.cos(yaw), math.sin(yaw), math.cos(tilt), math.sin(tilt)
    return ((c, s, 0.0), (-s * ct, c * ct, st), (s * st, -c * st, ct))


def corners(square, x, y, z, yaw, tilt=0.0):
    """The black square's corners, in OpenCV's order, in the frame a tag is placed in: its centre at (x, y, z) and its
    marker axes as marker_axes(yaw, tilt) gives them."""
    (xx, xy, xz), (yx, yy, yz), _ = marker_axes(yaw, tilt)
    h = square / 2
    return [
        (x + xx * u + yx * v, y + xy * u + yy * v, z + xz * u + yz * v) for u, v in ((-h, h), (h, h), (h, -h), (-h, -h))
    ]
