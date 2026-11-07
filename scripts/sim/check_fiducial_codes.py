#!/usr/bin/env python3
"""Check the task scene's fiducial codes (robot_platform.simulation.fiducials.CODES) against OpenCV.

For every marker in the table: its cells must equal OpenCV's own drawing of that ID, and OpenCV's detector must read
the table's cells, drawn with a white quiet zone, as that ID. Needs OpenCV 4.7 or newer (cv2.aruco), which the
development environment does not include, so run it with a Python that has OpenCV:

    python scripts/sim/check_fiducial_codes.py

Prints one line per marker; exits 1 if any differs.
"""

from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from robot_platform.simulation.fiducials import CODES, cells, grid  # noqa: E402

PIXELS_PER_CELL = 20


def main() -> int:
    failures = 0
    for family, codes in CODES.items():
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        for tag_id in sorted(codes):
            table = np.array(cells(family, tag_id))
            opencv = (cv2.aruco.generateImageMarker(dictionary, tag_id, grid(family), borderBits=1) > 127).astype(int)
            image = np.kron(table * 255, np.ones((PIXELS_PER_CELL, PIXELS_PER_CELL))).astype(np.uint8)
            image = cv2.copyMakeBorder(image, *[2 * PIXELS_PER_CELL] * 4, cv2.BORDER_CONSTANT, value=255)
            _, ids, _ = detector.detectMarkers(image)
            read = [] if ids is None else ids.ravel().tolist()
            ok = np.array_equal(table, opencv) and read == [tag_id]
            failures += not ok
            print(
                f'{family} ID {tag_id}: cells {"match" if np.array_equal(table, opencv) else "DIFFER"}, '
                f'detector reads {read} -> {"ok" if ok else "FAIL"}'
            )
    print(f'OpenCV {cv2.__version__}: {"all match" if not failures else f"{failures} failed"}')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
