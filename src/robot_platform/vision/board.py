"""The table's ArUco board: which tags, where they lie, and their corners.

A board file (config/devices/minerva-table-board.json) gives each tag's black-square centre in the board frame, flat
on the table (z = 0), and how well the layout is known. A tag's corners run in OpenCV's order, the code's top-left,
top-right, bottom-right and bottom-left, with the marker's x to the right of the printed code and y toward its top.
Tags not in the file (the box's tags 6 and 7) are not part of the board.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

DEFAULT_BOARD = Path(__file__).resolve().parents[3] / 'config' / 'devices' / 'minerva-table-board.json'


@dataclass(frozen=True)
class Board:
    family: str
    square: float
    tags: dict  # id -> (x, y, yaw): the black square's centre (metres) and its marker's turn about z (radians)
    sigma_m: float
    sigma_yaw: float

    def corners(self, tag, offset=(0., 0., 0.)):
        """(4, 3) corners of a tag, OpenCV's order, its centre moved by (dx, dy) and turned by dyaw."""
        x, y, yaw = self.tags[tag]
        dx, dy, dyaw = offset
        c, s, h = math.cos(yaw + dyaw), math.sin(yaw + dyaw), self.square / 2
        return np.array([[x + dx + c * u - s * v, y + dy + s * u + c * v, 0.]
                         for u, v in ((-h, h), (h, h), (h, -h), (-h, -h))])

    def layout(self, offsets=None):
        """{tag: (4, 3) corners} with optional per-tag (dx, dy, dyaw) offsets."""
        offsets = offsets or {}
        return {tag: self.corners(tag, offsets.get(tag, (0., 0., 0.))) for tag in self.tags}


def board_from_dict(data):
    if data.get('kind') != 'minerva_table_board':
        raise ValueError('Expected a minerva_table_board file')
    def value(key):
        entry = data[key]
        return entry['value'] if isinstance(entry, dict) else entry
    family, square = str(value('family')), float(value('square_m'))
    if not 0 < square < .5:
        raise ValueError('square_m must be a size in metres')
    tags = {}
    for key, tag in data['tags'].items():
        x, y = (float(v) for v in tag['center_m'])
        yaw = math.radians(float(tag.get('yaw_deg', 0.)))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'Tag {key}: center_m and yaw_deg must be finite')
        tags[int(key)] = (x, y, yaw)
    if len(tags) < 2:
        raise ValueError('A board needs at least two tags')
    sigma_m, sigma_yaw = float(value('placement_sigma_m')), math.radians(float(value('yaw_sigma_deg')))
    if not (sigma_m > 0 and sigma_yaw > 0):
        raise ValueError('The layout sigmas must be positive')
    return Board(family, square, tags, sigma_m, sigma_yaw)


def load_board(path=DEFAULT_BOARD):
    return board_from_dict(json.loads(Path(path).read_text(encoding='utf-8')))
