"""The wrist camera rig file (config/devices/minerva-wrist-cameras.json): the URDF and board, which link each camera
is fixed to and its image size, and how Helm takes a still."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RIG = ROOT / 'config/devices/minerva-wrist-cameras.json'


def _value(entry):
    return entry['value'] if isinstance(entry, dict) else entry


def load_rig(path=DEFAULT_RIG):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('kind') != 'minerva_wrist_cameras':
        raise ValueError('Expected a minerva_wrist_cameras file')
    still = {key: float(_value(v)) for key, v in data['still'].items()}
    still['frames'] = int(still['frames'])
    for name, camera in data['cameras'].items():
        if camera.get('arm') not in ('left', 'right') or not camera.get('link') or len(camera.get('size', ())) != 2:
            raise ValueError(f'{name}: give its arm (left or right), link and size')
    return dict(urdf=ROOT / data['urdf'], board=ROOT / data['board'], cameras=data['cameras'], still=still)
