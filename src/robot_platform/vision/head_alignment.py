"""The head camera's alignment on the table's board (docs/development/HEADSET_3D_PLAN.md, Alignment):

    python -m robot_platform.vision.head_alignment RUN_FOLDER [--rig config/devices/minerva-head-alignment.json]
        [--wrists FOLDER]

Reads RUN_FOLDER/stills.jsonl (helm.head_alignment): stills of the board from the head camera, each with the neck's
measured CAD angles over its frames (the head's pose feed, neck.head_pose) and the intrinsics the camera published
with them. Fits the camera's mount in the head link it is fixed to and where the board lies in Minerva's frame: the
wrist calibration's solve (vision.wrist_calibration) with the intrinsics held at the camera's own (the D435's factory
calibration). Writes RUN_FOLDER/result.json.

A two-joint neck leaves two things to the rest of the fit: a zero error of the pan joint turns the whole head about
its axis, which the board's placement takes up, and the tilt's, which the mount takes up. So the head alone cannot
tell its pan zero from where the board lies; the arms' own fit of the board (the latest solved wrist calibration
under --wrists) can, and the result gives the difference, with the part of it about the pan axis.

Set OPENBLAS_NUM_THREADS=1 before running (Helm does).
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np

from robot_platform.models import ModelProfile
from robot_platform.neck.head_pose import ALIGNMENT_KIND, cad_mount
from .board import load_board
from .chain import Chain
from .geometry import invert, rotation, rotvec
from .wrist_calibration import CalibrationError, CameraSpec, Still, calibrate

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RIG = ROOT / 'config/devices/minerva-head-alignment.json'
CAMERA = 'head'
PAN = 'neck_z'
# Turned or tilted less than this over a run's stills, the head barely moves the camera about that joint's axis, and
# the mount's position along the other comes out poorly.
MIN_SPREAD_DEG = 10.


def _value(entry):
    return entry['value'] if isinstance(entry, dict) else entry


def load_rig(path=DEFAULT_RIG):
    """The alignment's settings: the head model's URDF and front, the camera's link and CAD origin (the head profile's
    `camera`), the board, and how a still is taken."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('kind') != 'minerva_head_alignment':
        raise ValueError('Expected a minerva_head_alignment file')
    profile = json.loads((ROOT / data['head_profile']).read_text(encoding='utf-8'))
    model = ModelProfile.load(ROOT / profile['model_profile'])
    camera = profile.get('camera') or {}
    if not camera.get('link') or len(camera.get('origin_m', ())) != 3:
        raise ValueError('The head profile names no camera link and origin')
    still = {key: float(_value(v)) for key, v in data['still'].items()}
    still['frames'] = int(still['frames'])
    return dict(urdf=model.urdf, front=model.front_direction, link=camera['link'], origin_m=camera['origin_m'],
                board=ROOT / data['board'], still=still)


def load_stills(folder):
    """The run's head stills (Still), with the intrinsics and head reference each was taken with."""
    stills, published, references = [], [], []
    for line in (Path(folder) / 'stills.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get('camera') != CAMERA:
            continue
        corners = {int(t): np.asarray(q, float).reshape(4, 2) for t, q in (record.get('tags') or {}).items()}
        stills.append(Still(CAMERA, record['joints'], corners, name=record['name']))
        published.append(record.get('intrinsics'))
        references.append(record.get('reference_sha256'))
    return stills, published, references


def held_camera(published):
    """Nine camera numbers and the image size from the intrinsics the camera published with every still: fx, fy, cx,
    cy, no distortion (the D435's colour stream has none). Refused when a still has none or they differ."""
    if not published or any(not isinstance(k, dict) for k in published):
        raise CalibrationError('A still has no intrinsics from the camera: the head camera publishes them since its '
                               'colour registration update; capture again')
    first = published[0]
    keys = ('fx', 'fy', 'cx', 'cy', 'width', 'height')
    try:
        values = [float(first[key]) for key in keys]
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError('The camera\'s published intrinsics lack fx, fy, cx, cy or the image size') from exc
    if not all(math.isfinite(v) for v in values) or min(values) <= 0:
        raise CalibrationError('The camera\'s published intrinsics are not positive and finite')
    if any(abs(float(k.get(key, math.nan)) - v) > 1e-6 for k in published[1:] for key, v in zip(keys, values)):
        raise CalibrationError('The camera\'s intrinsics changed between stills (another resolution?): start a new run')
    if any(abs(float(c)) > 0 for c in first.get('coeffs') or ()):
        raise CalibrationError('The camera publishes lens distortion, which this solve does not model')
    fx, fy, cx, cy, width, height = values
    return np.array([fx, fy, cx, cy, 0., 0., 0., 0., 0.]), (int(width), int(height))


def _joint_frame(chain, name):
    """The robot-frame pose of a joint's frame on the chain, all joints at zero: its origin and axis."""
    result = np.eye(4)
    for joint, kind, fixed, axis in chain.path:
        result = result @ fixed
        if joint == name:
            return result[:3, 3], result[:3, :3] @ axis
    raise ValueError(f'No joint {name} on the chain to {chain.tip}')


def _difference(a, b):
    """How far rigid transform b is from a: millimetres and degrees."""
    d = invert(a) @ b
    return dict(translation_mm=round(float(np.linalg.norm(a[:3, 3] - b[:3, 3])) * 1e3, 2),
                rotation_deg=round(math.degrees(float(np.linalg.norm(rotvec(d[:3, :3])))), 3))


def latest_wrist_placement(root):
    """(run name, 4x4) of the board's placement in Minerva's frame from the newest solved wrist calibration under
    `root` (helm.calibration's runs), its recommended model; None when there is none."""
    root = Path(root)
    runs = sorted(p for p in root.glob('*') if (p / 'result.json').is_file()) if root.is_dir() else []
    for run in reversed(runs):
        try:
            result = json.loads((run / 'result.json').read_text(encoding='utf-8'))
            model = (result.get('models') or {}).get(result.get('recommended')) or {}
            matrix = np.array(model['board']['placement']['matrix'], float)
        except (OSError, ValueError, KeyError, TypeError):
            continue
        if matrix.shape == (4, 4) and np.isfinite(matrix).all():
            return run.name, matrix
    return None


def against_arms(head_placement, arms_placement, chain, middle):
    """The head's placement of the board against the arms', at the board's `middle` (board frame): how far apart the
    two put it, and how much of that a pan zero error explains (a turn about the pan axis, through its origin). The
    rest is the head's kinematics against the arms' (the neck's place on the frame), or the board read differently."""
    d = arms_placement @ invert(head_placement)  # The head's idea of Minerva's frame to the arms'.
    origin, axis = _joint_frame(chain, PAN)
    turn = rotvec(d[:3, :3])
    pan = float(turn @ axis)
    head_point = (head_placement @ np.r_[middle, 1.])[:3]
    arms_point = (arms_placement @ np.r_[middle, 1.])[:3]
    panned = origin + rotation(axis * pan) @ (head_point - origin)  # The head's point, its pan zero taken out.
    return dict(board_mm=round(float(np.linalg.norm(arms_point - head_point)) * 1e3, 2),
                rotation_deg=round(math.degrees(float(np.linalg.norm(turn))), 3),
                pan_deg=round(math.degrees(pan), 3),
                other_mm=round(float(np.linalg.norm(arms_point - panned)) * 1e3, 2),
                other_deg=round(math.degrees(float(np.linalg.norm(turn - pan * axis))), 3))


def solve_run(folder, rig=DEFAULT_RIG, wrists=None):
    """Fit the run's head stills; writes and returns RUN_FOLDER/result.json's contents."""
    folder = Path(folder)
    config = load_rig(rig)
    board = load_board(config['board'])
    stills, published, references = load_stills(folder)
    result = dict(solved_at=datetime.now(timezone.utc).isoformat(), run=folder.name, stills=len(stills), notes=[],
                  link=config['link'])
    try:
        if len(set(references)) > 1:
            raise CalibrationError("The head's reference changed during the run: start a new run")
        camera, size = held_camera(published)
        chain = Chain(config['urdf'], config['link'])
        report = calibrate(board, [CameraSpec(CAMERA, chain, size)], stills, intrinsics={CAMERA: camera})
    except CalibrationError as exc:
        result['error'] = str(exc)
    else:
        result.update(reference_sha256=references[0], report=report,
                      **_findings(report, stills, chain, config, board, camera, wrists))
    (folder / 'result.json').write_text(json.dumps(result, indent=1), encoding='utf-8')
    return result


def _findings(report, stills, chain, config, board, camera, wrists):
    """What the fit says beyond its report: the mount against the CAD guess, how far the camera stood, the pixel
    error as millimetres at the board, where the pan axis is on the board (a tape measure can check it), whether the
    head turned and tilted enough, and the board against the arms' fit of it."""
    found, notes = {}, []
    mount = np.array(report['cameras'][CAMERA]['mount']['matrix'])
    placement = np.array(report['board']['placement']['matrix'])
    cad = cad_mount(chain.pose({name: 0. for name in chain.joints}), config['front'], config['origin_m'])
    found['cad'] = dict(matrix=np.round(cad, 7).tolist(), **_difference(cad, mount))
    # The camera's distance to the board's middle in each still, and the error it makes of a pixel there.
    middle = np.r_[np.mean([board.tags[t][:2] for t in board.tags], axis=0), 0.]
    distances = [float(np.linalg.norm((invert(chain.pose(s.joints) @ mount) @ placement @ np.r_[middle, 1.])[:3]))
                 for s in stills]
    found['distance_m'] = dict(median=round(float(np.median(distances)), 3), low=round(min(distances), 3),
                               high=round(max(distances), 3))
    focal = float(np.mean(camera[:2]))
    found['rms_mm'] = round(report['cameras'][CAMERA]['rms_px'] * float(np.median(distances)) / focal * 1e3, 2)
    origin, _ = _joint_frame(chain, PAN)
    found['pan_axis_in_board_mm'] = [round(float(v) * 1e3, 1) for v in (invert(placement) @ np.r_[origin, 1.])[:3]]
    spread = {}
    for name in chain.joints:
        values = [float(s.joints[name]) for s in stills]
        spread[name] = round(math.degrees(max(values) - min(values)), 1)
        if spread[name] < MIN_SPREAD_DEG:
            notes.append(f'{name} moved only {spread[name]}° over the stills: {"turn" if name == PAN else "tilt"} '
                         f'the head more between them (at least {MIN_SPREAD_DEG:.0f}°)')
    found['spread_deg'] = spread
    arms = latest_wrist_placement(wrists) if wrists else None
    if arms is not None:
        found['arms'] = dict(run=arms[0], **against_arms(placement, arms[1], chain, middle))
    else:
        notes.append("No solved wrist calibration to compare: the head alone cannot tell its pan zero from where the "
                     "board lies")
    found['notes'] = notes
    return found


def summary(result):
    """What Helm's page shows of a solve."""
    if result.get('error'):
        return dict(error=result['error'], solved_at=result.get('solved_at'), stills=result.get('stills'))
    report, camera = result['report'], result['report']['cameras'][CAMERA]
    cross = (report.get('cross_validation') or {}).get('cameras', {}).get(CAMERA, {})
    placement = report['board']['placement']
    return dict(solved_at=result['solved_at'], run=result['run'], stills=len(camera['stills']),
                rms_px=camera['rms_px'], holdout_px=cross.get('rms_px'), rms_mm=result['rms_mm'],
                outliers=[s['name'] for s in camera['stills'] if s.get('outlier')],
                mount_mm=[round(v * 1e3, 1) for v in camera['mount']['xyz_m']], mount_rpy_deg=camera['mount']['rpy_deg'],
                mount_sigma_mm=camera['mount'].get('sigma_xyz_mm'),
                mount_sigma_deg=camera['mount'].get('sigma_rotation_deg'),
                cad=dict(translation_mm=result['cad']['translation_mm'], rotation_deg=result['cad']['rotation_deg']),
                board_mm=[round(v * 1e3, 1) for v in placement['xyz_m']], board_rpy_deg=placement['rpy_deg'],
                pan_axis_in_board_mm=result['pan_axis_in_board_mm'], distance_m=result['distance_m'],
                spread_deg=result['spread_deg'], arms=result.get('arms'), notes=result.get('notes', []))


def alignment_document(result, *, saved_at):
    """The head's camera file from a solve's result.json (helm.head_alignment saves it): the mount neck.head_pose reads,
    the board's place in Minerva's frame, and how well they fit."""
    report = result['report']
    camera, cross = report['cameras'][CAMERA], (report.get('cross_validation') or {}).get('cameras', {})
    return dict(schema=1, kind=ALIGNMENT_KIND, link=result['link'], reference_sha256=result['reference_sha256'],
                mount=camera['mount'], board_in_robot=report['board']['placement'], intrinsics=camera['intrinsics'],
                fit=dict(rms_px=camera['rms_px'], holdout_px=cross.get(CAMERA, {}).get('rms_px'),
                         rms_mm=result['rms_mm'], stills=len(camera['stills'])),
                cad=dict(translation_mm=result['cad']['translation_mm'], rotation_deg=result['cad']['rotation_deg']),
                arms=result.get('arms'), run=result['run'], solved_at=result['solved_at'], saved_at=saved_at)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('run', type=Path)
    parser.add_argument('--rig', type=Path, default=DEFAULT_RIG)
    parser.add_argument('--wrists', type=Path, help="The wrist calibration's runs, to compare the board's placement")
    args = parser.parse_args(argv)
    result = solve_run(args.run, args.rig, args.wrists)
    print(json.dumps(summary(result), indent=1))
    return 1 if result.get('error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
