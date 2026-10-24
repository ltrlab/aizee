"""Solve a calibration run's stills:

    python -m robot_platform.vision.solve RUN_FOLDER [--rig config/devices/minerva-wrist-cameras.json]

Reads RUN_FOLDER/stills.jsonl (helm.calibration) and fits the wrist cameras twice: on the URDF's kinematics as they
are ('plain') and with the arms' joint zero offsets fitted ('joint_offsets', J1-J5; the mount absorbs J6). The one
with the lower cross-validated error is `recommended`. A third, plain fit puts each camera on J6's parent link
instead: if that fits better, the camera does not turn with J6 (`link_check`). Writes RUN_FOLDER/result.json.

Set OPENBLAS_NUM_THREADS=1 before running (Helm does): multithreaded BLAS only slows problems this small.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np

from .board import load_board
from .chain import Chain
from .rig import DEFAULT_RIG, load_rig
from .wrist_calibration import CalibrationError, CameraSpec, Still, calibrate


def load_stills(folder):
    stills = []
    for line in (Path(folder) / 'stills.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        corners = {int(t): np.asarray(q, float).reshape(4, 2) for t, q in (record.get('tags') or {}).items()}
        stills.append(Still(record['camera'], record['joints'], corners, name=record['name']))
    return stills


def robot_in_board(report, specs):
    """Where the robot sits on the board, as a tape measure would check it: each arm's J1 axis (its shoulder joint's
    origin) in the board frame, mm (x along the near side from tag 0's outer corner toward tag 1, y toward the far
    side, so the robot's side is negative, z above the table), and the robot's turn about the vertical against the
    nominal layout (its left along the board's -x), degrees, counterclockwise from above."""
    board_in_base = np.array(report['board']['placement']['matrix'])
    base_in_board = np.linalg.inv(board_in_base)
    shoulders = {}
    for spec in specs:
        name, kind, fixed, _ = spec.chain.path[0]
        if kind == 'fixed':
            continue
        point = base_in_board @ np.r_[fixed[:3, 3], 1.]
        shoulders[name] = [round(float(v) * 1e3, 1) for v in point[:3]]
    left = base_in_board[:3, :3] @ np.array([1., 0., 0.])
    yaw = math.degrees(math.atan2(left[1], left[0])) - 180.
    return dict(j1_mm=shoulders, yaw_deg=round((yaw + 180.) % 360. - 180., 2))


def _holdout(report):
    cameras = (report.get('cross_validation') or {}).get('cameras') or {}
    values = [c['rms_px'] for c in cameras.values()]
    return float(np.mean(values)) if values else float(report['fit']['rms_px'])


def solve_run(folder, rig=DEFAULT_RIG):
    config = load_rig(rig)
    board = load_board(config['board'])
    stills = load_stills(folder)
    names = [name for name in config['cameras'] if any(s.camera == name for s in stills)]
    specs = [CameraSpec(name, Chain(config['urdf'], config['cameras'][name]['link']),
                        tuple(config['cameras'][name]['size'])) for name in names]
    models, notes = {}, []
    for key, joint_offsets in (('plain', None), ('joint_offsets', True)):
        try:
            models[key] = calibrate(board, specs, stills, joint_offsets=joint_offsets)
            models[key]['robot_in_board'] = robot_in_board(models[key], specs)
        except CalibrationError as exc:
            models[key] = {'error': str(exc)}
    solved = {key: report for key, report in models.items() if 'error' not in report}
    recommended = min(solved, key=lambda key: _holdout(solved[key])) if solved else None
    link_check = {}
    if 'plain' in solved:
        try:
            parents = [CameraSpec(spec.name, Chain(config['urdf'], spec.chain.before_last_joint()), spec.size)
                       for spec in specs]
            other = calibrate(board, parents, stills, folds=0)
            for spec, alternative in zip(specs, parents):
                link_check[spec.name] = dict(
                    link=spec.chain.tip, rms_px=solved['plain']['cameras'][spec.name]['rms_px'],
                    parent_link=alternative.chain.tip, parent_rms_px=other['cameras'][spec.name]['rms_px'])
                if link_check[spec.name]['parent_rms_px'] < .8 * link_check[spec.name]['rms_px']:
                    notes.append(f'{spec.name} fits better on {alternative.chain.tip} (before J6): check the mount')
        except CalibrationError as exc:
            link_check = {'error': str(exc)}
    result = dict(solved_at=datetime.now(timezone.utc).isoformat(), run=Path(folder).name, stills=len(stills),
                  models=models, recommended=recommended, link_check=link_check, notes=notes)
    (Path(folder) / 'result.json').write_text(json.dumps(result, indent=1), encoding='utf-8')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('run', type=Path)
    parser.add_argument('--rig', type=Path, default=DEFAULT_RIG)
    args = parser.parse_args(argv)
    result = solve_run(args.run, args.rig)
    for key, report in result['models'].items():
        if 'error' in report:
            print(f'{key}: {report["error"]}')
            continue
        for name, camera in report['cameras'].items():
            cross = (report.get('cross_validation') or {}).get('cameras', {}).get(name, {})
            print(f'{key} {name}: {camera["rms_px"]} px (held out {cross.get("rms_px")}), mount '
                  f'{[round(v * 1e3, 1) for v in camera["mount"]["xyz_m"]]} mm {camera["mount"]["rpy_deg"]} deg')
    print('recommended:', result['recommended'], *result['notes'], sep='\n  ')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
