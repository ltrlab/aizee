"""helm --sim with this checkout's actuator file, replayed on the real teleop it was fitted on, reproduces the
2026-10-15 report's held-out scores (docs/development/SIM2REAL.md).

The recordings, the report and Helm's operator runs folder (with the verified directions the fit used) are not in
the repository, so this is not part of the development gate. Run it by name:

    .venv-dev/Scripts/python.exe -m pytest tests/evidence/test_sim2real_20261015.py

MINERVA_EVIDENCE (default C:/Users/ltr/Workspace/minerva-evidence, as mirrored there) and MINERVA_OPERATOR_RUNS
locate them.
"""
import hashlib
import json
import os
from pathlib import Path

import pytest

from robot_platform.characterization import sim2real

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
ACTUATORS = DEVICES / 'simulation' / 'minerva-production-01-actuators.json'
EVIDENCE = Path(os.environ.get('MINERVA_EVIDENCE', 'C:/Users/ltr/Workspace/minerva-evidence'))
RUNS = Path(os.environ.get('MINERVA_OPERATOR_RUNS',
                           'C:/Users/ltr/.codex/worktrees/dev-baseline/aizee/build/manual-tuning/operator-runs'))
REPORT = EVIDENCE / 'analysis/sim2real-20261015/report.json'
ARM = [f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)]

pytestmark = pytest.mark.skipif(not REPORT.is_file() or not RUNS.is_dir(),
                                reason='needs the 2026-10-15 evidence and the operator runs folder')


@pytest.fixture(scope='module')
def replayed():
    document = json.loads(ACTUATORS.read_text(encoding='utf-8'))
    assert hashlib.sha256(REPORT.read_bytes()).hexdigest() == document['source']['report']['sha256']
    report = json.loads(REPORT.read_text(encoding='utf-8'))
    built = sim2real.simulator(DEVICES / 'minerva-joint-calibration.json', RUNS, actuators=ACTUATORS)
    # The files the fit was made against: the servo calibration and the verified directions, so every arm joint's
    # gravity scale applies.
    assert built['identity']['servo_sha256'] == document['source']['servo']['sha256']
    assert built['identity']['directions'] == document['source']['directions']
    assert built['identity']['actuators']['used'] == ARM and set(built['identity']['actuators']) == {
        'path', 'sha256', 'used'}
    recordings = [EVIDENCE / 'jetson/evidence' / Path(r['session'].replace('\\', '/')).name
                  for r in document['source']['recordings']]
    comparison = sim2real.compare(recordings, built, ARM, calibrate_joints=False)
    assert [r['samples_sha256'] for r in comparison['recordings']] == [
        r['samples_sha256'] for r in document['source']['recordings']]
    return document, report, comparison


@pytest.mark.parametrize('motor', ARM)
def test_the_simulator_with_its_actuator_file_reproduces_the_reports_held_out_scores(replayed, motor):
    """The simulator as helm --sim builds it with the file (its plant's delay and gravity scale included) scores on
    the held-out real windows what the report scored for the calibration, to rounding."""
    document, report, comparison = replayed
    simulated = comparison['joints'][motor]['current_simulator']
    calibrated = report['joints'][motor]['calibrated']['holdout']
    assert document['motors'][motor]['held_out']['calibrated'] == calibrated
    for part in ('all', 'moving', 'holding'):
        assert simulated[part]['windows'] == calibrated[part]['windows']
        for key in ('rms_mrad_p50', 'max_mrad_p50', 'max_mrad_p95'):
            assert simulated[part][key] == pytest.approx(calibrated[part][key], rel=1e-9, abs=1e-12), (part, key)
