"""Reproducible, hardware-free development gate. Run from any directory."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid
import venv
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / '.venv-dev' / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
CORE = [
    'tests/unit/test_approach.py', 'tests/unit/test_joint_box_bound.py',
    'tests/unit/test_runtime_collision.py', 'tests/contract/test_guarded_controller.py',
    'tests/contract/test_collision_envelope.py', 'tests/contract/test_motion_envelope.py',
    'tests/contract/test_guarded_lifecycle_session.py', 'tests/contract/test_rehearsal.py',
    'tests/unit/test_mujoco_plant.py', 'tests/unit/test_physics_audit.py',
    'tests/contract/test_mujoco_receiver.py',
    'tests/contract/test_direct_teleop.py',
    'tests/contract/test_manual_teleop.py', 'tests/unit/test_manual_tuning.py',
    'tests/unit/test_manual_teleop_lifecycle.py', 'tests/unit/test_manual_tuning_api.py',
    'tests/unit/test_manual_tuning_rejections.py',
    'tests/unit/test_teleop_ui_budget.py', 'tests/unit/test_helm_recording.py',
    'tests/unit/test_teleop_recording.py',
    'tests/unit/test_direct_teleop.py', 'tests/unit/test_direct_latency.py', 'tests/unit/test_leader_bridge.py',
    'tests/unit/test_qualify_direct_motion.py', 'tests/unit/test_position_only_trial.py',
    'tests/unit/test_teleop_lifecycle.py', 'tests/unit/test_guarded_config.py',
    'tests/unit/test_helm_teleop.py', 'tests/unit/test_live_receiver.py',
    'tests/contract/test_direct_benchmark.py', 'tests/unit/test_benchmark_direct_loop.py',
    'tests/unit/test_direct_readiness.py', 'tests/unit/test_ssh_worktree.py',
    'tests/unit/test_minerva_check.py', 'tests/unit/test_helm_check.py',
    'tests/unit/test_ops_release.py', 'tests/unit/test_joint_calibration.py',
    'tests/contract/test_helm_settings.py', 'tests/unit/test_helm_settings_api.py',
    'tests/unit/test_helm_telemetry.py', 'tests/unit/test_helm_app.py', 'tests/unit/test_helm_live_view.py', 'tests/unit/test_qualify_manual_tuning.py', 'tests/unit/test_plain_data.py',
    'tests/unit/test_joint_capture_preflight.py', 'tests/unit/test_leader_link.py', 'tests/unit/test_helm_edge.py',
    'tests/unit/test_openrb_leader_setup.py',
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture(*args):
    return subprocess.check_output(args, cwd=ROOT, text=True, encoding='utf-8').strip()


def source_identity():
    # Includes tracked and untracked source, excluding ignored build outputs.
    paths = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others',
                                     '--exclude-standard'], cwd=ROOT).decode().split('\0')
    digest = hashlib.sha256()
    for name in sorted(set(filter(None, paths))):
        path = ROOT / name
        digest.update(name.encode())
        digest.update(bytes.fromhex(sha(path)) if path.is_file() else b'<deleted>')
    return {'commit': capture('git', 'rev-parse', 'HEAD'),
            'branch': capture('git', 'branch', '--show-current'),
            'status': capture('git', 'status', '--porcelain'),
            'content_sha256': digest.hexdigest()}


def rust_environment():
    env = os.environ.copy()
    tool_root = Path(env.get('ROBOT_RUST_TOOLCHAIN_ROOT', ROOT / 'rust/target/jetson-toolchain'))
    if tool_root.is_dir():
        env['CARGO_HOME'] = str(tool_root / 'cargo-home')
        env['RUSTUP_HOME'] = str(tool_root / 'rustup-home')
        env['PATH'] = str(tool_root / 'cargo-home/bin') + os.pathsep + env['PATH']
    if not shutil.which('cargo', path=env['PATH']):
        raise RuntimeError('Install Rust 1.98.1, or set ROBOT_RUST_TOOLCHAIN_ROOT to an existing toolchain root')
    env['CARGO_TARGET_DIR'] = str(ROOT / 'rust/target')
    env['ROBOT_CONTROL_HARNESS'] = str(ROOT / 'rust/target/debug' / (
        'control_harness.exe' if os.name == 'nt' else 'control_harness'))
    lib = 'robot_collision.dll' if os.name == 'nt' else (
        'librobot_collision.dylib' if sys.platform == 'darwin' else 'librobot_collision.so')
    env['ROBOT_COLLISION_LIBRARY'] = str(ROOT / 'rust/target/release' / lib)
    return env


def setup():
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('The development lock targets CPython 3.12. Run setup with Python 3.12.')
    subprocess.run(['git', 'lfs', 'pull'], cwd=ROOT, check=True)
    if not PYTHON.exists():
        venv.EnvBuilder(with_pip=True).create(PYTHON.parent.parent)
    if not capture(str(PYTHON), '--version').startswith('Python 3.12.'):
        raise RuntimeError('Existing .venv-dev uses another Python version; recreate it with Python 3.12')
    subprocess.run([str(PYTHON), '-m', 'pip', 'install', '--require-hashes',
                    '-r', str(ROOT / 'requirements/dev.lock')], cwd=ROOT, check=True)
    subprocess.run([str(PYTHON), '-m', 'pip', 'install', '--no-deps', '--no-build-isolation',
                    '-e', '.'], cwd=ROOT, check=True)


def check(offline=False):
    run_dir = ROOT / 'build/dev-check' / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True)
    report = {'schema_version': 1, 'gate': 'offline-control', 'passed': False, 'commands': []}

    def run(args, env):
        args = list(map(str, args))
        args[0] = shutil.which(args[0], path=env['PATH']) or args[0]
        print('+ ' + ' '.join(args), flush=True)
        with (run_dir / 'output.log').open('a', encoding='utf-8') as log:
            log.write('+ ' + ' '.join(args) + '\n')
            log.flush()
            result = subprocess.run(args, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        report['commands'].append({'args': args, 'returncode': result.returncode})
        if result.returncode:
            raise RuntimeError(f'Command failed ({result.returncode}); see {run_dir / "output.log"}')

    try:
        report['source_before'] = source_identity()
        report['lock_sha256'] = sha(ROOT / 'requirements/dev.lock')
        if not PYTHON.is_file():
            raise RuntimeError('Run python scripts/dev.py setup first')
        env = rust_environment()
        temp = run_dir / 'temp'
        temp.mkdir()
        env.update(TEMP=str(temp), TMP=str(temp), TMPDIR=str(temp), PYTHONNOUSERSITE='1')
        # Remove external Python path overrides; an editable install must belong here.
        env.pop('PYTHONPATH', None)
        run([PYTHON, '-c', 'from pathlib import Path; import robot_platform; '
             'assert Path(robot_platform.__file__).resolve().parent == '
             'Path("src/robot_platform").resolve(), "Wrong editable install"'], env)
        run([PYTHON, '-c', 'from robot_platform.models import UrdfModel, ModelProfile; '
             'UrdfModel(ModelProfile.load("urdf/minerva/model.json"))'], env)
        report['python'] = capture(str(PYTHON), '--version')
        if not report['python'].startswith('Python 3.12.'):
            raise RuntimeError('The development gate requires Python 3.12')
        report['packages'] = json.loads(capture(str(PYTHON), '-m', 'pip', 'list', '--format=json'))
        rustc = shutil.which('rustc', path=env['PATH'])
        if not rustc:
            raise RuntimeError('Rust compiler missing from the selected toolchain')
        report['rustc'] = subprocess.check_output([rustc, '--version'], env=env, text=True).strip()
        if not report['rustc'].startswith('rustc 1.98.1 '):
            raise RuntimeError('Use the baseline Rust compiler 1.98.1; install with rustup and select it')
        flags = ['--locked'] + (['--offline'] if offline else [])
        run(['cargo', 'test', '-p', 'robot_control', '-p', 'robot_collision', *flags], env)
        run(['cargo', 'build', '-p', 'robot_control', '--bin', 'control_harness', *flags], env)
        run(['cargo', 'build', '-p', 'robot_collision', '--release', *flags], env)
        report['binaries'] = {key: {'path': env[key], 'sha256': sha(env[key])}
                              for key in ('ROBOT_CONTROL_HARNESS', 'ROBOT_COLLISION_LIBRARY')}
        # A broken ABI must fail before tests can skip missing native prerequisites.
        run([PYTHON, '-c', 'import ctypes, os; '
             'lib=ctypes.CDLL(os.environ["ROBOT_COLLISION_LIBRARY"]); '
             'assert lib.rpc_abi_version() == 3; import fcl, numpy, scipy, pytest, mujoco, hypothesis'], env)
        run([PYTHON, '-m', 'pytest', '-q', '--strict-markers', '-o', 'xfail_strict=true',
             '--basetemp', run_dir / 'pytest', '--junitxml', run_dir / 'tests.xml', *CORE], env)
        cases = ET.parse(run_dir / 'tests.xml').findall('.//testcase')
        skipped = [case.get('name') for case in cases if case.find('skipped') is not None]
        report['tests'] = {'collected': len(cases), 'skipped': skipped}
        if not cases or skipped:
            raise RuntimeError(f'Required gate must execute every test; skipped: {skipped}')
        report['source_after'] = source_identity()
        if report['source_before'] != report['source_after']:
            raise RuntimeError('Source changed during verification; rerun in an isolated worktree')
        report['passed'] = True
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        (run_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'Report: {run_dir / "report.json"}', flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['setup', 'check'])
    parser.add_argument('--offline', action='store_true', help='Use cached Cargo dependencies only')
    args = parser.parse_args()
    if args.command == 'setup':
        setup()
        return 0
    return check(args.offline)


if __name__ == '__main__':
    raise SystemExit(main())
