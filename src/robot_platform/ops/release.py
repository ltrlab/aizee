"""Robot releases: a small, verified, immutable bundle of what the robot runs.

A release holds the Python platform (without web assets), the device profiles,
the robot model's core files and the cross-built ARM64 binaries, plus a
manifest with the sha256 of every file. CAD meshes stay on the build PC: the
build checks the collision proxies and the limit review against them and
records their digests (assets.lock.json), so the robot loads kinematics and
proxies without them. A release never changes after it is built.

Build and verify on the PC; `verify_tree` and `activate` also run on the robot
with the standard library and the platform's own modules.
"""
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
TARGET = 'aarch64-unknown-linux-gnu'
MODEL = 'urdf/minerva/model.json'
CALIBRATION = 'config/devices/minerva-joint-calibration.json'
MANIFEST, LOCK = 'release.json', 'assets.lock.json'
BINARIES = ('guarded_controller', 'control_harness', 'motor_watchdog_trial')
LIBRARY = 'librobot_collision.so'
# Each artifact is only as stale as its own crate: binaries come from control_core,
# the library from collision.
RUST_SOURCES = {'bin/': 'rust/control_core', 'lib/': 'rust/collision'}
EM_AARCH64 = 183


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _key(path, root):
    return Path(path).resolve().relative_to(Path(root).resolve()).as_posix()


def _git(*args, root=ROOT):
    result = subprocess.run(['git', *args], cwd=root, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def _aarch64(path):
    header = Path(path).read_bytes()[:20]
    return (len(header) == 20 and header[:4] == b'\x7fELF' and header[4] == 2 and header[5] == 1
            and int.from_bytes(header[18:20], 'little') == EM_AARCH64)


def collect(root=ROOT):
    """{release path: source file} for everything the robot runs."""
    from robot_platform.models import ModelProfile
    root = Path(root)
    files = {}
    for path in sorted((root / 'src/robot_platform').rglob('*.py')):
        parts = path.relative_to(root / 'src/robot_platform').parts
        if '__pycache__' not in parts and 'static' not in parts:
            files[_key(path, root)] = path
    for path in sorted((root / 'config/devices').glob('*.json')):
        files[_key(path, root)] = path
    profile = ModelProfile.load(root / MODEL)
    for path in (root / MODEL, profile.urdf, profile.collision_model, profile.limit_review):
        files[_key(path, root)] = Path(path)
    built = root / 'rust/target' / TARGET / 'release'
    for name in BINARIES:
        files[f'bin/{name}'] = built / name
    files[f'lib/{LIBRARY}'] = built / LIBRARY
    return files


def check_binaries(files, root=ROOT):
    """Cross-built binaries must exist, be AArch64 and be newer than their crate's Rust sources."""
    newest = {prefix: max(path.stat().st_mtime for path in (Path(root) / source).rglob('*')
                          if path.is_file() and path.suffix in ('.rs', '.toml'))
              for prefix, source in RUST_SOURCES.items()}
    for key, path in files.items():
        prefix = key[:key.find('/') + 1]
        if prefix in RUST_SOURCES:
            if not path.is_file() or not _aarch64(path):
                raise ValueError(f'{key}: missing or not an AArch64 build. Run '
                                 'scripts/build_jetson.ps1 -Offline -Package robot_control, robot_collision')
            if path.stat().st_mtime < newest[prefix]:
                raise ValueError(f'{key} is older than the Rust sources. Run '
                                 'scripts/build_jetson.ps1 -Offline -Package robot_control, robot_collision')


def verify_model(root=ROOT):
    """Full checks against the CAD meshes on the build PC; returns the mesh digests and facts."""
    from robot_platform.control.joint_calibration import load_plan
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.simulation.runtime_model import RuntimeCollisionModel
    profile = ModelProfile.load(Path(root) / MODEL)
    model = UrdfModel(profile)  # Hashes every mesh against the limit review.
    runtime = RuntimeCollisionModel(profile)  # Proxies enclose the source CAD.
    load_plan(Path(root) / CALIBRATION)  # Calibration, reference and model agree.
    digests = {_key(path, root): _sha(path.read_bytes()) for path in model.assets}
    facts = {'model_id': profile.id, 'urdf_sha256': model.sha256, 'collision_model_revision': runtime.revision,
             'meshes': len(digests)}
    return digests, facts


def build(out_dir, root=ROOT, *, robot='minerva-production-01', now=None):
    """Write <id>.tar.gz into out_dir; return (manifest, archive path)."""
    root, out_dir = Path(root), Path(out_dir)
    files = collect(root)
    check_binaries(files, root)
    digests, facts = verify_model(root)
    lock = json.dumps(dict(sorted(digests.items())), indent=1).encode()
    contents = {key: path.read_bytes() for key, path in files.items()}
    contents[LOCK] = lock
    listing = {key: {'sha256': _sha(data), 'size': len(data)} for key, data in sorted(contents.items())}
    content_sha = _sha(json.dumps({k: v['sha256'] for k, v in listing.items()}, sort_keys=True).encode())
    commit = _git('rev-parse', 'HEAD', root=root) or 'unknown'
    dirty = bool(_git('status', '--porcelain', root=root))
    stamp = (now or datetime.now(timezone.utc)).strftime('%Y%m%dT%H%M%SZ')
    release_id = f'{stamp}-{commit[:7]}' + (f'-dirty{content_sha[:6]}' if dirty else '')
    manifest = {'schema': 1, 'kind': 'robot_release', 'id': release_id, 'robot': robot,
                'created_at': datetime.now(timezone.utc).isoformat(), 'content_sha256': content_sha,
                'source': {'commit': commit, 'branch': _git('branch', '--show-current', root=root), 'dirty': dirty},
                'verified': {**facts, 'assets_lock_sha256': _sha(lock)},
                'requires': {'python': '>=3.10', 'modules': ['numpy', 'scipy', 'msgpack']},
                'files': listing}
    out_dir.mkdir(parents=True, exist_ok=True)
    archive = out_dir / f'{release_id}.tar.gz'
    executables = {f'bin/{name}' for name in BINARIES}
    with tarfile.open(archive, 'w:gz') as tar:
        for key, data in [*sorted(contents.items()), (MANIFEST, json.dumps(manifest, indent=1).encode())]:
            info = tarfile.TarInfo(key)
            info.size, info.mode, info.mtime = len(data), 0o755 if key in executables else 0o644, 0
            tar.addfile(info, io.BytesIO(data))
    return manifest, archive


def release_drift(manifest, root=ROOT):
    """Release files whose content differs between a deployed manifest and this tree.

    Mesh digests are not compared (meshes stay here); a file missing locally,
    such as an unbuilt binary, is left to check_binaries.
    """
    listed = collect(root)
    files = {key: path for key, path in listed.items() if path.is_file()}
    deployed = {key: entry['sha256'] for key, entry in manifest['files'].items() if key != LOCK}
    return sorted([f'changed {key}' for key in files.keys() & deployed.keys()
                   if _sha(files[key].read_bytes()) != deployed[key]]
                  + [f'new {key}' for key in files.keys() - deployed.keys()]
                  + [f'removed {key}' for key in deployed.keys() - listed.keys()],
                  key=lambda item: item.split(' ', 1)[1])


def verify_tree(root):
    """Problems with an unpacked release (empty when every file matches the manifest)."""
    root = Path(root)
    try:
        manifest = json.loads((root / MANIFEST).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        return [f'manifest unreadable: {exc}']
    problems = []
    for key, entry in manifest.get('files', {}).items():
        path = root / key
        if not path.is_file():
            problems.append(f'missing {key}')
        elif path.stat().st_size != entry['size'] or _sha(path.read_bytes()) != entry['sha256']:
            problems.append(f'changed {key}')
    listed = set(manifest.get('files', {})) | {MANIFEST}
    for path in root.rglob('*'):
        key = path.relative_to(root).as_posix()
        if path.is_file() and key not in listed and '__pycache__' not in path.parts:
            problems.append(f'unexpected {key}')
    lock = root / LOCK
    if lock.is_file() and _sha(lock.read_bytes()) != manifest.get('verified', {}).get('assets_lock_sha256'):
        problems.append('asset lock does not match the verified manifest')
    return problems


def activate(root):
    """Use a verified release in this process: its mesh digests and native binaries."""
    from robot_platform.models.urdf import register_verified_assets
    root = Path(root).resolve()
    problems = verify_tree(root)
    if problems:
        raise RuntimeError('Release failed verification: ' + '; '.join(problems[:5]))
    lock = json.loads((root / LOCK).read_text(encoding='utf-8'))
    register_verified_assets({root / key: sha for key, sha in lock.items()})
    os.environ['ROBOT_COLLISION_LIBRARY'] = str(root / 'lib' / LIBRARY)
    os.environ['ROBOT_CONTROL_HARNESS'] = str(root / 'bin' / 'control_harness')
    return json.loads((root / MANIFEST).read_text(encoding='utf-8'))
