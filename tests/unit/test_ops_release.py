"""Robot releases: allowlist build, manifest verification, activation and pruning."""
import json
import os
import tarfile

import pytest

from robot_platform.models.urdf import register_verified_assets, verified_release_active
from robot_platform.ops.deploy import RELEASE_ID, stale_releases
from robot_platform.ops.release import LOCK, MANIFEST, ROOT, TARGET, _sha, activate, build, check_binaries, verify_tree

ARM64 = ROOT / 'rust/target' / TARGET / 'release'


def write_release(root, files, lock=None):
    """A minimal unpacked release with a correct manifest."""
    lock = json.dumps(lock or {}).encode()
    contents = {**files, LOCK: lock}
    for key, data in contents.items():
        (root / key).parent.mkdir(parents=True, exist_ok=True)
        (root / key).write_bytes(data)
    manifest = {'id': 'x', 'files': {k: {'sha256': _sha(v), 'size': len(v)} for k, v in contents.items()},
                'verified': {'assets_lock_sha256': _sha(lock)}}
    (root / MANIFEST).write_text(json.dumps(manifest))
    return manifest


def test_verification_catches_missing_changed_and_unexpected_files(tmp_path):
    write_release(tmp_path, {'src/a.py': b'print(1)', 'bin/tool': b'\x7fELF'})
    assert verify_tree(tmp_path) == []
    (tmp_path / 'src/__pycache__').mkdir()
    (tmp_path / 'src/__pycache__/a.pyc').write_bytes(b'cache')  # Python may add bytecode.
    assert verify_tree(tmp_path) == []
    (tmp_path / 'src/a.py').write_bytes(b'print(2)')
    (tmp_path / 'bin/tool').unlink()
    (tmp_path / 'extra.txt').write_text('?')
    assert sorted(verify_tree(tmp_path)) == ['changed src/a.py', 'missing bin/tool', 'unexpected extra.txt']
    assert verify_tree(tmp_path / 'absent')[0].startswith('manifest unreadable')


def test_activation_registers_the_verified_meshes_and_native_paths(tmp_path, monkeypatch):
    # setenv (not delenv) so pytest restores both after activate() overwrites them.
    monkeypatch.setenv('ROBOT_COLLISION_LIBRARY', 'unset')
    monkeypatch.setenv('ROBOT_CONTROL_HARNESS', 'unset')
    write_release(tmp_path, {'urdf/m/x.urdf': b'<robot/>'}, lock={'urdf/m/meshes/a.stl': 'ab' * 32})
    try:
        activate(tmp_path)
        assert verified_release_active()
        assert os.environ['ROBOT_COLLISION_LIBRARY'].endswith(os.path.join('lib', 'librobot_collision.so'))
        assert os.environ['ROBOT_CONTROL_HARNESS'].endswith(os.path.join('bin', 'control_harness'))
    finally:
        register_verified_assets({})
    (tmp_path / 'urdf/m/x.urdf').write_bytes(b'<robot name="changed"/>')
    with pytest.raises(RuntimeError, match='failed verification'):
        activate(tmp_path)


def test_pruning_keeps_the_newest_and_never_the_current_or_previous():
    names = [f'2026092{d}T000000Z-abcdef{d}' for d in range(6)]
    assert all(RELEASE_ID.match(name) for name in names)
    assert stale_releases(names, names[5], names[4], 3) == names[:3]
    # Rolled back to an old release: it and the one it replaced both stay.
    assert stale_releases(names, names[1], names[5], 3) == [names[0], names[2]]
    assert stale_releases(names + ['not-a-release'], names[5], None, 2) == names[:4]


@pytest.mark.skipif(not (ARM64 / 'guarded_controller').is_file(), reason='Cross-build the ARM64 binaries first')
def test_a_built_release_is_small_complete_and_verifiable(tmp_path):
    manifest, archive = build(tmp_path / 'out')
    assert RELEASE_ID.match(manifest['id'])
    keys = set(manifest['files'])
    assert {'bin/guarded_controller', 'bin/control_harness', 'lib/librobot_collision.so', LOCK,
            'config/devices/minerva-teleop.json', 'urdf/minerva/minerva.urdf'} <= keys
    assert not any(key.endswith(('.stl', '.glb', '.gltf', '.js', '.css', '.html')) for key in keys)
    assert manifest['verified']['meshes'] > 0 and archive.stat().st_size < 10e6
    with tarfile.open(archive) as tar:
        tar.extractall(tmp_path / 'release', filter='data')
    assert verify_tree(tmp_path / 'release') == []
    again, _ = build(tmp_path / 'again')
    assert again['content_sha256'] == manifest['content_sha256']  # Same sources, same content.


def test_each_binary_is_only_as_stale_as_its_own_crate(tmp_path):
    elf = b'\x7fELF\x02\x01' + bytes(12) + (183).to_bytes(2, 'little') + bytes(44)
    sources = {'rust/control_core/src/lib.rs': 100, 'rust/collision/src/lib.rs': 50}
    for key, stamp in sources.items():
        (tmp_path / key).parent.mkdir(parents=True)
        (tmp_path / key).write_text('//')
        os.utime(tmp_path / key, (stamp, stamp))
    files = {'bin/guarded_controller': tmp_path / 'guarded_controller', 'lib/librobot_collision.so': tmp_path / 'lib.so'}
    for path, stamp in zip(files.values(), (200, 75)):
        path.write_bytes(elf)
        os.utime(path, (stamp, stamp))
    check_binaries(files, tmp_path)  # The library predates control_core, not its own crate.
    os.utime(files['lib/librobot_collision.so'], (40, 40))
    with pytest.raises(ValueError, match='librobot_collision.so is older'):
        check_binaries(files, tmp_path)
    files['bin/guarded_controller'].write_bytes(b'MZ' + bytes(62))
    with pytest.raises(ValueError, match='not an AArch64 build'):
        check_binaries(files, tmp_path)
