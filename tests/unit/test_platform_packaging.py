"""Exercise shipped wheel contents and robot-side dependency preflight."""
from email.parser import Parser
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

from packaging.requirements import Requirement
import pytest

from robot_platform.ops import release, smoke

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope='module')
def wheel(tmp_path_factory):
    out = tmp_path_factory.mktemp('platform-wheel')
    subprocess.run([sys.executable, '-m', 'pip', 'wheel', '--no-index', '--no-deps',
                    '--no-build-isolation', '--wheel-dir', str(out), str(ROOT)],
                   check=True, capture_output=True, text=True)
    return next(out.glob('*.whl'))


def test_installed_wheel_serves_all_helm_assets_outside_the_checkout(wheel, tmp_path):
    installed = tmp_path / 'installed'
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--no-index', '--no-deps',
                    '--target', str(installed), str(wheel)], check=True, capture_output=True, text=True)
    assets = {p.name: p.read_bytes() for p in (ROOT / 'src/robot_platform/helm/static').iterdir() if p.is_file()}
    with ZipFile(wheel) as archive:
        for name, content in assets.items():
            assert archive.read('robot_platform/helm/static/' + name) == content
    code = ('from pathlib import Path; import sys; '
            f'sys.path.insert(0, {str(installed)!r}); '
            'import robot_platform.helm.server as server; '
            f'assert Path(server.__file__).is_relative_to(Path({str(installed)!r})); '
            f'assert all((server.STATIC / name).is_file() for name in {list(assets)!r}); '
            'assert "<html" in (server.STATIC / "index.html").read_text(encoding="utf-8").lower()')
    subprocess.run([sys.executable, '-I', '-c', code], cwd=tmp_path, check=True,
                   capture_output=True, text=True)


def test_wheel_declares_capture_and_helm_dependencies_and_scene_physics(wheel):
    with ZipFile(wheel) as archive:
        metadata = Parser().parsestr(archive.read(next(p for p in archive.namelist()
                                                      if p.endswith('.dist-info/METADATA'))).decode())
    requirements = [Requirement(item) for item in metadata.get_all('Requires-Dist')]

    def names(extra):
        return {item.name.lower() for item in requirements
                if item.marker is None or item.marker.evaluate({'extra': extra})}

    assert {'numpy', 'pyzmq', 'pillow', 'msgpack', 'h5py'} <= names('capture')
    assert {'numpy', 'pyzmq', 'pillow', 'msgpack', 'h5py', 'aiohttp', 'paramiko',
            'pyserial', 'trimesh', 'scipy'} <= names('helm')
    physics = next(item for item in requirements if item.name == 'mujoco'
                   and item.marker.evaluate({'extra': 'physics'}))
    assert '3.15.0' in physics.specifier and '3.14.0' not in physics.specifier


def test_missing_declared_dependency_fails_smoke_before_model_or_receiver(tmp_path, monkeypatch):
    manifest = {'id': 'fixture', 'source': {'commit': 'fixture'},
                'requires': {'modules': ['_aizee_missing_capture_dependency_']}}
    monkeypatch.setattr(release, 'activate', lambda root: manifest)
    with pytest.raises(ModuleNotFoundError, match='_aizee_missing_capture_dependency_'):
        smoke.run(tmp_path)


def test_dependency_preflight_does_not_import_host_simulation(monkeypatch):
    loaded = []
    from importlib import import_module

    def capture(name):
        loaded.append(name)
        return import_module(name)

    monkeypatch.setattr(smoke, 'import_module', capture)
    result = smoke.check_requirements({'requires': {'modules': ['json', 'pathlib']}})
    assert loaded == ['json', 'pathlib']
    assert set(result) == set(loaded)
