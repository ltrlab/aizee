"""A verified release loads Minerva's collision model without meshes, trimesh or FCL,
and keep-out queries match the full, mesh-verified load exactly."""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.models.urdf import register_verified_assets
from robot_platform.simulation.collision import CollisionError
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.runtime_model import RuntimeCollisionModel

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / 'urdf/minerva'
CORE = ('model.json', 'minerva.urdf', 'collision/broadphase-v1.2.json', 'limits/collision-v1.2.json')

pytestmark = pytest.mark.skipif(not library_path().is_file(), reason='Build the robot_collision library first')


@pytest.fixture(scope='module')
def full():
    return RuntimeCollisionModel(ModelProfile.load(MODEL_DIR / 'model.json'))


@pytest.fixture
def release(tmp_path):
    """A model directory as a release ships it: core files only, plus the verified mesh digests."""
    target = tmp_path / 'urdf/minerva'
    for name in CORE:
        (target / name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(MODEL_DIR / name, target / name)
    model = UrdfModel(ModelProfile.load(MODEL_DIR / 'model.json'))
    digests = {str(target / path.relative_to(MODEL_DIR.resolve())): __import__('hashlib').sha256(
        path.read_bytes()).hexdigest() for path in model.assets}
    yield target, digests
    register_verified_assets({})


def poses(model, count=6, seed=7):
    generator = np.random.default_rng(seed)
    for _ in range(count):
        yield {name: float(generator.uniform(*model.model.by_name[name]['control_bounds']))
               for name in model.scene.independent_joints}


def test_a_kinematics_only_load_matches_the_full_one(full, release):
    target, digests = release
    register_verified_assets(digests)
    light = RuntimeCollisionModel(ModelProfile.load(target / 'model.json'))
    assert light.kinematics_only and not full.kinematics_only
    assert light.revision == full.revision and light.shapes == full.shapes and light.pairs == full.pairs
    assert light.keepout_scoped_pairs == full.keepout_scoped_pairs and light.pair_kinds == full.pair_kinds
    assert (light.policy_revision, light.keepout_policy_revision) == (full.policy_revision, full.keepout_policy_revision)
    for ours, theirs in zip(light.keepout_motion_corners, full.keepout_motion_corners):
        assert np.array_equal(ours, theirs)
    heavy, lean = NativeCollisionBackend(full, geometry_mode='keepout'), NativeCollisionBackend(light, geometry_mode='keepout')
    try:
        for pose in poses(full):
            assert np.array_equal(light.transforms(pose), full.transforms(pose))
            a, b = heavy.proximity(pose, threshold=.1), lean.proximity(pose, threshold=.1)
            assert a['near_pairs'] == b['near_pairs'] and a['minimum_distance_m'] == b['minimum_distance_m']
    finally:
        heavy.close()
        lean.close()
    with pytest.raises(CollisionError, match='kinematics only'):
        light.refine(next(poses(full)), [light.pairs[0]], threshold=.1)


def test_a_verified_release_needs_no_trimesh_fcl_or_meshes(release, tmp_path):
    target, digests = release
    (tmp_path / 'digests.json').write_text(json.dumps(digests))
    script = f'''
import json, sys
sys.modules["trimesh"] = None  # Importing either now fails, as on the robot.
sys.modules["fcl"] = None
from robot_platform.models import ModelProfile
from robot_platform.models.urdf import register_verified_assets
from robot_platform.simulation.native_collision import NativeCollisionBackend
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
register_verified_assets(json.loads(open({str(tmp_path / 'digests.json')!r}).read()))
model = RuntimeCollisionModel(ModelProfile.load({str(target / 'model.json')!r}))
backend = NativeCollisionBackend(model, geometry_mode="keepout")
pose = {{name: 0. for name in model.scene.independent_joints}}
print(model.kinematics_only, "near_pairs" in backend.proximity(pose, threshold=.1))
'''
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, cwd=ROOT,
                            env={**__import__('os').environ, 'PYTHONPATH': str(ROOT / 'src')})
    assert result.returncode == 0, result.stderr
    assert result.stdout.split() == ['True', 'True']


def test_an_unverified_model_without_meshes_is_refused(release):
    target, _digests = release
    register_verified_assets({})
    with pytest.raises(Exception, match='mesh|Mesh|geometry'):
        RuntimeCollisionModel(ModelProfile.load(target / 'model.json'))
