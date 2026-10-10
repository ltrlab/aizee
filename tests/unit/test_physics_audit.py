from pathlib import Path

import pytest

from robot_platform.models.physics_audit import audit_urdf


def urdf(tmp_path, *, mass='1', inertia='ixx=".01" ixy="0" ixz="0" iyy=".02" iyz="0" izz=".02"'):
    path = tmp_path / 'fixture.urdf'
    path.write_text(f'''<robot name="fixture"><link name="base"><inertial>
      <mass value="{mass}"/><inertia {inertia}/></inertial>
      <collision><geometry><box size=".1 .2 .3"/></geometry></collision>
      </link></robot>''')
    return path


def test_plausible_inputs_pass_structural_audit_without_claiming_validation(tmp_path):
    report = audit_urdf(urdf(tmp_path))
    assert report['structurally_usable_for_dynamics'] and not report['errors']
    assert report['physical_validation'] == 'unvalidated'


@pytest.mark.parametrize('mass', ['1e-9', '-1', 'nan', 'inf'])
def test_placeholder_or_invalid_mass_is_rejected(tmp_path, mass):
    assert audit_urdf(urdf(tmp_path, mass=mass))['errors']


@pytest.mark.parametrize('inertia', [
    'ixx="1" ixy="2" ixz="0" iyy="1" iyz="0" izz="1"',
    'ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="3"',
    'ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"',
])
def test_nonphysical_full_tensor_is_rejected(tmp_path, inertia):
    assert audit_urdf(urdf(tmp_path, inertia=inertia))['errors']


def test_active_minerva_is_explicitly_blocked_for_dynamics():
    root = Path(__file__).resolve().parents[2]
    report = audit_urdf(root / 'urdf/minerva/minerva.urdf')
    assert len(report['links']) == 17
    assert report['total_declared_mass_kg'] == pytest.approx(17e-9)
    assert len([e for e in report['errors'] if 'placeholder' in e]) == 17
    assert not report['structurally_usable_for_dynamics']
