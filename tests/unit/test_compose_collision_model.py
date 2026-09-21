"""Collision imports must preserve the visual model's mechanical coordinates."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
trimesh=pytest.importorskip('trimesh')
from robot_platform.models import ModelProfile,UrdfModel

spec=importlib.util.spec_from_file_location('compose_collision_model',Path(__file__).resolve().parents[2]/'scripts/compose_collision_model.py')
composer=importlib.util.module_from_spec(spec);spec.loader.exec_module(composer)


def fixture(tmp_path,name,root,child,axis='0 0 1',collision=True):
    folder=tmp_path/name;folder.mkdir()
    trimesh.creation.box(extents=[.1,.2,.3]).export(folder/'shape.stl')
    geometry='<geometry><mesh filename="shape.stl"/></geometry>'
    path=folder/'model.urdf'
    path.write_text(f'''<robot name="fixture"><link name="{root}"/>
    <link name="{child}"><visual>{geometry}<material name="paint"><color rgba=".1 .2 .3 1"/></material></visual>
    {('<collision>'+geometry+'</collision>') if collision else ''}</link>
    <joint name="hinge" type="revolute"><parent link="{root}"/><child link="{child}"/>
    <origin xyz="0 .5 0"/><axis xyz="{axis}"/><limit lower="-1" upper="1"/></joint></robot>''')
    return ModelProfile.from_urdf(path)


def test_compose_renamed_links_preserves_visuals_and_joint_coordinates(tmp_path):
    visual=fixture(tmp_path,'visual','base','arm')
    collision=fixture(tmp_path,'simplified','stand','shell')
    originals={p:p.read_bytes() for p in [visual.urdf,collision.urdf]}
    out=tmp_path/'combined.urdf';report=tmp_path/'composition.json'
    result=composer.compose(visual,collision,out,report)
    model=UrdfModel(ModelProfile.from_urdf(out))
    assert model.links==['base','arm']
    assert model.joints==UrdfModel(visual).joints
    root=ET.parse(out).getroot();link=root.find("link[@name='arm']")
    assert link.find('visual/geometry/mesh').get('filename')=='visual/shape.stl'
    assert link.find('collision/geometry/mesh').get('filename')=='simplified/shape.stl'
    assert link.find('visual/material/color').get('rgba')=='.1 .2 .3 1'
    assert result['collision_shape_mapping']==[{'source':'shell/0','composed':'arm/0'}]
    assert all(p.read_bytes()==raw for p,raw in originals.items())


def test_compose_rejects_axis_change_before_writing(tmp_path):
    visual=fixture(tmp_path,'visual','base','arm')
    collision=fixture(tmp_path,'simplified','stand','shell',axis='0 0 -1')
    out=tmp_path/'combined.urdf';report=tmp_path/'composition.json'
    with pytest.raises(ValueError,match='Joint frame differs'):
        composer.compose(visual,collision,out,report)
    assert not out.exists() and not report.exists()


def test_compose_rejects_missing_collision_coverage(tmp_path):
    visual=fixture(tmp_path,'visual','base','arm')
    collision=fixture(tmp_path,'simplified','stand','shell',collision=False)
    with pytest.raises(ValueError,match='no replacement collision geometry'):
        composer.compose(visual,collision,tmp_path/'combined.urdf',tmp_path/'composition.json')
