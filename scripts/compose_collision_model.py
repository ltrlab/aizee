"""Combine detailed URDF visuals with a compatible local collision-only URDF.

Joint names, coordinates and topology must agree; link names may differ. This
does no mesh simplification, CAD retrieval, dynamics estimation or exclusion.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from robot_platform.models import ModelProfile, UrdfModel


def compose(visual_profile, collision_profile, output, report_path):
    output,report_path=Path(output).resolve(),Path(report_path).resolve()
    if output==report_path:
        raise ValueError('Output and report must be different files')
    if output.exists() or report_path.exists():
        raise ValueError('Choose new output and report files')
    visual=UrdfModel(visual_profile,apply_review=False)
    collision=UrdfModel(collision_profile,apply_review=False)
    if set(visual.by_name)!=set(collision.by_name):
        raise ValueError('Joint names differ between visual and collision models')
    mapping={visual.root:collision.root}
    mapping.update({a['child']:collision.by_name[name]['child'] for name,a in visual.by_name.items()})
    if len(set(mapping.values()))!=len(collision.links):
        raise ValueError('Link mapping must be one-to-one')
    for name,a in visual.by_name.items():
        b=collision.by_name[name]
        if mapping[a['parent']]!=b['parent'] or a['type']!=b['type'] or a['mimic']!=b['mimic'] or a['bounds']!=b['bounds']:
            raise ValueError(f'Incompatible joint topology or coordinates: {name}')
        for va,vb in [(a['axis'],b['axis']),*( (a['origin'][k],b['origin'][k]) for k in ('xyz','rpy'))]:
            if not np.allclose(va,vb,atol=1e-10,rtol=0):
                raise ValueError(f'Joint frame differs: {name}')
    root=ET.parse(visual_profile.urdf).getroot()
    collision_links={node.get('name'):node for node in ET.parse(collision_profile.urdf).getroot().findall('link')}
    assets={}
    def localize(node,profile):
        for mesh in node.findall('.//mesh'):
            source=profile.resolve_mesh(mesh.get('filename'))
            if not source.is_relative_to(output.parent):
                raise ValueError('Compose output in a directory containing both source asset trees')
            relative=source.relative_to(output.parent).as_posix()
            mesh.set('filename',relative)
            assets[relative]=hashlib.sha256(source.read_bytes()).hexdigest()
    shape_mapping=[]
    for link in root.findall('link'):
        name=link.get('name')
        for original in link.findall('collision'):link.remove(original)
        localize(link,visual_profile)
        replacements=collision_links[mapping[name]].findall('collision')
        if link.findall('visual') and not replacements:
            raise ValueError(f'Visible link has no replacement collision geometry: {name}')
        for index,item in enumerate(replacements):
            item=deepcopy(item);localize(item,collision_profile);link.append(item)
            shape_mapping.append({'source':f'{mapping[name]}/{index}','composed':f'{name}/{index}'})
    ET.indent(root)
    payload=ET.tostring(root,encoding='utf-8',xml_declaration=True)+b'\n'
    report=dict(schema_version=1,kind='composed_collision_model',
                visual_urdf_sha256=visual.sha256,collision_urdf_sha256=collision.sha256,
                composed_urdf_sha256=hashlib.sha256(payload).hexdigest(),
                link_mapping=mapping,collision_shape_mapping=shape_mapping,assets=assets,
                joint_frames_unchanged=True,visuals_unchanged=True,
                collision_geometry_reviewed=False,physical_validation=False,
                inertial_source='Detailed source retained; no physical dynamics validation implied')
    output.parent.mkdir(parents=True,exist_ok=True)
    report_path.parent.mkdir(parents=True,exist_ok=True)
    with output.open('xb') as stream:stream.write(payload)
    with report_path.open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual-profile',type=Path,required=True)
    parser.add_argument('--collision-urdf',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    result=compose(ModelProfile.load(args.visual_profile),ModelProfile.from_urdf(args.collision_urdf),args.output,args.report)
    print(json.dumps({'urdf_sha256':result['composed_urdf_sha256'],'collision_shapes':len(result['collision_shape_mapping']),
                      'joint_frames_unchanged':True}))


if __name__=='__main__':main()
