"""Write a reviewable MJCF from a URDF without changing its frames or inertials.

Every URDF link becomes a named MuJoCo body at the link frame, so MuJoCo's forward
kinematics can be compared numerically with the project's URDF kinematics. Inertials
are copied as declared: exporter placeholders stay placeholders, and MujocoPlant
still refuses a model whose moving bodies carry them. Mimic relations become joint
equality constraints. Meshes are referenced in place, never copied or rescaled.

Contacts are off (contype/conaffinity 0) until a reviewed contact model exists,
and no damping, friction, armature or actuator is invented. Generating the MJCF is
not physical validation; the manifest says what the model is and is not.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .urdf import ModelProfile, UrdfModel, asset_digest, numbers

GENERATOR = 'robot_platform.models.mjcf/v1'
# MJCF geom groups: the viewer can toggle source visuals and source collision meshes.
VISUAL_GROUP, COLLISION_GROUP = 2, 3
# MuJoCo's STL decoder rejects files with more faces than this.
MUJOCO_STL_MAX_FACES = 200_000
# MujocoPlant's default step. A mimic is a rigid mechanical coupling (rack and
# pinion jaws), so its equality uses the stiffest time constant MuJoCo keeps stable
# (two steps; the refsafe flag raises it if a caller lengthens the step).
TIMESTEP_S = .0005
MIMIC_SOLREF = (2 * TIMESTEP_S, 1.)
DERIVED_MESH_DIR = 'meshes'


def stl_face_count(path):
    with Path(path).open('rb') as stream:
        header = stream.read(84)
    count = int.from_bytes(header[80:84], 'little') if len(header) == 84 else -1
    if count < 1 or Path(path).stat().st_size != 84 + 50 * count:
        raise ValueError(f'Expected a binary STL: {Path(path).name}')
    return count


def stl_to_obj(path):
    """The same triangles as OBJ text with exactly shared vertices merged; no decimation."""
    raw = Path(path).read_bytes()
    count = stl_face_count(path)
    records = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)),
                                                 ('attr', '<u2')]), count=count, offset=84)
    corners = records['v'].reshape(-1, 3).astype(float)
    vertices, index = np.unique(corners, axis=0, return_inverse=True)
    faces = index.reshape(-1, 3)
    lines = [f'# Lossless OBJ of {Path(path).name}: {count} triangles, exact shared vertices merged.\n']
    lines += [f'v {x!r} {y!r} {z!r}\n' for x, y, z in vertices.tolist()]
    lines += [f'f {a + 1} {b + 1} {c + 1}\n' for a, b, c in faces.tolist()]
    return ''.join(lines).encode(), count


def rpy_matrix(rpy):
    """URDF rpy: fixed-axis roll about X, then pitch about Y, then yaw about Z."""
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_quat(matrix):
    """Unit quaternion (w, x, y, z) of a rotation matrix, with w >= 0."""
    m = np.asarray(matrix, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        q = [s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.] * 4
        q[0] = (m[k, j] - m[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (m[j, i] + m[i, j]) / s
        q[1 + k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def _frame(element, origin):
    element.set('pos', text(origin['xyz']))
    if any(origin['rpy']):
        element.set('quat', text(matrix_quat(rpy_matrix(origin['rpy']))))


def _relative(path, start):
    return Path(os.path.relpath(path, start)).as_posix()


def _inertials(root):
    result = {}
    for link in root.findall('link'):
        nodes = link.findall('inertial')
        if not nodes:
            continue
        if len(nodes) > 1:
            raise ValueError(f"{link.get('name')}: multiple inertial elements")
        node = nodes[0]
        frame = node.find('origin')
        rpy = numbers(frame.get('rpy', '0 0 0') if frame is not None else '0 0 0', 3, 'inertial rpy')
        xyz = numbers(frame.get('xyz', '0 0 0') if frame is not None else '0 0 0', 3, 'inertial xyz')
        mass = node.find('mass')
        tensor = node.find('inertia')
        if mass is None or tensor is None:
            raise ValueError(f"{link.get('name')}: inertial needs mass and inertia")
        xx, xy, xz, yy, yz, zz = numbers(
            [tensor.get(k) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')], 6, 'inertia')
        # URDF expresses the tensor in the inertial frame; MJCF gets it in the
        # body (link) frame so no second orientation convention is involved.
        r = rpy_matrix(rpy)
        body = r @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ r.T
        result[link.get('name')] = dict(
            pos=xyz, mass=numbers([mass.get('value')], 1, 'mass', True)[0],
            fullinertia=[body[0, 0], body[1, 1], body[2, 2], body[0, 1], body[0, 2], body[1, 2]])
    return result


def build_mjcf(profile, output, *, meshes=True):
    """Return (MJCF text, manifest, derived meshes) for writing at `output`.

    Reads only. Derived meshes map a path relative to the output directory to its
    bytes: a lossless OBJ for each STL over MuJoCo's face limit. `meshes=False`
    keeps bodies, joints, inertials and constraints only: the same kinematic tree
    without loading CAD, for fast frame comparisons.
    """
    output = Path(output).resolve()
    model = UrdfModel(profile, apply_review=False)
    raw = profile.urdf.read_bytes()
    root = ET.fromstring(raw)
    inertials = _inertials(root)
    by_child = {j['child']: j for j in model.joints}
    children = {name: [] for name in model.links}
    for joint in model.joints:
        children[joint['parent']].append(joint)

    mujoco = ET.Element('mujoco', model=root.get('name') or profile.id)
    ET.SubElement(mujoco, 'compiler', angle='radian', autolimits='true', inertiafromgeom='false',
                  balanceinertia='false', boundmass='0', boundinertia='0', fusestatic='false',
                  discardvisual='false', meshdir=_relative(profile.urdf.parent, output.parent))
    ET.SubElement(mujoco, 'option', timestep=repr(TIMESTEP_S), gravity='0 0 -9.81')
    ET.SubElement(mujoco, 'default')  # Explicitly empty: no inherited joint/geom physics.
    asset = ET.SubElement(mujoco, 'asset')
    mesh_names, assets, files, derived, derivations = {}, {}, {}, {}, []

    def mesh_file(path):
        """The file MuJoCo loads for a source mesh: itself, or its lossless OBJ."""
        if path not in files:
            if path.suffix.lower() != '.stl':
                raise ValueError(f'MuJoCo export supports STL meshes only: {path.name}')
            assets[path] = asset_digest(path)
            files[path] = path
            faces = stl_face_count(path)
            if faces > MUJOCO_STL_MAX_FACES:
                content, _ = stl_to_obj(path)
                relative = f'{DERIVED_MESH_DIR}/{path.stem}.obj'
                if relative in derived:
                    raise ValueError(f'Derived mesh name collision: {relative}')
                derived[relative] = content
                files[path] = output.parent / relative
                derivations.append({'source': _relative(path, output.parent), 'source_sha256': assets[path],
                                    'derived': relative, 'derived_sha256': hashlib.sha256(content).hexdigest(),
                                    'triangles': faces, 'method': 'binary STL to OBJ, exact shared vertices merged'})
        return files[path]

    def mesh_name(desc):
        key = (desc['asset'], tuple(desc['scale']))
        if key not in mesh_names:
            path = model.assets[desc['asset']]
            # Visual and collision sets reuse file names; the source path tells them apart.
            name = _relative(path.with_suffix(''), profile.urdf.parent)
            if desc['scale'] != [1., 1., 1.]:
                name += '@' + 'x'.join(repr(float(v)) for v in desc['scale'])
            mesh_names[key] = name
            element = ET.SubElement(asset, 'mesh', name=name, file=_relative(mesh_file(path), profile.urdf.parent))
            if desc['scale'] != [1., 1., 1.]:
                element.set('scale', text(desc['scale']))
        return mesh_names[key]

    geometry = {}
    for desc in model.geometry:
        geometry.setdefault(desc['link'], []).append(desc)

    def add_geoms(body, link):
        if not meshes:
            return
        counts = {}
        for desc in geometry.get(link, []):
            role = desc['role']
            counts[role] = counts.get(role, 0) + 1
            geom = ET.SubElement(body, 'geom', name=f'{link}/{role}/{counts[role] - 1}')
            if desc['shape'] == 'mesh':
                geom.set('type', 'mesh')
                geom.set('mesh', mesh_name(desc))
            elif desc['shape'] == 'box':
                geom.set('type', 'box')
                geom.set('size', text(np.asarray(desc['size']) / 2))
            elif desc['shape'] == 'sphere':
                geom.set('type', 'sphere')
                geom.set('size', text([desc['radius']]))
            else:
                geom.set('type', 'cylinder')
                geom.set('size', text([desc['radius'], desc['length'] / 2]))
            _frame(geom, desc['origin'])
            geom.set('group', str(VISUAL_GROUP if role == 'visual' else COLLISION_GROUP))
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            if role == 'visual':
                geom.set('rgba', text(desc['color']))

    def add_body(parent, link):
        body = ET.SubElement(parent, 'body', name=link)
        joint = by_child.get(link)
        if joint is not None:
            _frame(body, joint['origin'])
            if joint['type'] != 'fixed':
                element = ET.SubElement(body, 'joint', name=joint['name'],
                                        type='slide' if joint['type'] == 'prismatic' else 'hinge',
                                        axis=text(joint['axis']))
                if joint['bounds'] is not None:
                    element.set('range', text(joint['bounds']))
        if link in inertials:
            values = inertials[link]
            ET.SubElement(body, 'inertial', pos=text(values['pos']), mass=repr(float(values['mass'])),
                          fullinertia=text(values['fullinertia']))
        add_geoms(body, link)
        for joint in children[link]:
            add_body(body, joint['child'])

    worldbody = ET.SubElement(mujoco, 'worldbody')
    add_body(worldbody, model.root)
    mimics = [model.by_name[n] for n in model.mimic_order]
    if mimics:
        equality = ET.SubElement(mujoco, 'equality')
        for joint in mimics:
            m = joint['mimic']
            # MuJoCo: q1 - q1ref = a0 + a1*(q2 - q2ref); every ref here is zero.
            ET.SubElement(equality, 'joint', name=f"{joint['name']}/mimic", joint1=joint['name'],
                          joint2=m['joint'], polycoef=text([m['offset'], m['multiplier'], 0, 0, 0]),
                          solref=text(MIMIC_SOLREF))
    custom = ET.SubElement(mujoco, 'custom')
    ET.SubElement(custom, 'text', name='source_urdf_sha256', data=model.sha256)
    ET.SubElement(custom, 'text', name='generator', data=GENERATOR)
    ET.indent(mujoco)
    xml = ('<!-- Generated by robot_platform.models.mjcf from the URDF; do not edit by hand. -->\n'
           + ET.tostring(mujoco, encoding='unicode') + '\n')

    placeholders = sorted(name for name, values in inertials.items() if values['mass'] <= 1e-6)
    manifest = {
        'schema_version': 1, 'kind': 'mjcf_export', 'generator': GENERATOR, 'model_id': profile.id,
        'source_urdf': _relative(profile.urdf, output.parent), 'source_urdf_sha256': model.sha256,
        'mjcf_sha256': hashlib.sha256(xml.encode()).hexdigest(), 'meshes_included': bool(meshes),
        'mesh_assets': [{'file': _relative(path, output.parent), 'sha256': sha}
                        for path, sha in sorted(assets.items(), key=lambda item: str(item[0]))],
        'derived_meshes': derivations,
        'independent_joints': list(model.independent),
        'mimic_equalities': [{'follower': j['name'], 'source': j['mimic']['joint'],
                              'multiplier': j['mimic']['multiplier'], 'offset': j['mimic']['offset']}
                             for j in mimics],
        'assumptions': ['mimic couplings are rigid: stiffest stable equality, no backlash or compliance',
                        'joint ranges are the URDF limits, not measured hard stops',
                        'gravity is -Z in the URDF root frame; the mounting pose is unverified'],
        'placeholder_inertial_links': placeholders,
        'physical_validation': 'unvalidated',
        'dynamics_usable': not placeholders,
        'contacts': 'disabled: no reviewed contact model',
        'not_modeled': ['joint damping', 'joint friction', 'rotor/gear armature', 'actuators',
                        'motor-to-joint transmissions (receiver bindings are separate)'],
    }
    return xml, manifest, derived


def write_mjcf(profile, output, *, meshes=True):
    """Write a new MJCF, its derived meshes and manifest; refuses to overwrite any."""
    output = Path(output).resolve()
    manifest_path = output.with_suffix('.manifest.json')
    xml, manifest, derived = build_mjcf(profile, output, meshes=meshes)
    targets = [output, manifest_path, *(output.parent / name for name in derived)]
    if any(path.exists() for path in targets):
        raise FileExistsError('Refusing to overwrite an MJCF export; write a new revision directory')
    output.parent.mkdir(parents=True, exist_ok=True)
    for name, content in derived.items():
        (output.parent / name).parent.mkdir(parents=True, exist_ok=True)
        with (output.parent / name).open('xb') as stream:
            stream.write(content)
    with output.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(xml)
    with manifest_path.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path, help='model.json profile (for example urdf/minerva/model.json)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--no-meshes', action='store_true', help='kinematic tree only')
    args = parser.parse_args()
    manifest = write_mjcf(ModelProfile.load(args.profile), args.output, meshes=not args.no_meshes)
    print(json.dumps({k: manifest[k] for k in ('mjcf_sha256', 'dynamics_usable', 'physical_validation')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
