"""Read-only physical-input audit for URDF; passing is not physical validation."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .urdf import ModelProfile, UrdfModel, numbers, origin


def audit_urdf(path, *, minimum_mass_kg=1e-6):
    if not math.isfinite(minimum_mass_kg) or minimum_mass_kg <= 0:
        raise ValueError('Minimum mass must be finite and positive')
    path = Path(path).resolve()
    raw = path.read_bytes()
    report = {'schema_version': 1, 'urdf': str(path), 'sha256': hashlib.sha256(raw).hexdigest(),
              'minimum_mass_kg': minimum_mass_kg, 'physical_validation': 'unvalidated',
              'errors': [], 'warnings': [], 'links': [], 'mimic_joints': []}
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        report['errors'].append(f'Invalid XML: {exc}')
        report['structurally_usable_for_dynamics'] = False
        return report
    try:
        UrdfModel(ModelProfile.from_urdf(path), apply_review=False)
    except (ValueError, OSError) as exc:
        report['errors'].append(f'Kinematics/assets: {exc}')
    for link in root.findall('link'):
        name = link.get('name')
        entry = {'name': name}
        report['links'].append(entry)
        try:
            inertials = link.findall('inertial')
            if len(inertials) != 1:
                raise ValueError('needs one explicit inertial element (review fixed dummy frames separately)')
            inertial = inertials[0]
            entry['origin'] = origin(inertial)
            masses, tensors = inertial.findall('mass'), inertial.findall('inertia')
            if len(masses) != 1 or len(tensors) != 1:
                raise ValueError('needs one mass and one inertia tensor')
            mass = numbers([masses[0].get('value')], 1, 'mass', True)[0]
            entry['mass_kg'] = mass
            if mass <= minimum_mass_kg:
                report['errors'].append(f'{name}: mass {mass:g} kg is below the audit minimum; likely an exporter placeholder')
            xx, xy, xz, yy, yz, zz = numbers(
                [tensors[0].get(k) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')], 6, 'inertia')
            principal = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
            entry['principal_inertia_kg_m2'] = principal.tolist()
            if principal[0] <= 0:
                raise ValueError('inertia tensor must be positive definite')
            if principal[2] > principal[0] + principal[1] + 1e-9 * principal[2]:
                raise ValueError('principal inertias violate the triangle inequality')
            if not link.findall('collision'):
                report['warnings'].append(f'{name}: no collision geometry; check contact coverage')
        except ValueError as exc:
            report['errors'].append(f'{name}: {exc}')
    for joint in root.findall('joint'):
        name, kind = joint.get('name'), joint.get('type')
        if joint.find('mimic') is not None:
            report['mimic_joints'].append(name)
        if kind != 'fixed':
            try:
                limit = joint.find('limit')
                if limit is None:
                    raise ValueError('needs explicit effort and velocity limits')
                numbers([limit.get('effort'), limit.get('velocity')], 2, 'effort/velocity limits', True)
            except ValueError as exc:
                report['errors'].append(f'{name}: {exc}')
    report['total_declared_mass_kg'] = sum(link.get('mass_kg', 0.) for link in report['links'])
    report['structurally_usable_for_dynamics'] = not report['errors']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--minimum-mass-kg', type=float, default=1e-6)
    args = parser.parse_args()
    report = audit_urdf(args.urdf, minimum_mass_kg=args.minimum_mass_kg)
    text = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    else:
        print(text, end='')
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
