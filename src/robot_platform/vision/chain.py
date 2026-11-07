"""A URDF's kinematic chain from its root link to one link: where that link is for given joint positions.

Only the joints on the path are read, and positions are taken as given: a calibration must place a pose the robot
actually reached, so nothing here clamps or rejects positions against URDF limits. Joint `offsets` (radians or metres,
by name) are added to the positions; a calibration can fit them.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from robot_platform.models.urdf import origin
from .geometry import rotation, rotations, rpy_matrix, transform


class Chain:
    def __init__(self, urdf_path, tip):
        root = ET.fromstring(Path(urdf_path).read_bytes())
        by_child = {}
        for element in root.findall('joint'):
            child = element.find('child').get('link')
            by_child[child] = element
        links = {link.get('name') for link in root.findall('link')}
        if tip not in links:
            raise ValueError(f'No link {tip!r} in {urdf_path}')
        path, parents, link = [], [], tip
        while link in by_child:
            element = by_child[link]
            if element.find('mimic') is not None:
                raise ValueError(f'Joint {element.get("name")} on the path to {tip} is a mimic joint')
            kind = element.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError(f'Unsupported joint type {kind!r}: {element.get("name")}')
            o = origin(element)
            axis_element = element.find('axis')
            axis = np.array(
                [float(v) for v in (axis_element.get('xyz') if axis_element is not None else '1 0 0').split()]
            )
            if kind != 'fixed' and not math.isclose(float(np.linalg.norm(axis)), 1.0, abs_tol=1e-6):
                raise ValueError(f'Joint {element.get("name")} axis must be a unit vector')
            path.append((element.get('name'), kind, transform(rpy_matrix(*o['rpy']), o['xyz']), axis))
            link = element.find('parent').get('link')
            parents.append(link)
        self.root, self.tip = link, tip
        self.path = tuple(reversed(path))
        self.joints = tuple(name for name, kind, _, _ in self.path if kind != 'fixed')
        # The link each joint on the path hangs from, in path order.
        self.parents = tuple(reversed(parents))

    def before_last_joint(self):
        """The link the last moving joint turns about: where a camera fixed before that joint would sit."""
        moving = [k for k, (_, kind, _, _) in enumerate(self.path) if kind != 'fixed']
        if not moving:
            raise ValueError(f'No moving joint on the path to {self.tip}')
        return self.parents[moving[-1]]

    def pose(self, positions, offsets=None):
        """T_root_tip (4x4) for {joint name: position}; every moving joint on the path must be given."""
        offsets = offsets or {}
        result = np.eye(4)
        for name, kind, fixed, axis in self.path:
            result = result @ fixed
            if kind == 'fixed':
                continue
            if name not in positions:
                raise ValueError(f'No position for joint {name}')
            value = float(positions[name]) + float(offsets.get(name, 0.0))
            if not math.isfinite(value):
                raise ValueError(f'Joint {name} position must be finite')
            if kind == 'prismatic':
                result = result @ transform(np.eye(3), axis * value)
            else:
                result = result @ transform(rotation(axis * value), (0.0, 0.0, 0.0))
        return result

    def positions(self, states):
        """(N, J) positions of {joint: position} states, in self.joints order."""
        try:
            return np.array([[float(state[name]) for name in self.joints] for state in states], float).reshape(
                -1, len(self.joints)
            )
        except KeyError as exc:
            raise ValueError(f'No position for joint {exc.args[0]}') from exc

    def poses(self, positions, offsets=None):
        """(N, 4, 4) T_root_tip for (N, J) positions in self.joints order, plus optional (J,) offsets."""
        positions = np.asarray(positions, float)
        if offsets is not None:
            positions = positions + np.asarray(offsets, float)
        result = np.broadcast_to(np.eye(4), (len(positions), 4, 4)).copy()
        j = 0
        for _, kind, fixed, axis in self.path:
            result = result @ fixed
            if kind == 'fixed':
                continue
            motion = np.broadcast_to(np.eye(4), (len(positions), 4, 4)).copy()
            if kind == 'prismatic':
                motion[:, :3, 3] = positions[:, j : j + 1] * axis
            else:
                motion[:, :3, :3] = rotations(positions[:, j : j + 1] * axis)
            result = result @ motion
            j += 1
        return result
