"""The room around the task scene: the committed backdrop and floor are what the seeded painting makes, MuJoCo shows each
backdrop face where the painting meant it, the lights are warm without being brighter, and a look changes what the
cameras see and nothing the physics uses."""
import json
from pathlib import Path

import numpy as np
import pytest

from robot_platform.simulation import room
from robot_platform.simulation.task_scene import ROOM_BACKDROP, MinervaTaskScene
from robot_platform.simulation.workcell import Workcell, load_workcell

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    return mapping


def test_the_committed_backdrop_and_floor_are_what_the_room_paints():
    from PIL import Image
    faces = room.backdrop()
    assert set(faces) == set(room.FACES)
    for face, pixels in faces.items():
        assert pixels.shape == (room.SIZE, room.SIZE, 3)
        assert np.array_equal(np.asarray(Image.open(room.ASSETS / f'{face}.png').convert('RGB')), pixels), face
    assert np.array_equal(np.asarray(Image.open(room.ASSETS / 'floor.png').convert('RGB')), room.floor_texture())
    # Blurred: neighbouring pixels differ little, as an out-of-focus backdrop's do.
    assert all(np.abs(np.diff(pixels.astype(int), axis=1)).mean() < 1.5 for pixels in faces.values())


def test_mujoco_shows_each_backdrop_face_where_the_painting_meant_it(tmp_path):
    """Faces of random 8 x 8 px blocks; cameras looking along each axis: every pixel MuJoCo renders at a block's middle
    has the colour face_directions says lies in that direction."""
    import mujoco
    from PIL import Image
    rng = np.random.default_rng(3)
    size, block = 32, 8
    faces = {}
    for face in room.FACES:
        blocks = rng.integers(20, 236, (size // block, size // block, 3), dtype=np.uint8)
        faces[face] = np.repeat(np.repeat(blocks, block, axis=0), block, axis=1)
        Image.fromarray(faces[face]).save(tmp_path / f'{face}.png')
    files = ' '.join(f'file{face}="{(tmp_path / f"{face}.png").as_posix()}"' for face in room.FACES)
    views = {'+x': ((0, -1, 0), (0, 0, 1)), '-x': ((0, 1, 0), (0, 0, 1)), '+y': ((1, 0, 0), (0, 0, 1)),
             '-y': ((-1, 0, 0), (0, 0, 1)), '+z': ((1, 0, 0), (0, -1, 0)), '-z': ((1, 0, 0), (0, 1, 0))}
    cameras = ''.join(f'<camera name="{name}" xyaxes="{" ".join(map(str, (*x, *y)))}" fovy="90"/>'
                      for name, (x, y) in views.items())
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><statistic extent="2" center="0 0 0"/><asset><texture type="skybox" {files}/>'
        f'</asset><visual><global offwidth="64" offheight="64"/></visual><worldbody>'
        f'<geom type="sphere" size=".01" pos="0 0 -5" rgba="0 0 0 0"/>{cameras}</worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    directions = {face: room.face_directions(face, size) for face in room.FACES}
    renderer = mujoco.Renderer(model, 64, 64)
    checked = 0
    try:
        for name, (x, y) in views.items():
            renderer.update_scene(data, camera=name)
            image = renderer.render()
            right, up = np.array(x, float), np.array(y, float)
            forward = -np.cross(right, up)
            for row in range(4, 64, 8):
                for col in range(4, 64, 8):
                    a, b = (col + .5) / 32 - 1, 1 - (row + .5) / 32
                    direction = forward + a * right + b * up
                    direction /= np.linalg.norm(direction)
                    face = max(room.FACES, key=lambda f: float(directions[f][size // 2, size // 2] @ direction))
                    fr, fc = np.unravel_index(np.argmax(directions[face] @ direction), (size, size))
                    if min(fr % block, fc % block) < 2 or max(fr % block, fc % block) > block - 3:
                        continue  # Near a block's edge: texture filtering may mix two blocks.
                    assert np.abs(image[row, col].astype(int) - faces[face][fr, fc]).max() <= 3, (name, row, col)
                    checked += 1
    finally:
        renderer.close()
    assert checked >= 90  # Of 384 sampled pixels, those well inside a block.


def test_warm_lights_are_as_bright_as_white():
    for kelvin in (3900., room.LOOK['light_kelvin'], 5100.):
        rgb = room.kelvin_rgb(kelvin)
        assert float(rgb @ (.2126, .7152, .0722)) == pytest.approx(1.)
        assert rgb[0] > rgb[1] > rgb[2]  # Warm: more red than blue.
    assert np.allclose(room.kelvin_rgb(6600.), 1., atol=.03)


def test_a_look_changes_what_the_cameras_see_and_nothing_the_physics_uses():
    """A workcell's own look (as a drawn layout gives one): the robot's grey and the floor's tone on their colours, the
    lights' warmth on their colour, the room's tone on the backdrop's pixels; every physics array as without it."""
    base = load_workcell(MEASURED)
    look = {'robot_grey': .19, 'room_tone': (1.06, .97, .93), 'floor_tone': (.92, 1.04, 1.08), 'light_kelvin': 5000.}
    plain = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad', workcell=base)
    looked = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad',
                              workcell=Workcell.from_description(dict(base.describe(), look=look)))
    try:
        for key in ('geom_size', 'geom_pos', 'geom_quat', 'geom_friction', 'geom_solref', 'geom_solimp', 'geom_contype',
                    'geom_conaffinity', 'body_mass', 'body_inertia', 'opt.timestep'):
            first, second = (getattr(scene.model.opt, key[4:]) if key.startswith('opt.') else
                             getattr(scene.model, key) for scene in (plain, looked))
            assert np.array_equal(first, second), key
        assert plain.capabilities['contact_geometry_sha256'] == looked.capabilities['contact_geometry_sha256']
        assert plain.capabilities['scene_sha256'] != looked.capabilities['scene_sha256']
        assert looked.capabilities['look'] == {key: list(value) if isinstance(value, tuple) else value
                                               for key, value in look.items()}
        material = looked._id('material', 'task/robot/dark_grey')
        assert np.allclose(looked.model.mat_rgba[material], (.19, .19, .19, 1.))
        floor = looked._id('geom', 'task/floor')
        assert np.allclose(looked.model.geom_rgba[floor], (.92, 1.04, 1.08, 1.))
        assert np.allclose(looked.model.light_diffuse[0], .7 * room.kelvin_rgb(5000.), atol=1e-6)
        assert np.allclose(looked.model.vis.headlight.ambient, .35 * room.kelvin_rgb(5000.), atol=1e-6)
        texture = plain._id('texture', ROOM_BACKDROP)
        start, count = int(plain.model.tex_adr[texture]), int(plain.model.tex_width[texture] *
                                                               plain.model.tex_height[texture] * 3)
        before = plain.model.tex_data[start:start + count].reshape(-1, 3).astype(float)
        after = looked.model.tex_data[start:start + count].reshape(-1, 3).astype(float)
        assert np.allclose(after, np.clip(np.round(before * (1.06, .97, .93)), 0, 255))
    finally:
        plain.close()
        looked.close()
