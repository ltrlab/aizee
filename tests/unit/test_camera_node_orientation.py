"""Publisher image orientation and existing depth-consumer compatibility; no SDK/device access."""

import importlib.util
from io import BytesIO
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

from robot_platform.transport.wire import pack_camera, unpack_camera

ROOT = Path(__file__).resolve().parents[2]


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def camera_node(monkeypatch):
    # SDK imports are stubbed before loading the publisher. No real pipeline,
    # device enumeration, service or socket is opened by these tests.
    monkeypatch.setitem(sys.modules, 'pyrealsense2', SimpleNamespace())
    monkeypatch.syspath_prepend(str(ROOT / 'python'))
    return load_file('camera_orientation_under_test', ROOT / 'python/nodes/camera_node.py')


@pytest.mark.parametrize('rotation', [0, 180])
@pytest.mark.parametrize('output_resolution', [None, (32, 24)])
def test_actual_publisher_rotates_rgb_depth_and_projection_together(
    camera_node, monkeypatch, rotation, output_resolution
):
    color = np.zeros((48, 64, 3), dtype=np.uint8)
    color[:24, :32], color[:24, 32:] = (240, 20, 10), (10, 230, 20)
    color[24:, :32], color[24:, 32:] = (20, 10, 240), (220, 220, 30)
    depth = np.arange(48, dtype=np.uint16).reshape(6, 8) * 31
    original_color, original_depth = color.copy(), depth.copy()
    intrinsics = SimpleNamespace(fx=4.0, fy=5.0, ppx=1.25, ppy=2.75, width=8, height=6)
    frame = SimpleNamespace(
        get_timestamp=lambda: 100.0,
        get_color_frame=lambda: SimpleNamespace(get_data=lambda: color),
        get_depth_frame=lambda: SimpleNamespace(get_data=lambda: depth),
        first_or_default=lambda stream: None,
    )
    profile = SimpleNamespace(
        get_stream=lambda stream: SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(get_intrinsics=lambda: intrinsics)
        ),
        get_device=lambda: SimpleNamespace(first_depth_sensor=lambda: SimpleNamespace(get_depth_scale=lambda: 0.001)),
    )
    pipeline = SimpleNamespace(
        start=lambda config: profile, wait_for_frames=lambda **kwargs: frame, poll_for_frames=lambda: None
    )
    sdk = SimpleNamespace(
        pipeline=lambda: pipeline,
        config=lambda: SimpleNamespace(enable_stream=lambda *args: None),
        stream=SimpleNamespace(color=1, depth=2, accel=3, gyro=4),
        format=SimpleNamespace(rgb8=1, z16=2),
    )
    monkeypatch.setattr(camera_node, 'rs', sdk)
    node = camera_node.CameraNode(
        image_rotation_deg=rotation,
        color_resolution=(64, 48),
        depth_resolution=(8, 6),
        output_resolution=output_resolution,
        jpeg_quality=100,
    )
    node.initialize_camera()
    messages = []

    def send(parts):
        messages.append(unpack_camera(parts))
        node.running = False

    node.zmq_socket = SimpleNamespace(send_multipart=send)
    node.process_frames()
    assert len(messages) == 1
    message = messages[0]
    assert message['image_rotation_deg'] == rotation
    assert message['depth']['intrinsics_frame'] == 'sensor_optical'
    assert message['depth']['scale'] == 0.001
    assert (message['depth']['width'], message['depth']['height']) == (8, 6)
    expected_color = color if rotation == 0 else color[::-1, ::-1]
    expected_depth = depth if rotation == 0 else depth[::-1, ::-1]
    np.testing.assert_array_equal(
        np.frombuffer(message['depth']['data_bytes'], dtype=np.uint16).reshape(6, 8), expected_depth
    )
    decoded = np.asarray(Image.open(BytesIO(message['color']['data_bytes'])))
    expected_color = (
        np.asarray(Image.fromarray(expected_color).resize(output_resolution)) if output_resolution else expected_color
    )
    assert decoded.shape == expected_color.shape
    # JPEG is lossy; compare centers well away from the quadrant boundaries.
    for y in (decoded.shape[0] // 4, decoded.shape[0] * 3 // 4):
        for x in (decoded.shape[1] // 4, decoded.shape[1] * 3 // 4):
            np.testing.assert_allclose(decoded[y, x].astype(float), expected_color[y, x], atol=3)
    original_k = dict(fx=4.0, fy=5.0, cx=1.25, cy=2.75, width=8, height=6)
    k = message['depth']['intrinsics']
    for u, v in ((0, 0), (7, 5), (3, 2)):
        up, vp = (u, v) if rotation == 0 else (7 - u, 5 - v)
        np.testing.assert_allclose(
            [(up - k['cx']) / k['fx'], (vp - k['cy']) / k['fy']],
            [(u - original_k['cx']) / 4.0, (v - original_k['cy']) / 5.0],
        )
    np.testing.assert_array_equal(color, original_color)
    np.testing.assert_array_equal(depth, original_depth)


@pytest.mark.parametrize('rotation', [0, 180])
def test_existing_depth_decoder_preserves_original_sensor_ray(camera_node, rotation):
    images = load_file('orientation_depth_consumer', ROOT / 'python/scripts/collect_demo_app/images.py')
    color = np.full((6, 8, 3), 80, dtype=np.uint8)
    depth = np.zeros((6, 8), dtype=np.uint16)
    depth[1, 2] = 1000
    color, depth = camera_node._rotate_rgbd(color, depth, rotation)
    k = camera_node._rotate_depth_intrinsics(dict(fx=4.0, fy=5.0, cx=1.25, cy=2.75, width=8, height=6), rotation)
    jpeg = BytesIO()
    Image.fromarray(color).save(jpeg, format='JPEG')
    msg = unpack_camera(
        pack_camera(
            dict(
                timestamp=1.0,
                image_rotation_deg=rotation,
                color=dict(format='jpeg', width=8, height=6),
                depth=dict(width=8, height=6, scale=0.001, intrinsics=k, intrinsics_frame='sensor_optical'),
            ),
            jpeg.getvalue(),
            depth.tobytes(),
        )
    )
    cache = dict(gripper=None, gripper_time=0.0, scene=msg, scene_time=1.0)
    stop, thread, lock, decoded, _ = images._start_image_decoder(
        threading.Lock(), cache, (8, 6), always_on=True, scene_proj_stride=1
    )
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with lock:
                points = decoded['scene_pts_cam']
            if points is not None:
                break
            time.sleep(0.001)
        np.testing.assert_allclose(points, [[(2 - 1.25) / 4.0, (1 - 2.75) / 5.0, 1.0]], atol=1e-7)
    finally:
        stop.set()
        thread.join(timeout=2.0)
        assert not thread.is_alive()


@pytest.mark.parametrize('rotation', [0, 180])
@pytest.mark.parametrize('output_resolution', [None, (32, 24)])
def test_the_publisher_registers_depth_to_the_color_picture_it_sends(
    camera_node, monkeypatch, rotation, output_resolution
):
    # Helm's headset textures each depth point with its color pixel: the color camera's intrinsics for the raster
    # as published (rotated, scaled) and the depth sensor's pose in the color camera's optical frame.
    color = np.zeros((48, 64, 3), dtype=np.uint8)
    depth = np.full((6, 8), 1000, dtype=np.uint16)
    depth_k = SimpleNamespace(fx=4.0, fy=5.0, ppx=3.25, ppy=2.75, width=8, height=6)
    color_k = SimpleNamespace(fx=40.0, fy=42.0, ppx=31.5, ppy=23.25, width=64, height=48)
    angle = 0.05  # The color camera turned slightly about y, and 15 mm to the side, as a D435's sits.
    rotation_rows = [[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]]
    column_major = [rotation_rows[row][col] for col in range(3) for row in range(3)]
    extrinsics = SimpleNamespace(rotation=column_major, translation=[0.015, 0.001, -0.002])

    def stream(kind):
        k = color_k if kind == 1 else depth_k
        return SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(get_intrinsics=lambda: k),
            get_extrinsics_to=lambda other: extrinsics,
        )

    frame = SimpleNamespace(
        get_timestamp=lambda: 100.0,
        get_color_frame=lambda: SimpleNamespace(get_data=lambda: color),
        get_depth_frame=lambda: SimpleNamespace(get_data=lambda: depth),
        first_or_default=lambda stream: None,
    )
    profile = SimpleNamespace(
        get_stream=stream,
        get_device=lambda: SimpleNamespace(first_depth_sensor=lambda: SimpleNamespace(get_depth_scale=lambda: 0.001)),
    )
    pipeline = SimpleNamespace(
        start=lambda config: profile, wait_for_frames=lambda **kwargs: frame, poll_for_frames=lambda: None
    )
    monkeypatch.setattr(
        camera_node,
        'rs',
        SimpleNamespace(
            pipeline=lambda: pipeline,
            config=lambda: SimpleNamespace(enable_stream=lambda *args: None),
            stream=SimpleNamespace(color=1, depth=2, accel=3, gyro=4),
            format=SimpleNamespace(rgb8=1, z16=2),
        ),
    )
    node = camera_node.CameraNode(
        image_rotation_deg=rotation,
        color_resolution=(64, 48),
        depth_resolution=(8, 6),
        output_resolution=output_resolution,
    )
    node.initialize_camera()
    messages = []

    def send(parts):
        messages.append(unpack_camera(parts))
        node.running = False

    node.zmq_socket = SimpleNamespace(send_multipart=send)
    node.process_frames()
    message = messages[0]
    np.testing.assert_allclose(message['depth']['to_color']['rotation'], rotation_rows)
    np.testing.assert_allclose(message['depth']['to_color']['translation'], [0.015, 0.001, -0.002])
    k, ck = message['depth']['intrinsics'], message['color']['intrinsics']
    width, height = output_resolution or (64, 48)
    assert (ck['width'], ck['height']) == (width, height) == (message['color']['width'], message['color']['height'])
    # A depth pixel of the published raster lands, through the published registration, on the color pixel the
    # original sensors give, rotated and scaled as the published color raster is.
    for u, v in ((0, 0), (7, 5), (3, 2)):
        z = 1.0
        point = np.array([(u - k['cx']) / k['fx'] * z, (v - k['cy']) / k['fy'] * z, z])
        seen = np.array(rotation_rows) @ point + extrinsics.translation
        published = (ck['fx'] * seen[0] / seen[2] + ck['cx'], ck['fy'] * seen[1] / seen[2] + ck['cy'])
        uo = 40.0 * seen[0] / seen[2] + 31.5
        vo = 42.0 * seen[1] / seen[2] + 23.25
        if rotation == 180:
            uo, vo = 63 - uo, 47 - vo
        expected = ((uo + 0.5) * width / 64 - 0.5, (vo + 0.5) * height / 48 - 0.5)
        np.testing.assert_allclose(published, expected, atol=1e-9)


def test_without_an_sdk_registration_the_stream_goes_on_without_it(camera_node, monkeypatch):
    profile = SimpleNamespace(
        get_stream=lambda kind: SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(
                get_intrinsics=lambda: SimpleNamespace(fx=4.0, fy=5.0, ppx=1.25, ppy=2.75, width=8, height=6)
            )
        )
    )  # No get_extrinsics_to.
    monkeypatch.setattr(camera_node, 'rs', SimpleNamespace(stream=SimpleNamespace(color=1, depth=2)))
    assert camera_node._color_registration(profile, profile.get_stream(2), 0) == (None, None)


@pytest.mark.parametrize('invalid', [True, False, 90, -180, 360, 180.0, '180', None])
def test_bad_rotation_fails_before_any_camera_initialization(camera_node, invalid):
    with pytest.raises(ValueError, match='integer 0 or 180'):
        camera_node.CameraNode(image_rotation_deg=invalid)


def test_default_and_yaml_rotation(camera_node):
    assert camera_node.CameraNode().image_rotation_deg == 0
    assert camera_node._kwargs_from_config({})['image_rotation_deg'] == 0
    kwargs = camera_node._kwargs_from_config({'camera': {'id': 'scene_cam', 'image_rotation_deg': 180}})
    assert camera_node.CameraNode(**kwargs).image_rotation_deg == 180


def test_cli_rotation_is_passed_to_publisher(camera_node, monkeypatch):
    supplied = []
    monkeypatch.setattr(
        camera_node, 'CameraNode', lambda **kw: supplied.append(kw) or SimpleNamespace(run=lambda: None)
    )
    monkeypatch.setattr(camera_node.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(sys, 'argv', ['camera_node.py', '--image-rotation-deg', '180'])
    camera_node.main()
    assert supplied[0]['image_rotation_deg'] == 180
