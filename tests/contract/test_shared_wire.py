"""Wire compatibility checks independent of hardware or ZMQ sockets."""

import msgpack

from robot_platform.transport import wire


def test_message_bytes_match_existing_protocol():
    # Golden bytes: a named command map and binary payload, not JSON/base64.
    message = {"Enable": True, "payload": b"\x00\xff"}
    expected = b"\x82\xa6Enable\xc3\xa7payload\xc4\x02\x00\xff"
    assert wire.pack_msg(message) == expected
    assert wire.unpack_msg(expected) == message


def test_camera_frames_preserve_order_bytes_and_caller_metadata():
    header = {
        "timestamp": 1.25,
        "color": {"format": "jpeg", "width": 2, "height": 1},
        "depth": {"format": "z16"},
    }
    color, depth = b"\xff\xd8jpeg", b"\x00\x01\x02\x03"
    frames = wire.pack_camera(header, color, depth)
    assert frames == [
        msgpack.packb({**header, "parts": ["color", "depth"]}, use_bin_type=True),
        color,
        depth,
    ]
    decoded = wire.unpack_camera(iter(frames))
    assert decoded == {
        "timestamp": 1.25,
        "color": {**header["color"], "data_bytes": color},
        "depth": {**header["depth"], "data_bytes": depth},
    }
    assert "parts" not in header
    assert "data_bytes" not in header["color"]


def test_optional_camera_frames_and_empty_input():
    assert wire.unpack_camera([]) == {}
    assert wire.unpack_camera(wire.pack_camera({"sequence": 4})) == {"sequence": 4}
    assert wire.unpack_camera(wire.pack_camera({}, depth_bytes=b"")) == {
        "depth": {"data_bytes": b""}
    }


def test_legacy_wire_imports_reexport_shared_functions():
    from python.common import wire as legacy

    for name in ("pack_msg", "unpack_msg", "pack_camera", "unpack_camera"):
        assert getattr(legacy, name) is getattr(wire, name)
