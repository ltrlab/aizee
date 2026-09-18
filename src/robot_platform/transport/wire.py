"""Shared ZMQ wire format for the AIZEE pipeline.

Cameras, telemetry, and commands all flowed as `send_json(dict)` /
`recv_string()` -> `json.loads()`.  For cameras, the dict embedded a
base64-encoded JPEG (and optionally a base64-encoded depth buffer), so
every frame paid:
  - publisher base64 encode + json.dumps,
  - 33% bandwidth inflation,
  - consumer json.loads of a 100KB+ string,
  - consumer base64.b64decode.

This module replaces that with:
  - msgpack instead of JSON for the small dicts,
  - ZMQ multipart for cameras: header msgpack + raw JPEG/depth bytes.

A `parts` list in the header names the frames that follow in order.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional

import msgpack


def pack_msg(obj: Any) -> bytes:
    """msgpack-encode a plain dict/scalar for telem/cmd/UPS channels."""
    return msgpack.packb(obj, use_bin_type=True)


def unpack_msg(buf: bytes) -> Any:
    """Inverse of pack_msg."""
    return msgpack.unpackb(buf, raw=False)


def pack_camera(
    header: dict,
    color_bytes: Optional[bytes] = None,
    depth_bytes: Optional[bytes] = None,
) -> list:
    """Encode a camera message as ZMQ multipart frames.

    Returns a list suitable for `socket.send_multipart(...)`:
      [msgpack(header_with_parts), color_bytes?, depth_bytes?]

    `header` should contain the metadata for whichever streams are
    present (e.g. `header["color"] = {"format": "jpeg", "width": ...,
    "height": ...}`).  Don't put the raw bytes in the header — pass
    them as the `color_bytes` / `depth_bytes` arguments.
    """
    parts: list = []
    bodies: list = []
    if color_bytes is not None:
        parts.append("color")
        bodies.append(color_bytes)
    if depth_bytes is not None:
        parts.append("depth")
        bodies.append(depth_bytes)
    header = dict(header)
    header["parts"] = parts
    return [msgpack.packb(header, use_bin_type=True), *bodies]


def unpack_camera(frames: Iterable[bytes]) -> dict:
    """Decode multipart frames produced by `pack_camera`.

    Returns a dict shaped like the old JSON message, but with the raw
    bytes attached as `msg["color"]["data_bytes"]` / `msg["depth"]
    ["data_bytes"]` instead of base64-encoded strings under
    `msg["color"]["data"]`.
    """
    flist = list(frames)
    if not flist:
        return {}
    header = msgpack.unpackb(flist[0], raw=False)
    out = dict(header)
    parts = out.pop("parts", [])
    for name, body in zip(parts, flist[1:]):
        meta = out.get(name)
        if isinstance(meta, dict):
            meta = dict(meta)
        else:
            meta = {}
        meta["data_bytes"] = body
        out[name] = meta
    return out
