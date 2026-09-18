"""Compatibility imports for the shared wire contract.

Install the repository package with ``python -m pip install -e .``.
New code imports :mod:`robot_platform.transport.wire` directly.
"""

from robot_platform.transport.wire import pack_camera, pack_msg, unpack_camera, unpack_msg

__all__ = ["pack_camera", "pack_msg", "unpack_camera", "unpack_msg"]
