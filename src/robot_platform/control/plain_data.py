"""Copies of the plain data the control loop passes around (receiver requests, replies, feedback, joint tables)."""
from copy import deepcopy

_ATOMS = (int, float, str, bool, type(None))


def plain_copy(value):
    """An independent copy of plain data (dicts, lists, tuples, numbers, strings), as copy.deepcopy makes it.

    deepcopy's per-object memo and keep-alive bookkeeping cost more than the copy itself. With both arms it was
    most of a direct-teleop cycle's Python time on the Jetson, and the tuning rollback snapshots pushed an Apply
    cycle past its 10 ms slot (2026-10-11). Anything other than these exact types (subclasses such as named
    tuples included) is handed to deepcopy, so no value is ever shared with the original.
    """
    if isinstance(value, _ATOMS):
        return value
    kind = type(value)
    if kind is dict:
        return {key: plain_copy(item) for key, item in value.items()}
    if kind is list:
        return [plain_copy(item) for item in value]
    if kind is tuple:
        return tuple(plain_copy(item) for item in value)
    return deepcopy(value)
