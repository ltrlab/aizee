"""cmd_sender.py — 100 Hz command re-emitter for the Minerva follower.

Simpler than AIZEE's tracking cmd-sender: the main loop computes the 17-DoF
target (from the teleop source) and drops a prebuilt command into the holder;
this thread re-emits it at 100 Hz so the follower's PD loop stays fed between
30 Hz main-loop ticks. Command schema matches minerva_policy_node.send_command
so the follower's motor_control sees identical messages from teleop and policy.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence, Tuple

import zmq

from common.minerva_constants import KD, KP, MINERVA_JOINTS, NUM_MINERVA_JOINTS
from common.wire import pack_msg


def build_arm_command(
    positions: Sequence[float],
    kp: Sequence[float] = KP,
    kd: Sequence[float] = KD,
) -> dict:
    return {
        "type": "arm_joints",
        "joint_names": list(MINERVA_JOINTS),
        "positions": [float(x) for x in positions],
        "velocities": [0.0] * NUM_MINERVA_JOINTS,
        "kp": list(kp),
        "kd": list(kd),
    }


def send_direct(sock, msg: dict, lock: Optional[threading.Lock] = None) -> None:
    """One-shot send (enable/disable/e-stop). Safe fire-and-forget. Pass the
    shared send lock so a direct send never races the 100 Hz re-emitter on the
    same (non-thread-safe) PUSH socket."""
    try:
        if lock is not None:
            with lock:
                sock.send(pack_msg(msg), zmq.NOBLOCK)
        else:
            sock.send(pack_msg(msg), zmq.NOBLOCK)
    except Exception:
        pass


def start_cmd_sender(
    sock, hz: int = 100,
) -> Tuple[threading.Event, threading.Thread, threading.Lock, dict, threading.Lock]:
    lock = threading.Lock()          # guards holder["bundle"]
    send_lock = threading.Lock()     # serialises sock.send across threads (zmq sockets aren't thread-safe)
    holder: dict = {"bundle": None}  # main loop sets holder["bundle"]
    stop = threading.Event()
    period = 1.0 / max(hz, 1)

    def _run() -> None:
        next_t = time.perf_counter() + period
        while not stop.is_set():
            with lock:
                bundle = holder.get("bundle")
            if bundle is not None:
                try:
                    with send_lock:
                        sock.send(pack_msg(bundle), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                except Exception:
                    pass
            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                stop.wait(sleep_t)
            next_t += period
            if next_t < time.perf_counter():
                next_t = time.perf_counter() + period

    thread = threading.Thread(target=_run, daemon=True, name="CmdTx")
    thread.start()
    return stop, thread, lock, holder, send_lock


__all__ = ["build_arm_command", "send_direct", "start_cmd_sender"]
