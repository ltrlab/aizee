"""Explicit e-stop fanout; each destination has independent retry state."""
from robot_platform.transport.wire import pack_msg


def parse_state(line):
    import json
    if line.startswith("#"):
        for item in line[1:].split():
            if item in ("nc=0", "nc=1"):
                return item == "nc=0"
    elif line.startswith("{"):
        try:
            state = json.loads(line).get("estop")
            return state if type(state) is bool else None
        except (ValueError, AttributeError):
            pass
    return None


class EstopFanout:
    def __init__(self, sockets):
        self.sockets = dict(sockets)
        self.state = None
        self.pending = set()

    def update(self, state):
        if type(state) is not bool:
            raise ValueError("e-stop state must be boolean")
        if state != self.state:
            self.state = state
            self.pending = set(self.sockets)

    def flush(self):
        import zmq
        errors = {}
        if self.state is None:
            return errors
        command = pack_msg({"type": "emergency_stop" if self.state else "clear_emergency_stop"})
        # Repeat asserted stops to cover restarted controllers. Clear never
        # enables motion and is sent once per destination.
        for endpoint in self.sockets if self.state else tuple(self.pending):
            try:
                self.sockets[endpoint].send(command, flags=zmq.NOBLOCK)
                self.pending.discard(endpoint)
            except zmq.ZMQError as exc:
                self.pending.add(endpoint)
                errors[endpoint] = str(exc)
        return errors


def bridge_main():
    import argparse
    import time
    import serial
    import zmq
    parser = argparse.ArgumentParser(description="Serial e-stop to MessagePack; repeat --zmq for each controller")
    parser.add_argument("--port", default="/dev/estop-receiver")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--zmq", action="append")
    args = parser.parse_args()
    ctx, sockets, ser = zmq.Context(), {}, None
    try:
        for endpoint in dict.fromkeys(args.zmq or ["tcp://localhost:5555"]):
            sock = ctx.socket(zmq.PUSH)
            sockets[endpoint] = sock
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.IMMEDIATE, 1)
            sock.setsockopt(zmq.SNDHWM, 1)
            sock.connect(endpoint)
        fanout = EstopFanout(sockets)
        last_log = retry_at = 0.0
        while True:
            now = time.monotonic()
            if ser is None and now >= retry_at:
                try:
                    ser = serial.Serial(args.port, args.baud, timeout=0.02)
                except serial.SerialException:
                    retry_at = now + 2
            if ser is not None:
                try:
                    state = parse_state(ser.readline().decode(errors="replace").strip())
                    if state is not None:
                        fanout.update(state)
                except serial.SerialException:
                    ser.close()
                    ser = None
                    retry_at = now + 1
            errors = fanout.flush()
            if errors and now - last_log > 2:
                print(f"[estop] delivery pending: {errors}", flush=True)
                last_log = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            ser.close()
        for sock in sockets.values():
            sock.close(linger=0)
        ctx.term()
