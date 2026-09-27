"""Acknowledged commissioning transport. No renewal, reconnect replay or auto-arm."""
from robot_platform.transport.wire import pack_msg, unpack_msg


class GuardedZmqTransport:
    def __init__(self, endpoint, *, timeout_ms=500):
        import zmq
        if not endpoint.startswith("tcp://127.0.0.1:"):
            raise ValueError("Guarded commissioning uses loopback; use an SSH tunnel for remote access")
        if not 1 <= timeout_ms <= 5000:
            raise ValueError("timeout_ms must be between 1 and 5000")
        self.endpoint, self.timeout_ms = endpoint, timeout_ms
        self.context, self.socket, self.last = zmq.Context(), None, None
        self.on_result = None  # Called with every result, e.g. to collect its recorder rows.

    def _connect(self):
        import zmq
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.IMMEDIATE, 1)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.connect(self.endpoint)

    def request(self, request):
        import zmq
        if self.socket is None:
            self._connect()
        try:
            self.socket.send(pack_msg({"protocol_version": 1, "request": request}))
            result = unpack_msg(self.socket.recv())
        except zmq.ZMQError as exc:
            self.socket.close(linger=0)
            self.socket = None
            raise TimeoutError("Guarded exchange failed; command acceptance is unknown; no command was retried") from exc
        self.last = result
        if self.on_result is not None:
            self.on_result(result)
        if result.get("error"):
            raise RuntimeError(result["error"])
        return result

    def exchange(self, request):
        result = self.request(request)
        if result.get("reply") is None:
            raise RuntimeError("Receiver did not acknowledge a controller request")
        return result["reply"]

    def close(self):
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
