#!/usr/bin/env python3
"""
ROS 2 LewanSoul LX‑16A Head Servo Driver
========================================

* Controls one or more **LewanSoul LX‑16A** servos on `/dev/ttyUSB1` using the
  `lewansoul_lx16a` SDK (single‑wire, 115200 baud by default).
* **Subscribes** to `/head_group_cmd` (`sensor_msgs/JointState`, radians).
* **Publishes**   `/joint_states` & `/diagnostics`.
* Auto‑scans ID range (default 1‑10) if no mapping parameters are provided.
* No ROM register reads (angle limits, etc.) beyond simple *get_position* for
  feedback – keeps protocol traffic minimal.

Parameters
----------
```yaml
serial_port:   "/dev/ttyUSB1"
baudrate:      115200
scan_range:    "1:10"          # ping IDs if joint_map / joint_ids unset
joint_map:     []              # list of {name,id}
joint_ids:     []              # simple list, names auto‑generated
state_poll_hz: 25.0
```

Range & conversion
------------------
LX‑16A maps **0 – 1000** internal units ≈ **0 – 240 degrees**.

```python
pulse = rad * 1000 / (240° in rad)  = rad * 1000 / 4.18879
```
"""
import math, threading
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from serial import Serial
from lewansoul_lx16a import ServoController, TimeoutError  # type: ignore

# ───────────────────────── constants / helpers ──────────────────────────── #
_FULL_RANGE_RAD = math.radians(240.0)           # 4.18879
_SCALE_PULSE = 1000.0 / _FULL_RANGE_RAD         # ≈ 238.73


def rad_to_pulse(rad: float) -> int:
    """Clamp rad → 0‑1000 pulses."""
    val = int(max(0, min(1000, rad * _SCALE_PULSE)))
    return val


def pulse_to_rad(pulse: int) -> float:
    return (pulse / 1000.0) * _FULL_RANGE_RAD

# ───────────────────────────────── Node ──────────────────────────────────── #
class LX16AHeadDriver(Node):
    def __init__(self):
        super().__init__('lx16a_head_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('scan_range', '1:10')
        self.declare_parameter('joint_map', [])
        self.declare_parameter('joint_ids', [])
        self.declare_parameter('state_poll_hz', 25.0)

        port     = self.get_parameter('serial_port').value
        baud     = int(self.get_parameter('baudrate').value)
        poll_hz  = float(self.get_parameter('state_poll_hz').value)

        # Serial + controller
        try:
            ser = Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            raise SystemExit(f'Cannot open {port}: {e}')
        self.ctrl = ServoController(ser, timeout=0.2)
        self.get_logger().info(f'Port {port} opened @ {baud} baud')

        # Servo discovery
        self.joint_names, self.ids = self._discover_servos()
        self.servos = {n: self.ctrl.servo(i) for n, i in self.ids.items()}
        self.get_logger().info(f'Head servos: {", ".join(f"{n}(id={i})" for n,i in self.ids.items())}')

        # ROS I/O
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.cmd_sub   = self.create_subscription(JointState, '/head_group_cmd', self._cmd_cb, 10)
        self.state_pub = self.create_publisher  (JointState, '/joint_states', qos)
        self.diag_pub  = self.create_publisher  (DiagnosticArray, '/diagnostics', 10)

        self._lock = threading.Lock()
        self.create_timer(1.0 / poll_hz, self._poll_loop)

    # ───────── Servo discovery ──────────
    def _discover_servos(self):
        jm = self.get_parameter('joint_map').value
        if jm:
            if isinstance(jm[0], str):
                import yaml; jm = [yaml.safe_load(s) for s in jm]
            names = [j['name'] for j in jm]
            ids   = {j['name']: int(j['id']) for j in jm}
            return names, ids

        jids = self.get_parameter('joint_ids').value
        if jids:
            names = [f'head_{i:02d}' for i in jids]
            return names, dict(zip(names, map(int, jids)))

        start, end = map(int, (self.get_parameter('scan_range').value or '1:10').split(':'))
        self.get_logger().info(f'Scanning LX‑16A IDs {start}–{end}')
        names, ids = [], {}
        for sid in range(start, end + 1):
            try:
                found_id = self.ctrl.get_servo_id(sid, timeout=0.05)
                if found_id == sid:
                    nm = f'head_{sid:02d}'
                    names.append(nm); ids[nm] = sid
            except TimeoutError:
                continue
        if not ids:
            raise SystemExit('No LX‑16A servos detected')
        return names, ids

    # ───────── Command callback ──────────
    def _cmd_cb(self, msg: JointState):
        with self._lock:
            for name, rad in zip(msg.name, msg.position):
                servo = self.servos.get(name)
                if not servo:
                    continue
                servo.move(rad_to_pulse(rad), time=0)  # immediate

    # ───────── Poll loop ──────────
    def _poll_loop(self):
        js   = JointState(); js.header.stamp = self.get_clock().now().to_msg()
        diag = DiagnosticArray(header = js.header)
        with self._lock:
            for name, servo in self.servos.items():
                try:
                    pulse = servo.get_position(timeout=0.05)
                    js.name.append(name); js.position.append(pulse_to_rad(pulse))
                    st = DiagnosticStatus(name=f'Servo {name}', level=DiagnosticStatus.OK, message='OK')
                    st.values.append(KeyValue(key='pulse', value=str(pulse)))
                except TimeoutError:
                    st = DiagnosticStatus(name=f'Servo {name}', level=DiagnosticStatus.ERROR, message='Timeout')
                diag.status.append(st)
        self.state_pub.publish(js); self.diag_pub.publish(diag)

    def destroy_node(self):
        self.ctrl._serial.close()
        super().destroy_node()

# ───────────────────────── main ──────────────────────────── #

def main():
    rclpy.init()
    node = LX16AHeadDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
