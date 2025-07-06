#!/usr/bin/env python3
"""
LX‑224 Head Bridge (ROS 2 → serial)
===================================

* Subscribes : `/head_cmd` (sensor_msgs/JointState, radians).
* Clamps, optionally **inverts**, and scales each joint to its calibrated
  pulse limits, then streams pulses to LX‑224 servos.

–– Update ––
* `head_yaw` **and** `head_pitch` directions are now **reversed** (multiply
  radians by −1 before mapping).  Toggle by editing `INVERTED_JOINTS` set.

Parameters
----------
- **port** (str)  : serial port                (default `/dev/ttyUSB1`)
- **baud** (int)  : baud rate                  (default `115200`)
- **limits_file** (str): YAML file with per‑joint `min` / `max` pulses

YAML example:
```yaml
head_yaw:   {id: 1, min: 17,  max: 985}
head_pitch: {id: 2, min: 392, max: 978}
head_roll:  {id: 3, min: 16,  max: 981}
```
"""
import math, yaml, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial import Serial
from lewansoul_lx16a import ServoController, TimeoutError

_FULL_RANGE_RAD = math.radians(240.0)  # 0‑1000 pulses ≃ 240 °
INVERTED_JOINTS = {"head_yaw", "head_pitch"}  # ← change sign for these

class LX224Bridge(Node):
    def __init__(self):
        super().__init__('lx224_head_bridge')
        # ───── parameters ─────
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('limits_file', './lx224_limits.yaml')

        port  = self.get_parameter('port').value
        baud  = int(self.get_parameter('baud').value)
        lfile = self.get_parameter('limits_file').value

        # ───── load limits ─────
        with open(lfile, 'r') as fh:
            cfg = yaml.safe_load(fh)
        self.limits = {j: {k: int(v) for k, v in spec.items()} for j, spec in cfg.items()}

        # ───── attach servos ─────
        ser = Serial(port, baudrate=baud, timeout=0.1)
        ctrl = ServoController(ser, timeout=0.05)
        self.servos = {}
        for name, spec in self.limits.items():
            try:
                ctrl.get_servo_id(spec['id'], timeout=0.05)
                self.servos[name] = ctrl.servo(spec['id'])
            except TimeoutError:
                self.get_logger().warn(f'Servo id {spec["id"]} ({name}) not responding')
        if not self.servos:
            raise SystemExit('No LX‑224 servos detected!')

        self.create_subscription(JointState, '/head_cmd', self._cmd_cb, 10)
        self.get_logger().info(f'Bridge active on {port}@{baud}; inverted joints: {INVERTED_JOINTS}')

    # ── helpers ──
    @staticmethod
    def _rad_to_pulse(rad: float, lo: int, hi: int) -> int:
        span, center = hi - lo, (hi + lo) / 2
        return max(lo, min(hi, int(rad / _FULL_RANGE_RAD * span + center)))

    # ── callback ──
    def _cmd_cb(self, msg: JointState):
        for name, rad in zip(msg.name, msg.position):
            spec = self.limits.get(name)
            servo = self.servos.get(name)
            if not spec or not servo:
                continue
            if name in INVERTED_JOINTS:
                rad = -rad
            pulse = self._rad_to_pulse(rad, spec['min'], spec['max'])
            servo.move(pulse)

# ───── main ─────

def main():
    rclpy.init()
    try:
        rclpy.spin(LX224Bridge())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
