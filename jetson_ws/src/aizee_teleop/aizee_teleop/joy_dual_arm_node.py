#!/usr/bin/env python3
"""
joy_dual_arm_node  – Xbox-One controller → MoveIt Servo (dual Aizee arms).

Mapping = PickNik / MoveIt Servo example (axes & buttons enumerated below).
Hold LB to steer left arm only, RB for right arm only, nothing for both.
Press MENU once (button 11, “≡”) to start both Servo nodes.

Author: LTR Labs
"""

from __future__ import annotations
import rclpy, numpy as np
from rclpy.node         import Node
from sensor_msgs.msg    import Joy, JointState
from geometry_msgs.msg  import TwistStamped
from std_srvs.srv       import Trigger

# ───────────── XBOX-One map from your C++ sample ──────────────────────────
class Ax:     # axes indices
    LX, LY         = 0, 1        # left stick
    RX, RY         = 2, 3        # right stick
    RT, LT         = 4, 5        # triggers  (idle = 0.0)
    D_PAD_X, D_PAD_Y = 6, 7      # d-pad

class Btn:    # button indices
    A, B, X, Y         = 0, 1, 3, 4
    LB, RB             = 6, 7
    MENU               = 11       # “≡”  (we use this as “start-servo”)
# ───────────────────────────────────────────────────────────────────────────

_DEADZONE   = 0.08     # ignore tiny stick noise
_LIN_GAIN   = 1.0      # m s⁻¹ per full deflection
_ANG_GAIN   = 1.5      # rad s⁻¹ per full deflection
_LIN_CLAMP  = 0.25
_ANG_CLAMP  = 1.0
_FILL_HZ    = 10.0     # missing-joint filler

_MISSING_JOINTS = [
    "bl_wheel_joint", "br_wheel_joint", "fl_wheel_joint", "fr_wheel_joint",
    "gantry_base_joint", "gantry_swivel_joint",
    "head_swivel_joint", "head_tilt_joint", "head_pitch_joint",
]

# ───────────────────────────────────────────────────────────────────────────
class JoyDualArm(Node):
    def __init__(self) -> None:
        super().__init__("joy_dual_arm_node")

        # pubs
        self.pub_left  = self.create_publisher(
            TwistStamped, "/left_arm/servo_node/delta_twist_cmds", 10)
        self.pub_right = self.create_publisher(
            TwistStamped, "/right_arm/servo_node/delta_twist_cmds", 10)

        # start-servo clients
        self.cli_l = self.create_client(Trigger,
                                        "/left_arm/servo_node/start_servo")
        self.cli_r = self.create_client(Trigger,
                                        "/right_arm/servo_node/start_servo")
        self.servo_started = False

        # joy sub
        self.create_subscription(Joy, "/joy", self.cb_joy, 20)

        # dummy joint-state filler
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0/_FILL_HZ, self._fill_joints)

        self.get_logger().info("Joy-→Servo bridge (corrected map) ready")

    # ────────────────────────────────────────────────────────────────
    def cb_joy(self, msg: Joy) -> None:
        if not self.servo_started and len(msg.buttons) > Btn.MENU \
           and msg.buttons[Btn.MENU]:
            self._start_servo()

        # dead-zone helper
        dz = lambda v: 0.0 if abs(v) < _DEADZONE else v
        axes = [dz(a) for a in msg.axes]

        # linear:    x via triggers  (RT – LT)  ;  y / z via sticks
        vx = _LIN_GAIN * (axes[Ax.RT] - axes[Ax.LT])     # forward/back
        vy = _LIN_GAIN * axes[Ax.LX]
        vz = _LIN_GAIN * axes[Ax.LY]

        # angular:   roll = bumpers ; pitch / yaw via R-stick
        wx = _ANG_GAIN * axes[Ax.RY]                     # roll (x-rot)
        wy = _ANG_GAIN * axes[Ax.RX]                     # pitch
        wz = _ANG_GAIN * axes[Ax.D_PAD_X]                # yaw

        v_cmd = np.clip([vx, vy, vz], -_LIN_CLAMP, _LIN_CLAMP)
        w_cmd = np.clip([wx, wy, wz], -_ANG_CLAMP, _ANG_CLAMP)

        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.twist.linear.x,  tw.twist.linear.y,  tw.twist.linear.z  = v_cmd
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = w_cmd

        # arm select: hold LB / RB
        send_left  = len(msg.buttons) > Btn.LB and msg.buttons[Btn.LB]
        send_right = len(msg.buttons) > Btn.RB and msg.buttons[Btn.RB]

        if not (send_left or send_right):
            # self.pub_left.publish(tw)
            self.pub_right.publish(tw)
        else:
            # if send_left:  self.pub_left.publish(tw)
            if send_right: self.pub_right.publish(tw)

    # ────────────────────────────────────────────────────────────────
    def _start_servo(self) -> None:
        for cli in (self.cli_l, self.cli_r):
            if not cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warning("start_servo srv not ready")
                return
            cli.call_async(Trigger.Request())
        self.servo_started = True
        self.get_logger().info("Sent start_servo to both arms")

    def _fill_joints(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position = _MISSING_JOINTS, [0.0]*len(_MISSING_JOINTS)
        self.js_pub.publish(js)

# ───────────────────────────────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    rclpy.spin(JoyDualArm())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

