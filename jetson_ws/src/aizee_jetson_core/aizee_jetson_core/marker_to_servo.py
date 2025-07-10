#!/usr/bin/env python3
"""
Convert interactive-marker pose errors into TwistStamped commands
so that MoveIt Servo will chase the end-effector marker.

  • publishes on  /right_arm/servo_node/delta_twist_cmds
  • parameters:  base_frame  ,  ee_frame
"""

import math, rclpy, numpy as np
from rclpy.node         import Node
from geometry_msgs.msg  import TwistStamped
from visualization_msgs.msg import InteractiveMarkerFeedback   # works both ways
import tf2_ros, tf_transformations as tft
from std_srvs.srv import Trigger

MAX_LIN  = 0.20       # m/s  (tune ≤ Servo's scale.linear)
MAX_ANG  = 0.50       # rad/s (tune ≤ Servo's scale.rotational)

class MarkerToTwist(Node):
    def __init__(self):
        super().__init__("marker_to_servo")

        self.declare_parameter("base_frame", "gantry_base_link")
        self.declare_parameter("ee_frame",   "right_wrist_tool_link")
        self.base = self.get_parameter("base_frame").value
        self.ee   = self.get_parameter("ee_frame").value

        # TF ---------------------------------------------------------------
        self.tf_buf   = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3.0))
        self.tf_sub   = tf2_ros.TransformListener(self.tf_buf, self)

        # publishers / subscribers ----------------------------------------
        self.pub_twist = self.create_publisher(
            TwistStamped, "/right_arm/servo_node/delta_twist_cmds", 10)
            
        self.start_cli = self.create_client(
            Trigger, '/right_arm/servo_node/start_servo')
        self.servo_started = False

        self.sub_fb = self.create_subscription(
            InteractiveMarkerFeedback, "/interactive_markers/feedback",
            self._fb_cb, 10)

        self.get_logger().info("Ready – drag the marker!")

    # ------------------------------------------------------------------
    def _fb_cb(self, msg: InteractiveMarkerFeedback):
        if msg.event_type != InteractiveMarkerFeedback.POSE_UPDATE \
           or msg.marker_name != "ee_marker":
            return

        # 1. current EE pose in base frame --------------------------------
        try:
            tr = self.tf_buf.lookup_transform(
                self.base, self.ee, rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warning("TF lookup failed")
            return

        p_curr = np.array([tr.transform.translation.x,
                           tr.transform.translation.y,
                           tr.transform.translation.z])
        q_curr = np.array([tr.transform.rotation.x,
                           tr.transform.rotation.y,
                           tr.transform.rotation.z,
                           tr.transform.rotation.w])

        # 2. desired pose from marker -------------------------------------
        p_des  = np.array([msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z])
        q_des  = np.array([msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w])

        # 3. translational error  -----------------------------------------
        dp = p_des - p_curr
        # clamp – keeps velocities sane
        dp = np.clip(dp, -0.10, 0.10)

        # 4. rotational error (shortest-arc quaternion) -------------------
        q_err = tft.quaternion_multiply(
                    q_des, tft.quaternion_conjugate(q_curr))
        q_err = q_err / np.linalg.norm(q_err)   # guard against drift

        ang  = 2 * math.acos(np.clip(q_err[3], -1.0, 1.0))
        if ang > math.pi:           # put in [-π, π]
            ang -= 2*math.pi
        axis = np.array(q_err[:3])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.zeros(3)
        else:
            axis = axis / np.linalg.norm(axis)

        dw = ang * axis             # rot-vec (rad)

        # clamp angular as well
        dw = np.clip(dw, -0.5, 0.5)

        # 5. build TwistStamped ------------------------------------------
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.twist.linear.x,  tw.twist.linear.y,  tw.twist.linear.z  = dp * (MAX_LIN / 0.10)
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = dw * (MAX_ANG / 0.50)
        print(tw)
        self.pub_twist.publish(tw)
        
        if not self.servo_started and self.start_cli.wait_for_service(timeout_sec=0.0):
            self.start_cli.call_async(Trigger.Request())
            self.servo_started = True
            self.get_logger().info("start_servo called")

def main():
    rclpy.init()
    rclpy.spin(MarkerToTwist())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

