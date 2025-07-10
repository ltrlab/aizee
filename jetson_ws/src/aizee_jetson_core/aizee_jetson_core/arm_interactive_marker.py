#!/usr/bin/env python3
"""
Interactive 6-DoF marker on the end-effector.
Drag the cube – the marker publishes a PoseStamped you can feed to MoveIt Servo.
"""

import rclpy, tf2_ros, tf_transformations as tft
from rclpy.node          import Node
from geometry_msgs.msg   import PoseStamped
from geometry_msgs.msg   import Pose
from std_msgs.msg        import Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.duration import Duration

class EEInteractive(Node):
    def __init__(self):
        super().__init__("ee_interactive_marker")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("base_frame", "gantry_base_link")
        self.declare_parameter("ee_frame",   "right_wrist_tool_link")
        self.base = self.get_parameter("base_frame").value
        self.ee   = self.get_parameter("ee_frame").value

        # TF listener
        self.tf_buf   = tf2_ros.Buffer()
        self.tf_lstnr = tf2_ros.TransformListener(self.tf_buf, self)

        # publish PoseStamped so other nodes (e.g. Servo) can subscribe
        self.pose_pub = self.create_publisher(PoseStamped,
                                              "/ee_target_pose", 10)

        # Interactive-marker server (topic will be /interactive_markers)
        self.im_server = InteractiveMarkerServer(self, "interactive_markers")

        # Poll TF until it appears, then create the marker
        self.timer = self.create_timer(0.5, self._try_create_marker)
        self.get_logger().info(
            f'Looking for TF {self.base} → {self.ee} …')

    # ------------------------------------------------------------------
    def _try_create_marker(self):
        if not self.tf_buf.can_transform(
                self.base, self.ee, rclpy.time.Time(),
                timeout=Duration(seconds=0)):
            return                        # still waiting

        self.timer.cancel()              # TF is available ⇒ stop polling
        self.get_logger().info("TF available, building marker")

        tr = self.tf_buf.lookup_transform(
            self.base, self.ee, rclpy.time.Time())

        p = Pose()
        p.position.x = tr.transform.translation.x
        p.position.y = tr.transform.translation.y
        p.position.z = tr.transform.translation.z
        p.orientation = tr.transform.rotation          # quat already OK



        # start the marker at the current EE pose
        int_m = InteractiveMarker()
        int_m.header.frame_id = self.base
        int_m.name            = "ee_marker"
        int_m.scale           = 0.15
        int_m.pose            = p            

        # 6-DoF controls ---------------------------------------------------
        for axis, ori in zip(("x","y","z"), (
            tft.quaternion_from_euler(0, 1.57, 0),   # rotate X
            tft.quaternion_from_euler(1.57, 0, 0),   # rotate Y
            tft.quaternion_from_euler(0, 0, 0))):    # rotate Z
            ctrl = InteractiveMarkerControl()
            ctrl.name = f"rotate_{axis}"
            ctrl.orientation.w, ctrl.orientation.x, ctrl.orientation.y, ctrl.orientation.z = ori
            ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_m.controls.append(ctrl)

        for axis, ori in zip(("x","y","z"), (
            tft.quaternion_from_euler(0, 1.57, 0),   # move X
            tft.quaternion_from_euler(1.57, 0, 0),   # move Y
            tft.quaternion_from_euler(0, 0, 0))):    # move Z
            ctrl = InteractiveMarkerControl()
            ctrl.name = f"move_{axis}"
            ctrl.orientation.w, ctrl.orientation.x, ctrl.orientation.y, ctrl.orientation.z = ori
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_m.controls.append(ctrl)

            self.im_server.insert(int_m)                         # add marker
            self.im_server.setCallback(int_m.name, self._feedback_cb)
            self.im_server.applyChanges()      

    # ------------------------------------------------------------------
    def _feedback_cb(self, feedback):
        ps = PoseStamped(
            header = Header(frame_id=self.base, stamp=self.get_clock().now().to_msg()),
            pose   = feedback.pose)
        self.pose_pub.publish(ps)

# ----------------------------------------------------------------------
def main():
    rclpy.init()
    rclpy.spin(EEInteractive())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

