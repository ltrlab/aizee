#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class BaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('base_link_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_tf)
        self.t = 0.0

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        # example: make it circle around
        self.t += 0.05
        t.transform.translation.x = math.cos(self.t) * 1.0
        t.transform.translation.y = math.sin(self.t) * 1.0
        t.transform.translation.z = 0.0
        # no rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = BaseLinkBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()

