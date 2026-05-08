#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class StaticTransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('static_transform_broadcaster')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcaster_timer_callback)

    def broadcaster_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StaticTransformBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()