#!/usr/bin/env python

"""Author: Arian Kourangi """

import math

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(
            Path, '/waypoints_global', 10)
        self.timer_callback()
    def timer_callback(self):

        pose1 = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()

        pose1.pose.position.x = np.float64(0.2)
        pose1.pose.position.y = np.float64(0.2)
        pose1.pose.position.z = np.float64(0)
        pose1.pose.orientation.x = np.float64(0)
        pose1.pose.orientation.y = np.float64(0)
        pose1.pose.orientation.z = np.float64(0)
        pose1.pose.orientation.w = np.float64(1)

        pose2.pose.position.x = np.float64(0.5)
        pose2.pose.position.y = np.float64(0.5)
        pose2.pose.position.z = np.float64(0)
        pose2.pose.orientation.x = np.float64(0)
        pose2.pose.orientation.y = np.float64(0)
        pose2.pose.orientation.z = np.float64(0)
        pose2.pose.orientation.w = np.float64(1)

        pose3.pose.position.x = np.float64(1)
        pose3.pose.position.y = np.float64(1)
        pose3.pose.position.z = np.float64(0)
        pose3.pose.orientation.x = np.float64(0)
        pose3.pose.orientation.y = np.float64(0)
        pose3.pose.orientation.z = np.float64(0)
        pose3.pose.orientation.w = np.float64(1)

        poses = Path()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = 'odom'
        poses.poses.append(pose1)
        poses.poses.append(pose2)
        poses.poses.append(pose3)
        self.publisher.publish(poses)


def main():
    rclpy.init()
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()