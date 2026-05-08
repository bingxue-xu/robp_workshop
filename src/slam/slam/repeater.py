#!/usr/bin/env python

import math

import numpy as np
import copy

import rclpy
import cv2 as cv
import cv_bridge
from PIL import Image as im
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, quaternion_multiply
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from tf2_geometry_msgs import do_transform_point


class Repeater(Node):

    def __init__(self):
        super().__init__('repeater')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        # Create tf buffer
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10))
        # create tf listener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Period between tf updates from odom, should post tf of map-> odom at same freq
        callbackgroup2 = ReentrantCallbackGroup()
        self.ang_speed = 0
        period = 0.01
        self.trans = None
        self.create_subscription(
            sensor_msgs.Imu, '/imu/data_raw', self.imu_callback, 10, callback_group=callbackgroup2)
        self.create_subscription(
            sensor_msgs.LaserScan, '/scan', self.laser_callback, 10, callback_group=callbackgroup2)

        self.create_subscription(TransformStamped, '/map_to_odom',
                                 self.map_to_odom_callback, 10, callback_group=callbackgroup2)
        self.publisher = self.create_publisher(
            sensor_msgs.LaserScan, '/scan_filtered', 10)

        self.create_timer(period, self.timer_callback,
                          callback_group=callbackgroup2)

    def map_to_odom_callback(self, msg: TransformStamped):
        self.trans = msg

    def timer_callback(self):
        """ Repeats the transform of map-> odom """
        if self.trans is not None:
            trans = copy.deepcopy(self.trans)
            trans.header.stamp = self.get_clock().now().to_msg()
            # Send the transformation
            self._tf_broadcaster.sendTransform(trans)

    def laser_callback(self, msg: sensor_msgs.LaserScan):
        if self.ang_speed <= 0.08:
            self.publisher.publish(msg)
            # print("Published")
            # sself.get_logger().info('Repeating')
        else:
            pass
            # print("Not Published")

    def imu_callback(self, msg: sensor_msgs.Imu):
        self.ang_speed = abs(msg.angular_velocity.z)


def main():
    rclpy.init()
    node = Repeater()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
