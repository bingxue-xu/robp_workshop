#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)
        self._path_pub = self.create_publisher(Path, '/path', 10)
        self._path = Path()

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self.stamp = self.get_clock().now().to_msg()
        self.create_timer(0.1, self.timer_callback)
        
        self.declare_parameter('frequency', 20)
        self.declare_parameter('wheel_base', 0.311)
        self.declare_parameter('wheel_radius', 0.098425/2)
        self.declare_parameter('ticks_per_revolution', 48 * 64)


    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry.

        This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).

        Your task is to update the odometry based on the encoder data in 'msg'. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An encoders ROS message. To see more information about it 
        run 'ros2 interface show robp_interfaces/msg/Encoders' in a terminal.
        """


        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        ticks_per_rev = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        whell_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        K = 2*np.pi/ticks_per_rev
        D = (wheel_radius/2)*(K*(delta_ticks_right+delta_ticks_left))
        delta_theta = (wheel_radius/whell_base)*(K*(delta_ticks_right-delta_ticks_left))

        self._x = self._x + D*np.cos(self._yaw) 
        self._y = self._y + D*np.sin(self._yaw) 
        self._yaw = self._yaw + delta_theta 
        self.stamp = msg.header.stamp
        
        self.publish_path(self.stamp, self._x, self._y, self._yaw)

    def broadcast_transform(self, stamp, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform.

        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Broadcasted odom→base_link TF: {x}, {y}, {yaw}')

    def timer_callback(self):
        self.broadcast_transform(self.stamp, self._x, self._y, self._yaw)

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)
        self._path_pub.publish(self._path)
        


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()