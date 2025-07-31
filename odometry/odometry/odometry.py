#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10, callback_group=ReentrantCallbackGroup())
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10, callback_group=ReentrantCallbackGroup())
        self._path_pub = self.create_publisher(Path, '/path', 10)
        self._path = Path()

        self.i = 0
        self.drift = 0.0
        self.save_drift = False
        self.omega = 0.0

        self.linear_tick_threshold = 25
        self.angular_tick_threshold = 3

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self.stamp = self.get_clock().now().to_msg()
        
        # self.declare_parameter('frequency', 20)
        self.declare_parameter('wheel_base', 0.311)
        self.declare_parameter('wheel_radius', 0.098425/2)
        self.declare_parameter('ticks_per_revolution', 48 * 64)
        self.declare_parameter('use_imu', False)
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value


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

        is_stationary = (abs(delta_ticks_left) < self.linear_tick_threshold and 
            abs(delta_ticks_right) < self.linear_tick_threshold and
            abs(delta_ticks_left - delta_ticks_right) < self.angular_tick_threshold)
            # If the robot is not moving, we update the drift for stationary calibration
        if is_stationary:
            if not self.save_drift:
                self.get_logger().info('Stationary calibration started.')
            self.save_drift = True
        else:
            if self.save_drift:
                self.get_logger().info('Stationary calibration ended.')
            self.save_drift = False

        K = 2*np.pi/self.ticks_per_rev
        D = (self.wheel_radius/2)*(K*(delta_ticks_right+delta_ticks_left))
        delta_theta = (self.wheel_radius/self.wheel_base)*(K*(delta_ticks_right-delta_ticks_left))

        self._x = self._x + D*np.cos(self._yaw) 
        self._y = self._y + D*np.sin(self._yaw) 

        if self.use_imu:
            self._yaw = self.imu_yaw
            self.get_logger().info(f'IMU yaw: {self._yaw:.2f} rad')
        else:
            self._yaw = self._yaw + delta_theta 
        self.stamp = msg.header.stamp
    
        self.publish_path(self.stamp, self._x, self._y, self._yaw)
        self.broadcast_transform(self.stamp, self._x, self._y, self._yaw)

    def imu_callback(self, msg: Imu):
        """Takes IMU data and updates the odometry.

        This function is called every time the IMU is updated (i.e., when a message is published on the '/imu/data_raw' topic).

        Your task is to update the odometry based on the IMU data in 'msg'. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An IMU ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/Imu' in a terminal.
        """
        imu_yaw_change = 0.0
        if self.i == 0:
            [roll, pitch, self.yaw_init] = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.i += 1
        else:
            [roll, pitch, yaw] = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            imu_yaw_change = yaw - self.yaw_init

            if self.save_drift:
                self.drift = self._yaw - imu_yaw_change

        if self.use_imu:
            self.imu_yaw = imu_yaw_change + self.drift
        self.omega = msg.angular_velocity.z
        if abs(self.omega) > 0.1:
            self.get_logger().info(f'Is turning: {self.omega:.2f} rad/s')


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
