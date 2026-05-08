#!/usr/bin/env python

"""Author: Arian Kourangi """

import math

import numpy as np
from sympy import euler


import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from example_interfaces.srv import Trigger

import cv2 as cv
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformBroadcaster
# Set to True if running table
USE_ENCODERS_ROTATE = False


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, '/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        # Store the path here
        self._path = Path()
        callbackgroup = ReentrantCallbackGroup()
        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Encoders, '/motor/encoders', self.encoder_callback, 10, callback_group=callbackgroup)
        self.create_subscription(
            sensor_msgs.Imu, '/imu/data_raw', self.imu_callback, 10, callback_group=callbackgroup)
        self._yaw = 0.0
        self.stamp = self.get_clock().now().to_msg()
        # 2D pose

        # FROM SLAM
        self.mu = []  # [x, y, z, theta, v, landmark_x, landmark_y,landmark_nr ...]
        self.cov = []
        self.R = np.array(np.eye(3)).astype(float)  # Change motion model noise

        self.R[0, 0] = 0.1  # x std
        self.R[1, 1] = 0.1  # y std
        self.R[2, 2] = 0.01  # robot theta std

        self.Initialize()
        self.i = 0
        self.omega = 0
        self.yaw_zero = 0
        self.yaw = 0
        self.drift = 0
        self.save_drift = False
        self.srv = self.create_service(
            Trigger, '/reset_odom', self.reset_callback, callback_group=callbackgroup)
        self.sleep = self.create_publisher(std_msgs.Bool, '/sleep', 10)

    def Initialize(self):
        # initializing mu with only robot estimates
        self.mu.append(np.array([0] * 3).astype(float))
        # initializing cov with only robot covariances
        self.cov.append(0 * np.eye(3).astype(float))

    def imu_callback(self, msg: sensor_msgs.Imu):
        # TODO: IF I have time look into correcting for drift in gyro by setting temp yaw when standing still and checking the drift
        if self.i == 0:
            [roll, pitch, self.yaw_zero] = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.i += 1
            # Since we will not be starting oriented with x axis but rather y axis
            # self.yaw_zero += np.pi/2
        if self.save_drift:
            temp = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.drift = self.yaw_zero - self.yaw - temp[2]
        else:
            [roll, pitch, yaw] = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

            if not USE_ENCODERS_ROTATE:
                # GUSTAV LOOK HERE COMMENT WHEN RUNNING ON TBALE
                self.yaw = self.yaw_zero - yaw  - self.drift
        self.omega = -msg.angular_velocity.z
        # print('updating omega')

    def reset_callback(self, request, response):
        self.mu[-1][1] = 0
        self.mu[-1][0] = 0
        response.success = True
        return response

    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry.

        This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).

        """
        # self.mu.append(np.copy(self.mu[-1]))
        # self.cov.append(np.copy(self.cov[-1]))

        # The kinematic parameters for the differential configuration

        dt = 50 / 1000
        ticks_per_rev = 48 * 64
        wheel_radius = 0.098425/2
        base = 0.301  #

        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        wtr = 2*np.pi*delta_ticks_right / \
            (ticks_per_rev*dt)  # Right wheel angular vel
        wtl = 2*np.pi*delta_ticks_left / \
            (ticks_per_rev*dt)  # left wheel angular vel

        omega = (wheel_radius * (wtr-wtl)/base)
        # +self.omega)/2  # robot angular vel
        # omega = self.omega
        vt = wheel_radius * (wtr+wtl)/2  # Robot linear 
        if abs(vt) < 0.005 and abs(omega) < 0.02:
            sleep = std_msgs.Bool()
            sleep.data = True
            #makes Scan matcher and Detection sleep for 3 sec
            self.sleep.publish(sleep)
            self.save_drift = True
        else:
            self.save_drift = False

        if USE_ENCODERS_ROTATE:
            # yaw  #GUSTAV LOOK HERE UNCOMMMENT WHEN RUNNING ON TABLE
            self.mu[-1][2] += omega*dt
        # Normalize angle
        self.mu[-1][1] += vt*dt*np.sin(self.mu[-1][2])  # y
        self.mu[-1][0] += vt*dt*np.cos(self.mu[-1][2])  # x
        self.mu[-1][2] = np.mod(self.yaw + np.pi, 2 * np.pi) - np.pi

        self.stamp = msg.header.stamp

        # self.publish_path(
        #    self.stamp, self.mu[-1][0], self.mu[-1][1], self.mu[-1][2])
        self.broadcast_transform(
            self.stamp, self.mu[-1][0], self.mu[-1][1], self.mu[-1][2], 'odom', 'base_link')

        # Add in a new publisher that publishes a covariance as odommetry message

    def broadcast_transform(self, stamp, x, y, yaw, parent_frame, child_frame):
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
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

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

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)

    def timer_callback(self):
        self.broadcast_transform(
            self.stamp, self.mu[-1][0], self.mu[-1][1], self.mu[-1][2], 'odom', 'base_link')

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
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
