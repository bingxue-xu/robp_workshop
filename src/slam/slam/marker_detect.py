#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped
from arian_interfaces.msg import PointWithRadius
from arian_interfaces.srv import AddCamObs
from aruco_msgs.msg import MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
import std_msgs.msg as std_msgs
import time
class Aruco_detect(Node):
    def __init__(self):
        super().__init__('display_markers')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.current_target_marker_id = None
        ####### Service client for AddCamObs   ########
        self.cli_addcamobs = self.create_client(
            AddCamObs, '/add_cam_obs', callback_group=ReentrantCallbackGroup())
        while not self.cli_addcamobs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddCamObs.Request()
        cbr = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            MarkerArray, '/aruco/markers', self.callback, 10, callback_group=cbr)
        self.sub_sleep = self.create_subscription(std_msgs.Bool, '/sleep', self.sleep_callback, 10 ,callback_group=ReentrantCallbackGroup())
    def sleep_callback(self, msg: std_msgs.Bool):
        #if msg.data == True:
        #    time.sleep(3)
        #else:
        #    time.sleep(1)
        pass
    def callback(self, msg: MarkerArray):
        list = []
        trans = None
        for marker in msg.markers:
            trans = self.get_trans(marker)
            if trans is not None:
                #self.get_logger().info(marker.header.frame_id)
                posestamped = PoseStamped()
                posestamped.header = marker.header
                posestamped.pose = marker.pose.pose
    
                formed = do_transform_pose_stamped(posestamped, trans)
                T = TransformStamped()
                T.child_frame_id = f'/aruco_marker{marker.id}'
                T.header.frame_id = 'map'
                T.header.stamp = marker.header.stamp
                T.transform.translation.x = formed.pose.position.x
                T.transform.translation.y = formed.pose.position.y
                T.transform.translation.z = formed.pose.position.z
                T.transform.rotation.x = formed.pose.orientation.x
                T.transform.rotation.y = formed.pose.orientation.y
                T.transform.rotation.z = formed.pose.orientation.z
                T.transform.rotation.w = formed.pose.orientation.w
                self._tf_broadcaster.sendTransform(T)
                [roll, pitch, yaw] = euler_from_quaternion(
                    [T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w])
                yaw = yaw- np.pi/2
                if yaw < 0:
                    yaw = yaw+2*np.pi
    
                #self.get_logger().info(str(yaw))
    
                radpoint = PointWithRadius()
                radpoint.point.point = marker.pose.pose.position
                radpoint.point.header = marker.header
                radpoint.radius = 0.15
                radpoint.marker_id = marker.id
                radpoint.orientation = yaw
                radpoint.type = 'box'
                radpoint.target = True
                list.append(radpoint)
        self.send_req_cam_obs(list)

    def send_req_cam_obs(self, list):
        self.req.radpoints.points = list
        self.future = self.cli_addcamobs.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # print('Added point to obs list ', self.future.result().done)

    def get_trans(self, marker):
        trans = None
        if self.tf_buffer.can_transform('map', marker.header.frame_id, marker.header.stamp, timeout=rclpy.duration.Duration(seconds=4)):
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    marker.header.frame_id,
                    marker.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
            except (ConnectivityException) as C:
                self.get_logger().error(str(C))
            except (LookupException) as L:
                self.get_logger().error(str(L))
            except (ExtrapolationException) as E:
                self.get_logger().error(str(E))
            except (TransformException) as T:
                self.get_logger().error(str(T))
        return trans


def main():
    rclpy.init()
    node = Aruco_detect()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
