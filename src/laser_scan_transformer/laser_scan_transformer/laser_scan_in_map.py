#!/usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, quaternion_multiply
from tf2_geometry_msgs import do_transform_pose_stamped
import time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from aruco_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from tf2_geometry_msgs import do_transform_point

class LaserScanTransformer(Node):
    def __init__(self):
        super().__init__('laser_scan_transformer')
        callbackgroup = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=callbackgroup)
        self.cloud_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, '/scan_transformed', 10)
        
        
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        

    def scan_callback(self, msg: LaserScan):
        trans = None
        source_frame = msg.header.frame_id
        target_frame = 'map'

        if self.tf_buffer.wait_for_transform_async('map', msg.header.frame_id, msg.header.stamp):
            try:
                trans = self.tf_buffer.lookup_transform(
                            target_frame,
                            source_frame,
                             msg.header.stamp,timeout=rclpy.duration.Duration(seconds=1)) ##this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
            except(ConnectivityException) as c:
                print(c)
            except(LookupException) as L:
                print(L)
            except(ExtrapolationException) as E:
                print(E)
            except(TransformException) as T:
                print(T)

        if trans is not None:
            #quat=[trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
            #robot_yaw = self.get_yaw(quat)
            #robot_z = trans.transform.translation.z

            nr_messages = len(msg.ranges)
            #robot_x_coord = trans.transform.translation.x
            #robot_y_coord = trans.transform.translation.y
            Points = []
            for i in range(nr_messages):
                if msg.ranges[i] <= msg.range_min <= msg.range_max: #robot_yaw +
                    continue
                else:
                    total_angle =  msg.angle_min + +i*msg.angle_increment

                    x_coord = math.cos(total_angle) * msg.ranges[i] #robot_x_coord +
                    y_coord =  math.sin(total_angle) * msg.ranges[i] #robot_y_coord + 

                    point = PointStamped()
                    point.point.x = x_coord
                    point.point.y = y_coord
                    point.point.z = np.float64(0)
                    formed = do_transform_point(point,trans)
                    Points.append(np.array([formed.point.x, formed.point.y, formed.point.z]))
                
                stuff = self.get_cloud(np.array(Points), 'map')
                self.cloud_publisher.publish(stuff)
                print('publishing')

    def get_yaw(self,q):
        return math.atan2(2*(q[3]*q[2] + q[0]*q[1]), 1- 2*(q[1]*q[1] + q[2]*q[2]))
    

    def get_cloud(self, points, parent_frame):
        itemsize = np.dtype(np.float32).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(np.float32).tobytes()
        fields = [sensor_msgs.PointField(
            name='x', offset=0*itemsize, datatype=sensor_msgs.PointField.FLOAT32, count=1),sensor_msgs.PointField(
            name='y', offset=1*itemsize, datatype=sensor_msgs.PointField.FLOAT32, count=1),sensor_msgs.PointField(
            name='z', offset=2*itemsize, datatype=sensor_msgs.PointField.FLOAT32, count=1)]
        header = std_msgs.Header(frame_id=parent_frame)
        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )
def main():
    rclpy.init()
    node = LaserScanTransformer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
