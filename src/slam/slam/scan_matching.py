#!/usr/bin/env python

"""Author: Arian Kourangi """

from scipy.spatial import KDTree
import numpy as np
from .scan_match_class import ScanMatch
from .process_cloud import *
import rclpy
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg as std_msgs
import tf2_ros
from rclpy.node import Node
import csv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import PointStamped

from open3d import open3d as o3d
import time

class scan_matcher(Node):
    def __init__(self):
        super().__init__('scan_matching')
        self.new_scan = None
        self.new_pointcloud = None
        ###########     Parameters     ###########
        self.k = 0  # Counter
        self.refresh_rate = 3  # Every ith scan to use for localization
        self.map_rate = 2
        # How many total scans to be usedscans to use to build map. This metric needs to be updated
        self.map_size = 500
        self.map_done = False
        self.init_scans = 20
        self.current_id = 0
        self.end_id = 0
        self.tree = None
        self.flag = False
        #############                  ############
        self.map = np.zeros((round(self.map_size*1e4), 3))
        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=1000))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.pose = np.eye(4)
        callbackgroup = MutuallyExclusiveCallbackGroup()
        self.subscriber = self.create_subscription(
            sensor_msgs.LaserScan, '/scan_filtered', self.callback_lidar, 10, callback_group=callbackgroup)
        self.trans_pub = self.create_publisher(
            TransformStamped, '/map_to_odom', 10)
        self.sub = self.create_subscription(
            std_msgs.Bool, '/map_done', self.map_done_callback, 10, callback_group=callbackgroup)

        self.sub_sleep = self.create_subscription(std_msgs.Bool, '/sleep', self.sleep_callback,10, callback_group=callbackgroup)
    def sleep_callback(self, msg: std_msgs.Bool):
        if msg.data == True:
            time.sleep(0.1)

    def map_done_callback(self, msg: std_msgs.Bool):
        self.flag = msg.data

    def callback_lidar(self, msg: sensor_msgs.LaserScan):

        if not self.map_done and self.flag:  # self.k == self.map_size:

            # filter out outliers. For a map size of 100, map rate of 10 and taking in the first 10 scans 0,4 and 20 neighbors works alright
            # self.map = filter_points(self.map, 0.4, nr_neighbors=20)
            np.savetxt('spin.txt', self.map)
            self.get_logger().info('Map_Done!!')
            self.tree =KDTree(self.map[:self.current_id], copy_data=True)
            # self.map = filter_points(
            #    self.map, 0.5, 5)
            # self.map = preprocess_point_cloud(self.map)
            # self.map = filter_points(self.map, 1, 150)
            # _, self.map = extract_points_on_lines(
            #   self.map, 0.5, 0.5, 0.5)
            # self.map = smooth_point_cloud(self.map, 0.3)
            self.map_done = True
            self.refresh_rate = 5

        if self.k >= self.init_scans:

            if self.k % self.refresh_rate == 0 :#or (not self.map_done and self.k % self.map_rate == 0):

                self.new_scan = msg
                nr_messages = len(self.new_scan.ranges)
                ranges = []
                bearings = []

                for i in range(nr_messages):
                    if msg.ranges[i] <= msg.range_min + 0.10 or msg.ranges[i] >= msg.range_max:
                        continue
                    elif msg.range_min < msg.ranges[i] <= msg.range_max:
                        total_angle = msg.angle_min + i*msg.angle_increment  # robot_yaw +
                        # Normalize angle
                        total_angle = np.mod(
                            total_angle + np.pi, 2 * np.pi) - np.pi
                        ranges.append(msg.ranges[i])
                        bearings.append(total_angle)
                ranges = np.array(ranges)
                bearings = np.array(bearings)

                self.new_pointcloud = range_bearing_to_cartesian(
                    ranges, bearings)

                source_frame = self.new_scan.header.frame_id
                target_frame = 'odom'

                trans_new_to_odom = None
                if self.tfBuffer.can_transform(target_frame, source_frame, self.new_scan.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
                    try:
                        trans_new_to_odom = self.tfBuffer.lookup_transform(
                            target_frame,
                            source_frame,
                            self.new_scan.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
                    except (ConnectivityException) as C:
                        self.get_logger().error(str(C))
                    except (LookupException) as L:
                        self.get_logger().error(str(L))
                    except (ExtrapolationException) as E:
                        self.get_logger().error(str(E))
                    except (TransformException) as T:
                        self.get_logger().error(str(T))
                if trans_new_to_odom is not None:
                    # Now the transform matrix should just be the relative pose
                    Estimated_transform = transform_to_matrix(
                        trans_new_to_odom)  # This is base->odom

                    """Using Arians Scanmatcher ICP"""
                    if not self.map_done:

                        Match = ScanMatch(self.new_pointcloud,
                                          self.map[:self.current_id], 100, 1.5, 1e-10, 0.95,None,bool = False)
                    else:
                        Match = ScanMatch(self.new_pointcloud,
                                          self.map, 100, 1.5, 1e-10, 0.95,self.tree,bool = True)
                    Actual_transform = Match.icp(T=Estimated_transform)
                    if Actual_transform is not None:
                        # Just printing out the trasnsform for reference
                        #print('Arian_transform ', ' x= ', round(Actual_transform[0, 3], 4), ' y= ', round(Actual_transform[1, 3], 4), ' theta = ', round(np.arctan2(
                        #    Actual_transform[1, 0], Actual_transform[0, 0]), 4))

                        # diff = inv(odom->base*inv(map->base))
                        diff = np.linalg.inv(
                            np.dot(np.linalg.inv(Estimated_transform), Actual_transform))   # reg_p2p.transformation))  # Here estimated trans = base->odom and reg_p2p.transformation base->map
                        ####################
                        yaw = np.arctan2(diff[1, 0], diff[0, 0])
                        yaw = np.mod(
                            yaw + np.pi, 2 * np.pi) - np.pi
                        if not self.map_done:
                            self.create_map(
                                None, Actual_transform, self.new_pointcloud)
                            #print(self.k)
                        ################################
                        # yaw = 0.0
                        self.broadcast_transform(
                            self.new_scan.header.stamp, diff[0, 3], diff[1, 3], yaw, 'map', 'odom')  # Maybe switch odom and map
        if self.k < self.init_scans:
            self.create_map(msg, None, None, bool=False)
            #print(self.k)
        self.k += 1

    def create_map(self, msg: sensor_msgs.LaserScan, T, cloud, bool=True):
        """Takes in scans and creates a map to use, only want to use the first 100 scans or so"""
        if not bool:
            trans = None
            source_frame = msg.header.frame_id
            target_frame = 'odom'
            if self.tfBuffer.can_transform(target_frame, msg.header.frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
                try:
                    trans = self.tfBuffer.lookup_transform(
                        target_frame,
                        source_frame,
                        msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1))
                except (ConnectivityException) as C:
                    self.get_logger().error(str(C))
                except (LookupException) as L:
                    self.get_logger().error(str(L))
                except (ExtrapolationException) as E:
                    self.get_logger().error(str(E))
                except (TransformException) as T:
                    self.get_logger().error(str(T))

            if trans is not None:
                nr_messages = len(msg.ranges)
                ranges = []
                bearings = []
                for i in range(nr_messages):
                    if msg.ranges[i] <= msg.range_min + 0.10 or msg.ranges[i] >= msg.range_max:
                        continue
                    else:
                        total_angle = msg.angle_min + i*msg.angle_increment  # robot_yaw +
                        # Normalize angle
                        total_angle = np.mod(
                            total_angle + np.pi, 2 * np.pi) - np.pi
                        ranges.append(msg.ranges[i])
                        bearings.append(total_angle)

                ranges = np.array(ranges)
                bearings = np.array(bearings)
                local_scan = range_bearing_to_cartesian(ranges, bearings)
                # local_scan = filter_points(local_scan, 0.5, 5)

                """np.savetxt('scan' + str(self.sks) +
                           '.txt', local_scan)
                self.sks += 1
                """
                # First transform the scan from lidar to odom
                temp = np.array([0.0, 0.0, 0.0])
                for point in local_scan:
                    ps = PointStamped()
                    ps.point.x = point[0]
                    ps.point.y = point[1]
                    ps.point.z = point[2]
                    ps.header.frame_id = msg.header.frame_id
                    ps.header.stamp = msg.header.stamp
                    formed = do_transform_point(ps, trans)
                    temp = np.vstack(
                        (temp, np.array([formed.point.x, formed.point.y, 0.0])))
                self.end_id = self.current_id + len(temp[1:])

                self.map[self.current_id:self.end_id, :] = temp[1:]
                self.current_id = self.end_id

        else:
            hom = np.ones(len(cloud))
            source_temp = np.column_stack((cloud, hom))
            source_r = np.dot(T, source_temp.T)
            temp = np.transpose(source_r[0:3])
            self.end_id = self.current_id + len(temp)
            self.map[self.current_id:self.end_id, :] = temp
            self.current_id = self.end_id

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

        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        # self.tfBroadcaster.sendTransform(t)
        # self.tf_static_broadcaster.sendTransform(t)
        """Publish transform to map-to-odom topic and let repeater broadcast transform"""
        self.trans_pub.publish(t)


def range_bearing_to_cartesian(range_measurements, bearing_measurements):
    x = range_measurements * np.cos(bearing_measurements)
    y = range_measurements * np.sin(bearing_measurements)
    z = np.zeros_like(x)  # Assuming LiDAR is in 2D (z-coordinate is zero)
    return np.column_stack((x, y, z))


def get_diff_transform(source, target):
    difference = np.dot(np.linalg.inv(source), target)
    return difference


def transform_to_matrix(trans: TransformStamped):

    [pitch, roll, theta] = euler_from_quaternion(
        [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
    phi = 0
    R_11 = np.cos(phi)*np.cos(theta)
    R_12 = -np.cos(phi)*np.sin(theta)
    R_13 = np.sin(phi)
    R_21 = (np.sin(phi)*np.sin(phi)*np.cos(theta)+np.cos(phi)*np.sin(theta))
    R_22 = -(np.sin(phi)*np.sin(phi)*np.sin(theta)-np.cos(phi)*np.cos(theta))
    R_23 = np.sin(phi)*np.cos(phi)
    R_31 = 0
    R_32 = 0
    R_33 = 1
    T_x = trans.transform.translation.x
    T_y = trans.transform.translation.y
    T_z = 0.0

    transform = np.array([[R_11, R_12, R_13, T_x],
                          [R_21, R_22, R_23, T_y],
                          [R_31, R_32, R_33, T_z],
                          [0, 0, 0, 1]])
    return transform


def main():
    rclpy.init()
    node = scan_matcher()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
