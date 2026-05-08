#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d

import ctypes
import struct
from std_msgs.msg import Header
import sensor_msgs_py

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs

from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped, Point
from tf_transformations import quaternion_from_euler
from math import pi
import sensor_msgs_py.point_cloud2 as pc2



class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)
        
        self._filter_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PointStamped, '/estimated_pose', 10)


    # def detect_callback(self, msg: PointCloud2):
    #     self._filter_pub.publish(msg)

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # self._pub.publish(msg) # publish original point cloud to topic /camera/depth/color/ds_points
        # self.get_logger().info(f'publishing original pointcloud')

        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        xyz = gen[:,:3]
        rgb = np.empty(xyz.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f' , c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8) 
            rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8) 
            rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        rgb = rgb.astype(np.float32) / 255

        # Filter points based on distance <= 0.9m and height > 0.0m
        distance_condition = (np.linalg.norm(xyz[:,:2], axis=1) <= 1.5) &(xyz[:,2]>0.0)
        filtered_points = xyz[distance_condition]
        filtered_rgb = rgb[distance_condition]
        # self.get_logger().info(f'filtered points in 0.9m and above ground {filtered_points}')
        # self.get_logger().info(f'filtered rgb in 0.9m and above ground {filtered_rgb}')

        # Filter for red and green points
        red_color = np.array([225, 83, 91]) / 255.0
        green_color = np.array([0, 112, 95]) / 255.0

        red_condition = np.all(np.isclose(filtered_rgb, red_color, atol=0.1), axis=1)
        green_condition = np.all(np.isclose(filtered_rgb, green_color, atol=0.1), axis=1)

        red_points = filtered_points[red_condition]
        red_colors = filtered_rgb[red_condition]

        green_points = filtered_points[green_condition]
        green_colors = filtered_rgb[green_condition]


        if len(red_points) > 0:
            self.get_logger().info(f'\033[91m Red sphere detected!\033[0m Number of points: {len(red_points)}')
            
        if len(green_points) > 0:
            self.get_logger().info(f'\033[92m Green cube detected!\033[0m Number of points: {len(green_points)}')

        red_ds_cloud = self.convert_np_to_open3d(red_points, red_colors)
        green_ds_cloud = self.convert_np_to_open3d(green_points, green_colors)  

        if len(red_points) > 0:
            red_cloud = self.convert_open3d_to_ros_cloud(red_ds_cloud, msg.header.frame_id, msg.header.stamp)
            red_position = self.convert_open3d_to_ros_position(red_points, msg.header.frame_id, msg.header.stamp)
            if red_cloud is not None and red_position is not None:
                self._filter_pub.publish(red_cloud)
                self.publisher.publish(red_position)    
        elif len(green_points) > 0:
            green_cloud = self.convert_open3d_to_ros_cloud(green_ds_cloud, msg.header.frame_id, msg.header.stamp)
            green_position = self.convert_open3d_to_ros_position(green_points, msg.header.frame_id, msg.header.stamp)
            if green_cloud is not None and green_position is not None:
                self._filter_pub.publish(green_cloud)
                self.publisher.publish(green_position)
        
        
        # self._pub.publish(red_detected)
        # self.get_logger().info('*********************************published detected**********************************')



    def convert_np_to_open3d(self, points, colors):
        # Convert NumPy -> Open3D
        cloud = o3d.geometry.PointCloud()    
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)

        # Downsample the point cloud to 5 cm
        ds_cloud = cloud.voxel_down_sample(voxel_size=0.05)
        # self.get_logger().info(f'downsample ds cloud to 5 cm {ds_cloud}')
        return ds_cloud
    

    def convert_open3d_to_ros_position(self, points, frame_id, stamp):
        # points = np.asarray(ds_cloud.points)

        if len(points) == 0:
            self.get_logger().info(f'no points')
            return None

        average_pos = np.mean(points, axis=0)
        point_stamped = PointStamped()
        point_stamped.point = Point(x =float(average_pos[0]), y = float(average_pos[1]), z = float(average_pos[2]))
        # self.get_logger().info(f'downsample point
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = stamp   
        try: 
            trans = self.tf_buffer.lookup_transform('map', frame_id, point_stamped.header.stamp, timeout=rclpy.time.Duration(seconds = 0.5))
            position = tf2_geometry_msgs.do_transform_point(point_stamped, trans)
            return position
        except TransformException as ex:
            self.get_logger().info(f'could not transform positions {ex}')
            return None
        


    def convert_open3d_to_ros_cloud(self, ds_cloud, frame_id, stamp):

        # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        colors = np.asarray(ds_cloud.colors)
        # self.get_logger().info(f'colors {colors}')

        point_cloud = PointCloud2()
        point_cloud.header = Header()
        point_cloud.header.stamp = stamp
        point_cloud.header.frame_id = frame_id

        try: 
            trans = self.tf_buffer.lookup_transform('map', frame_id, point_cloud.header.stamp, timeout=rclpy.time.Duration(seconds = 0.5))
        except TransformException as ex:
            self.get_logger().info(f'could not transform point cloud{ex}')
            return

        if len(points) == 0:
            self.get_logger().info(f'no points')

        else: 
            for point in points:
                point_stamped = PointStamped()
                point_stamped.point.x = point[0] 
                point_stamped.point.y = point[1]
                point_stamped.point.z = point[2]
                point_stamped.header.frame_id = frame_id
                point_stamped.header.stamp = stamp

                point = tf2_geometry_msgs.do_transform_point(point_stamped, trans)

            self.get_logger().info(f'did transform point cloud')
        
        colors = (colors * 255).astype(np.uint8)
        # self.get_logger().info(f'colors {colors}')

        colors = colors[:, [2, 1, 0]]

        # Convert the colors to uint32 for bitwise operations
        colors_uint32 = colors.astype(np.uint32)

        # Pack the colors into a single uint32 field
        packed_colors = np.left_shift(colors_uint32[:, 0], 16) | np.left_shift(colors_uint32[:, 1], 8) | colors_uint32[:, 2]
        # self.get_logger().info(f'packed_colors {colors}')

        points_rgb = np.zeros(len(points), dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgb', np.uint32)])

        points_rgb['x'] = points[:, 0]
        points_rgb['y'] = points[:, 1]
        points_rgb['z'] = points[:, 2]
        points_rgb['rgb'] = packed_colors
        # points_rgb['r'] = colors[:, 0]
        # points_rgb['g'] = colors[:, 1]
        # points_rgb['b'] = colors[:, 2]

        # create fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            # PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            # PointField(name='b', offset=14, datatype=PointField.UINT8, count=1)
        ]

        detected_pointcloud = pc2.create_cloud(point_cloud.header, fields, points_rgb)
        # self.get_logger().info(f'convert open3d to point cloud {detected}')
        # for i in range(len(points_rgb)):
        #     self.get_logger().info(f"RGB[{points_rgb}]: {points_rgb[i]}")
        return detected_pointcloud

  


def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()