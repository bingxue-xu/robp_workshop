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

from ida_interfaces.srv import EstPose

import cv2

class Detection(Node):

    def __init__(self):
        """
        Subscribes to: 
        - /camera/depth/color/points
        Publishes to:
        - /filtered_points
        - /camera/depth/color/ds_points (Not used?)
        - /estimated_pose
        Service:
        - estimated_pose (Request: - , Response: geometry_msgs/PointStamped est_pose)
        """
        super().__init__('detection')
        self.point_cloude = np.array([])
        self.estimated_pose = None

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)
        
        self._filter_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)

        self.publisher = self.create_publisher(PointStamped, '/estimated_pose', 10)


        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create service
        self.srv = self.create_service(EstPose, 'estimated_pose', self.estPose_callback)


    def estPose_callback(self, request, response):
        """ No request. Sends estimated pose from cloud callback function as response in service. Can right now be either red or blue object."""

        if self.estimated_pose is not None:
            response.est_pose = self.estimated_pose
        else:
            self.get_logger().warn("No estimated pose available.")
            # If no estimated pose is available, you might want to set response.est_pose to some default value or leave it unset.
        return response

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Proceeding:
        - Converts data
        - Filter based on distance (d<2) and height (0<z<0.5)
        - Convert RGB --> BGR --> HSV
        - Filter colors based on HSV values
        - Check if enough points are detected that it can be object
        - Estimate object pose
        """


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

        # Filter points based on distance <= 2m and height > -1m < 1m
        distance_condition = (np.linalg.norm(xyz[:,[0,2]], axis=1) <= 2) &(xyz[:,1]>-1) &(xyz[:,1]<1)
        filtered_points = xyz[distance_condition]
        filtered_rgb = rgb[distance_condition]
        self.point_cloude = filtered_points

        ######################### HSV #######################################

        ## HSV color space (Hue, Saturation, Value)

        height = 1
        width = np.size(filtered_rgb, 0)

        # Convert RGB to BGR for OpenCV
        bgr_image = (filtered_rgb.reshape((height, width, 3))[:, :, [2, 1, 0]] * 255).astype(np.uint8)        

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)


        # RED

        # mask 1 lower part of spectrum representing red

        # H: 0-15
        # S: 130-255
        # V: 50-255
        
        low_red = np.array([0, 155, 122])
        high_red = np.array([8, 255, 255])

        red_mask1 = cv2.inRange(hsv_image, low_red, high_red)

        low_blue = np.array([98, 153, 74])
        high_blue = np.array([101, 255, 255])
        blue_mask = cv2.inRange(hsv_image, low_blue, high_blue)
        blue_pixels = np.count_nonzero(blue_mask)
        # mask 2 higher part of spectrum representing red

        # H: 170-255
        # S: 130-255
        # V: 50-255
        low_red2 = np.array([166, 109, 133])
        high_red2 = np.array([255, 255, 255])

        red_mask2 = cv2.inRange(hsv_image, low_red2, high_red2)

        # Combine into one mask for red
        red_mask = red_mask1 + red_mask2
        # Count the number of red pixels ()
        no_red_pixels = np.count_nonzero(blue_mask)
        print(f'Number of blue_pixels: {blue_pixels}')


        # GREEN

        # Only one mask

        # H: 77-92
        # S: 41-255
        # V: 41-255
        
        low_green = np.array([80, 126, 135])
        high_green = np.array([90, 255, 255])

        green_mask = cv2.inRange(hsv_image, low_green, high_green)
        
        no_green_pixels = np.count_nonzero(green_mask)
        print(f'Number of green pixels: {no_green_pixels}')

        ################################################################




        red_low = np.array([])


        # Flatten the mask to a 1D array since otherwise wrong dimension
        red_mask_flat = blue_mask.flatten()
        green_mask_flat = green_mask.flatten()

        # Extract points that are marked in mask
        red_condition = red_mask_flat != 0
        green_condition = green_mask_flat!= 0
        
        red_points = filtered_points[red_condition]


                # Find the indices of the points that satisfy the red_condition
        red_indices = np.where(red_condition)[0]

        # Calculate the average index
        average_index = int(np.mean(red_indices))

        print(f"Average index of red points in point cloud: {average_index}")
        
        red_colors = filtered_rgb[red_condition]

        green_points = filtered_points[green_condition]
        green_colors = filtered_rgb[green_condition]


        # Print text if object is detected
        if len(red_points) > 10:
            self.get_logger().info(f'\033[91m Red sphere detected!\033[0m Number of points: {len(red_points)}')
            
        if len(green_points) > 10:
            self.get_logger().info(f'\033[92m Green cube detected!\033[0m Number of points: {len(green_points)}')


        # Convert to right format
        red_ds_cloud = self.convert_np_to_open3d(red_points, red_colors)
        green_ds_cloud = self.convert_np_to_open3d(green_points, green_colors)  


        # Threshold (#red_points > 10) to filter out non-object points
        if len(red_points) > 10:
            # Convert to ros cloud and estimate pose
            red_cloud = self.convert_open3d_to_ros_cloud(red_ds_cloud, msg.header.frame_id, msg.header.stamp)
            red_position = self.convert_open3d_to_ros_position(red_points, msg.header.frame_id, msg.header.stamp)
            self.estimated_pose = red_position
            self.get_logger().info(f'Estimated position is: {self.estimated_pose}')

            # Publish cloud and position if we have estimated pose of object
            if red_cloud is not None and red_position is not None:
                self._filter_pub.publish(red_cloud)
                self.publisher.publish(red_position)

        # Threshold (#green_points > 10) to filter out non-object points            
        elif len(green_points) > 10:
            # Convert to ros cloud and estimate pose
            green_cloud = self.convert_open3d_to_ros_cloud(green_ds_cloud, msg.header.frame_id, msg.header.stamp)
            green_position = self.convert_open3d_to_ros_position(green_points, msg.header.frame_id, msg.header.stamp)
            self.estimated_pose = green_position            
            self.get_logger().info(f'Estimated position is: {self.estimated_pose}')

            # Publish cloud and position if we have estimated pose of object
            if green_cloud is not None and green_position is not None:
                self._filter_pub.publish(green_cloud)
                self.publisher.publish(green_position)
        
        


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
        print(point_stamped.point)

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