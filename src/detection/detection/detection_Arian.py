#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d

from message_filters import ApproximateTimeSynchronizer
import ctypes
import struct

from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import time
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import TransformStamped, PointStamped
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        #publisher for point cloud topic 
        self.cloud_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, 'pointcloud', 10)
        #tf buffers and stuff
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_height = None
        #self.ts = ApproximateTimeSynchronizer([, self.tf_listener],30,0.01)
        self.counter = 0


    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """
        self.counter +=1
        if self.counter == 3:
             self.get_height('camera_link',rclpy.time.Time())
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        xyz = gen[:, :3]
        rgb = np.empty(xyz.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        rgb = rgb.astype(np.float32) / 255

        # Convert NumPy -> Open3D
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)
        cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Downsample the point cloud to 5 cm
        ds_cloud = cloud.voxel_down_sample(voxel_size=0.005) #Chnaged downsamliing, 5cm makes no sense for such small objects

        # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        colors = np.asarray(ds_cloud.colors)

        source_frame = msg.header.frame_id
        # findinf trans from camera to map
        #print('Time',msg.header.stamp)

        # if we have transform, check points distance and rgb values
        if self.camera_height != None:
            all_points = []
            red = False
            green = False
            for i in range(len(points)):
                if 255*colors[i][0] > 255*colors[i][1]+100 and 255*colors[i][0] > 255*colors[i][2] + 100:
                    if abs(points[i][0]) < 0.3 and 0.9 <points[i][2] < 1.8 and self.camera_height - 0.06 < points[i][1]< self.camera_height:
                       #found red sphere
                        red = True
                        all_points.append(points[i])

                elif 255*colors[i][1] > 255*colors[i][0]+60 and 255*colors[i][1] > 255*colors[i][2]:
                    if abs(points[i][0]) < 0.1 and 0.9<points[i][2] < 1.8 and self.camera_height - 0.06 < points[i][1]< self.camera_height:
                        # found green cube
                        green = True
                        all_points.append(points[i])

            if red:
                self.get_logger().info('Detected Red Sphere!!')
            elif green:
                self.get_logger().info('Detected Green Cube!!')
            # print(np.array(all_points))
            trans = None
            if red or green:
                if self.tf_buffer.wait_for_transform_async('map', source_frame, rclpy.time.Time()): #want it to be msg time but doesnt work and says needs future trans
                    try:
                        trans = self.tf_buffer.lookup_transform(
                            'map',
                            source_frame,
                            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
                    except (ConnectivityException, LookupException):#, ExtrapolationException, TransformException):
                        pass
                    except(ExtrapolationException) as e:
                        print(e)
            if trans != None:
                LIST = []
                temp = PointStamped()
                temp.header = msg.header
                for point in all_points:
                    temp.point.x = point[0]
                    temp.point.y = point[1]
                    temp.point.z = point[2]
                    formed = do_transform_point(temp,trans)
                    LIST.append(np.array([formed.point.x, formed.point.y, formed.point.z]))

                stuff = self.get_cloud(np.array(LIST), 'map')
                self.cloud_publisher.publish(stuff)


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
    def get_height (self,source_frame,stamp):
        trans = None
        if self.tf_buffer.wait_for_transform_async('map', source_frame,stamp): #want it to be msg time but doesnt work and says needs future trans
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    stamp, timeout=rclpy.duration.Duration(seconds=1))
            except (ConnectivityException, LookupException):#, ExtrapolationException, TransformException):
                pass
            except(ExtrapolationException) as e:
                print('Getting height error',e)
        if trans != None:
            self.camera_height = trans.transform.translation.z
            print('Got z height!')
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
