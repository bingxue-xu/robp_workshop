#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Point, Vector3, PointStamped, Quaternion
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, UVCoordinate
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from PIL import Image as RawImage, ImageTransform
import io

from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
import struct


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def find_coeffs(pa, pb):
    matrix = []
    for p1, p2 in zip(pa, pb):
        matrix.append([p1[0], p1[1], 1, 0, 0, 0, -p2[0]*p1[0], -p2[0]*p1[1]])
        matrix.append([0, 0, 0, p1[0], p1[1], 1, -p2[1]*p1[0], -p2[1]*p1[1]])

    A = np.matrix(matrix, dtype=float)
    B = np.array(pb).reshape(8)

    res = np.dot(np.linalg.inv(A.T * A) * A.T, B)
    return np.array(res).reshape(8)


class Workspace(Node):

    def __init__(self):
        super().__init__('image_map')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._map_pub = self.create_publisher(
            PointCloud2, '/map', qos_profile=latching_qos)
        self._img_pub = self.create_publisher(Image, '/seen_image', 10)

        # self.map_timer = self.create_timer(2, self.map_timer_callback)
        self.sensor_timer = self.create_timer(
            1 / 30, self.sensor_timer_callback)

        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(pkg_dir, 'workspace2.jpg')
        self.image = RawImage.open(image_path)

        width = 2000
        height = 3000
        # coeffs = find_coeffs(
        # [(width, 0), (0, 0), (0, height), (width, height)],
        # [(5871, 5563), (5919, 717), (530, 1277), (530, 4707)])
        coeffs = find_coeffs(
            [(0, 0), (0, height), (width, height), (width, 0)],
            [(412, 538), (181, 2704), (2085, 2681), (1793, 504)])
        self.image = self.image.transform(
            (width, height), RawImage.PERSPECTIVE, coeffs, resample=RawImage.Resampling.BICUBIC)

        self.map_timer_callback()

    def map_timer_callback(self):
        width_meter = 2
        height_meter = 3

        width = 300
        height = int(width * height_meter/width_meter)

        pixel_size = height_meter / height

        img = self.image.resize(
            (width, height), RawImage.Resampling.LANCZOS)

        # # Gamma correct
        # gamma = 0.4
        # img = img.point(lambda x: ((x/255)**gamma)*255)

        points = []
        for v in range(height):
            for u in range(width):
                x = height_meter - pixel_size * v
                y = width_meter - pixel_size * u
                z = 0.0
                color = img.getpixel((u, v))
                r = color[0]
                g = color[1]
                b = color[2]
                a = 255
                rgb = struct.unpack(
                    'I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
                  ]

        header = Header()
        header.stamp = Time.to_msg(self.get_clock().now())
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points)

        self._map_pub.publish(pc2)

        # # publish map marker
        # marker = Marker()

        # marker.header.stamp = Time.to_msg(self.get_clock().now())
        # marker.header.frame_id = "map"

        # marker.ns = "map"
        # marker.id = 0
        # # marker.action = marker.ADD
        # # marker.lifetime =
        # marker.frame_locked = True

        # marker.type = Marker.TRIANGLE_LIST

        # marker.points = []
        # marker.colors = []
        # for v in range(0, height, 2):
        #     for u in range(0, width, 2):
        #         p1 = Point(x=height_meter - pixel_size * v,
        #                    y=width_meter - pixel_size * u, z=0.0)
        #         p2 = Point(x=p1.x + pixel_size, y=p1.y, z=0.0)
        #         p3 = Point(x=p1.x, y=p1.y + pixel_size, z=0.0)
        #         p4 = Point(x=p1.x + pixel_size, y=p1.y + pixel_size, z=0.0)

        #         pixel = img.getpixel((u, v))
        #         c1 = ColorRGBA(r=pixel[0]/255.0, g=pixel[1] /
        #                        255.0, b=pixel[2]/255.0, a=1.0)
        #         pixel = img.getpixel((u, v + 1))
        #         c2 = ColorRGBA(r=pixel[0]/255.0, g=pixel[1] /
        #                        255.0, b=pixel[2]/255.0, a=1.0)
        #         pixel = img.getpixel((u + 1, v))
        #         c3 = ColorRGBA(r=pixel[0]/255.0, g=pixel[1] /
        #                        255.0, b=pixel[2]/255.0, a=1.0)
        #         pixel = img.getpixel((u + 1, v + 1))
        #         c4 = ColorRGBA(r=pixel[0]/255.0, g=pixel[1] /
        #                        255.0, b=pixel[2]/255.0, a=1.0)

        #         marker.points.extend([p1, p3, p2, p2, p3, p4])
        #         marker.colors.extend([c1, c3, c2, c2, c3, c4])

        # self._map_pub.publish(marker)

        # marker.type = Marker.TRIANGLE_LIST

        # marker.scale = Vector3(x=1.0, y=1.0, z=1.0)

        # # marker.pose.position = Point(x=0.17, y=-0.56, z=0.0)
        # marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        # # marker.pose.orientation = quaternion_from_euler(0.0, 0.0, math.radians(90.0))

        # p1 = Point(x=0.0, y=0.0, z=0.0)
        # p2 = Point(x=0.0, y=2.0, z=0.0)
        # p3 = Point(x=3.0, y=0.0, z=0.0)
        # p4 = Point(x=3.0, y=0.0, z=0.0)
        # p5 = Point(x=0.0, y=2.0, z=0.0)
        # p6 = Point(x=3.0, y=2.0, z=0.0)
        # marker.points = [p1, p2, p3, p4, p5, p6]

        # marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # uv1 = UVCoordinate(u=1.0, v=1.0)
        # uv2 = UVCoordinate(u=0.0, v=1.0)
        # uv3 = UVCoordinate(u=1.0, v=0.0)
        # uv4 = UVCoordinate(u=1.0, v=0.0)
        # uv5 = UVCoordinate(u=0.0, v=1.0)
        # uv6 = UVCoordinate(u=0.0, v=0.0)
        # marker.uv_coordinates = [uv1, uv2, uv3, uv4, uv5, uv6]

        # full_image_low_res = self.image.resize((200, 300), RawImage.Resampling.LANCZOS)

        # # # Gamma correct
        # gamma = 0.4
        # full_image_low_res = full_image_low_res.point(lambda x: ((x/255)**gamma)*255)

        # buf = io.BytesIO()
        # full_image_low_res.save(buf, format='png')

        # marker.texture.data = buf.getvalue()
        # marker.texture_resource = "embedded://workspace.png"
        # marker.texture.format = "png"

        # self._map_pub.publish(marker)

    def sensor_timer_callback(self):
        # corners = ul_x, ul_y, ur_x, ur_y, lr_x, lr_y, ll_x, ll_y
        odom_corners = self.corners_in_robot_view()
        if odom_corners is None:
            return

        image_corners = self.map_to_image_pixels(odom_corners, pixels_per_meter=1000,
                                                 img_width=self.image.size[0], img_height=self.image.size[1])

        # [31,146,88,226,252,112,195,31]
        transform = [coord for point in image_corners for coord in point]
        sensor_image = self.image.transform((320, 180), ImageTransform.QuadTransform(
            transform), fillcolor=(135, 116, 101), resample=RawImage.Resampling.BICUBIC)
        seen_image_msg = Image()
        seen_image_msg.header.stamp = Time.to_msg(self.get_clock().now())
        seen_image_msg.header.frame_id = 'base_link'
        sensor_image_cv2 = cv2.cvtColor(
            np.array(sensor_image), cv2.COLOR_RGB2BGR)
        seen_image_msg = self.bridge.cv2_to_imgmsg(sensor_image_cv2, 'bgr8')
        self._img_pub.publish(seen_image_msg)

    def map_to_image_pixels(self, corners, pixels_per_meter, img_width, img_height):
        uvs = []
        for x, y in corners:
            u = img_width - (y * pixels_per_meter)
            v = img_height - (x * pixels_per_meter)
            uvs.append((int(round(u)), int(round(v))))

        return uvs

    def corners_in_robot_view(self):

        camera_x = 0.17
        height = 0.145
        width = height * (320 / 180)

        raw_pts = [
            (camera_x + height, width / 2, 0.0),
            (camera_x, width / 2, 0.0),
            (camera_x, -width / 2, 0.0),
            (camera_x + height, -width / 2, 0.0),
        ]

        cur_time = self.get_clock().now()

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame='map',
            source_frame='base_link',
            time=cur_time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=0.5)

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                cur_time.to_msg()
            )
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

        odom_corners = []
        for x, y, z in raw_pts:
            point = PointStamped()
            point.header.frame_id = 'base_link'
            point.header.stamp = rclpy.time.Time().to_msg()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            odom_points = do_transform_point(point, tf)
            odom_corners.append((odom_points.point.x, odom_points.point.y))

        return odom_corners


def main():
    rclpy.init()
    node = Workspace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
