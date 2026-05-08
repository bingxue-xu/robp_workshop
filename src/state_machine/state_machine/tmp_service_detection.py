from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from bing_interfaces.srv import TmpDetection

import math

class Detection(Node):

    def __init__(self):
        super().__init__('detection_service')

        self.pose = PointStamped()
        self.sub_object = self.create_subscription(PointStamped,'/estimated_pose', self.estimated_pose_callback, 10)
        self.srv = self.create_service(TmpDetection, '/detection_service', self.detection_callback)

    def detection_callback(self, request,response):
        response.estimated_pose = self.pose
        print(response.estimated_pose)
        return response

    def estimated_pose_callback(self, msg):
        self.pose= msg


def main():
    rclpy.init()

    detection_service = Detection()
    

    rclpy.spin(detection_service)


    rclpy.shutdown()


if __name__ == '__main__':
    main()      

