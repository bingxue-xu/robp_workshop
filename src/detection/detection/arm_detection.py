import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from cv_bridge import CvBridge, CvBridgeError
import cv2

THRESHOLD = 0.2
class ArmDetection(Node):
    def __init__(self):
        super().__init__('arm_detection')
        self.object_detected = False
        self.bridge = CvBridge()
        cbr = ReentrantCallbackGroup()

        # Subscribe to ImageRaw

        self.Image = None
        # Create a service
        self.srv = self.create_service(Trigger, 'claw_detection', self.arm_detect_callback, callback_group=cbr)
        self.servo = None
        self.create_subscription(JointState, "/servo_pos_publisher", self.servo_callback, 10, callback_group=cbr)

    def servo_callback(self, msg):
        self.servo = abs(msg.position[0])




    def arm_detect_callback(self, request, response):

        if self.servo < 14700.0:
            response.success = True
            response.message = "Object in claw"
        else:
            response.success = False
            response.message = "No object in claw"
        return response

def main():
    rclpy.init()
    node = ArmDetection()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()


