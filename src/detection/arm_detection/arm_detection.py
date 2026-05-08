import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs

from example_interfaces.srv import Trigger
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

THRESHOLD = 0.7
class ArmDetection(Node):
    def __init__(self):
        super().__init__('arm_detection')
        self.object_detected = False
        self.bridge = CvBridge()

        # Subscribe to ImageRaw
        self.create_subscription(Image, 'image_raw', self.image_callback, 10) # TODO: Change to right names

        self.Image = None
        # Create a service
        self.srv = self.create_service(Trigger, 'arm_detection', self.arm_detect_callback)

    def image_callback(self, img_msg):
        self.Image = img_msg
        
        

    def detect(self, img):
        # Convert to BGR
        bgr_img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
        bgr_img = bgr_img.astype(np.float32) / 255

        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        img_value = hsv_img[:,:,2]

        if np.mean(img_value) > THRESHOLD:
            self.object_detected = True
            self.get_logger().info("Object detected in claw")

            return True
        else:
            self.object_detected = False
            self.get_logger().info("No object detected in claw")
            return False

    def arm_detect_callback(self, request, response):
        self.get_logger().info(self.Image.header)

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.Image, "passthrough")
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))

        Bool = self.detect(cv_image)
        if self.object_detected == True:
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()


