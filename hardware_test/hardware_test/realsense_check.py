# hardware_ws/src/hardware_test/hardware_test/realsense_check.py

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from hardware_test.base_test import BaseTest

class RealsenseCheck(BaseTest):
    def __init__(self):
        super().__init__(
            node_name='realsense_check',
            robot_name=self.declare_parameter('robot_name', '').value,
            domain_id=self.declare_parameter('domain_id', 0).value,
            json_folder=self.declare_parameter('json_folder', '').value
        )
        self.hardware_key = 'Realsense'
        self.topic_name = self.declare_parameter('topic_name', '/camera/color/image_raw').value
        self.timeout_s = self.declare_parameter('timeout_s', 5.0).value

        # Bridge for converting ROS Image to OpenCV image
        self.bridge = CvBridge()

    def save_image(self, msg: Image):
        """
        Convert the ROS Image message to an OpenCV image and save as PNG.
        Filename format: <robot_name>_<topic_name_clean>_<timestamp>.png
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        topic_clean = self.topic_name.strip('/').replace('/', '_')
        timestamp = msg.header.stamp.sec  # Use seconds-level timestamp
        filename = f"{self.robot_name}_{topic_clean}_{timestamp}.png"
        save_dir = os.path.join(self.json_folder, self.robot_name)
        os.makedirs(save_dir, exist_ok=True)
        file_path = os.path.join(save_dir, filename)

        try:
            import cv2
            cv2.imwrite(file_path, cv_image)
            self.get_logger().info(f"Image saved to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def is_passing(self, msg: Image, **kwargs):
        """
        Save image first, then return True if width>0 and height>0.
        """
        self.save_image(msg)
        return msg.width > 0 and msg.height > 0

    def perform_test(self):
        """
        Use the base class run_test() method for overall logic.
        """
        return self.run_test(
            hardware_key=self.hardware_key,
            msg_type=Image,
            topic_name=self.topic_name,
            timeout_s=self.timeout_s
        )

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCheck()
    node.main_spin_and_exit()
