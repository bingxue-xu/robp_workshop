#!/usr/bin/env python3
"""
hardware_ws/src/hardware_test/hardware_test/realsense_check.py
"""
import os
import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hardware_test.base_test import BaseTest

class RealSenseCheck(BaseTest):
    def __init__(self):
        super().__init__(node_name='realsense_check')
        self.hardware_key = 'RealSense'
        self.bridge = CvBridge()
        self.saved = False

    def setup_parameters(self):
        robot_name = self.declare_parameter('robot_name', '').value
        domain_id = self.declare_parameter('domain_id', 0).value
        json_folder = self.declare_parameter(
            'json_folder', '~/dd2419/workshop_ws/src/hardware_test/test_results'
        ).value

        self.topic_name = self.declare_parameter(
            'topic_name', '/camera/camera/color/image_raw'
        ).value
        self.timeout_s = self.declare_parameter(
            'timeout_s', 50.0
        ).value

        self.update_config(
            robot_name=robot_name,
            domain_id=domain_id,
            json_folder=json_folder
        )

    def is_passing(self, msg: Image, **kwargs):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            detail = f"CV conversion failed: {e}"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

        flat = cv_img.reshape(-1)
        if np.all(flat == 0):
            detail = "Image is completely black"
            self.get_logger().error(detail)
            raise RuntimeError(detail)
        if np.all(flat == flat.max()):
            detail = "Image is completely white"
            self.get_logger().error(detail)
            raise RuntimeError(detail)

        if not self.saved:
            safe_name = self.robot_name + '_realsense_' + self.topic_name.strip('/').replace('/', '_') + '.png'
            out_path = os.path.join(self.json_folder + '/images', safe_name)
            try:
                cv2.imwrite(out_path, cv_img)
                self.get_logger().info(f"Saved sample image to {out_path}")
                self.saved = True
            except Exception as e:
                self.get_logger().warn(f"Could not save sample image: {e}")

        detail = f"Received valid color frame ({msg.width}×{msg.height})"
        self.get_logger().info(detail)
        return True, detail

    def perform_test(self):
        self.setup_parameters()
        return self.run_test(
            hardware_key=self.hardware_key,
            msg_type=Image,
            topic_name=self.topic_name,
            timeout_s=self.timeout_s
        )


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCheck()
    node.main_spin_and_exit()

if __name__ == '__main__':
    main()
