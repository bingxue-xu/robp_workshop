#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robp_interfaces.msg import PointPixel, HSVFilter

class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/seen_image', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.point_pub = self.create_publisher(PointPixel,'/found_point', 10)
        self.hsv_filter_pub = self.create_publisher(HSVFilter,'/hsv_filter', 10)

    def find_point_on_line(self, msg: Image):
        """
        Find the line and a target point on the line  
        :param msg: The image
        :return: Point pixel values (column, row) on the line
        """

        # Convert the image to a format that allows us to use the package OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # Convert the image to the HSV (hue, saturation, value) color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        height, width, _ = hsv_image.shape

        line_column = -1
        line_row = -1

        hsv_filter = HSVFilter()
        ############## START Modify the values inbetween here ################
        hsv_filter.hue_min = 40
        hsv_filter.hue_max = 60 # Max is 360
        hsv_filter.saturation_min = 100
        hsv_filter.saturation_max = 255
        hsv_filter.value_min = 0
        hsv_filter.value_max = 255
        ############## END Modify the values inbetween here ################
        self.hsv_filter_pub.publish(hsv_filter) 

        # Remove pixles outside the values above from the image
        tape_mask = cv2.inRange(hsv_image, (hsv_filter.hue_min / 2, hsv_filter.saturation_min, hsv_filter.value_min), (hsv_filter.hue_max / 2, hsv_filter.saturation_max, hsv_filter.value_max))

        ############## START Improve/change below ###########################
        for row in range(height):
            for column in range(width):
                if tape_mask[row, column]:
                    line_column = column
                    line_row = row
                    return line_column, line_row
        ############## END Improve/change below ###########################

        return line_column, line_row

    def image_callback(self, msg: Image):

        """
        Find the target point and publish it.
        :param msg: The image message
        :return: Point pixel values (column, row) 
        
        """
        column, row = self.find_point_on_line(msg)

        column = int(column)
        row = int(row)

        pt = PointPixel()
        pt.header = msg.header
        pt.column = column
        pt.row = row
        self.point_pub.publish(pt)
        self.get_logger().debug(f'Publishing point on image msg {column, row}')


def main():
    rclpy.init()
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
