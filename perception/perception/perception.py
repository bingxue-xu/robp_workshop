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
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.point_pub = self.create_publisher(PointPixel, '/found_point', 10)
        self.hsv_filter_pub = self.create_publisher(
            HSVFilter, '/hsv_filter', 10)

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

        
        ############## START Modify the values inbetween here ################
        ############## green_tape_mask ################
        green_hsv_filter = HSVFilter()

        green_hsv_filter.hue_min = 60           # Range: [0, 360]
        green_hsv_filter.hue_max = 180         # Range: [0, 360]
        green_hsv_filter.saturation_min = 50    # Range: [0, 255]
        green_hsv_filter.saturation_max = 255  # Range: [0, 255]
        green_hsv_filter.value_min = 50         # Range: [0, 255]
        green_hsv_filter.value_max = 255       # Range: [0, 255]

        # We'll publish the HSV filter only when we find a point using it
        # self.hsv_filter_pub.publish(green_hsv_filter)

        green_tape_mask = cv2.inRange(hsv_image, (green_hsv_filter.hue_min / 2, green_hsv_filter.saturation_min,
                                green_hsv_filter.value_min), (green_hsv_filter.hue_max / 2, green_hsv_filter.saturation_max, green_hsv_filter.value_max))

        
        ############## red_tape_mask ################
        # Red is special in HSV color space because it wraps around from 360° back to 0°
        # So we need two ranges: one for the lower end (0-40°) and one for the upper end (340-360°)
        
        # First range: 0-40°
        red_hsv_filter_1 = HSVFilter()
        red_hsv_filter_1.hue_min = 0           # Range: [0, 360]
        red_hsv_filter_1.hue_max = 10         # Range: [0, 360]
        red_hsv_filter_1.saturation_min = 50    # Range: [0, 255]
        red_hsv_filter_1.saturation_max = 200  # Range: [0, 255]
        red_hsv_filter_1.value_min = 100         # Range: [0, 255]
        red_hsv_filter_1.value_max = 255       # Range: [0, 255]

        red_tape_mask_1 = cv2.inRange(hsv_image, (red_hsv_filter_1.hue_min / 2, red_hsv_filter_1.saturation_min,
                                red_hsv_filter_1.value_min), (red_hsv_filter_1.hue_max / 2, red_hsv_filter_1.saturation_max, red_hsv_filter_1.value_max))

        # Second range: 340-360°
        red_hsv_filter_2 = HSVFilter()
        red_hsv_filter_2.hue_min = 340           # Range: [0, 360]
        red_hsv_filter_2.hue_max = 358         # Range: [0, 360]
        red_hsv_filter_2.saturation_min = 50    # Range: [0, 255]
        red_hsv_filter_2.saturation_max = 200  # Range: [0, 255]
        red_hsv_filter_2.value_min = 70         # Range: [0, 255]
        red_hsv_filter_2.value_max = 255       # Range: [0, 255]

        red_tape_mask_2 = cv2.inRange(hsv_image, (red_hsv_filter_2.hue_min / 2, red_hsv_filter_2.saturation_min,
                                red_hsv_filter_2.value_min), (red_hsv_filter_2.hue_max / 2, red_hsv_filter_2.saturation_max, red_hsv_filter_2.value_max))

        # Create a combined mask using bitwise OR
        red_tape_mask = cv2.bitwise_or(red_tape_mask_1, red_tape_mask_2)



        ############## yellow_tape_mask ################
        yellow_hsv_filter = HSVFilter()

        yellow_hsv_filter.hue_min = 40           # Range: [0, 360]
        yellow_hsv_filter.hue_max = 60         # Range: [0, 360]
        yellow_hsv_filter.saturation_min = 100    # Range: [0, 255]
        yellow_hsv_filter.saturation_max = 255  # Range: [0, 255]
        yellow_hsv_filter.value_min = 100         # Range: [0, 255]
        yellow_hsv_filter.value_max = 255       # Range: [0, 255]

        # self.hsv_filter_pub.publish(yellow_hsv_filter)

        yellow_tape_mask = cv2.inRange(hsv_image, (yellow_hsv_filter.hue_min / 2, yellow_hsv_filter.saturation_min,
                                yellow_hsv_filter.value_min), (yellow_hsv_filter.hue_max / 2, yellow_hsv_filter.saturation_max, yellow_hsv_filter.value_max))


        line_column = -1
        line_row = -1

        # Image size
        height = hsv_image.shape[0]
        width = hsv_image.shape[1]

        ############## START Improve/change below ###########################

        # With the given solution it will always pick a point on the line on the last row,
        # which is towards the bottom of the image. Is this a good solution? What would
        # the controller want?

        # Check for points on green mask first, then red, then yellow (priority order)
        masks_with_filters = [
            (green_tape_mask, green_hsv_filter),
            (red_tape_mask_2, red_hsv_filter_2),  
            (red_tape_mask_1, red_hsv_filter_1),
            (yellow_tape_mask, yellow_hsv_filter),
        ]
        
        for i, (tape_mask, hsv_filter) in enumerate(masks_with_filters):
            points = np.where(tape_mask > 0)
            if len(points[0]) > 0:  
                idx = len(points[0]) // 2
                line_row = points[0][idx]
                line_column = points[1][idx]
                
                # Debug print
                # print(f"Detected color index: {i}, publishing HSV filter: {hsv_filter}")
                self.hsv_filter_pub.publish(hsv_filter)
            
                return line_column, line_row

        ############## END Improve/change below ###########################

        # Return -1, -1 if no point was found on any mask
        # No need to publish HSV filter if no point was found
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


def main():
    rclpy.init()
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()