#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robp_interfaces.msg import PointPixel
from matplotlib import pyplot as plt

class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/seen_image', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.point_pub = self.create_publisher(PointPixel,'/found_point', 10)
        

        plt.ion()
        self.figure, ((self.ax_image, self.ax_mask, self.ax_line_point), (self.ax_hue, self.ax_saturation, self.ax_value)) = plt.subplots(2, 3)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


    def find_point_on_line(self, msg: Image):
        """
        Find the line and target point on line  
        :param msg: The image message
        :return: Point pixel values (column, row) and the line
        """

        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        height, width, _ = hsv_image.shape

        line_column = -1
        line_row = -1

        # Change these
        hue_min = 0
        hue_max = 30 # Max is 179
        saturation_min = 100
        saturation_max = 255
        value_min = 0
        value_max = 255

        tape_mask = cv2.inRange(hsv_image, (hue_min, saturation_min, value_min), (hue_max, saturation_max, value_max))

        # Improve/change below
        for row in range(height, height):
            for column in range(width):
                if tape_mask[row, column]:
                    line_column = column
                    line_row = row
        return line_column, line_row, tape_mask

    
    def plot(self, msg: Image, line_column : int, line_row : int, tape_mask):
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        hsv_image = cv2.bitwise_and(hsv_image,hsv_image,mask = tape_mask)
        tape_mask = cv2.bitwise_and(image,image,mask = tape_mask)

        hue_hist = cv2.calcHist([hsv_image], [0], None, [179], [1, 179])
        saturation_hist = cv2.calcHist([hsv_image], [1], None, [255], [1, 255])
        value_hist = cv2.calcHist([hsv_image], [2], None, [255], [1, 255])
        
        self.ax_image.cla()
        self.ax_mask.cla()
        self.ax_line_point.cla()
        self.ax_hue.cla()
        self.ax_saturation.cla()
        self.ax_value.cla()

        self.ax_image.title.set_text("Raw image")
        self.ax_image.set_xlabel("Column")
        self.ax_image.set_ylabel("Row")
        self.ax_image.imshow(image)

        self.ax_mask.title.set_text("Filtered image")
        self.ax_mask.imshow(tape_mask)

        annotated_img = image.copy()
        cv2.circle(annotated_img, (line_column, line_row), 6, (0, 255, 0), 4)
        cv2.putText(annotated_img, 'Point', (line_column, line_row), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)

        self.ax_line_point.title.set_text("Detected point")
        self.ax_line_point.imshow(annotated_img)

        self.ax_hue.title.set_text("'Hue' in filtered image")
        self.ax_hue.set_xlabel("Hue ")
        self.ax_hue.set_ylabel("Number of pixels")
        self.ax_hue.plot(hue_hist)

        self.ax_saturation.title.set_text("'Saturation' in filtered image")
        self.ax_saturation.set_xlabel("Saturation")
        self.ax_saturation.plot(saturation_hist)

        self.ax_value.title.set_text("'Value' in filtered image")
        self.ax_value.set_xlabel("Value")
        self.ax_value.plot(value_hist)
        
        # drawing updated values
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


    def image_callback(self, msg: Image):

        """
        Find the target point and publish it.
        :param msg: The image message
        :return: Point pixel values (column, row) 
        
        """
        column, row, tape_mask = self.find_point_on_line(msg)

        column = int(column)
        row = int(row)

        self.plot(msg, column, row, tape_mask)

        pt = PointPixel()
        pt.header = msg.header
        pt.width = int(column)
        pt.height = int(row)
        self.point_pub.publish(pt)
        self.get_logger().info(f'Publishing point on image msg {column, row}')


def main():
    rclpy.init()
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()