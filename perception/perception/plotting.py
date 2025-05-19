#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robp_interfaces.msg import PointPixel, HSVFilter
from matplotlib import pyplot as plt

class Plotting(Node):
    def __init__(self):
        super().__init__('plotting')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/seen_image', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(PointPixel, '/found_point', self.point_callback, 10)
        self.create_subscription(HSVFilter, '/hsv_filter', self.hsv_callback, 10)

        plt.ion()
        self.figure, ((self.ax_image, self.ax_mask, self.ax_line_point), (self.ax_hue, self.ax_saturation, self.ax_value)) = plt.subplots(2, 3)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

        self.image = None
        self.hsv_filter = None

    def point_callback(self, msg: PointPixel):
        if not self.image or not self.hsv_filter:
            return

        image = self.bridge.imgmsg_to_cv2(self.image, 'rgb8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        tape_mask = cv2.inRange(hsv_image, (self.hsv_filter.hue_min / 2, self.hsv_filter.saturation_min, self.hsv_filter.value_min), (self.hsv_filter.hue_max / 2, self.hsv_filter.saturation_max, self.hsv_filter.value_max))

        hsv_image = cv2.bitwise_and(hsv_image,hsv_image,mask = tape_mask)
        tape_mask = cv2.bitwise_and(image,image,mask = tape_mask)

        hue_hist = cv2.calcHist([hsv_image], [0], None, [359], [1, 359])
        saturation_hist = cv2.calcHist([hsv_image], [1], None, [255], [1, 255])
        value_hist = cv2.calcHist([hsv_image], [2], None, [255], [1, 255])

        hue_hist = np.repeat(hue_hist, 2, axis=0)
        
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
        cv2.circle(annotated_img, (msg.column, msg.row), 6, (0, 255, 0), 4)
        cv2.putText(annotated_img, 'Point', (msg.column, msg.row), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)

        self.ax_line_point.title.set_text("Detected point")
        self.ax_line_point.imshow(annotated_img)

        self.ax_hue.title.set_text("'Hue' in filtered image")
        self.ax_hue.set_xlabel("Hue ")
        self.ax_hue.set_ylabel("Number of pixels")
        self.ax_hue.set(xlim=(self.hsv_filter.hue_min - 1, self.hsv_filter.hue_max + 1))
        self.ax_hue.plot(hue_hist)

        self.ax_saturation.title.set_text("'Saturation' in filtered image")
        self.ax_saturation.set_xlabel("Saturation")
        self.ax_saturation.set(xlim=(self.hsv_filter.saturation_min - 1, self.hsv_filter.saturation_max + 1))
        self.ax_saturation.plot(saturation_hist)

        self.ax_value.title.set_text("'Value' in filtered image")
        self.ax_value.set_xlabel("Value")
        self.ax_value.set(xlim=(self.hsv_filter.value_min - 1, self.hsv_filter.value_max + 1))
        self.ax_value.plot(value_hist)
        
        # drawing updated values
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def hsv_callback(self, msg: HSVFilter):
        self.hsv_filter = msg

    def image_callback(self, msg: Image):
        self.image = msg


def main():
    rclpy.init()
    node = Plotting()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()