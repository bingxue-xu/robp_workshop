#!/usr/bin/env python3
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.style as mplstyle
from matplotlib import pyplot as plt
from robp_interfaces.msg import PointPixel, HSVFilter
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy

# ValueError: 'asdas' is not a valid value for backend; supported values are ['GTK3Agg', 'GTK3Cairo', 'GTK4Agg', 'GTK4Cairo', 'MacOSX', 'nbAgg', 'QtAgg', 'QtCairo', 'Qt5Agg', 'Qt5Cairo', 'TkAgg', 'TkCairo', 'WebAgg', 'WX', 'WXAgg', 'WXCairo', 'agg', 'cairo', 'pdf', 'pgf', 'ps', 'svg', 'template']

mplstyle.use('fast')


class Plotting(Node):
    def __init__(self):
        super().__init__('plotting')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/seen_image', self.image_callback, 1)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(
            PointPixel, '/found_point', self.point_callback, 1)
        self.create_subscription(
            HSVFilter, '/hsv_filter', self.hsv_callback, 1)

        self.figure, ((self.ax_image, self.ax_mask, self.ax_line_point),
                      (self.ax_hue, self.ax_saturation, self.ax_value)) = plt.subplots(2, 3)
        plt.show(block=False)

        self.image = None
        self.hsv_filter = None

        self.raw_data = None
        self.mask_data = None
        self.annotated_data = None
        self.hue_data = None
        self.saturation_data = None
        self.value_data = None

    def point_callback(self, msg: PointPixel):
        if not self.image or not self.hsv_filter:
            return

        start = time.time()

        image = self.bridge.imgmsg_to_cv2(self.image, 'rgb8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        tape_mask = cv2.inRange(hsv_image, (self.hsv_filter.hue_min / 2, self.hsv_filter.saturation_min, self.hsv_filter.value_min),
                                (self.hsv_filter.hue_max / 2, self.hsv_filter.saturation_max, self.hsv_filter.value_max))

        hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=tape_mask)
        tape_mask = cv2.bitwise_and(image, image, mask=tape_mask)

        hue_hist = cv2.calcHist([hsv_image], [0], None, [179], [1, 179])
        saturation_hist = cv2.calcHist([hsv_image], [1], None, [255], [1, 255])
        value_hist = cv2.calcHist([hsv_image], [2], None, [255], [1, 255])

        hue_hist = np.log(hue_hist + 1)
        saturation_hist = np.log(saturation_hist + 1)
        value_hist = np.log(value_hist + 1)

        hue_hist = np.repeat(hue_hist, 2, axis=0)

        if self.raw_data:
            self.raw_data.set_data(image)
            self.ax_image.draw_artist(self.ax_image.patch)
            self.ax_image.draw_artist(self.raw_data)
        else:
            self.ax_image.title.set_text("Raw image")
            self.ax_image.set_xlabel("Column")
            self.ax_image.set_ylabel("Row")
            self.raw_data = self.ax_image.imshow(image)

        if self.mask_data:
            self.mask_data.set_data(tape_mask)
            self.ax_mask.draw_artist(self.ax_mask.patch)
            self.ax_mask.draw_artist(self.mask_data)
        else:
            self.ax_mask.title.set_text("Filtered image")
            self.ax_mask.set_xlabel("Column")
            self.ax_mask.get_yaxis().set_visible(False)
            self.mask_data = self.ax_mask.imshow(tape_mask)

        annotated_img = image.copy()
        if msg.column >= 0 or msg.row >= 0:
            cv2.circle(annotated_img, (msg.column, msg.row), 3, (0, 255, 0), 4)
            msg.row = max(20, min(annotated_img.shape[0] - 20, msg.row))
            if msg.column <= annotated_img.shape[1] / 2:
                cv2.putText(annotated_img, 'Point', (msg.column + 20, msg.row + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(annotated_img, 'Point', (msg.column - 100, msg.row + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if self.annotated_data:
            self.annotated_data.set_data(annotated_img)
            self.ax_line_point.draw_artist(self.ax_line_point.patch)
            self.ax_line_point.draw_artist(self.annotated_data)
        else:
            self.ax_line_point.title.set_text("Detected point")
            self.ax_line_point.set_xlabel("Column")
            self.ax_line_point.get_yaxis().set_visible(False)
            self.annotated_data = self.ax_line_point.imshow(annotated_img)

        if self.hue_data:
            x = [x for x in range(0, 358)]
            self.hue_data.set_offsets(np.column_stack((x, hue_hist)))
            self.ax_hue.draw_artist(self.ax_hue.patch)
            self.ax_hue.draw_artist(self.hue_data)
            # self.ax_hue.draw_artist(self.ax_hue.xaxis) 
            # self.ax_hue.draw_artist(self.ax_hue.yaxis)
        else:
            self.ax_hue.title.set_text("'Hue' in filtered image")
            self.ax_hue.set_xlabel("Hue ")
            self.ax_hue.set_ylabel("Log number of pixels")
            self.ax_hue.set(xlim=(0, 360))
            self.ax_hue.set(ylim=(-1, 12))
            x = [x for x in range(0, 358)]
            self.hue_data = self.ax_hue.scatter(x=x, y=hue_hist, c=x, cmap='hsv', s=10, animated=True)

        if self.saturation_data:
            self.saturation_data.set_ydata(saturation_hist)
            self.ax_saturation.draw_artist(self.ax_saturation.patch)
            self.ax_saturation.draw_artist(self.saturation_data)
        else:
            self.ax_saturation.title.set_text("'Saturation' in filtered image")
            self.ax_saturation.set_xlabel("Saturation")
            self.ax_saturation.set(xlim=(0, 255))
            self.ax_saturation.set(ylim=(-1, 12))
            self.ax_saturation.get_yaxis().set_visible(False)
            self.saturation_data, = self.ax_saturation.plot(saturation_hist, color='green', animated=True)

        if self.value_data:
            self.value_data.set_ydata(value_hist)
            self.ax_value.draw_artist(self.ax_value.patch)
            self.ax_value.draw_artist(self.value_data)
        else:
            self.ax_value.title.set_text("'Value' in filtered image")
            self.ax_value.set_xlabel("Value")
            self.ax_value.set(xlim=(0, 255))
            self.ax_value.set(ylim=(-1, 12))
            self.ax_value.get_yaxis().set_visible(False)
            self.value_data, = self.ax_value.plot(value_hist, color='blue', animated=True)

        # drawing updated values
        # self.figure.canvas.draw()
        self.figure.canvas.update()
        self.figure.canvas.flush_events()

        end = time.time()
        print(f"Plotting {end - start}s")

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


if __name__ == '__main__':
    main()
