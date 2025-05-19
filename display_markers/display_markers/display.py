
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image
from robp_interfaces.msg import DutyCycles, PointPixel, HSVFilter
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import tf2_geometry_msgs


class Display(Node) :

    def __init__(self) :
        super().__init__('display')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        
        self.create_subscription(Image, '/seen_image', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.point_subs = self.create_subscription(PointPixel, '/found_point', self.point_callback, 10)
        self.point_subs = self.create_subscription(HSVFilter, '/hsv_filter', self.hsv_filter_callback, 10)
        self.dutycycle_sub = self.create_subscription(DutyCycles, '/motor/duty_cycles', self.dutycycle_callback, 10)
 
        self.img_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.img_masked_pub = self.create_publisher(Image, '/masked_image', 10)
        self.left_dutycycle_pub = self.create_publisher(Marker, '/left_duty_cycle', 10)
        self.right_dutycycle_pub = self.create_publisher(Marker, '/right_duty_cycle', 10)

        self.cv_rgb = None


    def image_callback(self, msg: Image):
        self.cv_rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def point_callback(self, msg: PointPixel) :

        """Annotate the image with the target point."""

        if self.cv_rgb is None:
            self.get_logger().warn('Waiting for image')
            return
        
        u = msg.column
        v = msg.row
        annotated_img = self.cv_rgb.copy()
        cv2.circle(annotated_img, (u, v), 6, (0, 255, 0), 4)
        cv2.putText(annotated_img, 'Point', (u, v), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)

        out_msg = self.bridge.cv2_to_imgmsg(annotated_img, 'rgb8')
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self.img_pub.publish(out_msg)
        self.get_logger().info(f'Publishing point on image msg {u, v}')

    def hsv_filter_callback(self, msg: HSVFilter) :

        """Mask the image with the HSV filter."""

        if self.cv_rgb is None:
            self.get_logger().warn('Waiting for image')
            return
        
        hsv_image = cv2.cvtColor(self.cv_rgb, cv2.COLOR_RGB2HSV)

        tape_mask = cv2.inRange(hsv_image, (msg.hue_min / 2, msg.saturation_min, msg.value_min), (msg.hue_max / 2, msg.saturation_max, msg.value_max))

        image = cv2.bitwise_and(self.cv_rgb,self.cv_rgb,mask = tape_mask)

        out_msg = self.bridge.cv2_to_imgmsg(image, 'rgb8')
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self.img_masked_pub.publish(out_msg)
        self.get_logger().debug(f'Publishing masked image msg')


    def dutycycle_callback(self, msg: DutyCycles):

        """Display duty cycle markers in rviz."""

        arrow_header = msg.header
        arrow_ns = 'duty_cycle'
        arrow_scale = Vector3(x=0.01, y=0.05, z=0.1)
        arrow_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # the left duty cycle
        left = msg.duty_cycle_left
        left_arrow = Marker()
        left_arrow.header = arrow_header
        left_arrow.ns = arrow_ns
        left_arrow.id = 0
        left_arrow.type = Marker.ARROW
        left_arrow.action = Marker.ADD
        left_arrow.scale = arrow_scale
        left_arrow.color = arrow_color
        left_start = Point(x=0.0, y=0.17, z=0.0)
        left_end = Point(x=25*left*arrow_scale.z, y=0.17, z=0.0)
        left_arrow.points = [left_start, left_end]

        # the right duty cycle
        right = msg.duty_cycle_right
        right_arrow = Marker()
        right_arrow.header = arrow_header
        right_arrow.ns = arrow_ns
        right_arrow.id = 1
        right_arrow.type = Marker.ARROW
        right_arrow.action = Marker.ADD
        right_arrow.scale = arrow_scale
        right_arrow.color = arrow_color
        right_start = Point(x=0.0, y=-0.17, z=0.0)
        right_end = Point(x=25*right*arrow_scale.z, y=-0.17, z=0.0)
        right_arrow.points = [right_start, right_end]

        # Publish the markers
        self.left_dutycycle_pub.publish(left_arrow)
        self.right_dutycycle_pub.publish(right_arrow)
        self.get_logger().info(f'Publishing duty cycle markers {left}, {right}')


def main() :
    rclpy.init()
    node = Display()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
