#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2

from robp_interfaces.msg import PointPixel
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.subscription = self.create_subscription(PointPixel, '/found_point', self.point_on_line_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/motor_controller/twist', 10)


    def point_on_line_control(self,  column : int, row : int):
        """
        Calulate the linear and angular velocity based on the the point on line

        :param column: The width coordinate of the point on line
        :param row: The height coordinate of the point on line
        :return: The linear and angular velocity
        """
        
        if column < 0 or row < 0:
            linear_velocity = 0.0
            angular_velocity = 1.0
            return linear_velocity, angular_velocity

        linear_velocity = 0.5
        angular_velocity = 0.0

        width =320 
        height = 180 

        error = 0
        error = (column - width / 2)

        gain = 1
        gain = 2/width
        angular_velocity = -gain * error

        return linear_velocity, angular_velocity

    def point_on_line_callback(self, msg: PointPixel):
        linear_velocity, angular_velocity = self.point_on_line_control(msg.width, msg.height)

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Publishing twist msg {twist_msg.linear}, {twist_msg.angular}')


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()