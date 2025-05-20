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

        self.subscription = self.create_subscription(
            PointPixel, '/found_point', self.point_on_line_callback, 10)
        self.publisher_ = self.create_publisher(
            Twist, '/motor_controller/twist', 10)

    def point_on_line_control(self,  column: int, row: int):
        """
        Calculate the linear and angular velocity based on a point on the line

        :param column: The width coordinate of the point on the line
        :param row: The height coordinate of the point on the line
        :return: The linear [0,1] and angular [-1, 1] velocities
        """

        # Make the robot rotate if we do not find the line
        if column < 0 or row < 0:
            linear_velocity = 0.0
            angular_velocity = 1.0
            return linear_velocity, angular_velocity

        linear_velocity = 0.5
        angular_velocity = 0.0

        # Image size
        width = 320
        height = 180

        error_side = 0
        error_side = (column - width / 2)

        # Controller 
        gain = 0.005
        angular_velocity = -gain * error_side

        return linear_velocity, angular_velocity

    def point_on_line_callback(self, msg: PointPixel):
        linear_velocity, angular_velocity = self.point_on_line_control(
            msg.column, msg.row)

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        self.publisher_.publish(twist_msg)


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
