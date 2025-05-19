#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist


class CartesianController(Node):

    def __init__(self):
        super().__init__('cartesian_controller')
        
        self.subscp_twist = self.create_subscription(Twist, '/motor_controller/twist', self.twist_callback, 10)
        self.subscp_encoder = self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)
        self.publish_dutycycle = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        self.declare_parameter('frequency', 20)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('wheel_radius', 0.04921)
        self.declare_parameter('ticks_per_revolution', 3072)

        self.desired_linear= 0.0
        self.desired_angular = 0.0
        self.estimated_v_left = 0.0
        self.estimated_v_right = 0.0

        self.int_err_left = 0
        self.int_err_right = 0

        self.alpha_left = 0.5
        self.beta_left = 0.00065
        self.alpha_right = 0.44
        self.beta_right = 0.0005

        
    def twist_callback(self, msg):
        self.desired_linear = msg.linear.x
        self.desired_angular = msg.angular.z
        self.get_logger().info(f'desired velocity {self.desired_linear}, {self.desired_angular}')

    def encoder_callback(self, msg):

        """Takes encoder readings and twist information update dutycycles."""
        
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        dt = 1.0 / frequency
        wb = self.get_parameter('wheel_base').get_parameter_value().double_value
        wr = self.get_parameter('wheel_radius').get_parameter_value().double_value
        ticks_per_revolution = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        radians_per_tick = 2 * np.pi / ticks_per_revolution  

        self.estimated_v_left = radians_per_tick * msg.delta_encoder_left * frequency * wr
        self.estimated_v_right = radians_per_tick * msg.delta_encoder_right * frequency * wr
        self.get_logger().info(f'estimated velocity {self.estimated_v_left}, {self.estimated_v_right}')

        desired_v_left = self.desired_linear - self.desired_angular * wb
        desired_v_right = self.desired_linear + self.desired_angular * wb

        error_left = desired_v_left - self.estimated_v_left
        self.int_err_left += error_left * dt
        pwm_left = self.alpha_left * error_left + self.beta_left * self.int_err_left

        error_right = desired_v_right - self.estimated_v_right
        self.int_err_right += error_right * dt
        pwm_right = self.alpha_right * error_right + self.beta_right * self.int_err_right

        abs_left = abs(pwm_left)
        abs_right = abs(pwm_right)
        # Might want to limit to 0.5 instead of 1.0
        if abs_left > 0.5 or abs_right > 0.5:
            pwm_left /= max(abs_left, abs_right)
            pwm_right /= max(abs_left, abs_right)

        msg = DutyCycles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.duty_cycle_left = pwm_left
        msg.duty_cycle_right = pwm_right
        self.publish_dutycycle.publish(msg)


def main():
    rclpy.init()
    node = CartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()