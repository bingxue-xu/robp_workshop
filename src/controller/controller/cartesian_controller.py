#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist


class CartesianController(Node):

    def __init__(self):
        super().__init__('cartesian_controller')
        self.des_lin = 0  # placeholder
        self.des_ang = 0  # placeholder
        self.endcode_right = 0  # placeholder
        self.endcode_left = 0  # placeholder

        self.r = 0.098425/2  # wheel radius
        self.ticks = 360  # ticks per revolution
        self.b = 0.30  # wheel base

        self.alpha_1 = 0.25  # Pos Gain for left wheel
        # self.beta_1 = 0.00083   #Neg Gain for left wheel
        # self.beta_1 = 0.004
        self.beta_1 = 0.0

        self.alpha_2 = 0.252  # Pos Gain for right wheel
        # self.beta_2 = 0.006   #Neg Gain for right wheel
        # self.beta_2 = 0.004
        self.beta_2 = 0.0

        self.int_error_1 = 0  # left
        self.int_error_2 = 0  # right

        self.publisher = self.create_publisher(
            DutyCycles, '/motor/duty_cycles', 10)

        self.subscription_twist = self.create_subscription(
            Twist, '/cmd_vel', self.callback_twist, 10)

        self.subscription_twist  # prevent unused variable warning

        self.subscription_encoders = self.create_subscription(
            Encoders, '/motor/encoders', self.callback_encoders, 10)

        self.subscription_encoders  # prevent unused variable warning
        self.dt = 0.05
        self.old_time_sec = 0
        self.old_time_nanosec = 0

    def callback_encoders(self, msg1):
        self.encode_left = msg1.delta_encoder_left
        self.endcode_right = msg1.delta_encoder_right

        msg = DutyCycles()

        est_v_2 = 2*np.pi*self.dt*self.endcode_right*self.r / \
            self.ticks  # estimated angular velocity for right wheel
        est_v_1 = 2*np.pi*self.dt*self.endcode_left*self.r / \
            self.ticks  # estimated angular velocity for left wheel

        # desired angular velocity for right wheel
        des_v_2 = self.b*self.des_ang + self.des_lin
        des_v_1 = 2*self.des_lin - des_v_2  # desired angular velocity for left wheel

        # left wheel
        error_left = des_v_1 - est_v_1
        self.int_error_1 += error_left*self.dt  # 0.1 = dt

        pwm_1 = self.alpha_1*error_left + self.beta_1*self.int_error_1

        # right wheel
        error_right = des_v_2 - est_v_2
        self.int_error_2 += error_right*self.dt
        pwm_2 = self.alpha_2*error_right + self.beta_2*self.int_error_2

        msg.duty_cycle_left = np.float64(pwm_1)
        msg.duty_cycle_right = np.float64(pwm_2)

        self.publisher.publish(msg)

        # self.dt = self.old_time_seconds
        # self.old_time_sec = msg1.header.stamp.seconds
        # self.old_time_nanosec = msg1.header.stamp.nanoseconds

        # msg1 = DutyCycles()
        # msg1.duty_cycle_left = np.float64(0)
        # msg1.duty_cycle_right = np.float64(0)
        # self.publisher.publish(msg1)

    def callback_twist(self, msg):
        self.des_lin = msg.linear.x
        self.des_ang = msg.angular.z


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
