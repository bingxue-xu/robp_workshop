#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from robp_boot_camp_interfaces.msg import ADConverter
from geometry_msgs.msg import Twist


class WallFollowingController(Node):

    def __init__(self):
        super().__init__('wall_following_controller')

        self.publisher = self.create_publisher(Twist, '/motor_controller/twist', 10) 

        self.subscriber = self.create_subscription(ADConverter,'/kobuki/adc',self.callback_ADC,10)
        self.subscriber

        self.rot = 0 #placeholder
        self.alpha = 0.05 #Gain

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = np.float64(1.0) #constant velocity
        msg.angular.z = np.float64(self.rot)

        self.publisher.publish(msg)


    def callback_ADC(self,msg):
        front = msg.ch1
        rear = msg.ch2

        if front < 400 and rear < 400:
            self.rot = 0.5
        else:
            self.rot = -self.alpha*(front-rear)
        self.timer_callback()


def main():
    rclpy.init()
    node = WallFollowingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()