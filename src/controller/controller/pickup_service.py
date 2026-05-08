#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray


class OpenLoopController(Node):

    def __init__(self):
        super().__init__('pickup_service')
        # The message will look like this:ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int16MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

        self.publisher = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        timer = 4
        self.i = 0
        if self.i <=2:
            self.timer = self.create_timer(timer, self.timer_callback)
        

    # TODO: Implement
        
    def timer_callback(self):
        data_sets = [[5000,12000,16000,5000,18000,20000,2000,2000,2000,2000,2000,2000],
                     [12000,12000,16000,5000,18000,20000,2000,2000,2000,2000,2000,2000],
                    [12000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000]]
        msg = Int16MultiArray()
        msg.data = data_sets[self.i]
        self.publisher.publish(msg)

        self.i += 1
             

def main():
    rclpy.init()
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()