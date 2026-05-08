from sympy import im
from example_interfaces.srv import Trigger
from std_msgs.msg import Int16MultiArray 
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from manipulator.invers_kinematics import RobotArm
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped

import math

class Pick_up(Node):

    def __init__(self):
        super().__init__('publsih_posestamped')

        self.sub_object = self.create_publisher(PointStamped,'/estimated_pose',10)
        self.timer = self.create_timer(1, self.estimated_pose_callback)
    

    def estimated_pose_callback(self):
        pose = PointStamped()
        pose.point.x = 0.1
        pose.point.y = -0.15
        pose.point.z = 0.01
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        self.sub_object.publish(pose)
def main():
    rclpy.init()    

    pick_up_service = Pick_up()
    

    rclpy.spin(pick_up_service)


    rclpy.shutdown()


if __name__ == '__main__':
    main()      

