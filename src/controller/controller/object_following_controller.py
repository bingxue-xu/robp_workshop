#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from robp_boot_camp_interfaces.msg import ADConverter
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
import tf2_geometry_msgs
from nav_msgs.msg import Path



"""
publish Twist so it aligns to the wall


linear_vel = < SOME CONSTANT >
angular_vel = alpha * ( distance_sensor1 - distance_sensor2 )

ros2 interface show robp_boot_camp_interfaces/msg/ADConverter
front: /kobuki/adc ch1
back: /kobuki/adc ch2
value 0-1023
d = 1.114e^(-0.004adc) adc value and d distance in meter

"""

class ObjectFollowingController(Node):

    def __init__(self):
        super().__init__('object_following_controller')

        self.point_stamped = PointStamped()
        self.r_pose = PoseStamped()

        self.tf_buffer = Buffer()
        self.tf_listner = TransformListener(self.tf_buffer,self)
        
        self.subscription = self.create_subscription(PointStamped, '/estimated_pose', self.pose_callback, 10)

        self.subscription = self.create_subscription(Path, '/path', self.robot_pose_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    

    def pose_callback(self, msg):
        self.get_logger().info(f'pose_callback')
        target_frame = 'map'

        self.get_logger().info(f'frame_id pose_callback {msg.header.frame_id}')
        self.get_logger().info(f'pose frame_id: {self.pose.header.frame_id}')

        self.point_stamped.header.stamp = msg.header.stamp
        self.point_stamped.header.frame_id = target_frame
        self.point_stamped.point.x = msg.point.x
        self.point_stamped.point.y = msg.point.y

        # check name for the target frame

        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         target_frame, 
        #         msg.header.frame_id, 
        #         msg.header.stamp, 
        #         timeout=rclpy.time.Duration(seconds = 0.5))
        #     self.point = tf2_geometry_msgs.do_transform_point(point_stamped, t)
        # except TransformException as ex:
        #     self.get_logger().info("Couldnt transform")


    def robot_pose_callback(self, msg):
        target_frame = 'map'

        self.pose = msg.poses[0]

        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, 
                self.pose.header.frame_id, 
                self.pose.header.stamp, 
                timeout=rclpy.time.Duration(seconds = 0.5))
            self.r_pose = tf2_geometry_msgs.do_transform_pose_stamped(self.pose, t)
        except TransformException as ex:
            self.get_logger().info("Couldnt transform")
        

    def timer_callback(self):

        angular_to_target = math.atan2(self.point_stamped.point.y - self.r_pose.pose.position.y, 
                                       self.point_stamped.point.x - self.r_pose.pose.position.x)
        distance_to_target = math.sqrt((self.point_stamped.point.y - self.r_pose.pose.position.y)**2 + 
                                       (self.point_stamped.point.x - self.r_pose.pose.position.x)**2)

        self.orientation_threshold = 0.1
        self.target_distance_threshold = 0.1

        if abs(angular_to_target) < self.orientation_threshold:
            self.desired_angular = 0 
            self.desired_linear = 0.5
        else:
            self.desired_linear = 0
            self.desired_angular = angular_to_target  

        if distance_to_target < self.target_distance_threshold:
            self.desired_linear = 0
            self.desired_angular = 0
            return

        msg = Twist()
        l_vel = 0.5
        alpha = 1

        # test some here if we should implement some kind of stop at the ang.velocity
                
        msg.linear.x = l_vel
        msg.angular.z = angular_to_target * alpha
        self.get_logger().info(f'publishing twist: {msg.linear.x}, {msg.angular.z}')
        self.publisher.publish(msg)




def main():
    rclpy.init()
    node = ObjectFollowingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()