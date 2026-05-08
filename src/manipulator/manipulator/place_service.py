
from example_interfaces.srv import Trigger
from std_msgs.msg import Int16MultiArray
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PointStamped
from manipulator.invers_kinematics import RobotArm
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from ida_interfaces.srv import EstPose
import tf2_geometry_msgs
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from arian_interfaces.msg import PointWithRadius


class Place(Node):

    def __init__(self):
        super().__init__('pick_up_service')
        self.publisher = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=10000))
        self.tf_listner = TransformListener(self.tf_buffer, self)
        cbr = ReentrantCallbackGroup()
        self.est_pose = PointStamped()
        self.current_target = PointWithRadius()
        self.sub_object = self.create_subscription(
            PointWithRadius, '/estimated_pose', self.estimated_pose_callback, 10, callback_group=cbr)
        self.sub_target = self.create_subscription(
            PointWithRadius, '/current_target', self.current_target_callback, 10, callback_group=cbr)

        self.srv = self.create_service(
            Trigger, 'place_service', self.pick_up_callback, callback_group=cbr)

    def pick_up_callback(self, request, response):
        inversK = RobotArm()
        traget = 'Robot_arm_base'
        if self.est_pose.marker_id == self.current_target.marker_id:
            latest_est_pose = self.est_pose.point
        else:
            self.get_logger().info(str(self.est_pose.id))
            response.success = False
            return response
            #latest_est_pose = self.current_target.point
        #latest_est_pose = self.est_pose.point

        try:
            t = self.tf_buffer.lookup_transform_full(traget, rclpy.time.Time(
            ), latest_est_pose.header.frame_id, latest_est_pose.header.stamp, "map", timeout=rclpy.duration.Duration(seconds=4.0))
            pose = tf2_geometry_msgs.do_transform_point(latest_est_pose, t)
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
            self.get_logger().info('Transform could not be found')
            response.success = False
            return response
        try:

            self.get_logger().info('x '+str(pose.point.x)+ ' y '+ str(pose.point.y) + ' z ' + str(pose.point.z))
            # self.pose.point.x,self.pose.point.y,self.pose.point.z # this should contain a pose from the object we try to pick up
            if pose.point.x < 0 and abs(pose.point.y) < 0.1:
                pose.point.x = -pose.point.x
            servo_angles = inversK.inverse_kinematics(
                (pose.point.x +0.07, pose.point.y, pose.point.z + 0.10))
            data_sets = [[16000, 12000, servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3], 300, 2000, 2000, 2000, 2000, 2000],
                         [1000, 12000, servo_angles[0], servo_angles[1], servo_angles[2],
                             servo_angles[3], 500, 500, 500, 500, 500, 500],
                         [1000, 12000, 12000, 12000, 12000, 12000, 1000, 1000, 1000, 1000, 1000, 1000]]

            msg = Int16MultiArray()
            for i in range(len(data_sets)):
                msg.data = data_sets[i]
                self.publisher.publish(msg)
                time.sleep(3)
            response.success = True
            return response
        except:
            response.success = False
            self.get_logger().info('Could not calculate inverse kinematics')
            return response

    def estimated_pose_callback(self, msg: PointWithRadius):
        self.est_pose = msg

    def current_target_callback(self, msg: PointWithRadius):
        self.current_target = msg


def main():
    rclpy.init()
    node = Place()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
