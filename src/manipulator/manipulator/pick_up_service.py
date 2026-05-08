
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
from arian_interfaces.srv import EstPose
import tf2_geometry_msgs
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from arian_interfaces.msg import PointWithRadius
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class Pick_up(Node):

    def __init__(self):
        super().__init__('pick_up_service')
        self.publisher = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=100))
        self.tf_listner = TransformListener(self.tf_buffer, self)
        self.est_pose = PointWithRadius()
        self.current_target = PointWithRadius()
        cbr = ReentrantCallbackGroup()
        self.sub_object = self.create_subscription(
            PointWithRadius, '/estimated_pose', self.estimated_pose_callback, 10, callback_group=cbr)
        self.srv = self.create_service(
            Trigger, 'pick_up_service', self.pick_up_callback, callback_group=cbr)
        self.est_pose_service = self.create_service(
            EstPose, '/estimated_pose_service', self.estimated_pose_service_callback, callback_group=cbr)
        self.sub_current_target = self.create_subscription(
            PointWithRadius, '/current_target', self.current_target_callback, 10, callback_group=cbr)

    def pick_up_callback(self, request, response):
        inversK = RobotArm()
        traget = 'Robot_arm_base'
        if self.current_target.id == self.est_pose.id:
            latest_est_pose = self.est_pose.point
        else:
            #latest_est_pose = self.current_target.point
            self.get_logger().info(str(self.est_pose.id))
            response.success = False
            return response
        #latest_est_pose = self.est_pose.point

        try:
            t = self.tf_buffer.lookup_transform_full(traget, rclpy.time.Time(
            ), latest_est_pose.header.frame_id, latest_est_pose.header.stamp, "map", timeout=rclpy.duration.Duration(seconds=1.0))
            pose = tf2_geometry_msgs.do_transform_point(latest_est_pose, t)
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
            print('Transform could not be found')
            response.success = False
            return response
        try:
            if pose.point.x > 0.23 or pose.point.y >0.17:
                response.success = False
                response.message = "Object is too far away"
                return response
            self.get_logger().info(str(pose.point.x)+ str(pose.point.y) + str(pose.point.z))
            # self.pose.point.x,self.pose.point.y,self.pose.point.z # this should contain a pose from the object we try to pick up
            servo_angles = inversK.inverse_kinematics(
                (pose.point.x, pose.point.y, pose.point.z-0.02))
            data_sets = [[1000, 12000, 12000, 12000, 12000, 12000, 500, 500, 500, 500, 500, 500],
                [100, 12000, servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3], 300, 2000, 2000, 2000, 2500, 2500],
                         [20000, 12000, servo_angles[0], servo_angles[1], servo_angles[2],
                             servo_angles[3], 1000, 1000, 1000, 1000, 1000, 1000],
                         [20000, 12000, 12000, 12000, 12000, 12000, 2000, 2000, 2000, 2000, 2000, 2000]]

            msg = Int16MultiArray()
            for i in range(len(data_sets)):
                msg.data = data_sets[i]
                self.publisher.publish(msg)
                time.sleep(3)
            response.success = True
            return response
        except:
            response.success = False
            return response

    def estimated_pose_callback(self, msg:PointWithRadius):
        self.est_pose = msg
        self.get_logger().info("Estimated pose: %s" % self.est_pose.id)

    def estimated_pose_service_callback(self, request, response:PointWithRadius):
        if self.current_target.id == self.est_pose.id:
            response.est_pose = self.est_pose
        
        else:
            response.est_pose = PointWithRadius()
            response.est_pose.point.point.x = 9999.9
        try:
            return response
        except:
            response.est_pose = PointWithRadius()
            response.est_pose.point.point.x = 9999.9
            return response

    def current_target_callback(self, msg:PointWithRadius):
        self.current_target = msg


def main():
    rclpy.init()

    pick_up_service = Pick_up()

    rclpy.spin(pick_up_service, executor=MultiThreadedExecutor())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
