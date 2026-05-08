
from arian_interfaces.srv import MoveArm
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
        super().__init__('move_arm')
        self.publisher = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.srv = self.create_service(
            MoveArm, 'move_arm', self.callback)


    def callback(self, request, response):
        self.move_arm(request)
        time.sleep(2)
        response.success = True
        return response
    def move_arm(self, request):
        closed = request.closed
        reset = request.reset
        if not closed and not reset:
            msg = Int16MultiArray()
            msg.data = [120,12000,2000,16000,8400,12000,500,1000,1000,1000,1000,1000]
            self.publisher.publish(msg)
        elif closed and not reset:
            msg = Int16MultiArray()
            msg.data = [18000,12000,2000,16000,8400,12000,500,1000,1000,1000,1000,1000]
            self.publisher.publish(msg)
        elif reset:
            msg = Int16MultiArray()
            msg.data = [12000,12000,12000,12000,12000,12000,500,1000,1000,1000,1000,1000]
            self.publisher.publish(msg)

def main():
    rclpy.init()

    pick_up_service = Pick_up()

    rclpy.spin(pick_up_service, executor=MultiThreadedExecutor())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
