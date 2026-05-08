import rclpy
import math
import time

import numpy as np
from rclpy.node import Node
# from slam.slam.drive import Drive
# from slam.slam.drive import Drive

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, Point, Twist, Pose
from nav_msgs.msg import Odometry, Path
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse

# from std_msgs.msg import Int16MultiArray, Float32MultiArray

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gustav_custom_interfaces.action import DriveTo

"""
math function

delta = atan(2Lsin(alpha)/d)

d: distance between middle of robot to TP (look ahead distance)
L: wheelbase
R: distance between ICR and TP (turning radius?)

Ideas for imporvement:
- Not use np for the array but the path direclty which should make the program perform 
faster



"""

SIMULATION = False  # true if run with turtlebot

# Constants
LOOKAHEAD = 0.25
WB = 0.3
VELOCITY = 0.12
GOAL_TOLERANCE = 0.10


class PurePursuitActionServer(Node):

    def __init__(self):
        super().__init__('pure_pursuit_action_server')

        self.rx = 0
        self.ry = 0
        self.yaw = 0
        self.waypoints = np.zeros((1, 2))
        self.lookahead_idx = 0
        self.tolerance = None
        self.goal = DriveTo.Goal()
        self.rate = self.create_rate(50)
        callbackgroup = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.waypoints_pub = self.create_publisher(Point, '/waypoints', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # if SIMULATION:
        # self.subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10, callback_group=callbackgroup)
#

        # init server
        self._action_server = ActionServer(
            self,
            DriveTo,
            'drive_to',
            execute_callback=self.pure_pursuit,
            callback_group=callbackgroup,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Pure Pursuit Action Server has been initiated in DD2419')
        self.get_logger().info(f'Simulation: {SIMULATION}')

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        self.tolerance = goal_request.tolerance
        self.path_callback(self.goal.path)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def feedback(self, goal_handle, current_pose):
        feedback_msg = DriveTo.Feedback()
        # instead of position we just send the euclidean distacne between curr pose and last point on path
        feedback_msg.pose.position.x = np.sqrt((current_pose[0] - self.goal.path.poses[-1].pose.position.x)**2 + (
            current_pose[1] - self.goal.path.poses[-1].pose.position.y)**2)
        # feedback_msg.pose.position.y = current_pose[1]

        quart = quaternion_from_euler(0, 0, self.yaw)

        feedback_msg.pose.orientation.x = quart[0]
        feedback_msg.pose.orientation.z = quart[1]
        feedback_msg.pose.orientation.y = quart[2]
        feedback_msg.pose.orientation.w = quart[3]

        goal_handle.publish_feedback(feedback_msg)

    def get_robot_position(self):
        target_frame = 'map'
        if SIMULATION:
            target_frame = 'odom'
        if self.tf_buffer.can_transform(target_frame, 'camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame,
                    'camera_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1))
                # self.get_logger().info(f' get robot position : {t.transform.translation.x}, {t.transform.translation.y}')
                euler = euler_from_quaternion(
                    [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                self.yaw = euler[2]
                return t.transform.translation.x, t.transform.translation.y
            except TransformException as ex:
                self.get_logger().info(
                    f"Couldn't transform robot position due to: {ex}")
        else:
            self.get_logger().info('No transform found')

        return None, None

    def path_callback(self, path):
        self.get_logger().info("Got a path")
        for pose in path.poses:
            tmp = np.array([pose.pose.position.x, pose.pose.position.y])
            if self.waypoints[0, 0] == 0 and self.waypoints[0, 1] == 0:
                self.waypoints[0] = tmp
            self.waypoints = np.vstack((self.waypoints, tmp))

        #for point in self.waypoints:
        #    self.get_logger().info(f'Waypoint: {point}')

    def find_distance(self, p0, p1):
        p0 = p0.astype(float)
        p1 = p1.astype(float)
        return np.sqrt(np.sum((p1 - p0)**2))

    def find_distance_index(self, idx, current_xy):
        distance = self.find_distance(current_xy, self.waypoints[idx])
        return distance

    # checks which of the waypoints is the closest to the current position
    def find_nearest_waypoint(self, current_xy):
        distance = self.find_distance(
            current_xy, self.waypoints[self.lookahead_idx:])
        nearest_idx = 0
        for i in range(len(self.waypoints[self.lookahead_idx:])):
            if self.find_distance(current_xy, self.waypoints[self.lookahead_idx+i]) < distance:
                distance = self.find_distance(
                    current_xy, self.waypoints[self.lookahead_idx+i])
                nearest_idx = i+self.lookahead_idx
        return nearest_idx

    # checks if the waypoint is within lookahead distance
    def idx_close_to_lookahead(self, idx, current_xy):
        while self.find_distance_index(idx, current_xy) < LOOKAHEAD and idx <len(self.waypoints) - 1:
            idx += 1
        if idx == 0:
            return idx
        return idx - 1

    def pure_pursuit(self, goal_handle):
        self.get_logger().info('Pure pursuit started')

        self.rx, self.ry = self.get_robot_position()

        curr_pose = np.array([self.rx, self.ry])
        start = True
        start_index = None
        msg = Twist()
        locked = False
        ################################## Start Sequence #################################
        # goal_pose = self.waypoints[0]
        # alpha = math.atan2(goal_pose[1] - self.ry, goal_pose[0] - self.rx) - self.yaw
        # alpha = np.mod(alpha + np.pi,2*np.pi)-np.pi
        # while abs(alpha) > np.pi/2:
        #    msg = Twist()
        #    msg.angular.z = 1.1 if alpha > 0 else -1.1
        #    msg.linear.x = 0.0
        #    self.publisher.publish(msg)
        #    self.rx, self.ry = self.get_robot_position()
        #    alpha = math.atan2(goal_pose[1] - self.ry, goal_pose[0] - self.rx) - self.yaw
        #    alpha = np.mod(alpha + np.pi,2*np.pi)-np.pi
        # msg = Twist()
        # msg.angular.z = 0.0
        # msg.linear.x = 0.0
        # self.publisher.publish(msg)
        ########################################################################################
        # self.rx, self.ry = self.get_robot_position()
        # curr_pose = np.array([self.rx, self.ry])

        try:
            while round(self.find_distance(curr_pose, self.waypoints[-1]),2) > self.tolerance:
                self.rx, self.ry = self.get_robot_position()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result = DriveTo.Result()
                    result.success = False
                    msg = Twist()
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
                    self.publisher.publish(msg)
                    return result

                # self.get_logger().info(f'current pose: {self.get_robot_position()}')

                curr_pose = np.array([self.rx, self.ry])

                self.feedback(goal_handle, curr_pose)

                nearest_idx = self.find_nearest_waypoint(curr_pose)
                if not locked:
                    self.lookahead_idx = self.idx_close_to_lookahead(
                        nearest_idx, curr_pose)
     

                goal_pose = self.waypoints[self.lookahead_idx]

                # Pure pursuit controller

                alpha = math.atan2(
                    goal_pose[1] - self.ry, goal_pose[0] - self.rx) - self.yaw
                # alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Ensure alpha is in [-pi, pi]
                alpha = np.mod(alpha + np.pi, 2*np.pi) - \
                    np.pi  # Ensure alpha is in [-pi, pi]
                tmp_lookahead = self.find_distance(goal_pose, curr_pose)
                steering_angle = math.atan2(
                    2 * WB * math.sin(alpha), tmp_lookahead)

                MAX_STEERING_ANGLE = np.pi
                VEL_MAX = 0.7

                if steering_angle > MAX_STEERING_ANGLE: 
                    steering_angle = MAX_STEERING_ANGLE - 0.05
                elif steering_angle < -MAX_STEERING_ANGLE:
                    steering_angle = -MAX_STEERING_ANGLE + 0.05

                # self.get_logger().info(f'Steering angle: {steering_angle}')

                norm_angle = np.abs(steering_angle / MAX_STEERING_ANGLE)

                vel = min(VEL_MAX, VELOCITY * 1/(norm_angle))
                # Remove self.lookahead_idx < len(self.waypoints)/2 and test
                if abs(alpha) > np.pi/4 and abs(tmp_lookahead)<= LOOKAHEAD+0.05 and start:
                    vel = 0.0
                    steering_angle = 1.2 if alpha > 0 else -1.2
                    locked = True
                else:
                    locked = False
                    start = False

                msg.linear.x = vel
                msg.angular.z = steering_angle

                self.publisher.publish(msg)

                #time.sleep(0.005)
                self.rate.sleep()
                #rclpy.spin_once(self)

        except IndexError:
            self.get_logger().info('Path completed')

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

        goal_handle.succeed()
        result = DriveTo.Result()
        result.success = True

        self.get_logger().info('Pure pursuit completed')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitActionServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
