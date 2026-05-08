
import rclpy
import numpy as np
import math
from rclpy.node import Node

from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup



"""
math function

delta = atan(2Lsin(alpha)/d)

d: distance between middle of robot to TP (look ahead distance)
L: wheelbase
R: distance between ICR and TP (turning radius?)


make this into the an action server prob
"""

# Constants 
LOOKAHEAD = 1
WB = 0.3
VELOCITY = 0.5
GOAL_TOLERANCE = 0.3

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        self.rx = 0
        self.ry = 0
        self.yaw = 0
        self.vel = 0
        self.v_prev_error = 0.0
        self.waypoints = np.zeros((1,2))
        callbackgroup = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscriber = self.create_subscription(Path, '/global_path', self.path_callback, 10,callback_group=callbackgroup)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_goal = self.create_publisher(Bool, '/pure_pursuit_result', 10)
        self.subscriber = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10, callback_group=callbackgroup)



    def pose_callback(self, msg:PoseStamped):
        self.rx = msg.pose.position.x
        self.ry = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom',
                                                        'base_link',
                                                        rclpy.time.Time().to_msg(),
                                                        timeout=rclpy.time.Duration(seconds=0.5))
            self.rx = transform.transform.translation.x
            self.ry = transform.transform.translation.y

            print(self.rx)
            print(self.ry)

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            quaternion = (qx, qy, qz, qw)
            euler = euler_from_quaternion(quaternion)
            self.yaw = euler[2]

        except:
            print("Something fucked up")
            pass
    
    def path_callback(self, path):
        for pose in path.poses:
            self.get_logger().info(f'waypoints: {self.waypoints}')
            tmp = np.array([pose.pose.position.x, pose.pose.position.y])
            if self.waypoints[0,0] == 0 and self.waypoints[0,1] == 0:
                self.waypoints[0] = tmp
            else:
                self.waypoints = np.vstack((self.waypoints, tmp))
        self.pure_pursuit()

    def find_distance(self, p0, p1):
        p0 = p0.astype(float)
        p1 = p1.astype(float)
        return np.sqrt(np.sum((p1 - p0)**2))

    def find_distance_index(self, idx, current_xy):
        distance = self.find_distance(current_xy, self.waypoints[idx])
        return distance

    #checks which of the waypoints is the closest to the current position
    def find_nearest_waypoint(self, current_xy):
        distance = self.find_distance(current_xy, self.waypoints[0])
        nearest_idx = 0
        for i in range(len(self.waypoints)):
            if self.find_distance(current_xy, self.waypoints[i]) < distance:
                distance = self.find_distance(current_xy, self.waypoints[i])
                nearest_idx = i
        return nearest_idx

    #checks if the waypoint is within lookahead distance
    def idx_close_to_lookahead(self,idx, current_xy):
        while self.find_distance_index(idx, current_xy) < LOOKAHEAD and idx < len(self.waypoints) - 1:
            idx += 1
        if idx == 0:
            return idx
        return idx - 1
    
    def interpolate(self, p0, p1, alpha):
        tmp = np.linspace(p0, p1, alpha, endpoint=False)
        return np.interp(tmp, p0, p1)




    #if this works poorly implement the same interpolation as in the irob assignment

    def pure_pursuit(self):

        #self.robot_pose()
        curr_pose = np.array([self.rx, self.ry])
        msg = Twist()

        try:
            while self.find_distance(curr_pose, self.waypoints[-1]) > GOAL_TOLERANCE:
                #self.robot_pose()
                curr_pose = np.array([self.rx, self.ry])

                nearest_idx = self.find_nearest_waypoint(curr_pose)
                lookahead_idx = self.idx_close_to_lookahead(nearest_idx, curr_pose)

                goal_pose = self.waypoints[lookahead_idx]

                print('curr pose: ' + str(curr_pose))
                print('goal pose: ' + str(goal_pose))

                alpha = math.atan2(goal_pose[1] - self.ry, goal_pose[0] - self.rx) - self.yaw
                alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Ensure alpha is in [-pi, pi]
                
                tmp_lookahead = self.find_distance(goal_pose, curr_pose)
                steering_angle = math.atan2(2 * WB * math.sin(alpha), tmp_lookahead)

                MAX_STEERING_ANGLE = 0.6
                VEL_MAX = 0.5

                if steering_angle > MAX_STEERING_ANGLE:
                    steering_angle = MAX_STEERING_ANGLE - 0.05
                elif steering_angle < -MAX_STEERING_ANGLE:
                    steering_angle = -MAX_STEERING_ANGLE + 0.05

                norm_angle = np.abs(steering_angle / MAX_STEERING_ANGLE)
                
                #maybe add a PID controller here
                vel = min(VEL_MAX, VELOCITY * 1/(norm_angle))

                # Publish messages
                msg.linear.x = vel
                msg.angular.z = steering_angle
                
                self.pub_vel.publish(msg)

        except IndexError:
            self.get_logger().info('Path completed')
        
        if self.find_distance(curr_pose, self.waypoints[-1]) > GOAL_TOLERANCE:
            self.pub_goal.publish(True)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
                
        self.pub_vel.publish(msg)
        

def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()