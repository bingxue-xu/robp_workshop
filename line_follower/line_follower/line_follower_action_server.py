import rclpy 
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
from robp_interfaces.action import LineFollower
from robp_interfaces.msg import PointPixel
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class LineFollowerActionServer(Node):

    """
    This action server continuously checks whether the robot's two wheels remain across the line and within the workspace in real time. 
    As long as at least one point on the line is found between the left and right wheels, the robot is considered to be “on track” 
    If the robot leaves the workspace or the line is no longer between the wheels, the action is aborted.

    To use this server, run this node and interact with it using an action client:
    ros2 action send_goal --feedback line_follower robp_interfaces/action/LineFollower "{flag: true}"

    """


    def __init__(self):
        super().__init__('line_follower_action_server')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            LineFollower,
            'line_follower',
            self.execute_callback
        )
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/line', self._line_callback, qos_profile=qos_profile)
        
        self.declare_parameter('wheel_base', 0.23)
        self.map_width = 3000
        self.map_height = 2000
        self.inflation = 50

        self.pts = None
        self.line_points = None


    def _line_callback(self, msg: PointCloud2):
        points = []
        for pt in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = pt  
            points.append([x, y])
        self.line_points = np.array(points) if points else None


    def execute_callback(self, goal_handle):
        start_time = self.get_clock().now()
        feedback_msg = LineFollower.Feedback()
        result = LineFollower.Result()

        self.get_logger().info('Clara, färdiga, gå...')

        # Wait for line points to be available (max 5 seconds)
        wait_start = self.get_clock().now()
        while self.line_points is None or len(self.line_points) == 0:
            cur_wait = self.get_clock().now()
            if (cur_wait - wait_start).nanoseconds * 1e-9 > 5.0:
                self.get_logger().error('Timeout waiting for line points')
                goal_handle.abort()
                result.result = False
                return result
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            while True:
                cur_time = self.get_clock().now()
                elapsed = (cur_time - start_time).nanoseconds * 1e-9  

                if elapsed > 100.0:
                    break

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.result = False
                    self.get_logger().info('Goal cancelled')
                    return result

                umpire_result = self.umpire()
                if umpire_result is None:
                    feedback_msg.status = 'Waiting for transform...'
                    goal_handle.publish_feedback(feedback_msg)
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                if not umpire_result:
                    feedback_msg.status = 'Ops, off of the line :('
                    goal_handle.publish_feedback(feedback_msg)
                    goal_handle.succeed()
                    result.result = False
                    return result

                feedback_msg.status = f'Running, elapsed={elapsed:.1f}s'
                goal_handle.publish_feedback(feedback_msg)

                rclpy.spin_once(self, timeout_sec=0.1)

            goal_handle.succeed()
            result.result = True
            self.get_logger().info('Time is up')
            return result

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            goal_handle.abort()
            return result
    

    def umpire(self) -> bool:
        """
        check wether the robot is within workspace and not devating the line 
        """
        
        cur_time = self.get_clock().now()

        tf_future = self.tf_buffer.wait_for_transform_async(
            'map', 'base_link', cur_time)
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1.0)

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', cur_time.to_msg())
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {e}')
            return None
        
        robot_pos_x = tf.transform.translation.x
        robot_pos_y = tf.transform.translation.y

        self.get_logger().debug(f'Robot position in map: {robot_pos_x}, {robot_pos_y}')

        min_x, max_x = -self.inflation, self.map_width + self.inflation
        min_y, max_y = -self.inflation, self.map_height + self.inflation

        if not (min_x <= robot_pos_x <= max_x and min_y <= robot_pos_y <= max_y):
            self.get_logger().error('out of bounds :(')
            return False

        if self.line_points is None or len(self.line_points) == 0:
            self.get_logger().error('No line points detected!')
            return False
        
        ## check if both wheels are on the line
        wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        left_y = robot_pos_y + wheel_base
        right_y = robot_pos_y - wheel_base

        line_y = self.line_points[:, 1]
        y_min = min(left_y, right_y)
        y_max = max(left_y, right_y)
        within_wheels = np.logical_and(line_y >= y_min, line_y <= y_max)
        if not np.any(within_wheels):
            self.get_logger().error('Both wheels are off the line!')
            return False

        ## Check if the robot is close to the line
        # dists = np.linalg.norm(self.line_points - np.array([robot_pos_x, robot_pos_y]), axis=1)
        # min_dist = dists.min()
        # if min_dist > 0.23:  # 10 cm threshold, adjust as needed
        #     self.get_logger().error(f'Robot is off the line! Min dist: {min_dist:.1f} m')
        #     return False
        return True
        

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()