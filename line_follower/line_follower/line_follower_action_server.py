import rclpy 
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

from robp_interfaces.action import LineFollower
from robp_interfaces.msg import PointPixel
import cv2
import numpy as np

class LineFollowerActionServer(Node):

    def __init__(self):
        super().__init__('line_follower_action_server')

        self.bridge = CvBridge()
        self._action_server = ActionServer(
            self,
            LineFollower,
            'line_follower',
            self.execute_callback
        )

        self._robot_pos_pixel = self.create_subscription(
            PointPixel,
            'robot_pos_pixel',
            self.robot_pos_callback,
            10
        )

        self.create_subscription(Image, '/seen_image', self._image_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._image_callback, 10)

        self.robot_pos_pixel = None
        self.pts = None

    def robot_pos_callback(self, msg: PointPixel):
        self.robot_pos_pixel = (msg.width, msg.height)
        self.get_logger().info(f'Robot position: {msg.width}, {msg.height}')


    def _image_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2((msg, 'rgb8'))
        hsv_image = cv2.cvtVolor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_image, (0, 100, 100), (30, 2555, 255))  

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        self.pts = cv2.findNonZero(mask)    


    def execute_callback(self, goal_handle):
        start_time = self.get_clock().now().nanoseconds * 1e-9
        move_flag = LineFollower.Goal()
        feedback_msg = LineFollower.Feedback()
        result = LineFollower.Result()

        try:
            while move_flag.flag:
                self.get_logger().info('Clara, färdiga, gå...')
                cur_time = self.get_clock().now().nanoseconds * 1e-9
                elapsed = cur_time - start_time

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancelled')
                    goal_handle.canceled()
                    result.result = False
                    return result

                if not self.umpire():
                    feedback_msg.status = 'Out of line :('
                    goal_handle.publish_feedback(feedback_msg)
                    result.result = False
                    goal_handle.succeed()
                    return result

                feedback_msg.status =  f'Running, elapsed={elapsed:.1f}s'
                goal_handle.publish_feedback(feedback_msg)

                if elapsed > 10.0:
                    self.get_logger().info('Time is up')
                    result.result = True
                    goal_handle.succeed()
                    return result
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            goal_handle.abort()
            return result
    
    def umpire(self)-> bool:

        if self.robot_pos_pixel is None:
            self.get_logger().error('Robot position not available')
            return False
        
        self.robot_pos_pixel = 1442, 3180
        robot_column, robot_row = self.robot_pos_pixel

        self.get_logger.info('Robot position in image: {robot_column}, {robot_row}')

        inflation = 500
        workspace_width = 2000
        workspace_height = 3000
        min_x, max_x = -inflation, workspace_width + inflation
        min_y, max_y = -inflation, workspace_height + inflation

        if not (min_x <= robot_column <= max_x and min_y <= max_y):
            self.get_logger().error('out of bounds :')
            return False
        else:
            if self.pts is not None:
                distances = np.linalg.norm(self.pts - (robot_column, robot_row), axis=1)
                max_dev = distances.max()
                if  max_dev > 350: 
                    self.get_logger().error('out of line :(, max deviation = {max_dev:.1f} pixel')  
                    return False

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