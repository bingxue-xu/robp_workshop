import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robp_interfaces.action import LineFollower
from robp_interfaces.msg import PointPixel


class LineFollowerActionClient(Node):

    def __init__(self):
        super().__init__('line_follower_action_client')
        self._action_client = ActionClient(self, LineFollower, 'line_follower')


    def send_goal(self, flag):
        goal_msg = LineFollower.Goal()
        goal_msg.flag = flag
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback_msg = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'.format(feedback_msg.status))
    

def main(args = None):
    rclpy.init(args=args)
    action_client = LineFollowerActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)



if __name__ == '__main__':
    main()
