import sys
from bing_interfaces.srv import GlobalPlanning
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(GlobalPlanning, 'global_planning')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GlobalPlanning.Request()


    def send_request(self):
        goal_position = PointStamped()
        goal_position.header.frame_id = 'map'
        goal_position.header.stamp = self.get_clock().now().to_msg()
        goal_position.point.x = 1.3
        goal_position.point.y = 0.2
        goal_position.point.z = 0.0
        self.req.goal = goal_position
        self.get_logger().info(f'send out request {goal_position}')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'srvice result: {self.future.result().global_path}')
        return self.future.result().global_path




def main():
    rclpy.init()

    global_planner_client = TestClient()
    response = global_planner_client.send_request()
    global_planner_client.get_logger().info(
        f'global path from current robot position to is {response}'
    )

if __name__ == '__main__':
    main()