
import re
import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
    

from gustav_custom_interfaces.srv import Explore
from gustav_custom_interfaces.srv import VisualGrid

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup




class ExplorerClient(Node):
    def __init__(self):
        super().__init__('test_client')

        self.explore_client = self.create_client(Explore, '/explore')

        while not self.explore_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('explore service not available, waiting again...')

        self.path_pub = self.create_publisher(Path, '/path', 10)
        # while not self.explore_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req_exp = Explore.Request()


    def send_explore_request(self):
        self.req_exp.obs_map = OccupancyGrid()
        self.future = self.explore_client.call_async(self.req_exp)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'explore result: {self.future.result()}')
        self.path_pub.publish(self.future.result().path)
        return self.future.result()


def main():
    rclpy.init()
    test_client = ExplorerClient()
    test_client.send_explore_request()
    # executor = MultiThreadedExecutor()
    # executor.add_node(test_client)
    test_client.destroy_node()
    rclpy.shutdown()


    # result = test_client.send_viz_request()
    # test_client.get_logger().info(f'result: {result}')

    # Shutdown and cleanup

if __name__ == '__main__':
    main()