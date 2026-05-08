
import re
import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
    

from gustav_custom_interfaces.srv import VisualGrid

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup




class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')

        callbackgroup = ReentrantCallbackGroup()

        # self.explore_client = self.create_client(Explorer, 'explore')
        self.obs_map = OccupancyGrid()

        self.viz_client = self.create_client(VisualGrid, '/visual_grid')

        while not self.viz_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('viz service not available, waiting again...')


        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10, callback_group=callbackgroup)
        self.map_pub = self.create_publisher(OccupancyGrid, '/viz_map', 10)
        # while not self.explore_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        self.req_viz = VisualGrid.Request()

    def map_callback(self, msg):
        self.get_logger().info('map received')
        self.obs_map = msg

    
    def get_map(self):
        return self.obs_map
    
    def send_viz_request(self):
        self.get_logger().info(f'obs_map height: {self.obs_map.info.height}, width: {self.obs_map.info.width}')
        self.req_viz.obs_map = self.obs_map
        self.future = self.viz_client.call_async(self.req_viz)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    test_client = TestClient()
    result = test_client.send_viz_request()
    test_client.get_logger().info(f'result: {result}')
    test_client.destroy_node()
    rclpy.shutdown()


    # result = test_client.send_viz_request()
    # test_client.get_logger().info(f'result: {result}')

    # Shutdown and cleanup

if __name__ == '__main__':
    main()