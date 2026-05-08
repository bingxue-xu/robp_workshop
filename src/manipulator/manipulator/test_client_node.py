import sys

from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli_pick = self.create_client(Trigger, 'pick_up_service')
        self.cli_place = self.create_client(Trigger, 'place_service')
        while not self.cli_pick.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('pick_up service not available, waiting again...')
        while not self.cli_place.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('place service not available, waiting again...')

    def send_pick_request(self):
        request = Trigger.Request() 
        self.future = self.cli_pick.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
    def send_place_request(self):
        request = Trigger.Request()  
        self.future = self.cli_place.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response_pick = minimal_client.send_pick_request()
    respons_place = minimal_client.send_place_request()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()