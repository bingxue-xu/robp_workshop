def main():
    print('Hi from ms1_mission_planner.')


if __name__ == '__main__':
    main()
#!/usr/bin/env python3


import sys

from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node
import time


class StateMachine(Node):

    def __init__(self):
        super().__init__('ms1_state_machine')
        self.cli_pick = self.create_client(Trigger, 'pick_up_service')
        self.cli_place = self.create_client(Trigger, 'place_service')
        self.cli_goto = self.create_client(Trigger, 'goto_service')

        while not self.cli_pick.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for pick up service...')
        while not self.cli_place.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for place service...')
        while not self.cli_goto.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for go to service...')

        self.state = 0 
        time.sleep(3)
        self.check_states()


    def send_pick_request(self):
        request = Trigger.Request() 
        future = self.cli_pick.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_place_request(self):
        request = Trigger.Request()  
        future = self.cli_place.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_goto_request(self):
        request = Trigger.Request()  
        future = self.cli_goto.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()



    def check_states(self):
        
        while rclpy.ok():
    
            if self.state == 0:
                self.get_logger().info('picking object...')
                response_pick = self.send_pick_request()
    
                if response_pick.success:
                    self.get_logger().info('Grabed!')
                    self.state = 1
                else:
                    self.get_logger().info('Failed to pick, trying again!')
                    self.state = 5
    

            if self.state == 1:
                self.get_logger().info('going to object...')
                response_goto = self.send_goto_request()
    
                if response_goto.success:
                    self.get_logger().info('Arrived!')
                    self.state = 2
                else:
                    self.get_logger().info('Lets try again!')
                    self.state = 5


            if self.state == 2:
                self.get_logger().info('placing object...')
                response_place = self.send_place_request()
    
                if response_place.success:
                    self.get_logger().info('Placed!')
                    self.state = 3
                else:
                    self.get_logger().info('Lets try again!')
                    self.state = 5  

    
            if self.state == 5:
                self.get_logger().info('state machine failed, check code')
    
            if self.state ==3:
                self.get_logger().info('state machine finished!')
                break


def main(args = None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    try:
        rclpy.spin(state_machine)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()