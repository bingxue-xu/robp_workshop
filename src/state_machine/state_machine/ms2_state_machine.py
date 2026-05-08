#!/usr/bin/env python3


import re
import sys
from cairo import Status

from matplotlib import spines

from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from arian_interfaces.srv import IsInWs
from bing_interfaces.srv import GlobalPlanning
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gustav_custom_interfaces.action import DriveTo
from ida_interfaces.srv import EstPose


class StateMachine(Node):

    '''
    state machine for mile stone 2. the purposed goal is to receive a target pose, check if it is inside the workspace, plan a path around abstacles and move to it to pick it up.
    
    init state, state = 0
    subscribing detection,if goal exist: state -> 2
    call IsInws service, inside -> 3
    call global_planning, glbal_path -> 4  
    call pure pursuit action server, arrived -> 5
    call pick, success -> 10
    fail -> 20
    '''

    def __init__(self):
        super().__init__('ms2_state_machine')
        self.goal = None
        self.response_global_path = None
        self.arrived = None
        # init state machine
        self.state = 0 
        self.pp_result = False

        self.cli_ws = self.create_client(IsInWs, '/is_in_ws')
        self.cli_global_path = self.create_client(GlobalPlanning, '/global_planning')
        self.cli_pick = self.create_client(Trigger, '/pick_up_service')
        self.cli_detection = self.create_client(EstPose, '/estimated_pose') 
        self.pp_action_client = ActionClient(
            self, 
            DriveTo,
            'drive_to', callback_group=ReentrantCallbackGroup())
        
        while not self.pp_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for PP Action Server...')
        while not self.cli_ws.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for IsInWs service...')
        while not self.cli_global_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for global planning service...')
        while not self.cli_pick.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for pick up service...')
        while not self.cli_detection.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for detection service...')


        #AdD FOR IDAS
            

        # while not self.cli_pursuit.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn('Waiting for pick up service...')

        time.sleep(3)
        self.get_logger().info('All services available')
        self.check_states()



    ##########action shit############
    def send_goal(self, goal):
        goal_msg = DriveTo.Goal()
        goal_msg.path = goal

        self.pp_action_client.wait_for_server()
        self._send_goal_future = self.pp_action_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return None
        else:
            self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback_msg = feedback_msg.pose
        self.get_logger().info('Feedback:'+ str(self.feedback_msg))
        return feedback_msg
    
    def get_result_callback(self, future):
        result = future.result().result.success
        self.get_logger().info('Result:'+ str(result))
        self.state = 5
        time.sleep(1)
        self.check_states()
        return result
  
    ############### action client end ##############################      

  

    # IsInWs client    
    def send_IsInWs_request(self):
        self.req = IsInWs.Request()
        self.req.point = self.goal
        self.future = self.cli_ws.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().inside
        
    
    # global planning client

    def send_global_planning_request(self):
        self.req = GlobalPlanning.Request()
        self.req.goal = self.goal
        #self.get_logger().info(f'send out request {self.req.goal}')
        self.future = self.cli_global_path.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        #self.get_logger().info(f'srvice result: {self.future.result().global_path}')
        return self.future.result().global_path

    
    # pick up client
    def send_pick_request(self):
        self.req = Trigger.Request() 
        future = self.cli_pick.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
        # detection client
    def send_detection_request(self):       
        self.req = EstPose.Request() 
        future = self.cli_detection.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        self.goal = future.result().est_pose
        return future.result().est_pose


    def check_states(self):
        
        while rclpy.ok():
                

            # detection
            if self.state == 0:
                while True:
                    self.get_logger().info('detecting...')
                    estimated_pose= self.send_detection_request()

                    if estimated_pose.point.x != 9999:
                        self.get_logger().info('Got a Goal pose!')
                        self.state = 2
                        break
                    else:
                        self.get_logger().info('Nothing')
                        self.state = 0
                        time.sleep(1)


            # IsInWs 
            if self.state == 2:
                self.get_logger().info('checking if it is in ws...')
                response_isinws= self.send_IsInWs_request()
    
                if response_isinws:
                    self.get_logger().info('Inside!')
                    self.state = 3
                else:
                    self.get_logger().info('Ouside ws, never mind')
                    self.state = 20
    
            # global path planning
            if self.state == 3:
                self.get_logger().info('planning global path...')
                global_path = self.send_global_planning_request()
    
                if global_path.poses[0] != None:
                    self.get_logger().info('Global path is aviable!')
                    self.state = 4
                else:
                    self.get_logger().info('No path!')
                    self.state = 20

            # pure pursuit
            if self.state == 4:
                self.get_logger().info('Going to the goal...')
                response_pursuit = self.send_goal(global_path)
                self.get_logger().info(f'response_pursuit: {response_pursuit}')
                #rclpy.spin_until_future_complete(self.pp_action_client, response_pursuit)
            

                #if response_pursuit is None:    
                #    self.get_logger().info('Failed to send goal')
        #
                #elif response_pursuit:
                #    self.get_logger().info('Arrived!')
                #    self.state = 5
                #else:
                #    self.get_logger().info('No way!')
                #    self.state = 20

            # pick
            if self.state == 5:
                self.get_logger().info('Picking...')
                response_pick = self.send_pick_request()
    
                if response_pick.success:
                    self.get_logger().info('Grabed!')
                    self.state = 10
                else:
                    self.get_logger().info('Slipped!')
                    self.state = 20
    
    
            if self.state == 10:
                self.get_logger().info('MS2 state machine finished :)))')

            if self.state == 20:
                self.get_logger().info('Lets try again!!!')
                self.state = 0
                self.check_states()

            break

def main():
    rclpy.init()
    state_machine = StateMachine()
    try:
        rclpy.spin(state_machine, executor=MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()