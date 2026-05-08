

from sympy import O
from behaviour_tree.behaviour_tree.bt2_behaviors import GlobalPlanner
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from explorer_aglorithm import *

from gustav_custom_interfaces.srv import Explorer
from explorer_algorithm import *

class ExplorerService(Node):
    def __init__(self):
        super().__init__('explorer')
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Explorer node has been started')

        self.viz_map = OccupancyGrid()
        self.global_map = OccupancyGrid()

        self.robot_frame = 'base_link'
        self.global_frame = 'map'

        self.rx = 0
        self.ry = 0
        self.yaw = 0


        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.srv_explorer = self.create_service(Explorer, '/explorer', self.explorer_callback)

        self.cli_global_planner = self.create_client(GlobalPlanner, '/local_planner')

        # Talk to local planner to get path
        while not self.cli_global_planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = GlobalPlanner.Request()
        
    def robot_pose(self):
        if self.tf_buffer.can_transform(self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            try: 
                t = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1)
                )
                self.yaw = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
                self.rx = t.transform.translation.x
                self.ry = t.transform.translation.y
            except: 
                self.get_logger().info('Error in getting robot pose')
            
    def get_map(self, msg: OccupancyGrid):
        pass
            
    def send_req_global(self, posestamped):
        self.req.goal = posestamped
        self.future = self.cli_global_planner.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def convert_path(self, path):
        data = path.data
        path_converted = []
        for point in data:
            x = point.position.x
            y = point.position.y
            yaw = self.euler_from_quaternion([point.rotation.x, point.rotation.y, point.rotation.z, point.rotation.w])[2]
            tmp = (x, y, yaw)
            path_converted.append(tmp)
        return path_converted

    def map_callback(self, msg: OccupancyGrid):
        self.global_map = msg
    
    def timer_callback(self):
        self.get_logger().info('Explorer node is still running')
    
    def explorer_callback(self, request, response):
        self.get_logger().info('Explorer service has been called')
        self.robot_pose()

        self.global_map = request.map


        rand_point = random_point(self.global_map)
        self.req.goal = 
        global_path = self.send_req_global(rand_point)
        path = self.convert_path(global_path)

        # Start exploration









        response.success = True

        return response