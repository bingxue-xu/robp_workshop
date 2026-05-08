from ast import Mult

from py import std
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from .a_star import Astar
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException
from rclpy.executors import MultiThreadedExecutor
from bing_interfaces.srv import GlobalPlanning
import copy
import std_msgs.msg as std_msgs

import numpy as np

from PIL import Image as im
from matplotlib import pyplot as plt


class GlobalPlanningService(Node):
    def __init__(self):
        super().__init__('global_planning')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_frame = 'base_link'
        self.global_frame = 'map'
        self.offset = np.int8(-10)
        self.resolution = 0.04
        self.map = OccupancyGrid()
        self.astar_grid = OccupancyGrid()

        self.radius = int(0.22/self.resolution)

        self.inflated_publisher = self.create_publisher(
            OccupancyGrid, '/map_inflated', 10)
        self.path_publisher = self.create_publisher(
            Path, '/global_path', 10)

        self.global_planner_srv = self.create_service(
            GlobalPlanning, '/global_planning', self.global_path_callback)

        self.sleep = self.create_publisher(std_msgs.Bool, '/sleep', 10)


    def get_robot_position(self):
        if self.tf_buffer.can_transform(self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
                self.get_logger().info(
                    f' get robot position : {t.transform.translation.x}, {t.transform.translation.y}')
                return t.transform.translation.x, t.transform.translation.y
            except TransformException as ex:
                self.get_logger().info(
                    f"Couldn't transform robot position due to: {ex}")
        return None, None

    def global_path_callback(self, request, response):
        self.map =request.map
        sleep = std_msgs.Bool()
        sleep.data = True
        #makes Scan matcher and Detection sleep for 3 sec
        #self.sleep.publish(sleep)
        #temp_map = copy.deepcopy(self.map)

        # Convert 1D OccupancyGrid to 2D grid array for A*
        start = self.get_robot_position() 
        #Position in real meters
        occupied_cells = []
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                id = self.grid_index(i, j)
                # Camera obstacles are marked with -50
                if 0 < self.map.data[id] < 100:
                    x, y = self.grid_to_map(i, j)

                    if self.check_distance(x, y, request.goal.point.x, request.goal.point.y, (self.radius+2)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius+2)*self.resolution):
                            occupied_cells.append((i, j,self.radius))
                        #Dont want to remove walls and shit from close to me
                        #elif not self.check_distance(x, y,start[0], start[1], 0.1):
                        #    self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)))
                    #Again dont want to remove walls and stuff
                    #elif not self.check_distance(x, y, request.goal.point.x, request.goal.point.y, 0.1):
                    #    self.map.data[id] = 0
                    else:
                        dist = np.sqrt((request.goal.point.x - x)**2 + (request.goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)))

                elif self.map.data[id] == -50:
                    x, y = self.grid_to_map(i, j)

                    if self.check_distance(x, y, request.goal.point.x, request.goal.point.y, (self.radius+2)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius+2)*self.resolution):
                            occupied_cells.append((i, j,self.radius))
                        elif not self.check_distance(x, y,start[0], start[1], 0.1):
                            self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)))

                    elif not self.check_distance(x, y, request.goal.point.x, request.goal.point.y, 0.1):
                        self.map.data[id] = 0
                    else:
                        dist = np.sqrt((request.goal.point.x - x)**2 + (request.goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)))

                elif self.map.data[id] == -51:
                    x, y = self.grid_to_map(i, j)

                    if not self.check_distance(x, y,start[0], start[1], 0.1):
                        self.map.data[id] = 0
                elif self.map.data[id] == -125: #Dont inflate boundary too much
                    x, y = self.grid_to_map(i, j)

                    if self.check_distance(x, y, request.goal.point.x, request.goal.point.y, (self.radius/3)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius/3)*self.resolution):
                            occupied_cells.append((i, j,int(self.radius//3)))
                        #If I accedentily find myself on the boundary
                        elif not self.check_distance(x, y,start[0], start[1], 0.1):
                            self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)//3))
                    elif not self.check_distance(x, y, request.goal.point.x, request.goal.point.y, 0.1):
                        self.map.data[id] = 0
                    else:
                        dist = np.sqrt((request.goal.point.x - x)**2 + (request.goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)//3))
                        
                elif self.map.data[id] == -10:
                    self.map.data[id] = 0

        # inflating map
        for cell in occupied_cells:
            x1, y1, radius = cell
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if np.sqrt(dx**2 + dy**2) <= radius:
                        x2 = x1 + dx
                        y2 = y1 + dy
                        if self.is_in_bounds(x2, y2):
                            self.map.data[self.grid_index(
                                x2, y2)] = np.int8(80)

        self.inflated_publisher.publish(self.map)
        grid = np.array(self.map.data).reshape(self.map.info.width,self.map.info.height, order = 'F')
        #self.get_logger().info(str(grid.shape))

        #grid = np.full((self.map.info.width, self.map.info.height), 1)
        #for i in range(len(self.map.data)):
        #    # Getting xy coords in occupancy coords
        #    x, y = self.cords_from_index(i)
        #    # Also add unexplored as free
        #    if self.map.data[i] == 0 or self.map.data[i] == -10:
        #        grid[x, y] = 0

        # Start path planning
        if start[0] is None or start[1] is None:
            self.get_logger().error('Could not obtain robot position for path planning')
            return
        else:
            start = self.map_to_grid(start[0], start[1])

        goal = self.map_to_grid(request.goal.point.x, request.goal.point.y)

        # debug
        self.get_logger().info(
            f' get goal position {request.goal.point.x} {request.goal.point.y} in grid {goal}')

        path = Astar(grid, start, goal)
        formed_path = [self.grid_to_map(point[0], point[1]) for point in path]

        response.global_path = self.publish_path(
            formed_path, self.map.header.frame_id)
        self.get_logger().info(f'finished global path planning')

        return response

    def publish_path(self, path, frame_id):

        print('Publishing path')
        global_path = Path()
        global_path.header.frame_id = frame_id
        global_path.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            global_path.poses.append(pose)

        self.path_publisher.publish(global_path)
        return global_path

    ######################### helper functions######################################################
    def check_distance(self, x1, y1, x2, y2, distance):
        """HELPER FUNCTION"""

        """Checks if distance between two points is less than a given distance"""
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) > distance

    def map_to_grid(self, x, y):
        """HELPER FUNCTION"""

        """Takes in x,y in map coordinates (m) and returns coordinates in grid map coordinates (cm) """
        x_coord = int((x - self.offset)/self.resolution)
        y_coord = int((y - self.offset)/self.resolution)
        return x_coord, y_coord

    def grid_to_map(self, x, y):
        """HELPER FUNCTION"""

        """Takes in coordinates in gridmap coordinates (cm) and converts them to coordinates in map frame (m)"""
        x_coord = x*self.resolution + self.offset
        y_coord = y*self.resolution + self.offset
        return x_coord, y_coord

    def grid_index(self, x, y):
        """HELPER FUNCTION"""

        """Calculates index in grid map array (map.data) based on gridmap coordinates"""
        index = int(x + y * 2*abs(self.offset) /
                    self.resolution)
        return index

    def cords_from_index(self, index):
        """HELPER FUNCTION"""

        """Returns coordinates IN GRID MAP coordinates based on te index in gridmap array (map.data)"""
        x = index % int(2*abs(self.offset)/self.resolution)
        y = round(index//int(2*abs(self.offset)/self.resolution))
        return x, y

    def is_in_bounds(self, x, y):
        """HELPER FUNCTION"""
        """Returns weather (x, y) is inside grid_map or not."""

        if abs(x) < self.map.info.width:
            if abs(y) < self.map.info.height:
                return True
        return False
    ######################### helper functions######################################################


def main():
    rclpy.init()
    global_planning = GlobalPlanningService()
    try:
        rclpy.spin(global_planning, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
