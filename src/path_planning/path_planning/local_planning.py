from ast import Mult
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
from .a_star import Astar
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException
from rclpy.executors import MultiThreadedExecutor
from bing_interfaces.srv import GlobalPlanning
from arian_interfaces.msg import PointWithRadius
import copy

import std_msgs.msg as std_msgs
import numpy as np

from PIL import Image as im
from matplotlib import pyplot as plt


class LocalPlanningService(Node):
    def __init__(self):
        super().__init__('local_planning')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.robot_frame = 'base_link'
        # self.global_frame = 'map'
        self.offset = np.int8(-2)
        self.resolution = 0.04
        self.map = OccupancyGrid()
        self.astar_grid = OccupancyGrid()
        self.current_target = None
        self.subscribe = self.create_subscription(PointWithRadius, '/current_target', self.target_callback, 10, callback_group=ReentrantCallbackGroup())
        self.radius = int(0.34/self.resolution)  # Bigger radius


        self.inflated_publisher = self.create_publisher(
            OccupancyGrid, '/local_map_inflated', 10)
        self.path_publisher = self.create_publisher(
            Path, '/local_path', 10)

        self.global_planner_srv = self.create_service(
            GlobalPlanning, '/local_planning', self.local_path_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.sleep = self.create_publisher(std_msgs.Bool, '/sleep', 10)

    def target_callback(self, msg: PointWithRadius):
        self.current_target = msg


    def get_trans(self, map_msg, goal_msg):
        """Get transform from goal to lidar and lidar to map frame. Should work for any goal point, even if its given in camera frame for the last leg"""
        trans_goal_to_lidar = None
        trans_lidar_to_map = None
        if self.tf_buffer.can_transform_full(map_msg.header.frame_id, map_msg.header.stamp, goal_msg.header.frame_id, goal_msg.header.stamp, 'map', timeout=rclpy.duration.Duration(seconds=1)):
            try:
                trans_goal_to_lidar = self.tf_buffer.lookup_transform_full(
                    map_msg.header.frame_id, map_msg.header.stamp, goal_msg.header.frame_id, goal_msg.header.stamp, 'map', timeout=rclpy.duration.Duration(seconds=1))
            except (ConnectivityException) as c:
                self.get_logger().info(c)
            except (LookupException) as L:
                self.get_logger().info(L)
            except (ExtrapolationException) as E:
                self.get_logger().info(E)
            except (TransformException) as T:
                self.get_logger().info(T)
        if self.tf_buffer.can_transform('map', map_msg.header.frame_id, map_msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
            try:
                trans_lidar_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    map_msg.header.frame_id,
                    map_msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1))
            except (ConnectivityException) as c:
                self.get_logger().info(c)
            except (LookupException) as L:
                self.get_logger().info(L)
            except (ExtrapolationException) as E:
                self.get_logger().info(E)
            except (TransformException) as T:
                self.get_logger().info(T)
        return trans_goal_to_lidar, trans_lidar_to_map

    def get_robot_position(self, msg):
        target_frame = msg.header.frame_id
        source_frame = 'base_link'
        stamp = msg.header.stamp
        if self.tf_buffer.can_transform(target_frame, source_frame, stamp, timeout=rclpy.duration.Duration(seconds=1.0)):
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, stamp, timeout=rclpy.duration.Duration(seconds=1))
                self.get_logger().info(
                    f' get robot position : {t.transform.translation.x}, {t.transform.translation.y}')
                return t.transform.translation.x, t.transform.translation.y
            except TransformException as ex:
                self.get_logger().info(
                    f"Couldn't transform robot position due to: {ex}")
        return None, None

    def local_path_callback(self, request, response):
        self.map = request.map
        #self.map = copy.deepcopy(self.map)
        sleep = std_msgs.Bool()
        sleep.data = False
        #makes Scan matcher and Detection sleep for 3 sec
        self.sleep.publish(sleep)
        # Start path planning
        # now instead of getting robot position, since we have a local map the start point is just the origin ish(trasnform between lidar and robot).
        # We also need to transform the goal point from its fram_id to self.map frame id at time temp_map_stamp
        # Do planning, and then get transform from self.map fram id to map frame at time temp_map_stamp

        # position in local_map, is always just the transform from robot to lidar
        start = self.get_robot_position(self.map)
        # getting relevant transforms
        trans_goal_to_lidar, trans_lidar_to_map = self.get_trans(
            self.map, request.goal)  # need to transform the goal point from its frame to lidar, and then transform the path from lidar to map for pp

        # Transforming goal form map to lidar
        if trans_goal_to_lidar is not None:
            formed_goal = do_transform_point(request.goal, trans_goal_to_lidar)
        else:
            #if we cant get the transform, we just return an empty path to say it filed and let it replan
            response.global_path = Path()
            return response
        if self.current_target is not None:
            trans_target_to_lidar,_ = self.get_trans(self.map,self.current_target.point)
            #NOTE: Difference between target and goal! Goal is the waypoint that could be just an intermediate step between the target point. Target is the actual target point
            formed_target = do_transform_point(self.current_target.point, trans_target_to_lidar)
 
        # Convert 1D OccupancyGrid to 2D grid array for A*
        occupied_cells = []
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                id = self.grid_index(i, j)
                x, y = self.grid_to_map(i, j)
                # Camera obstacles are marked with -50
                if 0 < self.map.data[id] < 100:
                    if self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, (self.radius+2)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius+2)*self.resolution):
                            occupied_cells.append((i, j,self.radius))
                        #Dont want to remove walls and shit from close to me
                        #if not self.check_distance(x, y,start[0], start[1], 0.1):
                        #    self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-1)))
                    #Again dont want to remove walls and stuff
                    #if not self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, 0.1):
                    #    self.map.data[id] = 0
                    else:
                        dist = np.sqrt((formed_goal.point.x - x)**2 + (formed_goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-1)))

                elif self.map.data[id] == -50:
                    if self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, (self.radius+2)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius+2)*self.resolution):
                            occupied_cells.append((i, j,self.radius))
                        elif not self.check_distance(x, y,start[0], start[1], 0.2):
                            self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-3)))

                    elif not self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, 0.1):
                        self.map.data[id] = 0
                    else:
                        dist = np.sqrt((formed_goal.point.x - x)**2 + (formed_goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)))

                elif self.map.data[id] == -51:
                    if not self.check_distance(x, y,start[0], start[1], 0.1):
                        self.map.data[id] = 0
                elif self.map.data[id] == -125: #Dont inflate boundary too much
                    if self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, (self.radius/2)*self.resolution):
                        # Only points that are radius + two cells away from goal point are inflated
                        if self.check_distance(x, y, start[0], start[1], (self.radius/2)*self.resolution):
                            occupied_cells.append((i, j,int(self.radius//2)))
                        #If I accedentily find myself on the boundary
                        elif not self.check_distance(x, y,start[0], start[1], 0.2):
                            self.map.data[id] = 0
                        else:
                            dist = np.sqrt((start[0] - x)**2 + (start[1] - y)**2)
                            occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)//2))
                    elif not self.check_distance(x, y, formed_goal.point.x, formed_goal.point.y, 0.1):
                        self.map.data[id] = 0
                    else:
                        dist = np.sqrt((formed_goal.point.x - x)**2 + (formed_goal.point.y - y)**2)
                        occupied_cells.append((i, j,abs(int(dist//self.resolution)-2)//2))


        # inflating map
        for cell in occupied_cells:
            x1, y1,radius = cell
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
        #grid = np.full((self.map.info.width, self.map.info.height), 1)
        #for i in range(len(self.map.data)):
        #    # Getting xy coords in occupancy coords
        #    x, y = self.cords_from_index(i)
        #    if self.map.data[i] == 0:
        #        grid[x, y] = 0
        if start[0] is None or start[1] is None:
            self.get_logger().error('Could not obtain robot position for path planning')
            return
        else:
            start = self.map_to_grid(start[0], start[1])
        goal = self.map_to_grid(formed_goal.point.x, formed_goal.point.y)

        # debug
        self.get_logger().info(
            f' get goal position {request.goal.point.x} {request.goal.point.y} in grid {goal}')

        path = Astar(grid, start, goal)
        formed_path = [self.grid_to_map(point[0], point[1]) for point in path]

        response.global_path = self.publish_path(
            formed_path, self.map, trans_lidar_to_map)
        self.get_logger().info(f'finished local path planning')

        return response

    def publish_path(self, path, msg, trans):

        print('Publishing path')
        global_path = Path()
        global_path.header.frame_id = 'map'
        global_path.header.stamp = msg.header.stamp

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            formed = do_transform_pose_stamped(pose, trans)
            global_path.poses.append(formed)

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
    local_planning = LocalPlanningService()
    try:
        rclpy.spin(local_planning, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
