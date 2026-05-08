
from copy import deepcopy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker
from math import ceil, cos, sin, atan2, fabs

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import numpy as np
import math
import random
import rclpy

from gustav_custom_interfaces.srv import Explore
from gustav_custom_interfaces.srv import VisualGrid

from .a_star import Astar
import std_msgs.msg as std_msgs
"""        
create and updare viz_map
call explorer_callback to get best path
    explorer_callback: (as a service)
        get robot pose
        send req to global planner
        calculate best path based on cells seen
        return best path
    
send best path to local planner or do pp
"""

### CONSTANTS###
FOV = 50
MAX_RANGE = 22
NR_POSES = 100
###############
# CONSTANTS FOR SIMULAITON AND TESTING


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info('Explorer node started...')

        self.viz_grid = None  # 2D matrix look into this
        self.global_grid = None
        self.global_map = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = [None]*(NR_POSES**2)
        self.counter = 0

        self.offset_x = 0
        self.offset_y = 0
        self.resolution = 0
        self.height = 0
        self.width = 0

        self.free = 0  # TODO:Where did these values come from?
        self.occupied = 80
        self.unexplored = -10

        self.first = True

        # TODO: DOES vizgrid and global grid have the same syntax? What is -1, 1 and 0?

        self.visible_obs = []
        self.obstacles = []

        self.radius = 0

        self.robot_frame = 'base_link'
        self.global_frame = 'map'
        cbr = ReentrantCallbackGroup()
        # self.sub_global_map = self.create_subscription(
        #    OccupancyGrid, '/map', self.map_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_viz_map = self.create_publisher(OccupancyGrid, '/viz_map', 10)
        self.path_pub = self.create_publisher(Path, '/explore_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz_point', 50)

        self.create_timer(0.1, self.timer_callback, callback_group=cbr)

        self.retrace_steps_srv = self.create_service(
            VisualGrid, '/visual_grid', self.retrace_steps_callback, callback_group=cbr)
        self.explore_srv = self.create_service(
            Explore, '/explore', self.explorer_callback)
        self.sleep = self.create_publisher(std_msgs.Bool, '/sleep', 10)

        # self.cli_retrace_steps = self.create_client(VisualGrid, '/visual_grid')

    # def map_callback(self, msg: OccupancyGrid):
    #    # self.global_map = msg
    #    # if self.first:
    #    #    self.init_var()
    #    pass

    def pub_marker(self, x, y, id, color):
        self.marker = Marker()

        self.get_logger().info('Publishing marker at %f, %f' % (x, y))

        self.marker.header.frame_id = "map"
        self.marker.id = id
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD

        self.marker.pose.position.x = float(x)
        self.marker.pose.position.y = float(y)

        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color.a = 1.0
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker_pub.publish(self.marker)
        # self.get_logger().info('Published marker at %d, %d' % (x, y))


    def robot_pose(self):
        """
        Calculates the robots position on the grid from its global pose
        """
        if self.tf_buffer.can_transform(self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2)
                )
                roll, pitch, yaw = euler_from_quaternion(
                    [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                # self.get_logger().info(f'roll: {roll}, pitch: {pitch}, yaw: {yaw}')
                world_x = t.transform.translation.x
                world_y = t.transform.translation.y
                # grid_x, grid_y = self.map_to_grid(world_x, world_y)
                return (world_x, world_y, yaw)
            except:
                self.get_logger().info(f'Error in getting robot pose error')

    # def map_callback(self, msg: OccupancyGrid):
    #     self.global_map = msg
    #     if self.first:
    #         self.height = self.global_map.info.height
    #         self.width = self.global_map.info.width
    #         self.viz_grid = [[-1 for _ in range(self.height)] for _ in range(self.width)] #2D matrix look into this
    #         self.offset_x = self.global_map.info.origin.position.x
    #         self.offset_y = self.global_map.info.origin.position.y
    #         self.resolution = self.global_map.info.resolution
    #         self.first = False

    def init_var(self):
        self.height = self.global_map.info.height
        self.width = self.global_map.info.width
        # 2D matrix look into this
        self.viz_grid = np.full((self.global_map.info.width,
                                 self.global_map.info.height), -1)
        self.offset_x = self.global_map.info.origin.position.x
        self.offset_y = self.global_map.info.origin.position.y
        self.resolution = self.global_map.info.resolution
        self.radius = int(0.20/self.resolution)
        self.first = False
        self.nr_cells_outside_ws = self.global_map.data.count(
            100) + self.global_map.data.count(-125)

    def timer_callback(self):
        """
        Update viz map based on robots current pose
        """
        #self.get_logger().info('Timer callback...')

        tmp = self.robot_pose()
        if tmp is None:
            self.get_logger().info('No robot pose...')
            return
        self.path[self.counter] = tmp
        self.counter += 1

        # self.get_logger().info(f'yaw: {tmp[2]}')

        # if self.counter < 3:
        #    self.path[self.counter] = tmp
        #    self.counter += 1
        #    self.get_logger().info(f'robot pose: {tmp[0], tmp[1] , tmp[2]}')
        #    return
        # prev = self.path[self.counter - 1]
        # if np.abs(tmp[2] - prev[2]) > np.pi/12:
        #    self.path[self.counter] = tmp
        #    self.counter += 1
        #    self.get_logger().info(f'robot pose: {tmp[0], tmp[1] , tmp[2]}')
        #    return
        # if np.abs((tmp[0] - prev[0])**2 + (tmp[1] - prev[1])**2) > 0.2:
        #    self.path[self.counter] = tmp
        #    self.counter += 1
        #    self.get_logger().info(f'robot pose: {tmp[0], tmp[1] , tmp[2]}')
        #    return

    def map_to_grid(self, x, y):
        """Takes in x,y in map coordinates (m) and returns coordinates in grid map coordinates (cells) """
        x_coord = int((x - self.offset_x)/self.resolution)
        y_coord = int((y - self.offset_y)/self.resolution)
        return x_coord, y_coord

    def grid_to_map(self, x, y):
        """Takes in coordinates in gridmap coordinates (cells) and converts them to coordinates in map frame (m)"""
        x_coord = x*self.resolution + self.offset_x
        y_coord = y*self.resolution + self.offset_y
        return x_coord, y_coord

    def grid_index(self, x, y):
        """Calculates index in grid map array (map.data) based on gridmap coordinates"""
        index = int(x + y * self.width)
        return index

    def cords_from_index(self, index):
        """Returns coordinates IN GRID MAP coordinates based on the index in gridmap array (map.data)"""
        x = index % self.width
        y = index // self.width
        return x, y

    def global_map_to_grid(self):
        """
        Convert global occupancy grid to 2D matrix
        where each self is the size of self.resolution
        """
        # Convert 1D OccupancyGrid to 2D grid array for A* only 0 and 1
        self.obstacles = []
        #self.get_logger().info(
        #    f'global map width {self.global_map.info.width}, height {self.global_map.info.height}')

        grid = np.full((self.global_map.info.width,
                       self.global_map.info.height), 1)

        for i in range(self.global_map.info.width):
            for j in range(self.global_map.info.height):
                id = self.grid_index(i, j)
                if self.global_map.data[id] == self.occupied or self.global_map.data[id] == -50:
                    self.obstacles.append((i, j))
                    # grid[i, j] = 1
                elif self.global_map.data[id] == self.free:
                    grid[i, j] = 0
                # To give Astar some breathing room
                elif self.global_map.data[id] == self.unexplored:
                    grid[i, j] = -1
        #self.get_logger().info(f'grid shape: {grid.shape}')

        return grid

    def seen_cells(self, obs_grid, viz_grid, pose, robot_pose=True):
        """
        Get the cells that are visible from the agent's current position in a cone infront of it.
        something here is fucked with the resolutoin and variation of the grid, at bit unclear what and how exaclty. 
        """
        visible_cells = []
        obs_list = []
        if robot_pose:
            grid_x, grid_y = self.map_to_grid(pose[0], pose[1])
        else:
            grid_x = pose[0]
            grid_y = pose[1]
        # self.get_logger().info(f'grid pose: {grid_x, grid_y}, map: {pose[0], pose[1]}')
        # self.get_logger().info(f'angle: {math.degrees(pose[2])}')
        for angle in range(-FOV//2, FOV//2, FOV//8):
            # Adjust the angle based on the direction the agent is facing
            adjusted_angle = math.degrees(pose[2]) + angle
            adjusted_angle = np.mod(adjusted_angle + 180, 360)-180
            # self.get_logger().info(f'adjusted_angle: {adjusted_angle}')
            for r in range(MAX_RANGE):
                # self.get_logger().info(f'adjusted angle: {adjusted_angle}')
                x = int((grid_x + r * math.cos(math.radians(adjusted_angle))))
                y = int((grid_y + r * math.sin(math.radians(adjusted_angle))))
                # line = self.bresenham_line((grid_y, grid_y), (x, y))
                line = self.raytrace((grid_x, grid_y), (x, y))
                for point in line:
                    if point[0] < 0 or point[0] >= self.width or point[1] < 0 or point[1] >= self.height:
                        break
                    if obs_grid[point[0]][point[1]] == 1:
                        obs_list.append(point)
                        break  # TODO: Why break here?
                    elif viz_grid[point[0]][point[1]] == 0:
                        continue
                    elif viz_grid[point[0]][point[1]] == -1:
                        visible_cells.append(point)
        return visible_cells, obs_list

    def bresenham_line(self, start, end):
        """Generate points along a line using Bresenham's line algorithm."""
        points = []
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        x, y = x1, y1
        if abs(dy) < abs(dx):
            # Slope < 1
            if dx < 0:
                x1, x2 = x2, x1
                y1, y2 = y2, y1
                dx, dy = -dx, -dy
                x, y = x1, y1

            p = 2*dy - dx
            while x <= x2:
                points.append((int(x), int(y)))  # Convert x and y to integers
                x += 1
                if p < 0:
                    p += 2*dy
                else:
                    y += 1 if y1 < y2 else -1
                    p += 2*(dy - dx)
        else:
            # Slope >= 1
            if dy < 0:
                x1, x2 = x2, x1
                y1, y2 = y2, y1
                dx, dy = -dx, -dy
                x, y = x1, y1

            p = 2*dx - dy
            while y <= y2:
                points.append((int(x), int(y)))  # Convert x and y to integers
                y += 1
                if p < 0:
                    p += 2*dx
                else:
                    x += 1 if x1 < x2 else -1
                    p += 2*(dx - dy)

        return points

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def convert_seen(self, cells):
        """Converts the cells that have been seen to 0 in the visibility grid."""
        # print('Converting seen cells...')
        #self.get_logger().info(f'Size of cells: {len(cells)}')
        for cell in cells:
            self.viz_grid[cell[0]][cell[1]] = 0
        # self.get_logger().info('Done converting seen cells...')

    def random_point(self):
        """Gets a random point on the grid that is not an obstacle."""
        # self.get_logger().info(f'height: {self.height}, width: {self.width}')
        # self.get_logger().info(f'vizgrid height: {len(self.viz_grid)}, width: {len(self.viz_grid[0])}')
        # Generate random indices
        iter = 0
        while True:
            if iter > 10000:
                break
            i, j = np.random.randint(
                0, self.viz_grid.shape[0]), np.random.randint(0, self.viz_grid.shape[1])
            if self.global_grid[i, j] == 0: #or self.global_grid[i, j] == -1:
                # Choosing a point in global grid that is not outside ws and not obstacle
                window = self.viz_grid[max(0, i-3):min(self.viz_grid.shape[0], i+4),
                                       max(0, j-3):min(self.viz_grid.shape[1], j+4)]
                if self.viz_grid[i, j] == -1 or np.any(window == -1):
                    # Choosing corresponding point in viz grid if it is unseen or if it hase unseen cells within 15 cm
                    return (i, j)
            iter += 1
        return (0, 0)

    def calc_info_gain(self, obs_grid, viz_grid, path):
        """
        Calculate the information gain of a path.
        Information gain is the number of new cells that will be seen along the path.
        Seen cells is a list of sets containing all the cells that will be seen from each point in the path.
        """
        info_gain = 0
        cells = []
        for point in path:
            x = point[0]
            y = point[1]
            direction = point[2]
            new_cells, obs_list = self.seen_cells(
                obs_grid, viz_grid, (x, y, direction), robot_pose=False)
            info_gain += len(new_cells)
            cells.append(new_cells)

        return info_gain, cells

    def retrace_steps_callback(self, request, response):
        """
        Retrace steps taken by robot
        """
        self.get_logger().info('Retrace steps callback...')
        self.global_map = request.obs_map
        sleep = std_msgs.Bool()
        sleep.data = True
        #makes Scan matcher and Detection sleep for 3 sec
        self.sleep.publish(sleep)
        #while self.global_map.info.height == 0:
        #    self.get_logger().info(f'Waiting for global map...')

        # update the global map
        # TODO: WHY COPY MAP LIKE THISJUST TAKE MAP MESSAGE?
        # for i in range(len(self.global_map.data)):
        #    self.global_map.data[i] = self.global_map.data[i]

        # if first run initialize variables
        if self.first:
            self.init_var()

        # convert global map to grid
        self.global_grid = self.global_map_to_grid()
        # self.get_logger().info(f'global grid widht: {len(self.global_grid)}, height {len(self.global_grid[0])}')

        counter = 0
        for point in self.path:
            #self.get_logger().info(f'counter: {counter}')
            if point is None:
                break
            visible_cells, self.visible_obs = self.seen_cells(
                self.global_grid, self.viz_grid, point)
            self.convert_seen(visible_cells)
            for obs in self.visible_obs:
                self.viz_grid[obs[0]][obs[1]] = 1

            counter += 1

        # update viz map based on where the robot has been
        Map = OccupancyGrid()

        Map.header = self.global_map.header

        Map.info = self.global_map.info

        Map.data = [-1]*(self.width*self.height)

        for i in range(self.width):
            for j in range(self.height):

                index = self.grid_index(i, j)
                if self.viz_grid[i][j] == -1:
                    Map.data[index] = -1
                if self.viz_grid[i][j] == 0:
                    Map.data[index] = 0
                if self.viz_grid[i][j] == 1:
                    Map.data[index] = 100

        self.pub_viz_map.publish(Map)
        response.percentage = (((self.viz_grid == 0).sum(
        ) + (self.viz_grid == 1).sum()) / (self.width*self.height - self.nr_cells_outside_ws))*100

        # self.get_logger().info('Done retrace steps callback...')

        # Resetting path to not go through all poses already dealt with
        self.path = [None]*(NR_POSES**2)
        self.counter = 0

        return response

    def is_in_bounds(self, x, y):
        """HELPER FUNCTION"""
        """Returns weather (x, y) is inside grid_map or not."""
        if abs(x) < self.width:
            if abs(y) < self.height:
                return True
        return False

    def inflate(self):
        for obs in self.obstacles:
            x1 = obs[0]
            y1 = obs[1]
            for dx in range(-self.radius, self.radius + 1):
                for dy in range(-self.radius, self.radius + 1):
                    if np.sqrt(dx**2 + dy**2) <= self.radius:
                        x2 = x1 + dx
                        y2 = y1 + dy
                        if self.is_in_bounds(x2, y2):
                            self.global_grid[y2][x2] = 1

    def explorer_callback(self, request, response):
        # check that retrace has been called before
        if self.first:
            self.get_logger().error(f'first is not set, run retrace steps first')
        sleep = std_msgs.Bool()
        sleep.data = True
        #makes Scan matcher and Detection sleep for 3 sec
        self.sleep.publish(sleep)
        best_point = None
        best_gain = 0
        self.get_logger().info('Explorer callback...')

        # tmp_grid = deepcopy(self.global_grid)
        # self.get_logger().info(f'size tmp grid {len(tmp_grid)}')
        # self.inflate()

        for _ in range(25):
            #self.get_logger().info(f'Exploring iteration... {_}')
            rand_point = self.random_point()
            robot_pose = self.robot_pose()
            grid_x, grid_y = self.map_to_grid(robot_pose[0], robot_pose[1])
            start = (grid_x, grid_y, robot_pose[2])
            # self.get_logger().info(f'robot pose: {start}')
            # self.get_logger().info(f'rand point: {rand_point}')
            # self.get_logger().info(
            #    f'rand point value: {self.viz_grid[rand_point[0]][rand_point[1]]}')
            # path = Astar(self.global_grid, start, rand_point)
            tmp_gain, tmp_cell = self.calc_info_gain(
                self.global_grid, self.viz_grid, [[rand_point[0], rand_point[1], 0], [rand_point[0], rand_point[1], np.pi/2], [rand_point[0], rand_point[1], -np.pi/2], [rand_point[0], rand_point[1], np.pi]])

            if tmp_gain > best_gain:
                best_gain = tmp_gain
                best_point = rand_point
        rx, ry = self.grid_to_map(best_point[0], best_point[1])
        self.pub_marker(rx, ry, best_point[0], (0.0, 1.0, 0.0))

        self.get_logger().info(f'Done exploring')

        if best_gain != 0:
            x, y = self.grid_to_map(best_point[0], best_point[1])
            Point = PointStamped()
            Point.header.frame_id = self.global_frame
            Point.header.stamp = self.get_clock().now().to_msg()
            Point.point.x = x
            Point.point.y = y
            # path.poses.append(pose)

            self.get_logger().info(f'published path...')
            # self.path_pub.publish(path)
            response.point = Point

            return response
        else:
            self.get_logger().info(f'No path found')
            response.point = PointStamped()
            response.point.point.x = 9999.0
            return response


def main():
    rclpy.init()

    explorer = Explorer()

    try:
        rclpy.spin(explorer, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
