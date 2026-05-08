import rclpy
import time
import random
import math

import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker

from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster



class TestMap(Node):
    def __init__(self):
        super().__init__('test_map')

        self.map = OccupancyGrid()
        self.obs_map = OccupancyGrid()

        self.broadcaster = StaticTransformBroadcaster(self)

        self.free_thresh = 0.25
        self.occupied_thresh = 0.65

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.viz_map_pub = self.create_publisher(OccupancyGrid, '/obs_map', 10)
        self.point_pub = self.create_publisher(Marker, '/viz_point', 10)

        self.subscription  # prevent unused variable warning

    def broadcast_static_transform(self):
        
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "odom"
        static_transformStamped.child_frame_id = "map"

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        static_transformStamped.transform.rotation.x = 0
        static_transformStamped.transform.rotation.y = 0
        static_transformStamped.transform.rotation.z = 0
        static_transformStamped.transform.rotation.w = 1

        self.broadcaster.sendTransform(static_transformStamped)

    def interpolate(self, start, end, t):
        return (1 - t) * start + t * end

    def calc_angle(self, start, end):
        return math.atan2(end.y - start.y, end.x - start.x)

    def map_callback(self, msg):
        self.get_logger().info('Received map: %d x %d' % (msg.info.width, msg.info.height))
        self.map = msg
        self.publish_viz_map()
    
    def publish_map(self):
        self.obs_map.header.frame_id = 'map'
        self.obs_map.header.stamp = self.get_clock().now().to_msg()

        self.obs_map.info.height = 100
        self.obs_map.info.width = 100
        self.obs_map.info.resolution = 0.05
        self.obs_map.info.origin.position = Point()
        self.obs_map.info.origin.position.x = -2.35
        self.obs_map.info.origin.position.y = -1.25
        self.obs_map.info.origin.position.z = 0.0
        self.obs_map.info.origin.orientation.x = 0.0
        self.obs_map.info.origin.orientation.y = 0.0
        self.obs_map.info.origin.orientation.z = 0.0
        self.obs_map.info.origin.orientation.w = 1.0
        self.obs_map.data = [-1] * (self.obs_map.info.width * self.obs_map.info.height)
        self.random_obs()

        self.map_pub.publish(self.obs_map)
        self.get_logger().info('Published map: %d x %d' % (self.obs_map.info.width, self.obs_map.info.height))
    
    def random_obs(self):
        random.seed(0)
        for _ in range(100):
            index = random.randint(0, self.obs_map.info.width * self.obs_map.info.height - 1)
            self.obs_map.data[index] = 1

    def publish_viz_map(self):
        self.viz_grid = self.global_map_to_grid()
        print(f'viz grid shape: {self.viz_grid.shape}')
        print(f'origin: ({self.map.info.origin.position.x}, {self.map.info.origin.position.y})')

        self.viz_map = OccupancyGrid()
        self.viz_map.header.frame_id = 'map'
        self.viz_map.header.stamp = self.get_clock().now().to_msg()
        self.viz_map.info.height = self.map.info.height
        self.viz_map.info.width = self.map.info.width
        self.viz_map.info.resolution = self.map.info.resolution
        self.viz_map.info.origin = self.map.info.origin
        for i in range(len(self.viz_grid)):
            for j in range(len(self.viz_grid[0])):
                index = self.grid_index(i, j, self.viz_map)
                self.viz_map.data[index] = self.viz_grid[i, j]
        self.viz_map_pub.publish(self.viz_map)

    def grid_index(self, x, y):
        """Calculates index in grid map array (map.data) based on gridmap coordinates"""
        index = int(x + y * self.map.info.width)
        return index

    def cords_from_index(self, index):
        """Returns coordinates IN GRID MAP coordinates based on the index in gridmap array (map.data)"""
        x = index % self.map.info.width
        y = index // self.map.info.width
        return x, y
    
    def map_to_grid(self, x, y):
        """Takes in x,y in map coordinates (m) and returns coordinates in grid map coordinates (cells) """
        x_coord = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
        y_coord = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
        return x_coord, y_coord

    def grid_to_map(self, x, y):
        """Takes in coordinates in gridmap coordinates (cells) and converts them to coordinates in map frame (m)"""
        x_coord = x*self.map.info.resolution + self.map.info.origin.position.x
        y_coord = y*self.map.info.resolution + self.map.info.origin.position.y
        return x_coord, y_coord
    
    """
    cell_x = pose_x / resolution + off_set_x
    cell_y = pose_y / resolution + off_set_y
    """
    
    def global_map_to_grid(self):
        """
        Convert global occupancy grid to 2D matrix
        where each self is the size of self.resolution
        """
        # Convert 1D OccupancyGrid to 2D grid array for A* only 0 and 1
        occupied_cells = []
        self.get_logger().info(f'global map width {self.map.info.width}, height {self.map.info.height}')
        
        # Convert 1D OccupancyGrid to 2D grid array for A*
        grid = np.full((self.map.info.height, self.map.info.width), -1)        
        self.get_logger().info(f'global map data length {len(self.map.data)}')
        for i in range(len(self.map.data)):
            x, y = self.cords_from_index(i)
            self.get_logger().info(f'x: {x}, y: {y}')
            self.get_logger().info(f'global map width {self.map.info.width}, height {self.map.info.height}')
            # if i == 50:
            #     break
            if self.map.data[i] > 0.65:
                occupied_cells.append((x, y))
                grid[y, x] = 1
            elif self.map.data[i] < 0.25:
                grid[y, x] = 0 

        print(grid[-1][-2])  
        return grid
    
    # def index_to_coords2(self, index):
    #     self.get_logger().info(f' width {self.map.info.width} height {self.map.info.height}')
    #     x = (index % self.map.info.width) * self.map.info.resolution + self.map.info.origin.position.x
    #     y = (index // self.map.info.width) * self.map.info.resolution + self.map.info.origin.position.y
    #     self.get_logger().info('Index %d -> coords %f, %f' % (index, x, y))
    #     return x, y

    
    def publish_markers(self):
        pass
    #     # self.pub_marker(self.map.info.origin.position.x, self.map.info.origin.position.y, 0, (1.0, 0.0, 0.0))
    #     self.pub_marker(0.0, 0.0, 1, (0.0, 1.0, 0.0))
    #     x, y = self.index_to_coords2(0)
    #     self.pub_marker(x, y, 2, (0.0, 0.0, 1.0))

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
        self.point_pub.publish(self.marker)
        self.get_logger().info('Published marker at %d, %d' % (x, y))


    def global_path_callback(self):

        #map = copy.deepcopy(self.map)

        # Convert 1D OccupancyGrid to 2D grid array for A*
        map_2d = np.full((self.obs_map.info.height, self.obs_map.info.width), -1)        
        for i in range(len(self.obs_map.data)):
            x, y = self.index_to_coords(self.obs_map.data[i])
            map_2d[x, y] = self.obs_map.data[i]


        occupied_cells = []
        for i in range(self.obs_map.info.width):
            for j in range(self.obs_map.info.height):
                id = self.grid_index(i, j)
                # Camera obstacles are marked with -50
                if 0 < temp_map.data[id] < 100 or temp_map.data[id] == -50:
                    x, y = self.grid_to_map(i, j)
                    if self.check_distance(x, y, request.goal.point.x, request.goal.point.y, (self.radius+2)*self.resolution):
                        # Only points that are radius + two cells away are inflated
                        occupied_cells.append((i, j))
        # inflating map
        for cell in occupied_cells:
            x1, y1 = cell
            for dx in range(-self.radius, self.radius + 1):
                for dy in range(-self.radius, self.radius + 1):
                    if np.sqrt(dx**2 + dy**2) <= self.radius:
                        x2 = x1 + dx
                        y2 = y1 + dy
                        if self.is_in_bounds(x2, y2):
                            temp_map.data[self.grid_index(
                                x2, y2)] = np.int8(80)

        self.inflated_publisher.publish(temp_map)



def main(args=None):
    rclpy.init(args=args)

    test_map = TestMap()
    
    try:
        rclpy.spin(test_map)
    except KeyboardInterrupt:
        test_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()