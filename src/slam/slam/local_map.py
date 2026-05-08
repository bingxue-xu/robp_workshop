#!/usr/bin/env python

"""Author: Arian Kourangi """

from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from math import ceil, cos, sin, atan2, fabs
import numpy as np
import rclpy
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg as std_msgs
import tf2_ros
from rclpy.node import Node
import csv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import PointStamped
from array import array
from arian_interfaces.srv import AddObsMap


class gridmap(Node):
    def __init__(self):
        super().__init__('local_map')

        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10000))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher = self.create_publisher(
            OccupancyGrid, '/local_map_arian', 10)

        # Probably need to set this hiegher. Tell Global mapper to use -10(unexplored) as free space. And let
        # the local map to decide what is free space and what is occupied space. This is because the local map is updated more frequently
        self.map = OccupancyGrid()
        self.occupied_space = np.int8(80)
        self.ws = np.int8(100)  # Everything Outside workspace
        self.camera_obs = np.int8(-50)  # Obstacle detected by Camera
        self.free_space = np.int8(0)
        self.unexplored = np.int8(0)  # Set to 0 tempt
        self.remove_camera_obs = np.int8(50)
        self.uninflated_space = np.int8(-51)
        self.boundary =np.int8(-125)

        self.points = []
        self.offset = np.int8(-2)
        self.resolution = 0.04
        self.k = 0  # Counter for tenth scan publish
        self.map_size = 1
        self.reff_scan = None
        self.curr_scan = None
        callbackgroup = MutuallyExclusiveCallbackGroup()
        self.trans = None
        self.unseens_points = []
        self.readfile()
        self.subscriber = self.create_subscription(
            sensor_msgs.LaserScan, '/scan_filtered', self.callback_lidar, 10, callback_group=callbackgroup)
        self.srv2 = self.create_service(
            AddObsMap, '/add_obs_local', self.servicecallback_add_obs, callback_group=callbackgroup)

    def readfile(self):
        data = []
        with open("/home/rosuser/dd2419_ws/src/slam/slam/workspace_2.tsv") as file:
            tsv_file = csv.reader(file, delimiter="\t")
            for line in tsv_file:
                data.append(line)
        pos1, pos2 = data[0][0], data[0][1]
        for i in range(1, len(data)):
            if pos1 == 'x' and pos2 == 'y':
                x = float(data[i][0])  # /self.resolution
                y = float(data[i][1])  # /self.resolution
            else:
                x = float(data[i][1])  # /self.resolution
                y = float(data[i][0])  # /self.resolution

            self.points.append([x, y])

        self.poly1 = Polygon(self.points)
        distance_between_pts = 0.1
        boundary = self.poly1.boundary
        boundary_length = boundary.length
        pts_boundary = [
            boundary.interpolate(n, False) for n
            in np.linspace(0, boundary_length, int(boundary_length / distance_between_pts) + 1)
        ]
        # print(pts_boundary)
        listarray = []
        for pp in pts_boundary:
            listarray.append([pp.x, pp.y])
        self.boundary_points = np.array(listarray)

        start = self.map_to_grid(2.4*np.cos(-np.pi/3),2.4*np.sin(-np.pi/3))
        end = self.map_to_grid(2.4*np.cos(-2*np.pi/3),2.4*np.sin(-2*np.pi/3))
        line_points = self.raytrace(start,end)

        for pnt in line_points:
            x,y = pnt
            free_points =self.raytrace(self.map_to_grid(0,0), (x,y))
            for free in free_points:
                x,y = self.grid_to_map(free[0], free[1])
                if np.sqrt(x*x + y*y) > 0.3:
                    self.unseens_points.append(free)
        # print(self.boundary_points)

    def reset(self, msg: sensor_msgs.LaserScan):
        self.map.info.resolution = self.resolution
        self.map.info.width = int(2*abs(self.offset)/self.resolution)
        self.map.info.height = int(2*abs(self.offset)/self.resolution)
        self.map.info.origin.position.x = np.float64(self.offset)
        self.map.info.origin.position.y = np.float64(self.offset)
        self.map.info.origin.position.z = np.float64(0)
        self.map.info.origin.orientation.x = np.float64(0)
        self.map.info.origin.orientation.y = np.float64(0)
        self.map.info.origin.orientation.z = np.float64(0)
        self.map.info.origin.orientation.w = np.float64(1)
        self.map.header.stamp = msg.header.stamp
        # Setting the timestamp of the local map to the reff scan, all other points are trasnformed to this time in spacetime
        self.map.header.frame_id = msg.header.frame_id

        temp = [self.unexplored]*int((2*abs(self.offset)/self.resolution)**2)
        temp = array('b', temp)
        self.map.data = temp
        #for free in self.unseens_points:
        #    x,y = free
        #    self.add_to_map(x, y, self.uninflated_space)


    def callback_lidar(self, msg: sensor_msgs.LaserScan):
        self.reset(msg)
        self.reff_scan = msg
        nr_messages = len(msg.ranges)
        Points = []
        for i in range(nr_messages):
            # self.reff_scan.range_max:
            if self.reff_scan.ranges[i] <= self.reff_scan.range_min  or self.reff_scan.ranges[i] >= abs(self.offset):
                continue
            else:
                total_angle = self.reff_scan.angle_min + i*self.reff_scan.angle_increment
                # Normalize angle
                total_angle = np.mod(
                    total_angle + np.pi, 2 * np.pi) - np.pi

                x_coord = np.cos(total_angle) * \
                    self.reff_scan.ranges[i]
                y_coord = np.sin(total_angle) * \
                    self.reff_scan.ranges[i]

                point = PointStamped()
                point.point.x = x_coord
                point.point.y = y_coord
                point.point.z = np.float64(0)
                point.header.stamp = self.reff_scan.header.stamp
                Points.append(
                    np.array([point.point.x, point.point.y, point.point.z]))

        trans = None

        if self.tfBuffer.can_transform(msg.header.frame_id, 'map', msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
            try:
                trans = self.tfBuffer.lookup_transform(
                    msg.header.frame_id,
                    'map',
                    msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
            except (ConnectivityException) as c:
                print(c)
            except (LookupException) as L:
                print(L)
            except (ExtrapolationException) as E:
                print(E)
            except (TransformException) as T:
                print(T)
        if trans is not None:
            for pnt in self.boundary_points:

                pointstamp = PointStamped()
                pointstamp.point.x = pnt[0]
                pointstamp.point.y = pnt[1]
                pointstamp.point.z = np.float64(0)
                pointstamp.header.stamp = msg.header.stamp
                pointstamp.header.frame_id = 'map'
                formed = do_transform_point(pointstamp, trans)
                if abs(formed.point.x) < abs(self.offset) and abs(formed.point.y) < abs(self.offset):
                    Points.append(
                        np.array([formed.point.x, formed.point.y, formed.point.z]))

        for point in Points:
            # Adding occupied space
            x, y = self.map_to_grid(point[0], point[1])
            self.add_to_map(x, y, self.boundary)
        
        #adding unseen space as camera obstacles



        self.publisher.publish(self.map)

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

        """Returns coordinates IN GRID MAP coordinates based on the index in gridmap array (map.data)"""
        x = index % self.map.info.width
        y = round(index//self.map.info.width)
        return x, y

    def is_in_bounds(self, x, y):
        """HELPER FUNCTION"""
        """Returns weather (x, y) is inside grid_map or not."""

        if abs(x) < self.map.info.width:
            if abs(y) < self.map.info.height:
                return True
        return False

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

    def generate_circle_points(self, center_x, center_y, radius, num_points, start_angle, stop_angle):
        """ HELPER FUNCTION """
        # Takes center point in map frame (m) and radius in meters and returns points on the circle in map frame (m)
        # Generate angles evenly spaced between 0 and 2*pi
        angles = np.linspace(start_angle, stop_angle, num_points)

        # Calculate x and y coordinates of points on the circle
        x_coords = center_x + radius * np.cos(angles)
        y_coords = center_y + radius * np.sin(angles)

        # Return a list of (x, y) tuples representing the points on the circle
        return [(x, y) for x, y in zip(x_coords, y_coords)]

    def add_to_map(self, x, y, value):
        """Takes in x,y in gridmap coord system then adds them to map
        First check what current value is in corresponding cell, if it is self.ws then ignore since it is outside the workspace
        Depending on what the current value of the cell is it will reset to free/occupied or decrese prob of being occupied """

        if self.is_in_bounds(x, y):

            # If in gridmap
            if self.map.data[self.grid_index(x, y)] != self.camera_obs:
                # If not camera detected obstacle
                self.map.data[self.grid_index(x, y)] = value

    def servicecallback_add_obs(self, request, response):
        """Takes in a point in map frame and adds it to the map as occupied space
        All mesurments need to be in meters"""
        radpoints = request.radpoints.points
        for radpoint in radpoints:
            point = radpoint.point  # pointstamped
            radius = radpoint.radius
            trans = None

            trans = None
            source_frame = point.header.frame_id
            source_time = point.header.stamp
            target_frame = self.reff_scan.header.frame_id
            target_time = self.reff_scan.header.stamp
            fixed_frame = 'map'

            if self.tfBuffer.can_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rclpy.duration.Duration(seconds=1)):
                try:
                    trans = self.tfBuffer.lookup_transform_full(
                        target_frame,
                        target_time,
                        source_frame,
                        source_time, fixed_frame, timeout=rclpy.duration.Duration(seconds=1))
                except (ConnectivityException) as C:
                    self.get_logger().error(str(C))
                except (LookupException) as L:
                    self.get_logger().error(str(L))
                except (ExtrapolationException) as E:
                    self.get_logger().error(str(E))
                except (TransformException) as T:
                    self.get_logger().error(str(T))
            if trans is not None:
                self.get_logger().info('Adding obstacle to map')
                # Getting center point in map frame (m)
                formed = do_transform_point(point, trans)
                quat = quaternion_from_euler(0, 0, radpoint.orientation)
                temp = PoseStamped()
                temp.pose.position = point.point
                temp.header = point.header
                temp.pose.orientation.x = quat[0]
                temp.pose.orientation.y = quat[1]
                temp.pose.orientation.z = quat[2]
                temp.pose.orientation.w = quat[3]
                temp_formed = do_transform_pose_stamped(temp, trans)


                new_q= [temp_formed.pose.orientation.x, temp_formed.pose.orientation.y, temp_formed.pose.orientation.z, temp_formed.pose.orientation.w]
                [_, _, orientation] = euler_from_quaternion(new_q)

                # only want relevant points so that it doesnt do some funky shit:
                if abs(formed.point.x) < abs(self.offset) and abs(formed.point.y) < abs(self.offset):
                    if radpoint.type != 'target box' and radpoint.type != 'close to target':
                        circle_points = self.generate_circle_points(
                            formed.point.x, formed.point.y, radius, 100, 0, 2*np.pi)
                        for pnt in circle_points:
                            x, y = self.map_to_grid(pnt[0], pnt[1])
                            self.add_to_map(x, y, self.camera_obs)
                    #if radpoint.type == 'target box':
                    #    radius = 1.0
                    #    circle_points1 = self.generate_circle_points(
                    #        formed.point.x, formed.point.y, radius, 1000, (orientation + np.pi/9 - 2*np.pi), (orientation - np.pi/9 ))
                    #    #circle_points2 = self.generate_circle_points(
                    #    #    formed.point.x, formed.point.y, radius - self.resolution, 1000, (orientation + np.pi/10 - 2*np.pi), (orientation - np.pi/10))
                    #    start = self.map_to_grid(formed.point.x + radius * np.cos(orientation +np.pi/9), formed.point.y + radius * np.sin(orientation +np.pi/9))
                    #    end = self.map_to_grid(formed.point.x + (0.3) * np.cos(orientation +np.pi/9), formed.point.y + 0.2* np.sin(orientation +np.pi/9))
                    #    start1 = self.map_to_grid(formed.point.x + radius * np.cos(orientation -np.pi/9), formed.point.y + radius * np.sin(orientation -np.pi/9))
                    #    end1 = self.map_to_grid(formed.point.x + (0.3) * np.cos(orientation -np.pi/9), formed.point.y + 0.2* np.sin(orientation -np.pi/9))
#
                    #    line_points1 = self.raytrace(start,end)
                    #    line_points2 = self.raytrace(start1,end1)
#
                    #    for pnt in line_points1:
                    #        x, y = pnt[0], pnt[1]
                    #        self.add_to_map(x, y, self.uninflated_space)
#
                    #    for pnt in line_points2:
                    #        x, y = pnt[0], pnt[1]
                    #        self.add_to_map(x, y, self.uninflated_space)
                    #    for pnt in circle_points1:
                    #        x, y = self.map_to_grid(pnt[0], pnt[1])
                    #        self.add_to_map(x, y, self.uninflated_space)
                    #    #for pnt in circle_points2:
                    #    #    x, y = self.map_to_grid(pnt[0], pnt[1])
                    #    #    self.add_to_map(x, y, self.uninflated_space)
        response.map = self.map
        return response


def main():
    rclpy.init()
    node = gridmap()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
