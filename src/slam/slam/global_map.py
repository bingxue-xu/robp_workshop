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
from arian_interfaces.srv import IsInWs
from arian_interfaces.srv import AddObsMap
from .process_cloud import *

import time


class gridmap(Node):
    def __init__(self):
        super().__init__('global_map')

        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=1000))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Probably need to set this hiegher. Tell Global mapper to use -10(unexplored) as free space. And let
        self.refresh_rate = 2  # Hz
        # the local map to decide what is free space and what is occupied space. This is because the local map is updated more frequently
        self.map = OccupancyGrid()
        self.occupied_space = np.int8(80)
        self.ws = np.int8(100)  # Everything Outside workspace
        self.camera_obs = np.int8(-50)  # Obstacle detected by Camera
        self.uninflated_space = np.int8(-51)
        self.remove_camera_obs = np.int8(50)
        self.free_space = np.int8(0)
        self.unexplored = np.int8(-10)  # Set to 0 tempt
        self.boundary = np.int8(-125)

        self.points = []
        self.offset = np.int8(-10)
        self.resolution = 0.04
        self.k = 0  # Counter for tenth scan publish
        self.poly1 = None
        callbackgroup1 = MutuallyExclusiveCallbackGroup()
        callbackgroup2 = ReentrantCallbackGroup()
        self.points_within_polygon = []
        self.read_file()

        self.subscriber = self.create_subscription(
            sensor_msgs.LaserScan, '/scan_filtered', self.callback_lidar, 10, callback_group=callbackgroup1)
        self.srv1 = self.create_service(
            IsInWs, '/is_in_ws', self.servicecallback, callback_group=callbackgroup2)
        self.srv2 = self.create_service(
            AddObsMap, '/add_obs_global', self.servicecallback_add_obs, callback_group=callbackgroup2)

    def read_file(self):

        data = []
        with open("/home/rosuser/dd2419_ws/src/slam/slam/workspace_2.tsv") as file:
            tsv_file = csv.reader(file, delimiter="\t")
            for line in tsv_file:
                data.append(line)
        pos1, pos2 = data[0][0], data[0][1]
        for i in range(1, len(data)):
            if pos1 == 'x' and pos2 == 'y':
                x = float(data[i][0])/self.resolution
                y = float(data[i][1])/self.resolution
            else:
                x = float(data[i][1])/self.resolution
                y = float(data[i][0])/self.resolution

            self.points.append([x, y])

        self.poly1 = Polygon(self.points)

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
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.header.frame_id = 'map'

        temp = [self.ws]*int((2*abs(self.offset)/self.resolution)**2)
        temp = array('b', temp)
        self.map.data = temp

        min_x, min_y, max_x, max_y = self.poly1.bounds

        x_coords = np.arange(min_x, max_x)
        y_coords = np.arange(min_y, max_y)

        for x in x_coords:
            for y in y_coords:
                point = Point(x, y)
                if self.poly1.contains(point):
                    self.points_within_polygon.append((x, y))

        for shit in self.points_within_polygon:
            x = int((shit[0]) - self.offset/self.resolution)
            y = int((shit[1]) - self.offset/self.resolution)
            self.map.data[self.grid_index(x, y)] = self.unexplored
        
        distance_between_pts = self.resolution
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

        for shit in self.boundary_points:
            x = int((shit[0]) - self.offset/self.resolution)
            y = int((shit[1]) - self.offset/self.resolution)
            self.map.data[self.grid_index(x, y)] = self.boundary
        """
        ## TESTING####
        x_real = shit[0]*self.resolution
        y_real = shit[1]*self.resolution
        x_grid, y_grid = self.map_to_grid(x_real, y_real)
        index = self.grid_index(x_grid, y_grid)
        x_rev, y_rev = self.cords_from_index(index)
        print('Coords in real map', x_real, y_real)
        print('Coords in gridmap', x_grid, y_grid,
              'index for polygon coordinates', index)
        print('Coords in grid from index :', index,
              '=', x_rev, y_rev)
        print('Coords real from grid coors = ',
              self.grid_to_map(x_rev, y_rev))
        
        """

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

    def callback_lidar(self, msg: sensor_msgs.LaserScan):

        trans = None
        source_frame = msg.header.frame_id
        target_frame = 'map'
        self.k += 1
        if self.tfBuffer.can_transform('map', msg.header.frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
            try:
                trans = self.tfBuffer.lookup_transform(
                    target_frame,
                    source_frame,
                    msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
            except (ConnectivityException) as c:
                print(c)
            except (LookupException) as L:
                print(L)
            except (ExtrapolationException) as E:
                print(E)
            except (TransformException) as T:
                print(T)
        if trans is not None and self.k % self.refresh_rate == 0:
            # quat = [trans.transform.rotation.x, trans.transform.rotation.y,
            #        trans.transform.rotation.z, trans.transform.rotation.w]
            # angles = euler_from_quaternion(quat)
            # robot_yaw = angles[2]
            nr_messages = len(msg.ranges)
            robot_x_coord = trans.transform.translation.x
            robot_y_coord = trans.transform.translation.y
            Points = []
            occ_points = []

            for i in range(nr_messages):
                if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                    continue
                elif msg.range_min < msg.ranges[i] <= 8.0:
                    total_angle = msg.angle_min + i*msg.angle_increment
                    # Normalize angle
                    total_angle = np.mod(
                        total_angle + np.pi, 2 * np.pi) - np.pi

                    x_coord = np.cos(total_angle) * \
                        msg.ranges[i]
                    y_coord = np.sin(total_angle) * \
                        msg.ranges[i]

                    point = PointStamped()
                    point.point.x = x_coord
                    point.point.y = y_coord
                    point.point.z = np.float64(0)
                    point.header.stamp = msg.header.stamp
                    formed = do_transform_point(point, trans)
                    occ_points.append(
                        np.array([formed.point.x, formed.point.y, formed.point.z]))
                    Points.append(
                        np.array([formed.point.x, formed.point.y, formed.point.z]))
                else:
                    total_angle = msg.angle_min + i*msg.angle_increment
                    # Normalize angle
                    total_angle = np.mod(
                        total_angle + np.pi, 2 * np.pi) - np.pi

                    x_coord = np.cos(total_angle) * \
                        msg.ranges[i]
                    y_coord = np.sin(total_angle) * \
                        msg.ranges[i]

                    point = PointStamped()
                    point.point.x = x_coord
                    point.point.y = y_coord
                    point.point.z = np.float64(0)
                    point.header.stamp = msg.header.stamp
                    formed = do_transform_point(point, trans)
                    Points.append(
                        np.array([formed.point.x, formed.point.y, formed.point.z]))
                    
            occ_points = filter_points(np.asarray(occ_points), 0.5, 1)

            for point in occ_points:
                # only want to add occupied if range is smaller than 5 m because of ouliers
                x, y = self.map_to_grid(point[0], point[1])
                self.add_to_map(x, y, self.occupied_space)
                #free_points = self.raytrace((self.map_to_grid(
                #   robot_x_coord, robot_y_coord)), (x, y))
                #for free in free_points:
                #   x_f, y_f = free
                #   self.add_to_map(x_f, y_f, self.free_space)
#
            for point in Points:
                # Adding occupied space
                x, y = self.map_to_grid(point[0], point[1])
                # self.add_to_map(x, y, self.occupied_space) # Only raytrace points further than 5 m away
                # Adding free space
                free_points = self.raytrace((self.map_to_grid(
                    robot_x_coord, robot_y_coord)), (x, y))
                for free in free_points:
                    x_f, y_f = free
                    self.add_to_map(x_f, y_f, self.free_space)
            self.map.header.stamp = msg.header.stamp

            self.publisher.publish(self.map)

    def is_in_polygon(self, x, y):
        """ Only for Service
        Takes in coordinates in map coord system and checks if the points corresponds to inside the polygon"""

        x = x/self.resolution
        y = y/self.resolution
        point = Point(x, y)
        if self.poly1.contains(point):
            # print('Inside polygon')
            return True
        else:
            # print('Outside polygon')
            return False

    def add_to_map(self, x, y, value):
        """Takes in x,y in gridmap coord system then adds them to map
        First check what current value is in corresponding cell, if it is self.ws then ignore since it is outside the workspace
        Depending on what the current value of the cell is it will reset to free/occupied or decrese prob of being occupied """

        if self.is_in_bounds(x, y):

            # If in gridmap
            if self.map.data[self.grid_index(x, y)] != self.ws and self.map.data[self.grid_index(x, y)] != self.boundary and self.map.data[self.grid_index(x, y)] != self.camera_obs and self.map.data[self.grid_index(x, y)] != self.uninflated_space:
                if value != self.remove_camera_obs:
                    self.map.data[self.grid_index(x, y)] = value

            if self.map.data[self.grid_index(x, y)] != self.ws and self.map.data[self.grid_index(x, y)] != self.boundary:
                if self.map.data[self.grid_index(x, y)] == self.camera_obs or self.map.data[self.grid_index(x, y)] == self.uninflated_space:
                    if value == self.remove_camera_obs:
                        self.map.data[self.grid_index(x, y)] = self.free_space
                    if value == self.camera_obs or value == self.uninflated_space:
                        self.map.data[self.grid_index(x, y)] = value

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

    def servicecallback(self, request, response):
        """Takes request point in any frame (make sure it is in meters) and returns if in WS"""
        point = request.point
        trans = None
        if self.tfBuffer.can_transform('map', point.header.frame_id, point.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
            try:
                trans = self.tfBuffer.lookup_transform(
                    'map',
                    point.header.frame_id,
                    point.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
            except (ConnectivityException) as C:
                self.get_logger().error(str(C))
            except (LookupException) as L:
                self.get_logger().error(str(L))
            except (ExtrapolationException) as E:
                self.get_logger().error(str(E))
            except (TransformException) as T:
                self.get_logger().error(str(T))

        if trans is not None:
            #self.get_logger().info('Checking if point is inside workspace')

            formed = do_transform_point(point, trans)

            response.inside = self.is_in_polygon(
                formed.point.x, formed.point.y)
            #if response.inside == True:
            #    self.get_logger().info('Point is inside workspace')
            #else:
            #    self.get_logger().info('Not inside workspace')
            return response
        else:
            response.inside = False
            return response

    def servicecallback_add_obs(self, request, response):
        """Takes in a point in map frame and adds it to the map as occupied space
        All mesurments need to be in meters"""
        # CLEAR MAP OF ALL CAMERA AND UNINFATED OBS
        if not request.add:
            for shit in self.points_within_polygon:
                x = int((shit[0]) - self.offset/self.resolution)
                y = int((shit[1]) - self.offset/self.resolution)
                self.add_to_map(x, y, self.remove_camera_obs)
        else:

            radpoints = request.radpoints.points
            for radpoint in radpoints:
                point = radpoint.point  # pointstamped
                radius = radpoint.radius
                trans = None
                if self.tfBuffer.can_transform('map', point.header.frame_id, point.header.stamp, timeout=rclpy.duration.Duration(seconds=1)):
                    try:
                        trans = self.tfBuffer.lookup_transform(
                            'map',
                            point.header.frame_id,
                            point.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
                    except (ConnectivityException) as C:
                        self.get_logger().error(str(C))
                    except (LookupException) as L:
                        self.get_logger().error(str(L))
                    except (ExtrapolationException) as E:
                        self.get_logger().error(str(E))
                    except (TransformException) as T:
                        self.get_logger().error(str(T))

                    # For regular camera obstacles:
                if trans is not None and radpoint.type != 'target box' and radpoint.type != 'close to target':
                    #self.get_logger().info('Adding obstacle to map')
                    # Getting center point in map frame (m)
                    formed = do_transform_point(point, trans)
                    circle_points = self.generate_circle_points(
                        formed.point.x, formed.point.y, radius, 1000, 0, 2*np.pi)
                    for pnt in circle_points:
                        x, y = self.map_to_grid(pnt[0], pnt[1])

                        self.add_to_map(x, y, self.camera_obs)
                # If type is target box
                if trans is not None and radpoint.type == 'target box':
                    print('Got target box')
                    radius = 1.0
                    #self.get_logger().info('Adding obstacle to map')
                    # Getting center point in map frame (m)
                    formed = do_transform_point(point, trans)
                    circle_points1 = self.generate_circle_points(
                        formed.point.x, formed.point.y, radius, 1000, (radpoint.orientation + np.pi/14 - 2*np.pi), (radpoint.orientation - np.pi/14 ))
                    circle_points2 = self.generate_circle_points(
                        formed.point.x, formed.point.y, radius - self.resolution, 1000, (radpoint.orientation + np.pi/14 - 2*np.pi), (radpoint.orientation - np.pi/14))
                    start = self.map_to_grid(formed.point.x + radius * np.cos(radpoint.orientation +np.pi/14), formed.point.y + radius * np.sin(radpoint.orientation +np.pi/14))
                    end = self.map_to_grid(formed.point.x + (0.35) * np.cos(radpoint.orientation +np.pi/14), formed.point.y + 0.2* np.sin(radpoint.orientation +np.pi/14))
                    start1 = self.map_to_grid(formed.point.x + radius * np.cos(radpoint.orientation -np.pi/14), formed.point.y + radius * np.sin(radpoint.orientation -np.pi/14))
                    end1 = self.map_to_grid(formed.point.x + (0.35) * np.cos(radpoint.orientation -np.pi/14), formed.point.y + 0.2* np.sin(radpoint.orientation -np.pi/14))
                   
                    line_points1 = self.raytrace(start,end)
                    line_points2 = self.raytrace(start1,end1)

                    for pnt in line_points1:
                        x, y = pnt[0], pnt[1]
                        self.add_to_map(x, y, self.uninflated_space)

                    for pnt in line_points2:
                        x, y = pnt[0], pnt[1]
                        self.add_to_map(x, y, self.uninflated_space)
                    for pnt in circle_points1:
                        x, y = self.map_to_grid(pnt[0], pnt[1])
                        self.add_to_map(x, y, self.uninflated_space)
                    for pnt in circle_points2:
                        x, y = self.map_to_grid(pnt[0], pnt[1])
                        self.add_to_map(x, y, self.uninflated_space)

        response.map = self.map
        return response

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

    def point_inside_polygon(self, x, y):
        """
        NOT USED ANYMORE, TOO heavy computationally
        Check if a point (x, y) is inside a polygon defined by its vertices.
        takes in points in gridmap coordinates
        :param x: x-coordinate of the point
        :param y: y-coordinate of the point
        :param poly: List of tuples representing the vertices of the polygon [(x1, y1), (x2, y2), ...]
        :return: True if the point is inside the polygon, False otherwise
        """
        x = x*self.resolution+self.offset
        y = y*self.resolution+self.offset
        poly = self.points
        n = len(poly)
        inside = False
        [p1x, p1y] = poly[0]
        for i in range(n+1):
            p2x, p2y = poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / \
                                (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside


def main():
    rclpy.init()
    node = gridmap()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(5))
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
