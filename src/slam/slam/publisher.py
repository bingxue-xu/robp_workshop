#!/usr/bin/env python

"""Author: Arian Kourangi """

import math

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, OccupancyGrid

from arian_interfaces.srv import IsInWs
from arian_interfaces.msg import PointWithRadius
from arian_interfaces.srv import AddCamObs
from arian_interfaces.srv import GetList
from arian_interfaces.srv import AddObsMap
import random
import time


class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(
            Path, '/waypoints_global', 10)
        self.publisher1 = self.create_publisher(
            OccupancyGrid, 'local_map_test', 10)
        # self.timer = self.create_timer(1, self.timer_callback)

        ####### Service client for IsInWs   ########
        self.cli = self.create_client(IsInWs, 'is_in_ws')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IsInWs.Request()

        # Service client for AddObs   ########
        self.cli_addobs = self.create_client(AddObsMap, '/add_obs_global')
        while not self.cli_addobs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req1 = AddObsMap.Request()

        ####### Service client for AddCamObs   ########
        self.cli_addcamobs = self.create_client(AddCamObs, '/add_cam_obs')
        while not self.cli_addcamobs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = AddCamObs.Request()

        ######## Service client for GetList   ########
        self.cli_getlist = self.create_client(GetList, '/get_list')
        while not self.cli_getlist.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req3 = GetList.Request()
        self.list = None

        ######## Service client for Remove camera Obs   ########
        self.cli_removecamobs = self.create_client(AddCamObs, '/remove_target')
        while not self.cli_removecamobs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req4 = AddCamObs.Request()

        self.cli_add_obs_local = self.create_client(
            AddObsMap, '/add_obs_local')
        while not self.cli_add_obs_local.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req5 = AddObsMap.Request()
        self.main()

    def main(self):
        self.send_req()

        time.sleep(2)
        list = ['blue cube', 'red cube']
        temp_list = []
        for i in list:
            temp_list.append(self.make_pseudo_meas(
                i, np.float64(list.index(i)), np.float64(list.index(i))))
        self.send_req_cam_obs(temp_list)

        time.sleep(2)
        self.send_req_get_list()

        # self.list.points[0].type = 'target box'
        # self.list.points[0].orientation = np.pi/2
        self.send_req_add_obs_global(True)
        self.send_req_add_obs_local(True)
        time.sleep(5)
        self.send_req_add_obs_global(False)
        time.sleep(2)
        list = ['blue cube', 'red cube']
        temp_list = []
        for i in list:
            temp_list.append(self.make_pseudo_meas(
                i, np.float64(list.index(i)+0.4), np.float64(list.index(i)+0.4)))

        self.send_req_cam_obs(temp_list)
        time.sleep(2)
        self.send_req_remove_cam_obs(self.list.points[0])
        self.send_req_get_list()
        time.sleep(2)
        self.send_req_add_obs_global(True)
        self.send_req_add_obs_local(True)

    def make_pseudo_meas(self, type, x, y):
        Point = PointStamped()
        Point.header.frame_id = 'map'
        Point.header.stamp = self.get_clock().now().to_msg()
        Point.point.x = x
        Point.point.y = y
        Point.point.z = 0.0
        radpoint = PointWithRadius()
        radpoint.point = Point
        radpoint.radius = 0.8
        radpoint.target = True
        radpoint.type = type
        return radpoint

    def send_req_add_obs_local(self, add):
        self.req5.add = add
        self.req5.radpoints = self.list
        self.future = self.cli_add_obs_local.call_async(self.req5)
        rclpy.spin_until_future_complete(self, self.future)
        self.publisher1.publish(self.future.result().map)

    def send_req_remove_cam_obs(self, radpoint):
        list = [radpoint]
        self.req4.radpoints.points = list
        self.future = self.cli_removecamobs.call_async(self.req4)
        rclpy.spin_until_future_complete(self, self.future)
        print('Removed point from obs list ', self.future.result().done)

    def send_req_get_list(self):
        self.req3.type = 'target'
        self.future = self.cli_getlist.call_async(self.req3)
        rclpy.spin_until_future_complete(self, self.future)
        self.list = self.future.result().list
        # print('List of obstacles = ', self.future.result().list)

    def send_req_cam_obs(self, list):
        self.req2.radpoints.points = list
        self.future = self.cli_addcamobs.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        print('Added point to obs list ', self.future.result().done)

    def send_req(self):
        Point = PointStamped()
        Point.header.frame_id = 'odom'
        Point.header.stamp = self.get_clock().now().to_msg()
        Point.point.x = -3.8
        Point.point.y = 7.0
        Point.point.z = 0.0
        self.req.point = Point
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print('Inside ws = ', self.future.result().inside)

    def send_req_add_obs_global(self, add):

        self.req1.radpoints = self.list
        self.req1.add = add
        self.future = self.cli_addobs.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future)
        print('Added point to map ')  # , self.future.result().map)

    def timer_callback(self):

        pose1 = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()

        pose1.pose.position.x = np.float64(0.2)
        pose1.pose.position.y = np.float64(0.2)
        pose1.pose.position.z = np.float64(0)
        pose1.pose.orientation.x = np.float64(0)
        pose1.pose.orientation.y = np.float64(0)
        pose1.pose.orientation.z = np.float64(0)
        pose1.pose.orientation.w = np.float64(1)

        pose2.pose.position.x = np.float64(0.5)
        pose2.pose.position.y = np.float64(0.5)
        pose2.pose.position.z = np.float64(0)
        pose2.pose.orientation.x = np.float64(0)
        pose2.pose.orientation.y = np.float64(0)
        pose2.pose.orientation.z = np.float64(0)
        pose2.pose.orientation.w = np.float64(1)

        pose3.pose.position.x = np.float64(1)
        pose3.pose.position.y = np.float64(1)
        pose3.pose.position.z = np.float64(0)
        pose3.pose.orientation.x = np.float64(0)
        pose3.pose.orientation.y = np.float64(0)
        pose3.pose.orientation.z = np.float64(0)
        pose3.pose.orientation.w = np.float64(1)

        poses = Path()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = 'odom'
        poses.poses.append(pose1)
        poses.poses.append(pose2)
        poses.poses.append(pose3)
        self.publisher.publish(poses)


def main():
    rclpy.init()
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
