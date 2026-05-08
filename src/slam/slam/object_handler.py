#!/usr/bin/env python

"""Author: Arian Kourangi """


import math
from tempfile import tempdir

import numpy as np
import time

from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from arian_interfaces.msg import PointWithRadius
from arian_interfaces.msg import PointWithRadiusArray
from arian_interfaces.srv import AddCamObs
from arian_interfaces.srv import GetList
from arian_interfaces.srv import IsInWs
from .hash import *
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import copy

COUNT_THRESHOLD = 2
# subscribe to global path topic, make sure that global path planner still publishes to it. Take in path and store it. If detect new object, compare and see is it within 0.5 m of any
# point in the path. If it is, raise flag on seperate topic. This topic will be listened to on in the BT behavoir for PP, if it gets a flag it will stop the robot. And the local and or global planning will start over
# Still pblish i object id is same as curr target id publish to est pose


class ObjectHandler(Node):
    def __init__(self):
        super().__init__('object_handler')
        self.get_logger().info("Object handler node started")
        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=1000))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.list = {}
        self.start_list()
        self.current_target = None
        self.image = None
        self.path = None
        cbr = ReentrantCallbackGroup()
        self.isinside = False
        self.bridge = cv_bridge.CvBridge()
        cbr2 = MutuallyExclusiveCallbackGroup()
        ####### Service client for IsInWs   ########    
        self.cli = self.create_client(
            IsInWs, 'is_in_ws', callback_group=MutuallyExclusiveCallbackGroup())
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IsInWs.Request()
        self.sub_sleep = self.create_subscription(std_msgs.Bool, '/sleep', self.sleep_callback, 10 ,callback_group=ReentrantCallbackGroup())

        self.publisher = self.create_publisher(
            PointWithRadius, '/estimated_pose', 10)
        self.publisher_flag = self.create_publisher(
            std_msgs.Bool, '/flag', 10)
        self.speaker = self.create_publisher(std_msgs.String, '/speaker', 10)
        self.subscription1 = self.create_subscription(
            PointWithRadius, '/current_target', self.callback_target, 10, callback_group=cbr)

        self.subscription2 = self.create_subscription(
            Path, '/global_path', self.callback_path, 10, callback_group=cbr)

        self.add_cam_obs_srv = self.create_service(
            AddCamObs, '/add_cam_obs', self.add_cam_obs_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_list_srv = self.create_service(
            GetList, '/get_list', self.get_list_callback, callback_group=cbr)
        self.remove_cam_obs_srv = self.create_service(
            AddCamObs, '/remove_target', self.remove_cam_obs_callback, callback_group=MutuallyExclusiveCallbackGroup())

    def callback_image(self, msg: Image):
        self.image = msg

    def callback_path(self, msg: Path):
        self.path = msg

    def start_list(self):
        list = ['red cube', 'red ball', 'blue cube', 'blue ball', 'green cube',
                'green ball', 'wooden cube', 'kiki', 'hugo', 'oakie', 'slush', 'muddles', 'binky', 'box']
        for shit in list:
            self.list[shit] = {}

    def send_req(self, point):
        self.req.point = point
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        self.future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        # print(future.result().inside)
        self.isinside = future.result().inside
        return future.result().inside

    def callback_target(self, msg: PointWithRadius):
        self.current_target = msg

    def sleep_callback(self, msg: std_msgs.Bool):
        if msg.data == True:
            time.sleep(0.3)
        
    def add_cam_obs_callback(self, request, response):
        self.get_logger().info("Adding camera observation")
        """Takes in a point, transforms it to map frame, checks if inws, and adds it to the list of obstacles and/or targets.
        All mesurments need to be in meters"""
        for radpoint in request.radpoints.points:
            orig_point = radpoint.point  # pointstamped
            self.isinside = None
            self.send_req(orig_point)
            found = False

            while self.isinside is None:
                pass
                # self.get_logger().info("Waiting for response")

            if self.isinside:
                # self.get_logger().info("Point is inside workspace")

                trans = None
                if self.tfBuffer.can_transform('map', orig_point.header.frame_id, orig_point.header.stamp, timeout=rclpy.duration.Duration(seconds=4)):
                    try:
                        trans = self.tfBuffer.lookup_transform(
                            'map',
                            orig_point.header.frame_id,
                            orig_point.header.stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
                    except (ConnectivityException) as C:
                        self.get_logger().error(str(C))
                    except (LookupException) as L:
                        self.get_logger().error(str(L))
                    except (ExtrapolationException) as E:
                        self.get_logger().error(str(E))
                    except (TransformException) as T:
                        self.get_logger().error(str(T))
                if trans is not None:
                    formed_point = do_transform_point(orig_point, trans)
                    radpoint.point = formed_point
                    radpoint.type = radpoint.type.lower()

                    """
                    if the current observed object is the same as the target, then publish its pose to estimated pose. this
                    should work for both detection and aruco marker. Detection will always leave marker id none for objects, whilst only aruco node will give them a marker_id to be used
                    when target is a aruco marker object
                    """
                    found = False
                    try:
                        if self.list[radpoint.type] == {}:
                            radpoint.id = radpoint.type + str(0)
                            radpoint.count = 1
                            self.list[radpoint.type][radpoint.id] = radpoint
                            #self.get_logger().info("Adding new object")
                            message = std_msgs.String()

                            message.data = f'Holy Moly I just saw a new {radpoint.type}'
                            self.speaker.publish(message)
                            self.save_image(radpoint)
                        else:
                            for k in self.list[radpoint.type]:
                                if self.check_dist(self.list[radpoint.type][k].point.point.x, radpoint.point.point.x, self.list[radpoint.type][k].point.point.y, radpoint.point.point.y) < 0.5:
                                    found = True
                                    radpoint.id = k
                                    self.get_logger().info('Object already in list')
                                    if radpoint.target: #if target (aruco or item), this will let aruco overwrite non target box
                                        self.list[radpoint.type][k].point = radpoint.point
                                        self.list[radpoint.type][k].count += 1
                                        self.list[radpoint.type][k].marker_id = radpoint.marker_id
                                        self.list[radpoint.type][k].target = radpoint.target
                                        self.list[radpoint.type][k].orientation = radpoint.orientation

                                    elif not radpoint.target: #If we see a box without aruco
                                        if self.list[radpoint.type][k].target: #If the current stored box is a target (aruco) increase count
                                            self.list[radpoint.type][k].count += 1
                                        elif not self.list[radpoint.type][k].target: #if the current stored target is not a target (aruco) update the pose, count and everything else
                                            self.list[radpoint.type][k].point = radpoint.point
                                            self.list[radpoint.type][k].count += 1
                                            self.list[radpoint.type][k].marker_id = radpoint.marker_id
                                            self.list[radpoint.type][k].target = radpoint.target

                            if not found:
                                radpoint.id = radpoint.type + \
                                    str(len(self.list[radpoint.type]))
                                radpoint.count = 1
                                self.list[radpoint.type][radpoint.id] = radpoint
                                message = std_msgs.String()
                                message.data = f'Holy Moly I just saw a new {radpoint.type}'
                                self.speaker.publish(message)
                                #self.get_logger().info("Adding new object")
                                self.save_image(radpoint)
                    except (KeyError):
                        self.get_logger().info('Not an acceptabel type of object')

                # Publish untransformed point if the radpoints id is same as target
                if self.current_target is not None:
                    # and radpoint.marker_id == self.current_target.marker_id:
                    if radpoint.id == self.current_target.id:# and radpoint.marker_id == self.current_target.marker_id:
                        temp = copy.deepcopy(radpoint)
                        temp.point = orig_point
                        temp.marker_id = self.current_target.marker_id
                        self.publisher.publish(temp)
                        self.get_logger().info(temp.id)
                # if path msg is not none and we see a New object
                if self.path is not None and not found:
                    for path in self.path.poses:
                        if self.check_dist(path.pose.position.x, radpoint.point.point.x, path.pose.position.y, radpoint.point.point.y) < 0.8:
                            flag = std_msgs.Bool()
                            flag.data = True
                            self.publisher_flag.publish(flag)
                            break
                # IF we have an image stored and we see a new object

            else:
                self.get_logger().info("Point is outside workspace")
        response.done = True
        return response

    def get_list_callback(self, request, response):
        """Takes in empty req and returns a list of all objects"""
        if request.type == 'obstacle':
            temp_list = []
            for k in self.list:
                for i in self.list[k]:
                    if self.list[k][i].count >= 2:
                        temp_list.append(self.list[k][i])
                response.list.points = temp_list
        elif request.type == 'target':
            temp_list = []
            for k in self.list:
                for i in self.list[k]:
                    if self.list[k][i].target and self.list[k][i].type == 'box':
                        temp_list.append(self.list[k][i])
                    elif self.list[k][i].target and self.list[k][i].type != 'box' and self.list[k][i].count >=COUNT_THRESHOLD:
                        likely_target = self.get_most_count(self.list[k][i])
                        if likely_target not in temp_list:
                            temp_list.append(likely_target)

            response.list.points = temp_list
        return response

    def get_most_count(self,object):
        temp = self.list
        best_count = object.count
        likely_object = object
        for k in temp:
            for i in temp[k]:
                if self.check_dist(temp[k][i].point.point.x, object.point.point.x, temp[k][i].point.point.y, object.point.point.y) < 0.1:
                    if temp[k][i].count > best_count:
                        best_count = temp[k][i].count
                        likely_object = temp[k][i]
        return likely_object
                    
    def remove_cam_obs_callback(self, request, response):
        self.get_logger().info("Removing camera observation")
        """Takes in a point and removes it from the list of obstacles"""
        temp = copy.deepcopy(self.list)
        temp_list = []
        for radpoint in request.radpoints.points:
            try:
                del temp[radpoint.type][radpoint.id]
                for k in temp:
                    for i in temp[k]:
                        if self.check_dist(temp[k][i].point.point.x, radpoint.point.point.x, temp[k][i].point.point.y, radpoint.point.point.y) < 0.1: #Delete everything within 5 cm
                            temp_list.append(temp[k][i])
                for shit in temp_list:
                    del temp[shit.type][shit.id]
                self.list = temp
                response.done = True
            except KeyError:
                self.get_logger().info("Object not in list")
                response.done = False
        return response

    def check_dist(self, x_old, x_new, y_old, y_new):
        dist = np.sqrt((x_old - x_new)**2 + (y_old - y_new)**2)
        return dist

    def save_image(self, radpoint):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(radpoint.image, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            cv2.imwrite(f"{radpoint.id}.jpg", cv2_img)


def main():
    rclpy.init()
    node = ObjectHandler()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
