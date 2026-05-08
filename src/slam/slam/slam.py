#!/usr/bin/env python

"""Author: Arian Kourangi """

import numpy as np

import rclpy
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from nav_msgs.msg import Path
import cv2 as cv
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformBroadcaster

from robp_interfaces.msg import Encoders


from typing import Iterable, Optional, Tuple


class SLAM(Node):

    def __init__(self):
        super().__init__('slam')
        # Create tf buffer
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10))
        # create tf listener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.trans = None
        # self.get_trans()

        self._path = Path()
        self._path_pub = self.create_publisher(Path, 'path', 10)

        callbackgroup = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            Encoders, '/motor/encoders', self.encoder_callback, 10, callback_group=callbackgroup)
        self.subscription = self.create_subscription(
            sensor_msgs.LaserScan, '/scan_filtered', self.laser_callback, 10, callback_group=callbackgroup)
        self.create_subscription(
            sensor_msgs.Imu, '/imu/data_raw', self.imu_callback, 10, callback_group=ReentrantCallbackGroup())

        # defining all variables used for SLAM

        self.mu = []  # [x, y, z, theta, v, landmark_x, landmark_y,landmark_nr ...]
        self.cov = []
        self.inf = 10 ** 10  # Used for initial cov of landmarks
        self.R = np.array(np.eye(4)).astype(float)  # Change motion model noise
        self.Q = np.array(np.eye(2)).astype(float)

        self.R[0, 0] = 0.1  # x std
        self.R[1, 1] = 0.1  # y std
        self.R[2, 2] = 0.01  # robot theta std
        self.R[3, 3] = 0.1  # velocity std

        self.Q[0, 0] = 0.01  # r std
        self.Q[1, 1] = 0.01  # theta std

        self.H_low = []
        self.H = []
        self.psi_inv = []
        self.pi = []
        self.HK = []
        self.innovation = []

        self.alpha = 0.5  # Mahalanobis threshold
        self.K = 0

        self.landmark_list = []
        self.zero = 0.0001
        self.z = [0, 0]
        self.i = 0
        self.orient = 0

        self.Initialize()

    def imu_callback(self, msg: sensor_msgs.Imu):
        if self.i == 0:
            [roll, pitch, self.yaw_zero] = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.i += 1
        else:
            [roll, pitch, yaw] = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.orient = -(yaw-self.yaw_zero)
        # self.omega = -msg.angular_velocity.z
        # print('updating omega')

    def Initialize(self):
        # initializing mu with only robot estimates
        self.mu.append(np.array([0] * 4).astype(float))
        # initializing cov with only robot covariances
        self.cov.append(0 * np.eye(4).astype(float))

    def encoder_callback(self, msg: Encoders):
        """Conitnously updates the state based on input from Encoders"""

        # time = imu.header.stamp
        # frame = imu.header.frame_id
        self.mu.append(np.copy(self.mu[-1]))
        self.cov.append(np.copy(self.cov[-1]))

        dt = 50 / 1000
        ticks_per_rev = 48 * 64
        wheel_radius = 0.098425/2
        base = 0.311  #

        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        wtr = 2*np.pi*delta_ticks_right / \
            (ticks_per_rev*dt)  # Right wheel angular vel
        wtl = 2*np.pi*delta_ticks_left / \
            (ticks_per_rev*dt)  # left wheel angular vel

        omega = wheel_radius * (wtr-wtl)/base  # robot angular vel
        vt = wheel_radius * (wtr+wtl)/2  # Robot linear vel

        # state prediction

        # Motion model
        self.mu[-1][3] = vt  # velocity
        # self.mu[-1][2] += omega*dt  # robot yaw/bearing
        self.mu[-1][2] = self.orient

        # normalizing angle
        self.mu[-1][2] = np.mod(self.mu[-1][2] + np.pi, 2 * np.pi) - np.pi

        # No z pos since we drive at constant altitude
        self.mu[-1][1] += self.mu[-1][3] * dt * np.sin(self.mu[-2][2])  # Y pos
        self.mu[-1][0] += self.mu[-1][3] * dt * np.cos(self.mu[-2][2])  # X pos

        # Jacobian of motion model
        G = np.array([[1, 0, -np.sin(self.mu[-2][2]) * self.mu[-1][3] * dt, 0],
                      [0, 1, np.cos(self.mu[-2][2]) * self.mu[-1][3] * dt, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        self.cov[-1][0:4, 0:4] = np.matmul(G, np.matmul(
            self.cov[-2][0:4, 0:4], np.transpose(G))) + self.R
        self.cov[-1][0:4, 4:] = np.matmul(G, self.cov[-2][0:4, 4:])
        self.cov[-1][4:, 0:4] = np.transpose(self.cov[-1][0:4, 4:])

        stamp = msg.header.stamp
        # self.publish_path(stamp, self.mu[-1][0], self.mu[-1][1],self.mu[-1][2])
        # self.broadcast_transform(stamp, self.mu[-1][0], self.mu[-1][1],self.mu[-1][2])
        # LAserSCAN and IMU are clashing and trying to update eachother before the other has finished so yes this doesn work

    def laser_callback(self, laser: sensor_msgs.LaserScan):
        """Takes laser message , offsets all points to fit in a 2000x2000 pixel grid, 
        then uses cv good features to extract all corners to be used as landmarks
        transforms the identified points from laser frame to map frame and passes them to update function"""
        self.get_trans()

        array = np.full((2000, 2000, 1), 255, dtype=np.uint8)
        nr_messages = len(laser.ranges)

        for i in range(nr_messages):
            # maximum 5 meters distance to scan
            if laser.ranges[i] <= 0.3 or laser.ranges[i] >= 10:
                continue
            else:
                total_angle = laser.angle_min + i*laser.angle_increment

                # convert to centimeter and add offset for image
                x_coord = round(np.cos(total_angle) *
                                laser.ranges[i]*100 + 1000)
                # convert to centimeter and add offset for image
                y_coord = round(np.sin(total_angle) *
                                laser.ranges[i]*100 + 1000)

                array[y_coord, x_coord] = 0

        corners = cv.goodFeaturesToTrack(array, 1, 0.9999, 5, blockSize=10)

        if corners is not None:
            """Can only do correction step if corners have been detected"""
            corners = np.intp(corners)
            point = PointStamped()
            points_list = []

            for i in corners:
                x, y = i.ravel()
                # convert back to meters and remove offset
                point.point.x = (x-1000)/100
                # convert back to meters and remove offset
                point.point.y = (y-1000)/100
                point.point.z = np.float64(0)
                # get pos of corners/landmarks in base_link frame
                formed = do_transform_point(point, self.trans)
                points_list.append(
                    [formed.point.x, formed.point.y, formed.point.z])

        # Have to make the extracted points into measruements
            meas = self.make_pseduo_meas(points_list)

            stamp = laser.header.stamp

            self.correction_step(meas, stamp)

    def make_pseduo_meas(self, points):
        """Takes the extracted edges as points and calculates the measurement that would give it in robot frame"""
        temp = []
        for point in points:
            r = np.sqrt(point[0]**2 + point[1]**2)

            # Maybe CHECH this one!! CHange to theta = np.sign(temp_y) * np.arccos(temp_x /np.sqrt((temp_x) ** 2 + (temp_y) ** 2))
            theta = np.arctan2(point[1], point[0])
            # normalizing angle
            theta = np.mod(theta + np.pi, 2 * np.pi) - np.pi

            temp.append([r, theta])
        return temp

    def correction_step(self, meas, stamp):
        self.pi = []
        self.psi_inv = []

        for j in meas:  # j = current meas [r,theta],
            self.add_observation_to_map(j)
            # These must be reset for every measurment
            self.pi = []
            self.psi_inv = []
            self.HK = []
            self.innovation = []

            # add new landmark to cov
            self.cov[-1] = np.concatenate((self.cov[-1],
                                          np.zeros((len(self.mu[-1]) - 2, 2))), axis=1)

            temp = np.concatenate(
                (np.zeros((2, len(self.mu[-1]) - 2)), self.inf * np.eye(2)), axis=1)
            self.cov[-1] = np.concatenate((self.cov[-1], temp), axis=0)

            # k =  landmark in map
            for k in range(round((len(self.mu[-1]) - 4) / 2)):
                self.measurment_prediction(j, k)

            self.data_association(stamp)

    def add_observation_to_map(self, landmark):
        """Takes in single measurment and adds the landmark to the state vector"""

        r = landmark[0]
        theta = landmark[1] + self.mu[-1][2]  # adding robot yaw to

        # normalizing angles
        theta = np.mod(theta + np.pi, 2 * np.pi) - np.pi

        pos_x = r * np.cos(theta) + self.mu[-1][0]
        pos_y = r * np.sin(theta) + self.mu[-1][1]

        # add landmark to mu
        self.mu[-1] = np.append(self.mu[-1], pos_x)
        self.mu[-1] = np.append(self.mu[-1], pos_y)

    def measurment_prediction(self, j, landmark_number):
        """Takes in current measuremtn j and landmark number we are predicting against
        j = current meas [r,theta]"""

        # get current landmark we are predicting against
        temp_x = self.mu[-1][4 + landmark_number * 2] - self.mu[-1][0]
        temp_y = self.mu[-1][5 + landmark_number * 2] - self.mu[-1][1]

        # measurment model
        r = np.sqrt((temp_x) ** 2 + (temp_y) ** 2)

        if temp_x == 0 and temp_y == 0:
            theta = 0
            print("SOMETHING WENT WRONG! 2")
        else:
            theta = np.sign(temp_y) * np.arccos(temp_x /
                                                np.sqrt((temp_x) ** 2 + (temp_y) ** 2)) - self.mu[-1][2]

        # normalizing angles
        theta = np.mod(theta + np.pi, 2 * np.pi) - np.pi

        self.z = [r, theta]

        # jacobian of measurement model
        temp_r = r

        sqrt_x2_y2 = np.sqrt(temp_x ** 2 + temp_y ** 2)

        self.H_low = np.array([[-1 * temp_x / temp_r, -1 * temp_y / temp_r, 0, 0, temp_x / temp_r,
                                temp_y / temp_r],
                               [temp_y / sqrt_x2_y2 ** 2, -1 * temp_x / sqrt_x2_y2 ** 2, -1, 0,
                                -1 * temp_y / sqrt_x2_y2 ** 2, temp_x / sqrt_x2_y2 ** 2]])

        self.H = np.zeros((4 + 2, len(self.mu[-1])))

        self.H[0:4, 0:4] = np.eye(4)
        self.H[4:6, 4 + 2 * landmark_number:6 +
               2 * landmark_number] = np.eye(2)

        # self.H_low = np.concatenate((self.H_low, np.array([[0.0] * 8])), axis=0)
        # self.H_low = np.concatenate((self.H_low, np.array([[0.0], [0.0], [0.0], [1.0]])), axis=1)

        self.H = np.matmul(self.H_low, self.H)

        self.HK.append(self.H)

        self.psi_inv.append(np.linalg.inv(
            np.matmul(
                self.H, (
                    np.matmul(
                        self.cov[-1], (
                            np.transpose(
                                self.H
                            )
                        )
                    )
                )
            )
            + self.Q
        ))

        # innovation and normalzing angles
        innovation = np.subtract(j, self.z)
        innovation[1] = np.mod(innovation[1] + np.pi, 2 * np.pi) - np.pi

        self.pi.append(np.matmul(
            np.transpose(innovation), (
                np.matmul(
                    self.psi_inv[-1], (
                        innovation
                    )
                )
            )
        ))
        self.innovation.append(innovation)

    def data_association(self, stamp):
        self.pi[-1] = self.alpha
        min_pi = min(self.pi)
        m = self.pi.index(
            min_pi) + 1  # python index ruins everything, m is the index of associated landmark and number of landmarks

        len_mu = round(max((len(self.mu[-1]) - 4) / 2 - 1, m))

        self.mu[-1] = self.mu[-1][:(len_mu * 2 + 4)]
        self.cov[-1] = self.cov[-1][:(len_mu * 2 + 4), :(len_mu * 2 + 4)]
        m = m - 1  # bring back to python indexing

        # print('cov, rows,cols', len(self.cov[-1]), len(self.cov[-1][0]))
        # print('H, rows,cols', len(self.HK[m]), len(self.HK[m][0]))
        # print('psi,rows,cols', len(self.psi_inv[m]), len(self.psi_inv[m][0]))

        self.K = np.matmul(self.cov[-1],
                           np.matmul(np.transpose(self.HK[m][:(len_mu * 2 + 4), :(len_mu * 2 + 4)]), self.psi_inv[m]))

        self.mu[-1] = self.mu[-1] + np.matmul(self.K, self.innovation[m])
        self.cov[-1] = self.cov[-1] - np.matmul(self.K, np.matmul(self.HK[m][:(len_mu * 2 + 4), :(len_mu * 2 + 4)],
                                                                  self.cov[-1]))
        """Find difference between est and true. FIrst trying with diff between odom-> base_link and true """
        trans = None
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                stamp, timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time() Instead use the message time. Always use the time form the message
        except (ConnectivityException) as c:
            print(c)
        except (LookupException) as L:
            print(L)
        except (ExtrapolationException) as E:
            print(E)
        except (TransformException) as T:
            print(T)
        if trans is not None:
            diff_x = -(self.mu[-1][0] - trans.transform.translation.x)
            diff_y = -(self.mu[-1][1] - trans.transform.translation.y)
            eulers = euler_from_quaternion(
                [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            diff_yaw = self.mu[-1][2] - eulers[2]
            diff_yaw = np.mod(diff_yaw + np.pi, 2 * np.pi) - np.pi

            # self.publish_path(stamp, self.mu[-1][0], self.mu[-1][1],self.mu[-1][2])
            self.broadcast_transform(
                stamp, diff_x, diff_y, diff_yaw, 'map', 'odom')
            print('updating')
            # self.broadcast_transform(stamp, self.mu[-1][0],self.mu[-1][1] ,self.mu[-1][2],'odom','base_link')

    def get_trans(self):
        """Gets trans from laer to base_link only once when node is initialized"""

        source_frame = 'laser'
        target_frame = 'base_link'
        if self.tf_buffer.wait_for_transform_async(target_frame, source_frame, rclpy.time.Time()):
            try:
                self.trans = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))  # this is getting the latest frame, rclpy.time.Time()

            except (ConnectivityException) as c:
                print(c)
            except (LookupException) as L:
                print(L)
            except (ExtrapolationException) as E:
                print(E)
            except (TransformException) as T:
                print(T)

    def broadcast_transform(self, stamp, x, y, yaw, parent_frame, child_frame):
        """Takes a 2D pose and broadcasts it as a ROS transform.

        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = SLAM()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
