from typing import Any
import py_trees as pt
import py_trees_ros as ptr
from py_trees.common import Status
import rclpy
from bing_interfaces.srv import GlobalPlanning
from arian_interfaces.srv import EstPose
import arian_interfaces
from arian_interfaces.srv import IsInWs, GetList, AddObsMap, AddCamObs, MoveArm
from arian_interfaces.msg import PointWithRadius, PointWithRadiusArray
from example_interfaces.srv import Trigger
from gustav_custom_interfaces.action import DriveTo
from gustav_custom_interfaces.srv import Explore, VisualGrid
from py_trees.blackboard import Blackboard
from action_msgs.msg import GoalStatus
import rclpy.action
import typing
import rclpy
from rclpy.node import Node
import py_trees
from rclpy.executors import SingleThreadedExecutor
from rclpy import exceptions
from rclpy.task import Future
from geometry_msgs.msg import Twist
import time
import numpy as np
import std_msgs.msg as std_msgs
from py_trees.decorators import Decorator

from geometry_msgs.msg import PointStamped,PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

# Did global planning, does both add obs global, global planning and remove obs global all in one  behavior. Might be better to devide up in several
# Need to do local planning as well
# Also need to do exploring

class CheckExploreFlag(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckExploreFlag, self).__init__(name=name)
        self.flag = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

    def initialise(self):
        self.flag = Blackboard().get('explore_flag')
        self.node.get_logger().info('initialising in CheckExploreFlag...')

    def update(self):
        if self.flag:
            self.node.get_logger().info('Flag is true')

            return pt.common.Status.SUCCESS
        else:
            self.node.get_logger().info('Flag is false')

            return pt.common.Status.FAILURE


class CheckClaws(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super(CheckClaws, self).__init__(name=name)
        self.future = None
        self.object = None
        self.pub = None
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_move = self.node.create_client(Trigger,'/claw_detection')
        self.pub = self.node.create_publisher(std_msgs.String,'/speaker',10)

        while not self.cli_move.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for claw detect service...')

    def initialise(self):
        req = Trigger.Request()
        self.object = Blackboard().get('curr_target')

        self.future = self.cli_move.call_async(req)

    def update(self):
        #self.logger.debug("%s.update()" % self.__class__.__name__)
        self.node.get_logger().info('updating in Claw Detect...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info('OBJECT DETECTED IN CLAW')
                    Blackboard().set('object_in_hand', self.object)
                    self.node.get_logger().info(
                        f'Picked up {self.object.id}')
                    message = std_msgs.String()

                    message.data = f'Roger that, I just Picked up a {self.object.type}'
                    self.pub.publish(message)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('NO OBJECT IN CLAW')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'CLAW DETECT service failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)

class MoveArm(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, closed,reset):
        super(MoveArm, self).__init__(name=name)
        self.future = None
        self.closed = closed
        self.reset = reset
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_move = self.node.create_client(arian_interfaces.srv.MoveArm,'/move_arm')

        while not self.cli_move.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for move arm service...')

    def initialise(self):
        req = arian_interfaces.srv.MoveArm.Request()
        req.closed = self.closed
        req.reset = self.reset
        self.future = self.cli_move.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #self.node.get_logger().info('updating in MoveArm...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info('Moved Arm')

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('Did not move arm')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'Move Arm service failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class CheckExplored(py_trees.behaviour.Behaviour):
    def __init__(self, name, threshold):
        super(CheckExplored, self).__init__(name=name)
        self.explored_percentage = 0.0
        self.threshold = threshold

    def initialise(self):
        self.explored_percentage = Blackboard().get('explored_percentage')

    def update(self):
        if self.explored_percentage >= self.threshold:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CallUsbCam(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CallUsbCam, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.cli_usb = self.node.create_client(Trigger, '/usb_cam_service')

        while not self.cli_usb.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for usb service...')
    def initialise(self):
        req = Trigger.Request()
        self.future = self.cli_usb.call_async(req)
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #self.node.get_logger().info('updating in CallUsbCam...')
        if self.future is None:
            return pt.common.Status.RUNNING
        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info('Usb Cam saw object!')

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(
                        'USB cam did not see target')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'Usb Cam service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING



    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)

class CheckObjecInHand(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObjecInHand, self).__init__(name=name)
        self.object_in_hand = None
    def setup(self, **kwargs):
        self.node = kwargs['node']
    def initialise(self):
        self.object_in_hand = Blackboard().get('object_in_hand')
        self.node.get_logger().info('initialising in CheckObjectInHand...')
    def update(self):
        if self.object_in_hand is not None:
            self.node.get_logger().info('SUCCESS! IN OBJECTINHAND')

            return pt.common.Status.SUCCESS
        
        else:
            self.node.get_logger().info('FAILURE IN OBJECTINHAND')

            return pt.common.Status.FAILURE


class UpdateRobotPosition(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super(UpdateRobotPosition, self).__init__(name=name)
        self.tf_buffer = None
        self.tf_listener = None
        self.robot_frame = 'base_link'
        self.global_frame = 'map'
        self.robot_position = None, None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.tfBuffer = Buffer()
        self.node.tfListener = TransformListener(self.node.tfBuffer, self.node)
        self.tf_listener = self.node.tfListener
        self.tf_buffer = self.node.tfBuffer

    def update(self):
        position = self.get_robot_position()
        if position is None:
            self.node.get_logger().info(f"Couldn't get robot position")
            return pt.common.Status.RUNNING
        else:
            Blackboard().set("robot_position", position)
            self.node.get_logger().info(f"update robot_position: {position}")
            return py_trees.common.Status.SUCCESS

    def get_robot_position(self):
        if self.tf_buffer.can_transform(self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
                self.node.get_logger().info(
                    f' get robot position : {t.transform.translation.x}, {t.transform.translation.y}')
                return t.transform.translation.x, t.transform.translation.y
            except (TransformException, ConnectivityException, LookupException, ExtrapolationException) as ex:
                self.node.get_logger().info(
                    f"Couldn't transform robot position due to: {ex}")
                return py_trees.common.Status.FAILURE
        return None, None


class CheckDistToTarget(pt.behaviour.Behaviour):
    """Calls object handler to get the latest list of objects and stores it in the blackboard
    Should be called everytime we want to do gloabl or local planning"""

    def __init__(self, name: str, threshold:float):
        super(CheckDistToTarget, self).__init__(name=name)
        self.tf_buffer = None
        self.tf_listener = None
        self.robot_frame = 'base_link'
        self.global_frame = 'map'
        self.robot_position = None, None
        self.est_pose = None
        self.future = None
        self.waypoints = []
        self.threshold = threshold

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.tfBuffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1000))
        self.node.tfListener = TransformListener(self.node.tfBuffer, self.node)
        self.tf_listener = self.node.tfListener
        self.tf_buffer = self.node.tfBuffer
        self.cli_est_pose = self.node.create_client(EstPose, '/estimated_pose_service')

        while not self.cli_est_pose.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for est pose service...')

    def initialise(self):
        req = EstPose.Request()
        self.future = self.cli_est_pose.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.node.get_logger().info('updating in Check Dist to target...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.est_pose.point.point.x != 9999.9:
                    self.node.get_logger().info('Got EST POSE!')
                    self.node.get_logger().info(str(response.est_pose.point.point.x))
                    if self.check_dist(response.est_pose.point) < self.threshold:
                        self.node.get_logger().info(
                            'Robot is close to target')
                        t = self.tf_buffer.lookup_transform_full(
                            self.global_frame,rclpy.time.Time(), response.est_pose.point.header.frame_id, response.est_pose.point.header.stamp,'map', timeout=rclpy.duration.Duration(seconds=5.0))
                        if t is not None:
                            formed = do_transform_point(response.est_pose.point, t)
                            temp = PoseStamped()
                            temp.header = formed.header
                            temp.pose.position = formed.point
                            self.waypoints.append(temp)
                            Blackboard().set('global_waypoints', self.waypoints)
    
                            return pt.common.Status.SUCCESS
                        else:
                            temp = PoseStamped()
                            target = Blackboard().get('curr_target')
                            temp.header = target.point.header
                            temp.pose.position = target.point.point
                            self.waypoints.append(temp)
                            Blackboard().set('global_waypoints', self.waypoints)

                            return pt.common.Status.SUCCESS
                    else:
                        self.node.get_logger().info(
                            'Robot is not close to target')
                        try:
                            t = self.tf_buffer.lookup_transform(
                                self.global_frame, response.est_pose.point.header.frame_id, response.est_pose.point.header.stamp, timeout=rclpy.duration.Duration(seconds=5.0))
                        except (ConnectivityException, LookupException,ExtrapolationException, TransformException) as C:
                            self.node.get_logger().error(str(C))
                            temp = PoseStamped()
                            target = Blackboard().get('curr_target')
                            temp.header = target.point.header
                            temp.pose.position = target.point.point
                            self.waypoints.append(temp)

                            Blackboard().set('global_waypoints', self.waypoints)
                            return pt.common.Status.FAILURE
      
                        if t is not None:
                            formed = do_transform_point(response.est_pose.point, t)
                            temp = PoseStamped()
                            temp.header = formed.header
                            temp.pose.position = formed.point
                            self.waypoints.append(temp)
                            self.node.get_logger().info('Transformed point')

                            Blackboard().set('global_waypoints', self.waypoints)
                            return pt.common.Status.FAILURE
                        # Updating the current target on the blackboard
                        else:
                            temp = PoseStamped()
                            target = Blackboard().get('curr_target')
                            temp.header = target.point.header
                            temp.pose.position = target.point.point
                            self.waypoints.append(temp)
                            self.node.get_logger().info('Could not transform point, using curr target')
                            Blackboard().set('global_waypoints', self.waypoints)
                            return pt.common.Status.FAILURE
                else:
                    self.node.get_logger().info('No EST POSE!')
                    temp = PoseStamped()
                    target = Blackboard().get('curr_target')
                    temp.header = target.point.header
                    temp.pose.position = target.point.point
                    self.waypoints.append(temp)
                    self.node.get_logger().info('Could not transform point, using curr target')
                    Blackboard().set('global_waypoints', self.waypoints)
                    return pt.common.Status.FAILURE   
            except Exception as e:
                self.node.get_logger().error(
                    f'Est Pose service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)

    def check_dist(self, est_pose):

        if self.tf_buffer.can_transform_full(self.robot_frame, rclpy.time.Time(), est_pose.header.frame_id, est_pose.header.stamp, self.global_frame, timeout=rclpy.duration.Duration(seconds=4.0)):
            try:
                t = self.tf_buffer.lookup_transform_full(self.robot_frame, rclpy.time.Time(
                ), est_pose.header.frame_id, est_pose.header.stamp, self.global_frame, timeout=rclpy.duration.Duration(seconds=4.0))
                formed = do_transform_point(est_pose, t)
                return np.sqrt(formed.point.x**2 + formed.point.y**2)
            except (TransformException, ConnectivityException, LookupException, ExtrapolationException) as ex:
                self.node.get_logger().info(
                    f"Couldn't transform robot position due to: {ex}")
                return py_trees.common.Status.FAILURE
        self.node.get_logger().info('Could not transform into robot_frame')
        return 10.0

class Check_Waypoints(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Check_Waypoints, self).__init__(name=name)
        self.waypoints = None

    def initialise(self):
        self.waypoints = Blackboard().get('global_waypoints')

    def update(self):
        if len(self.waypoints) == 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super(Wait, self).__init__(name=name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS


class PublishMessage(py_trees.behaviour.Behaviour):
    def __init__(self, name, turnvalue, forwardvalue, topic):
        super(PublishMessage, self).__init__(name=name)
        self.pub = None
        if topic == '/cmd_vel':
            self.message = Twist()
            self.message.angular.z = turnvalue
            self.message.linear.x = forwardvalue
            self.topic = topic
        elif topic == '/map_done':
            self.message = std_msgs.Bool()
            self.message.data = True
            self.topic = topic
        elif topic == '/speaker':
            self.message = std_msgs.String()
            self.message.data = 'WOOHOOO IM DONE BABY. REMEMBER KIDS, RECYCLING ROCKS'
            self.topic = topic
            
    def setup(self, **kwargs):
        self.node = kwargs['node']
        if self.topic == '/cmd_vel':
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        elif self.topic == '/map_done':
            self.pub = self.node.create_publisher(
                std_msgs.Bool, '/map_done', 10)
        elif self.topic == '/speaker':
            self.pub = self.node.create_publisher(
                std_msgs.String, '/speaker', 10)

    def update(self):
        self.pub.publish(self.message)
        return py_trees.common.Status.SUCCESS


class Initialize(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super(Initialize, self).__init__(name=name)

    def setup(self, **kwargs):
        self.node = kwargs['node']

    def initialise(self):
        Blackboard().set('robot_position', (0, 0))
        Blackboard().set('object_in_hand', None)
        Blackboard().set('curr_target', None)
        Blackboard().set('global_path', None)
        Blackboard().set('Target_list', [])
        Blackboard().set('Obstacle_list', [])
        Blackboard().set('local_path', None)
        Blackboard().set('global_waypoints', [])
        Blackboard().set('est_pose', None)
        Blackboard().set('local_path', None)
        Blackboard().set('explore_flag', False)
        Blackboard().set('obs_map', None)
        Blackboard().set('explored_percentage', 0.0)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.node.get_logger().info('updating in Intialize...')
        return pt.common.Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class RemoveTargetFromList(pt.behaviour.Behaviour):
    """Removes the object in hand from the target list after it has been placed in a box"""

    def __init__(self, name: str,Done):
        super(RemoveTargetFromList, self).__init__(name=name)
        self.future = None
        self.done = Done

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_remove_target = self.node.create_client(
            AddCamObs, '/remove_target')

        while not self.cli_remove_target.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for remove cam obs service...')

    def initialise(self):
        req = AddCamObs.Request()
        if self.done:
            #if this is at the end of the place tree then we want to remove what we just had in the hand
            req.radpoints.points.append(Blackboard().get('object_in_hand'))
        else:
            #If this is not at the end of the place sequence we want to remoe the current taget and choose a new one
            req.radpoints.points.append(Blackboard().get('curr_target'))
        self.future = self.cli_remove_target.call_async(req)


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.node.get_logger().info('updating in RemoveTargetFromList...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.done:
                    self.node.get_logger().info('Object placed in box has been deleted from target list')
                    # reset object_in_hand to None after removing it from target list
                    Blackboard().set('object_in_hand', None)
                    return pt.common.Status.SUCCESS

                elif not response.done:
                    self.node.get_logger().info(
                        'Object placed in box has not been deleted from target list')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'Remove target service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class SetCurrentTarget(pt.behaviour.Behaviour):
    """Sets the current target depening on if there is something in the hand or not, also publishes the current target to the topic /current_target"""

    def __init__(self, name: str):
        super(SetCurrentTarget, self).__init__(name=name)
        self.robot_x = None
        self.robot_y = None
        self.target_list = None
        self.object_in_hand = None
        self.pub = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.pub = self.node.create_publisher(
            PointWithRadius, '/current_target', 10)

    def initialise(self):
        self.object_in_hand = None
        self.node.get_logger().info('initialising in SetCurrentTarget...')
        self.target_list = Blackboard().get('Target_list')
        self.robot_x, self.robot_y = Blackboard().get('robot_position')
        self.object_in_hand = Blackboard().get('object_in_hand')

    def update(self):
        if self.object_in_hand is None:
            self.logger.debug("%s.update()" % self.__class__.__name__)
            self.node.get_logger().info('No object in hand, looking for closest target')
            distance = 10
            target = None
            for radpoint in self.target_list:
                if radpoint.type != 'box':
                    x = radpoint.point.point.x
                    y = radpoint.point.point.y
                    dist = ((self.robot_x-x)**2 + (self.robot_y-y)**2)**0.5
                    if dist < distance:  # and radpoint.type!= 'box':
                        distance = dist
                        target = radpoint
            if target is not None:
                Blackboard().set('curr_target', target)
                self.pub.publish(target)
                self.node.get_logger().info(
                    f'Set current target: {target.point}' + str(target.id))
                return pt.common.Status.SUCCESS
            elif target is None:
                self.node.get_logger().info('No target was found, need to do exploring')
                Blackboard().set('explore_flag', True)
                return pt.common.Status.FAILURE
        if self.object_in_hand is not None:
            self.node.get_logger().info('Object in hand, choosing the appropriate box')
            type = self.object_in_hand.type.lower()
            if type == 'blue cube' or type == 'red cube' or type == 'green cube' or type == 'wooden cube':
                mark = 1
            elif type == 'blue ball' or type == 'red ball' or type == 'green ball':
                mark = 2
            elif type == "hugo" or type == "oakie" or type == "kiki" or type == "slush" or type == "muddles" or type == "binky":
                mark = 3
            Found = False
            for radpoint in self.target_list:
                if radpoint.type == 'box':
                    self.node.get_logger().info(str(radpoint.marker_id))
                if radpoint.marker_id == mark:
                    Found = True
                    # Setting corresponding box to current target
                    Blackboard().set('curr_target', radpoint)
                    self.pub.publish(radpoint)
                    #self.node.get_logger().info(
                    #    f'Set current target: {radpoint}')
                    return pt.common.Status.SUCCESS
            if not Found:
                self.node.get_logger().info('No box was found, need to do exploring')
                Blackboard().set('explore_flag', True)
                return pt.common.Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class GetObstacleList(pt.behaviour.Behaviour):
    """Calls object handler to get the latest list of objects and stores it in the blackboard
    Should be called everytime we want to do gloabl or local planning"""

    def __init__(self, name: str):
        super(GetObstacleList, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_get_target = self.node.create_client(GetList, '/get_list')

        while not self.cli_get_target.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for get_list service...')

    def initialise(self):
        req = GetList.Request()
        req.type = 'obstacle'
        self.future = self.cli_get_target.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #self.node.get_logger().info('updating in GetObjectList...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if True:
                    self.node.get_logger().info('Got list of obstacles!')
                    Blackboard().set('Obstacle_list', response.list.points)

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No obstacles were found')
                    return pt.common.Status.SUCCESS
            except Exception as e:
                self.node.get_logger().error(
                    f'Get list service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class GetTargetList(pt.behaviour.Behaviour):
    """Calls the object handler for the latest list of targets and saves them to the blackboard
    Should be called every time we want to get a new target, either after picking somehting up or after placing something in a box"""

    def __init__(self, name: str):
        super(GetTargetList, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_get_target = self.node.create_client(GetList, '/get_list')

        while not self.cli_get_target.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for get_list service...')

    def initialise(self):
        req = GetList.Request()
        req.type = 'target'
        self.future = self.cli_get_target.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        #self.node.get_logger().info('updating in GetTargetList...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.list.points != []:
                    self.node.get_logger().info('Got list of targets!')
                    Blackboard().set('Target_list', response.list.points)

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No targets were found')
                    return pt.common.Status.SUCCESS
            except Exception as e:
                self.node.get_logger().error(
                    f'Get list service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class CheckWS(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super(CheckWS, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_is_in_ws = self.node.create_client(IsInWs, '/is_in_ws')

        while not self.cli_is_in_ws.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for is_in_ws service...')

    def initialise(self):
        req = IsInWs.Request()
        req.point = Blackboard().get("curr_target").point  # Pointwithradius
        self.future = self.cli_is_in_ws.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.node.get_logger().info('updating in CheckWS...')

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.inside:
                    self.node.get_logger().info('Object is inside WS.')
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('Object is outside WS.')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'is_in_ws service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class GlobalPlanner(pt.behaviour.Behaviour):
    """Starts global planning, takes current target and current obstacle list, calls addobs service of gloabl map, 
    gets map and sends map and goal to global planner"""

    def __init__(self, name: str):
        super(GlobalPlanner, self).__init__(name=name)
        self.future_addobs = None
        self.future_global_planning = None
        self.current_target = None
        self.obstacle_list = None
        self.map_sent = False
        self.req = None
        self.future_remove_obs = None
        self.removed = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_global_planning = self.node.create_client(
            GlobalPlanning, '/global_planning')
        self.cli_add_obs_global = self.node.create_client(
            AddObsMap, '/add_obs_global')

        while not self.cli_global_planning.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for global planning service...')
        while not self.cli_add_obs_global.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for add obs global service...')

    def initialise(self):
        self.map_sent = False
        self.future_global_planning = None
        self.future_remove_obs = None
        self.removed = False
        # first slice of global path shouild be the 25th point since it will be roughly 1m away (25*0.04 = 1m)
        self.start_point = 13
        self.req = GlobalPlanning.Request()
        self.current_target = Blackboard().get("curr_target")
        self.req.goal = self.current_target.point
        # remove current target from obstacle list
        temp_list = []
        self.obstacle_list = Blackboard().get("Obstacle_list")
        if self.obstacle_list == []:
            stuff = PointWithRadius()
            stuff.point.point.x = 0.0
            stuff.point.point.y = 0.0
            stuff.radius = 0.0
            self.obstacle_list.append(stuff)

        if self.obstacle_list != []:
            for radpoint in self.obstacle_list:
                if radpoint.id != self.current_target.id:
                    temp_list.append(radpoint)
                elif radpoint.id == self.current_target.id and radpoint.type == 'box':
                    radpoint.type = 'target box'
                    temp_list.append(radpoint)

        self.node.get_logger().info(
            f'Global planning get goal from blackboard: {self.req.goal}')
        req_add_obs = AddObsMap.Request()
        req_add_obs.add = True
        req_add_obs.radpoints.points = temp_list
        self.future_addobs = self.cli_add_obs_global.call_async(req_add_obs)

    def get_dist(self, obs):
        return ((self.current_target.point.point.x-obs.point.point.x)**2 + (self.current_target.point.point.y-obs.point.point.y)**2)**0.5

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # self.node.get_logger().info('updating in global planning...')

        if self.future_addobs is None:
            return pt.common.Status.RUNNING

        if self.future_addobs.done() and not self.map_sent:
            try:
                response = self.future_addobs.result()
                if response.map is not None:
                    self.node.get_logger().info('Got global map!')
                    self.req.map = response.map
                    self.future_global_planning = self.cli_global_planning.call_async(
                        self.req)
                    self.map_sent = True
                    return pt.common.Status.RUNNING
                    # Blackboard().set('global_map', response.map)
                else:
                    self.node.get_logger().info('No global map was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'add obs global service call failed {e}')
                return pt.common.Status.FAILURE

        if self.future_global_planning is None:
            return pt.common.Status.RUNNING
        if self.future_global_planning.done() and not self.removed:
            try:
                response = self.future_global_planning.result()
                if response.global_path.poses != []:
                    Blackboard().set("global_path", response.global_path)
                    self.node.get_logger().info(f'Stored global path on blackboard.')
                    # PROB NEED TO DO BETTER SLICING OF GLOBAL PATH
                    if len (response.global_path.poses) <= self.start_point+1:
                        global_waypoints = [response.global_path.poses[-1]]
                    elif (len(response.global_path.poses)-1) % self.start_point == 0:
                        global_waypoints = response.global_path.poses[self.start_point::self.start_point]
                        #global_waypoints.append(response.global_path.poses[-1])

                    elif (len(response.global_path.poses[self.start_point:])-1) % self.start_point != 0:
                        if (len(response.global_path.poses[self.start_point:])-2) % self.start_point == 0 or (len(response.global_path.poses[self.start_point:])-3) % self.start_point == 0:
                            global_waypoints = response.global_path.poses[self.start_point:-self.start_point:self.start_point]
                            global_waypoints.append(response.global_path.poses[-1])
                        else:
                            global_waypoints = response.global_path.poses[self.start_point::self.start_point]
                            global_waypoints.append(response.global_path.poses[-1])
                    Blackboard().set("global_waypoints", global_waypoints)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No global path was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'global planning service call failed {e}')
                return pt.common.Status.FAILURE

        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class LocalPlanner(pt.behaviour.Behaviour):
    """Starts global planning, takes current target and current obstacle list, calls addobs service of gloabl map, 
    gets map and sends map and goal to global planner"""

    def __init__(self, name: str):
        super(LocalPlanner, self).__init__(name=name)
        self.future_addobs = None
        self.future_local_planning = None
        self.current_target = None
        self.obstacle_list = None
        self.map_sent = False
        self.req = None
        self.future_remove_obs = None
        self.removed = False
        self.waypoints = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_local_planning = self.node.create_client(
            GlobalPlanning, '/local_planning')
        self.cli_add_obs_global = self.node.create_client(
            AddObsMap, '/add_obs_local')

        while not self.cli_local_planning.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for local planning service...')
        while not self.cli_add_obs_global.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for add obs local service...')

    def initialise(self):
        self.map_sent = False
        self.future_local_planning = None
        self.future_remove_obs = None
        self.removed = False
        self.req = GlobalPlanning.Request()
        waypoints = Blackboard().get("global_waypoints")
        self.current_target = Blackboard().get("curr_target")
        temp = Blackboard().get("object_in_hand")
        if temp != None:
            self.ob_in_hand = temp
        else:
            self.ob_in_hand = None
        # Posestamped, need to be convertet to pointstamped
        try:
            curr_waypoint = waypoints[0]
        except IndexError:
            self.node.get_logger().info(
                f'No waypoints in global_waypoints list')
            return pt.common.Status.FAILURE
        temp = PointStamped()
        temp.header = curr_waypoint.header
        temp.point.x = curr_waypoint.pose.position.x
        temp.point.y = curr_waypoint.pose.position.y
        temp.point.z = curr_waypoint.pose.position.z
        self.req.goal = temp
        # remove current target from obstacle list
        temp_list = []
        self.obstacle_list = Blackboard().get("Obstacle_list")
        if self.obstacle_list == []:
            stuff = PointWithRadius()
            stuff.point.point.x = 0.0
            stuff.point.point.y = 0.0
            stuff.radius = 0.0
            self.obstacle_list.append(stuff)

        if self.obstacle_list != []:
            for radpoint in self.obstacle_list:
                if self.current_target is not None:
                    if radpoint.id != self.current_target.id:
                        if self.ob_in_hand == None:
                            temp_list.append(radpoint)
                        elif radpoint.id != self.ob_in_hand.id:
                            temp_list.append(radpoint)
                    elif radpoint.id == self.current_target.id and radpoint.type == 'box':
                        radpoint.type = 'target box'
                        temp_list.append(radpoint)
                else:
                    temp_list.append(radpoint)

        self.node.get_logger().info(
            f'Local planning get goal from blackboard: {self.req.goal}')
        req_add_obs = AddObsMap.Request()
        req_add_obs.add = True
        req_add_obs.radpoints.points = temp_list
        self.future_addobs = self.cli_add_obs_global.call_async(req_add_obs)

    def get_dist(self, obs):
        return ((self.current_target.point.point.x-obs.point.point.x)**2 + (self.current_target.point.point.y-obs.point.point.y)**2)**0.5

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # self.node.get_logger().info('updating in global planning...')

        if self.future_addobs is None:
            return pt.common.Status.RUNNING

        if self.future_addobs.done() and not self.map_sent:
            try:
                response = self.future_addobs.result()
                if response.map is not None:
                    self.node.get_logger().info('Got local map!')
                    self.req.map = response.map
                    self.future_local_planning = self.cli_local_planning.call_async(
                        self.req)
                    self.map_sent = True
                    return pt.common.Status.RUNNING
                    # Blackboard().set('global_map', response.map)
                else:
                    self.node.get_logger().info('No local map was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'add obs global service call failed {e}')
                return pt.common.Status.FAILURE

        if self.future_local_planning is None:
            return pt.common.Status.RUNNING
        if self.future_local_planning.done() and not self.removed:
            try:
                response = self.future_local_planning.result()
                if response.global_path.poses != []:
                    Blackboard().set("local_path", response.global_path)
                    self.node.get_logger().info(f'Stored local path on blackboard.')

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No local path was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'global planning service call failed {e}')
                return pt.common.Status.FAILURE

        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class RemoveGlobalObs(pt.behaviour.Behaviour):
    """Removes the currently added camera obstacles form the global map, should be done right after global planning"""

    def __init__(self, name: str):
        super(RemoveGlobalObs, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_remove_obs = self.node.create_client(
            AddObsMap, '/add_obs_global')

        while not self.cli_remove_obs.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for remove obs service...')

    def initialise(self):
        req = AddObsMap.Request()
        req.add = False
        self.future = self.cli_remove_obs.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.map != None:
                    self.node.get_logger().info('Removed points from global map')

                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No obstacles were found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'Get list service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class PickUp(pt.behaviour.Behaviour):
    """After pickup need to set object in hand to curr target!!"""

    def __init__(self, name: str):
        super(PickUp, self).__init__(name=name)
        self.future = None
        self.object = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_pickup = self.node.create_client(Trigger, '/pick_up_service')
        #self.pub = self.node.create_publisher(std_msgs.String,'/speaker',10)

        while not self.cli_pickup.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for pick up service...')

    def initialise(self):
        req = Trigger.Request()
        self.object = Blackboard().get('curr_target')
        self.future = self.cli_pickup.call_async(req)

    def update(self):
        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    #Blackboard().set('object_in_hand', self.object)
                    #self.node.get_logger().info(
                    #    f'Picked up {self.object.id}')
                    #message = std_msgs.String()
#
                    #message.data = f'Roger that, I just Picked up a {self.object.type}'
                    #self.pub.publish(message)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(
                        f'Failed to pick up {self.object.id}')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'pick up service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)

# first service behaviour, a lot of debugging


class Place(pt.behaviour.Behaviour):
    """After pickup need to set object in hand to curr target!!"""

    def __init__(self, name: str):
        super(Place, self).__init__(name=name)
        self.future = None
        self.object = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_place = self.node.create_client(Trigger, '/place_service')
        self.pub = self.node.create_publisher(std_msgs.String,'/speaker',10)

        while not self.cli_place.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for place service...')

    def initialise(self):
        req = Trigger.Request()
        self.object = Blackboard().get('object_in_hand')
        self.future = self.cli_place.call_async(req)

    def update(self):
        if self.future is None:
            self.node.get_logger().info('Updating in PLace')
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    # Setting object in hand to none
                    # Blackboard().set('object_in_hand', None)
                    self.node.get_logger().info('Object placed in box')
                
                    message = std_msgs.String()
                    message.data = f'Roger that, I just Placed {self.object.type} in a box'
                    self.pub.publish(message)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().error(
                        f'place service call failed')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'place service call failed because of exception {e}')
                return pt.common.Status.FAILURE
        elif not self.future.done():
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class DetectGoal(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super(DetectGoal, self).__init__(name=name)
        self.future = None

    def setup(self, **kwargs):
        print("Available keys in kwargs:", kwargs.keys())

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name)
            self.logger.error(error_message)
            raise KeyError(error_message) from e

        self.logger.debug(f"{self.qualified_name}.setup()")

        self.cli_detection = self.node.create_client(
            EstPose, '/estimated_pose')

        while not self.cli_detection.wait_for_service(timeout_sec=5):
            self.node.get_logger().info('waiting for detection service...')
        self.node.get_logger().info('detection service is avaiable')

    def initialise(self):
        req = EstPose.Request()
        self.future = self.cli_detection.call_async(req)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.future is None:
            return pt.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                if response.est_pose is not None:
                    self.node.get_logger().info(
                        f'Detected estimated pose: {response.est_pose}')
                    Blackboard().set("est_pose", response.est_pose)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('Cannot get goal pose')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f'Service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


class PurePursuit(pt.behaviour.Behaviour):
    """
    An action client interface that draws goals from the blackboard. 
    Simplified and costomed version of action_clients in py_trees_ros
    """

    def __init__(self,
                 name: str,
                 action_type: typing.Any,
                 action_name: str,
                 key: str,
                 tolerence : float = 0.12,
                 generate_feedback_message: typing.Callable[[
                     typing.Any], str] = None,
                 wait_for_server_timeout_sec: float = -3.0
                 ):
        super().__init__(name)
        self.action_type = action_type
        self.action_name = action_name
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        self.generate_feedback_message = generate_feedback_message
        self.flag = None
        self.node = None
        self.tolerance = tolerence
        self.action_client = None
        self.waypoints = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        self.flag_subscriber = self.node.create_subscription(
            std_msgs.Bool, '/flag', self.flag_callback, 10)
        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.action_client.wait_for_server(
                timeout_sec=self.wait_for_server_timeout_sec)
        else:
            iterations = 0
            period_sec = -1.0*self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.action_client.wait_for_server(
                    timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for action server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.action_name,
                            self.qualified_name
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.action_name)
            self.node.get_logger().error("{}[{}]".format(
                self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to action server [{}]".format(
                self.action_name)
            self.node.get_logger().info("{}[{}]".format(
                self.feedback_message, self.qualified_name))

    def initialise(self):
        """
        Reset the internal variables and kick off a new goal request.
        """
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.result_status = None
        self.feedback = None
        self.flag = False
        self.waypoints = Blackboard().get("global_waypoints")
        if len(self.waypoints) == 1:
            self.last = True
        else:
            self.last = False


        try:
            goal = DriveTo.Goal()
            goal.path = Blackboard().get("local_path")
            goal.tolerance = self.tolerance
            self.send_goal_request(goal)
            self.feedback_message = "sent goal request"
        except KeyError:
            pass  # self.send_goal_future will be None, check on that

    def update(self):

        if self.send_goal_future is None:
            self.feedback_message = "no goal to send"
            return pt.common.Status.FAILURE
        if self.goal_handle is not None and not self.goal_handle.accepted:
            self.feedback_message = "goal rejected"
            return pt.common.Status.FAILURE

        if self.result_status is None:
            # self.node.get_logger().info("I am still running for a result...")
            if self.flag:
                self.feedback_message = "flag is true"
                self.node.get_logger().info("Flag is true")
                self.send_cancel_request()
                return pt.common.Status.FAILURE
            # if we get to within 0.2 m of the end of the path resturn success so that we can start a  new cycle of pp
            elif self.feedback is not None and not self.last:
                if self.feedback.pose.position.x < 0.45:
                    self.node.get_logger().info("I am close to the goal")
                    self.waypoints.pop(0)  # Remove the latest waypoint
                    Blackboard().set("global_waypoints", self.waypoints)
                    self.node.get_logger().info("Action behaviour completed!")

                    return pt.common.Status.SUCCESS
                pass
            return pt.common.Status.RUNNING
        else:
            if self.result_status:
                self.feedback_message = "action behaviour successfully completed"
                self.node.get_logger().info("Action behaviour completed!")
                self.waypoints.pop(0)  # Remove the latest waypoint
                Blackboard().set("global_waypoints", self.waypoints)
                return pt.common.Status.SUCCESS
            else:
                self.feedback_message = "failed"
                return pt.common.Status.FAILURE

    def flag_callback(self, msg):
        self.flag = msg.data

    def send_goal_request(self, goal: typing.Any):

        self.feedback_message = "sending goal ..."
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):

        if future.result() is None:
            self.feedback_message = "goal request failed :[ [{}]\n{!r}".format(
                self.qualified_name, future.exception())
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "goal rejected :( [{}]".format(
                self.qualified_name)
            return
        else:
            self.feedback_message = "goal accepted :) [{}]".format(
                self.qualified_name)

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        self.result_status = future.result().result.success

    def terminate(self, new_status: pt.common.Status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status,
                                new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        if (
            self.status == pt.common.Status.RUNNING and
            new_status == pt.common.Status.INVALID
        ):
            self.send_cancel_request()

    def shutdown(self):
        """
        Clean up the action client when shutting down.
        """
        self.action_client.destroy()

    def feedback_callback(self, msg: typing.Any):
        """
        Default generator for feedback messages from the action server. This will
        update the behaviour's feedback message with a stringified version of the
        incoming feedback message.

        Args:
            msg: incoming feedback message (e.g. move_base_msgs.action.MoveBaseFeedback)
        """
        #self.node.get_logger().info('Current dist : ' + str(msg.feedback.pose.position.x))
        self.feedback = msg.feedback
        if self.generate_feedback_message is not None:
            # self.feedback_message = "feedback: {}".format(
            #    self.generate_feedback_message(msg))
            self.node.get_logger().debug(
                '{} [{}]'.format(
                    self.feedback_message,
                    self.qualified_name
                )
            )

    def send_cancel_request(self):
        """
        Send a cancel request to the server. This is triggered when the
        behaviour's status switches from :attr:`~py_trees.common.Status.RUNNING` to
        :attr:`~py_trees.common.Status.INVALID` (typically a result of a priority
        interrupt).
        """
        self.feedback_message = "cancelling goal ... [{}]".format(
            self.qualified_name)
        self.node.get_logger().debug(self.feedback_message)

        self.node.get_logger().info('Cancel started')
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            self.node.get_logger().info('Cancel SENT!')

            future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future: Future):
        """
        Immediate callback for the result of a cancel request. This will
        set the behaviour's feedback message accordingly.

        Args:
            future: incoming cancellation result delivered from the action server
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.feedback_message = "goal successfully cancelled [{}]".format(
                self.qualified_name)
            self.node.get_logger().info('Cancel SENT!')

        else:
            self.feedback_message = "goal failed to cancel [{}]".format(
                self.qualified_name)
        self.node.get_logger().debug('... {}'.format(self.feedback_message))


class ExplorePoint(pt.behaviour.Behaviour):
    """
    call explore service, takes in the obs_map, slice it every 1m and set the last point as curr_target, so that local_planning can serve for pure pursuit

        args:
            'obs_map' <-- blackboard <-- update_obs_map behavior

        return:
            global_waypoints for local_planner
            curr_target for local_planner
    """

    def __init__(self, name: str):
        super(ExplorePoint, self).__init__(name=name)
        self.future_explore = None
        self.curr_target = None
        self.start_point = 25

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_explore = self.node.create_client(
            Explore, '/explore'
        )
        while not self.cli_explore.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for explore service...')
        self.pub = self.node.create_publisher(
            PointWithRadius, '/current_target', 10)

    def initialise(self):
        self.start_point = 0
        self.req = Explore.Request()
        self.req.obs_map = Blackboard().get('obs_map')
        self.future_explore = self.cli_explore.call_async(self.req)
        self.node.get_logger().info('Explore service called')

    def update(self):
        if self.future_explore is None:
            return pt.common.Status.RUNNING
        if self.future_explore.done():
            try:
                response = self.future_explore.result()
                if response.point.point.x != 9999.0:
                    target = PointWithRadius()
                    target.point = response.point
                    Blackboard().set('curr_target', target)
                    self.pub.publish(target)
                    self.node.get_logger().info(
                        f'Get best point from eplore service, point = {response.point}')
                    return pt.common.Status.SUCCESS

                else:
                    self.node.get_logger().info('No global path was found')
                    return pt.common.Status.FAILURE

            except Exception as e:
                self.node.get_logger().error(
                    f'explore service call failed {e}')
                return pt.common.Status.FAILURE

        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future_explore = None
        return super().terminate(new_status)


class VizGrid(pt.behaviour.Behaviour):
    """
    call retrace_steps service, takes in the obs_map, output updated viz map and caculate the explored percentage and put it on the blackboard 

    args:
        'obs_map' <-- blackboard <-- update_obs_map behavior

    return:
        updated viz_grid
        explored percentage
    """

    def __init__(self, name: str):
        super(VizGrid, self).__init__(name=name)
        self.vis_future = None
        self.obs_map = None
        self.explore_flag = True  # for testing

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_viz_grid = self.node.create_client(
            VisualGrid, '/visual_grid'
        )
        while not self.cli_viz_grid.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for viz grid service...')
        self.speaker = self.node.create_publisher(std_msgs.String, '/speaker', 10)

    def initialise(self):
        # TODO 'explore_flag' should be rised by object handle or the explored percentage less than 50%
        # self.explore_flag = Blackboard().get('explore_flag')
        if not self.explore_flag:
            self.node.get_logger().info('Exploration flag is not set, skipping...')
            return pt.common.Status.SUCCESS

        self.obs_map = Blackboard().get('obs_map')
        req = VisualGrid.Request()
        req.obs_map = self.obs_map
        self.vis_future = self.cli_viz_grid.call_async(req)
        self.node.get_logger().info('Viz grid service called')

    def update(self):
        if not self.explore_flag:  # no exploration needed
            pt.common.Status.SUCCESS

        if self.vis_future is None:
            return pt.common.Status.RUNNING
        elif self.vis_future.done():
            try:
                response = self.vis_future.result()  # 1D array

                if response.percentage >= 0.0:
                    explored_percentage = response.percentage
                    Blackboard().set("explored_percentage", explored_percentage)
                    self.node.get_logger().info(
                        f'explored_percentage : {explored_percentage:.2f}%')
                    message = std_msgs.String()
                    message.data = f'Look at that, I have explored {explored_percentage:.2f}%'

                    self.speaker.publish(message)
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No viz grid was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'retrace_steps service call failed {e}'
                )
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.vis_future = None
        return super().terminate(new_status)


class UpdateObsMap(pt.behaviour.Behaviour):
    """
    call '/add_obs_global' to get a global map with obstacles

    args:
        'Obstacle_list' <-- blackboard

    return:
        'obs_map' --> blackboard


    while driving, assume classification node is a client of (AddCamObs, '/add_cam_obs') and will automaticly AddCamObs from camera in object_handler list, 
    after run GeObstacleList behavior, it set('Obstacle_list') on blackboard, we add this list and over write previous global map and set "obs_map" on blackbord. 
    if a new object is found during PP, a flag will be rised and it will stop the robot, so it wont run into the found. 
    TODO: check pp action client goal_cancel, flag and cancel mechanism haven't implemented yet

    """

    def __init__(self, name: str):
        super(UpdateObsMap, self).__init__(name=name)
        self.future_addobs = None
        self.obstacle_list = None
        self.future_remove_obs = None
        self.removed = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cli_add_obs_global = self.node.create_client(
            AddObsMap, '/add_obs_global')

        while not self.cli_add_obs_global.wait_for_service(timeout_sec=1):
            self.node.get_logger().info('waiting for add obs global service...')

    def initialise(self):
        self.map_sent = False
        self.future_global_planning = None
        self.future_remove_obs = None
        self.removed = False

        temp_list = []
        self.obstacle_list = Blackboard().get("Obstacle_list")
        if self.obstacle_list == []:
            stuff = PointWithRadius()
            stuff.point.point.x = 0.0
            stuff.point.point.y = 0.0
            stuff.radius = 0.0
            self.obstacle_list.append(stuff)

        if self.obstacle_list != []:
            for radpoint in self.obstacle_list:
                temp_list.append(radpoint)

        req_add_obs = AddObsMap.Request()
        req_add_obs.add = True
        req_add_obs.radpoints.points = temp_list
        self.future_addobs = self.cli_add_obs_global.call_async(req_add_obs)

    def get_dist(self, obs):
        return ((self.current_target.point.point.x-obs.point.point.x)**2 + (self.current_target.point.point.y-obs.point.point.y)**2)**0.5

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # self.node.get_logger().info('updating in global planning...')

        if self.future_addobs is None:
            return pt.common.Status.RUNNING

        if self.future_addobs.done() and not self.map_sent:
            try:
                response = self.future_addobs.result()
                if response.map is not None:
                    Blackboard().set('obs_map', response.map)
                    self.node.get_logger().info('Got obs map!')
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info('No global map was found')
                    return pt.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(
                    f'add obs global service call failed {e}')
                return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future = None
        return super().terminate(new_status)


##################### Helper Behaviours #########################
class SliceGlobalWaypoints(pt.behaviour.Behaviour):
    """
        takes in a path, cut into waypoints every 1m, set the last point as curr_target, so that local_planning can serve for pure pursuit        
        args:
            'path' <-- blackboard <-- update_obs_map behavior

        return:
            global_waypoints for local_planner
            curr_target for local_planner
    """

    def __init__(self, name: str):
        super(SliceGlobalWaypoints, self).__init__(name=name)
        self.future_explore = None
        self.curr_target = None
        self.path = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

    def initialise(self):
        self.start_point = 0
        self.path = Blackboard().get('global_path')

        try:
            if self.path.poses != []:
                print(
                    f'Get global waypoints from blackboard, waypoints len = {len(self.path.poses)}')
                # slice the path every 25 points from the start point
                self.waypoints = self.path.poses[self.start_point::25]
                # set last point as curr_target
                if self.waypoints[-1] != self.path.poses[-1]:
                    self.waypoints.append(self.path.poses[-1])
            else:
                self.node.get_logger().info(f'no path was found')

        except Exception as e:
            self.node.get_logger().error(f'path poses call failed {e}')
            return pt.common.Status.FAILURE

    def update(self):
        if self.path is None:
            return pt.common.Status.RUNNING
        if self.path:

            if not self.waypoints:
                self.node.get_logger().info(f'no waypoints anymore')
                return pt.common.Status.SUCCESS
            if len(self.waypoints) > 1:
                Blackboard().set('curr_target', self.waypoints[1])
                self.node.get_logger().info(
                    f'Blackboard set curr_target = {self.waypoints[1]}')
                self.waypoints = self.waypoints[1:]
                Blackboard().set('global_waypoints', self.waypoints)
                return pt.common.Status.RUNNING
            else:
                if self.waypoints:
                    Blackboard().set('curr_target', self.waypoints[0])
                self.waypoints = []
                Blackboard().set('global_waypoints', self.waypoints)
                self.node.get_logger().info(
                    f'Blackboard set curr_target , this is the last point in the path')
                return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.future_explore = None
        return super().terminate(new_status)


class Counter(pt.behaviour.Behaviour):
    def __init__(self, name: str, duration: int):
        super(Counter, self).__init__(name)
        self.duration = duration
        self.counter = 0

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.timer = self.node.create_timer(
            1.0, self.timer_callback)  # Timer set for 1 second

    def timer_callback(self):
        if self.counter < self.duration:
            self.counter += 1
            self.node.get_logger().info(f'Count: {self.counter}')
        else:
            self.node.get_logger().info('Finished counting!')
            self.timer.cancel()  # Cancel the timer when done

    def initialise(self):
        self.counter = 0  # Reset the counter on initialisation
        print('initialising in Count...')

    def update(self):
        if self.counter >= self.duration:
            Blackboard().set('count_flag', True)
            self.node.get_logger().info('another round Count finished')
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        if self.timer:
            self.timer.cancel()
        super().terminate(new_status)


class RepeatUntilGoalReached(Decorator):
    """
    A decorator that repeats the execution of its child until the threshold is met.

    Args:
        child: The child behaviour or subtree to decorate.
        goal_condition: A callable that returns True if the goal is reached, else False.
        name: The decorator name.
    """

    def __init__(self, name: str, child: pt.behaviour.Behaviour, threshold: int):
        super().__init__(name=name, child=child)
        self.threshold = threshold

    def initialise(self) -> None:
        self.left_waypoints_len = len(Blackboard().get('global_waypoints'))

    def update(self):
        """
        Check the goal condition and decide whether to continue execution.

        Returns:
            py_trees.common.Status.RUNNING if the goal is not yet met and the child is to be repeated.
            The child's return status (SUCCESS or FAILURE) if the goal is met.
        """

        child_status = self.decorated.status
        # Check if the child's status is valid
        if child_status not in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE, py_trees.common.Status.RUNNING]:
            self.logger.error(f"Invalid child status: {child_status}")
            return py_trees.common.Status.INVALID

        if self.decorated.status == pt.common.Status.SUCCESS:
            self.left_waypoints_len = len(Blackboard().get('global_waypoints'))
            if self.left_waypoints_len <= self.threshold:
                return pt.common.Status.SUCCESS
            else:
                self.logger.info(
                    f"left waypoints from blackboard {self.left_waypoints_len}")

        elif self.decorated.status == pt.common.Status.FAILURE:
            self.logger.info(f"child failed, force to repeat")
            return pt.common.Status.RUNNING

        else:
            # Goal is not met, force child to repeat by returning RUNNING
            return py_trees.common.Status.RUNNING


class PrintBlackboardVariable(pt.behaviour.Behaviour):
    def __init__(self, variable_name, name="PrintBlackboardVariable"):
        super(PrintBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name
        self.value = None

    def initialise(self):
        self.value = Blackboard().get(self.variable_name)
        self.logger.info(
            f"blackboard variable {self.variable_name}: {self.value}")

    def update(self):
        if self.value is None:
            self.logger.warning(
                f"blackboard variable {self.variable_name} is None )")
            return pt.common.Status.FAILURE
        self.logger.info(
            f"get new info from blackboard {self.variable_name}: {self.value}")
        return pt.common.Status.SUCCESS


class RepeatCustom(pt.decorators.Decorator):
    """
    Repeat.

    :data:`~py_trees.common.Status.SUCCESS` is
    :data:`~py_trees.common.Status.RUNNING` up to a specified number at
    which point this decorator returns :data:`~py_trees.common.Status.SUCCESS`.

    :data:`~py_trees.common.Status.FAILURE` is always
    :data:`~py_trees.common.Status.FAILURE`.

    Args:
        child: the child behaviour or subtree
        num_success: repeat this many times (-1 to repeat indefinitely)
        name: the decorator name
    """

    def __init__(self, name: str, child: pt.behaviour.Behaviour, num_success: int):
        super().__init__(name=name, child=child)
        self.success = 0

    def initialise(self) -> None:
        """Reset the currently registered number of successes."""
        self.success = 0
        self.num_success = len(Blackboard().get('global_waypoints'))



    def update(self) -> pt.common.Status:
        """
        Repeat until the nth consecutive success.

        Returns:
            :data:`~py_trees.common.Status.SUCCESS` on nth success,
            :data:`~py_trees.common.Status.RUNNING` on running, or pre-nth success
            :data:`~py_trees.common.Status.FAILURE` failure.
        """
        if self.decorated.status == pt.common.Status.FAILURE:
            self.feedback_message = f"failed, aborting [status: {self.success} success from {self.num_success}]"
            return pt.common.Status.FAILURE
        elif self.decorated.status == pt.common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = (
                f"success [status: {self.success} success from {self.num_success}]"
            )
            if self.success == self.num_success:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.RUNNING
        else:  # RUNNING
            self.feedback_message = (
                f"running [status: {self.success} success from {self.num_success}]"
            )
            return pt.common.Status.RUNNING
