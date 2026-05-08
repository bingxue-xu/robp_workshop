import py_trees as pt
import py_trees_ros as ptr
import rclpy
from .bt2_behaviors import *
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from py_trees.blackboard import Blackboard


class MS2BT(Node):
    def __init__(self):
        super().__init__('ms2_behavior_tree')

        root = pt.composites.Sequence(name="MS2_behavior_tree", memory=True)
        #######################################################################################################################

        """
        EVRYTHGIN INSIDE HERE SHOULD BE ITS OWN TREE AND WHEN IT IS FINISHED IT RETURNS SUCCESS AnD THEN WE START THE REAL TREEE
        
        
        
        """
        start_seq = pt.composites.Sequence(name="seqence1", memory=True)
        # Wait for 10 seconds
        init = Initialize(name='Init')
        start_seq.add_child(init)
        robp = UpdateRobotPosition(name='RobotPosition')
        start_seq.add_child(robp)
        wait_10s = Wait("Wait 10s", 3)  # Change to 10 sec
        start_seq.add_child(wait_10s)

        # Publish messages 8 times with specific timings
        turn_seq = pt.composites.Sequence(name="turn_seq", memory=True)
        publish = PublishMessage(
            f"Publish {1}", turnvalue=1.5, forwardvalue=0.0)
        turn_seq.add_child(publish)
        wait_15 = Wait(f"Wait 1.5s {1}", 1.1)
        turn_seq.add_child(wait_15)
        publish = PublishMessage(
            f"Publish {2}", turnvalue=0.0, forwardvalue=0.0)
        turn_seq.add_child(publish)
        wait_3s = Wait(f"Wait 3s {1}", 1.5)
        turn_seq.add_child(wait_3s)

        turn_decorator = pt.decorators.Repeat(
            name="Repeat 8 times", child=turn_seq, num_success=8)
        #start_seq.add_child(turn_decorator)

        root.add_child(start_seq)
###########################################################################################################################################
        seq2 = pt.composites.Sequence(name="Sequence2", memory=True)

        temp = PointWithRadius()
        temp.point.header.frame_id = "map"
        temp.point.header.stamp = self.get_clock().now().to_msg()
        temp.point.point.x = 1.0
        temp.point.point.y = 1.9
        setpoint1 = pt.behaviours.SetBlackboardVariable(
            name='setpoint1', variable_name='curr_target', variable_value=temp, overwrite=True)
        seq2.add_child(setpoint1)
        wait_for_point = pt.behaviours.WaitForBlackboardVariable(
            name='Wait_for_point1', variable_name='curr_target')
        seq2.add_child(wait_for_point)

        root.add_child(seq2)

        plan_seq = pt.composites.Sequence(name="Plan Sequence", memory=True)
        get_obj = GetObstacleList(name='Get Object')
        global_planning = GlobalPlanner(name="Global Planning")
        plan_seq.add_child(get_obj)
        plan_seq.add_child(global_planning)

        remove_points = RemoveGlobalObs(name='Remove Global')
        plan_seq.add_child(remove_points)
        Local_plan_seq = pt.composites.Sequence(
            name="Local Plan Sequence", memory=True)
        drive = pt.composites.Sequence(name='Drive', memory=True)
        get_obj2 = GetObstacleList(name='Get Object666')
        drive.add_child(get_obj2)
        local_planning = LocalPlanner(name='Local_planenr')
        drive.add_child(local_planning)
        # wait = pt.behaviours.WaitForBlackboardVariable(name='Wait for Blackboard', variable_name='local_path')

        # drive.add_child(wait)
        pp = PurePursuit(
            name="Pure Pursuit21",
            action_type=DriveTo,
            action_name="/drive_to",
            key="global_path",
            generate_feedback_message=lambda feedback: f"Current Pose: ({feedback.pose.position.x}, {feedback.pose.position.y})")
        drive.add_child(pp)
        drive_decorator = pt.decorators.Retry(
            name='Retry locla planning once', child=drive, num_failures=0)
        Local_plan_seq.add_child(drive_decorator)
        Check = Check_Waypoints(name="Check waypoints")
        Local_plan_seq.add_child(Check)
        local_Decorator = pt.decorators.Retry(
            name='Retry local planning', child=Local_plan_seq, num_failures=0)
        plan_seq.add_child(local_Decorator)
        plan_decorator = pt.decorators.Retry(
            name='Retry global planning', child=plan_seq, num_failures=0)
        root.add_child(plan_decorator)

        seq3 = pt.composites.Sequence(name="Sequence3", memory=True)

        temp = PointWithRadius()
        temp.point.header.frame_id = "map"
        temp.point.header.stamp = self.get_clock().now().to_msg()
        temp.point.point.x = 0.0
        temp.point.point.y = 3.0
        setpoint2 = pt.behaviours.SetBlackboardVariable(
            name='setpoint2', variable_name='curr_target', variable_value=temp, overwrite=True)
        seq3.add_child(setpoint2)
        ###
        wait_for_point = pt.behaviours.WaitForBlackboardVariable(
            name='Wait_for_point2', variable_name='curr_target')
        seq3.add_child(wait_for_point)

        root.add_child(seq3)
        plan_seq = pt.composites.Sequence(name="Plan Sequence1", memory=True)
        get_obj = GetObstacleList(name='Get Object2')
        global_planning = GlobalPlanner(name="Global Planning412")
        plan_seq.add_child(get_obj)
        plan_seq.add_child(global_planning)

        remove_points = RemoveGlobalObs(name='Remove Global123')
        plan_seq.add_child(remove_points)
        Local_plan_seq = pt.composites.Sequence(
            name="Local Plan Sequence123", memory=True)
        drive = pt.composites.Sequence(name='Driv4e', memory=True)
        get_obj2 = GetObstacleList(name='Get Object66')
        drive.add_child(get_obj2)
        local_planning = LocalPlanner(name='Local_plan213enr')
        drive.add_child(local_planning)
        # wait = pt.behaviours.WaitForBlackboardVariable(name='Wait 23for Blackboard', variable_name='local_path')
#
        # drive.add_child(wait)
        pp = PurePursuit(
            name="Pure Pursuit2121",
            action_type=DriveTo,
            action_name="/drive_to",
            key="global_path",
            generate_feedback_message=lambda feedback: f"Current Pose: ({feedback.pose.position.x}, {feedback.pose.position.y})")
        drive.add_child(pp)
        drive_decorator = pt.decorators.Retry(
            name='Retry3 locla planning once', child=drive, num_failures=1)
        Local_plan_seq.add_child(drive_decorator)
        Check = Check_Waypoints(name="Check waypoints4")
        Local_plan_seq.add_child(Check)
        local_Decorator = pt.decorators.Retry(
            name='R41etry local planning', child=Local_plan_seq, num_failures=10)
        plan_seq.add_child(local_Decorator)
        plan_decorator = pt.decorators.Retry(
            name='Retr42y global planning', child=plan_seq, num_failures=1)
        root.add_child(plan_decorator)

        seq4 = pt.composites.Sequence(name='Sequence 4', memory=True)
        Get_target = GetTargetList(name='Get_Ta48957rget')
        Get_obstacle = GetObstacleList(name='Get_Obstac349857le')
        temp = PointWithRadius()
        temp.type = 'blue cube'
        set_in_hand = pt.behaviours.SetBlackboardVariable(
            name='object in hand', variable_name='object_in_hand', variable_value=temp, overwrite=True)
        wait_for_set = pt.behaviours.WaitForBlackboardVariable(
            name='Wait_for_in_hand', variable_name='object_in_hand')
        Set_target = SetCurrentTarget(name='Set_Target')
        seq4.add_children(
            [Get_target, Get_obstacle, set_in_hand, wait_for_set, Set_target])
        wait_for_point2 = pt.behaviours.WaitForBlackboardVariable(
            name='Wait_for_point3', variable_name='curr_target')

        seq4.add_child(wait_for_point2)
        root.add_child(seq4)
        plan_seq = pt.composites.Sequence(name="Plan Sequ123ence", memory=True)
        get_obj = GetObstacleList(name='Get Object56765')
        global_planning = GlobalPlanner(name="Global Pl5675anning")
        plan_seq.add_child(get_obj)
        plan_seq.add_child(global_planning)

        remove_points = RemoveGlobalObs(name='Remove457 Global')
        plan_seq.add_child(remove_points)
        Local_plan_seq = pt.composites.Sequence(
            name="Local Plan Se76quence", memory=True)
        drive = pt.composites.Sequence(name='Driv754e', memory=True)
        get_obj2 = GetObstacleList(name='Get Object645766')
        drive.add_child(get_obj2)
        local_planning = LocalPlanner(name='Local_planen457r')
        drive.add_child(local_planning)
        # wait = pt.behaviours.WaitForBlackboardVariable(name='Wait4574 for Blackboard', variable_name='local_path')
#
        # drive.add_child(wait)
        pp = PurePursuit(
            name="Pure Pursuit214747",
            action_type=DriveTo,
            action_name="/drive_to",
            key="global_path",
            generate_feedback_message=lambda feedback: f"Current Pose: ({feedback.pose.position.x}, {feedback.pose.position.y})")
        drive.add_child(pp)
        drive_decorator = pt.decorators.Retry(
            name='Re457457try locla planning once', child=drive, num_failures=1)
        Local_plan_seq.add_child(drive_decorator)
        Check = Check_Waypoints(name="Check waypoint4577s")
        Local_plan_seq.add_child(Check)
        local_Decorator = pt.decorators.Retry(
            name='Retry4754 local planning', child=Local_plan_seq, num_failures=10)
        plan_seq.add_child(local_Decorator)
        plan_decorator = pt.decorators.Retry(
            name='Retry glob457547al planning', child=plan_seq, num_failures=1)
        root.add_child(plan_decorator)

        self.tree = pt.trees.BehaviourTree(root)
        self.tree.setup(timeout=15, node=self)

        self.tick_tree_timer = self.create_timer(0.1, self.tick_tree)

    def tick_tree(self):
        # print("Ticking the behavior tree...")
        self.tree.tick()

        if self.tree.root.status == pt.common.Status.SUCCESS:
            self.get_logger().info('Behavior Tree Execution Finished: Success')
            self.tick_tree_timer.cancel()


def main():
    rclpy.init()
    tree = MS2BT()
    executor = SingleThreadedExecutor()
    executor.add_node(tree)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
