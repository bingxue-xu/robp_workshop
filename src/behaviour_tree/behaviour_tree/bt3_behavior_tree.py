from socket import AF_NETROM
import py_trees as pt
import py_trees_ros as ptr
import rclpy
from .bt2_behaviors import *
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from py_trees.blackboard import Blackboard
import copy
from arian_interfaces.msg import PointWithRadius
PLACETREE_GOAL_TOL = 0.14
PICKTREE_GOAL_TOL = 0.08
class MS3BT(Node):
    def __init__(self):
        super().__init__('ms3_behavior_tree')
        root = pt.composites.Sequence(name="MS3_behavior_tree", memory=True)

        ########## INIT##################
        init_seq = pt.composites.Sequence(name="init_seq", memory=True)
        init = Initialize(name='init')
        wait = Wait(name='wait', duration=1)
        init_seq.add_child(init)
        init_seq.add_child(wait)
        oneshot = pt.decorators.OneShot(
            name='init oneshot', child=init_seq, policy=pt.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)
        
        ######################################################


        ################# MAIN TASK###############################
        main_task_seq = pt.composites.Sequence(
            name="main_task_seq", memory=True)

        PickPlaceExploreSelector = pt.composites.Selector(
            name="PickPlaceExploreSeq", memory=True)

        PickPlaceSeq = pt.composites.Selector(
            name="PickPlaceSeq", memory=True)

        Pick_Tree = pt.composites.Sequence(name="Pick_Tree", memory=True)
        Place_Tree = pt.composites.Sequence(name="Place_Tree", memory=True)

        ###################### PICKTREE#######################################################

        ObjectInHand0 = CheckObjecInHand(name='ObjectInHand0')
        ObjectInHand0_decorator = pt.decorators.Inverter(
            name='object_decorator', child=ObjectInHand0)

        GetRobotPose0 = UpdateRobotPosition(name='GetRobotPose0')
        GetTargetList0 = GetTargetList(name='GetTargetList0')
        SetCurrentTarget0 = SetCurrentTarget(name='SetCurrentTarget0')
        PlanAndDrive0 = self.PlanAndDrive(tol = PICKTREE_GOAL_TOL)


        PickSelector = pt.composites.Selector(
            name="PickSelector", memory=True)
        ################ PickSEQ1#######################
        PickSeq1 = pt.composites.Sequence(name="PickSeq1", memory=True)
        CheckDist1 = CheckDistToTarget(name='CheckDist_1', threshold=0.20)
        PickUp1 = PickUp(name='PickUp1')
        Decor1 = pt.decorators.Retry(name='Decorator_pick1', child=PickUp1,num_failures=5)
        CheckClaws1 = CheckClaws(name='CheckClaws')
        back1 = self.BackUp(1.0)

        PickSeq1.add_children([CheckDist1,Decor1,CheckClaws1, back1])
        #################################################
        Fallback1 = self.PickUpFallback()


        #####PICKSEQ2 ###############################
        PickSeq2 = pt.composites.Sequence(name="PickSeq2", memory=True)
        back2 = self.BackUp(3.5)
        CheckDist2 = CheckDistToTarget(name = 'CheckDist2', threshold=0.20)
        CheckDistDecor123 = pt.decorators.FailureIsSuccess(name = 'CheckDistDecor123', child=CheckDist2)
        DrivePick = self.DriveToGoal(tol=PICKTREE_GOAL_TOL)
        CheckDist3 = CheckDistToTarget(name = 'CheckDist3', threshold=0.20)
        PickUp2 = PickUp(name='PickUp2')
        Decor2 = pt.decorators.Retry(name='Decorator_pick2', child=PickUp2,num_failures=5)
        CheckClaws2 = CheckClaws(name='CheckClaws2')
        Back3 = self.BackUp(1.0)
        PickSeq2.add_children([back2,CheckDistDecor123,DrivePick,CheckDist3,Decor2,CheckClaws2,Back3])
        ############################
        Fallback2 = self.PickUpFallback()

        #If we still fail to pick up the object we want to delete it from the object handler
        #and try to pick up the next object
        RemoveTargetPick = RemoveTargetFromList(name='RemoveTargetPick',Done=False)
        RemoveDecor = pt.decorators.SuccessIsFailure(name='RemoveDecor',child=RemoveTargetPick)
        PickSelector.add_children([PickSeq1,Fallback1,PickSeq2,Fallback2,RemoveDecor])

        # TODO: MAKE BACKUP BEHAVIOUR
        Pick_Tree.add_children(
            [ObjectInHand0_decorator, GetRobotPose0, GetTargetList0, SetCurrentTarget0, PlanAndDrive0, PickSelector])

        ######################################################################

        ######################## PLACE TREE #################################
        ObjectInHand1 = CheckObjecInHand(name='ObjectInHand1')
        GetTargetList111 = GetTargetList(name='GetTargetList111')
        SetCurrentTarget1 = SetCurrentTarget(name='SetCurrentTarget1')
        PlanAndDrive1 = self.PlanAndDrive(tol = PLACETREE_GOAL_TOL)

        PlaceSelector = pt.composites.Selector(
            name="PlaceSelector", memory=True)

        PlaceSeq1 = pt.composites.Sequence(name="PlaceSeq1", memory=True)
        PlaceSeq2 = pt.composites.Sequence(name="PlaceSeq2", memory=True)
        waitfortrans = Wait(name='waitfortrans', duration=3)
        CheckDist_2 = CheckDistToTarget(name='CheckDist_2', threshold = 0.38)
        Place0 = Place(name='Place0')
        Place0Decorator = pt.decorators.Retry(
            name='Place0Decorator', child=Place0, num_failures=5)
        RemoveTargetFromList0 = RemoveTargetFromList(
            name='RemoveTargetFromList0', Done=True)
        Backup2 = self.BackUp(1.0)
        PlaceSeq1.add_children(
            [waitfortrans,CheckDist_2,Place0Decorator, RemoveTargetFromList0, Backup2])
        Backagain = self.BackUp(3.5)
        waitfortrans2 = Wait(name='waitfortrans2', duration=3)
        CheckDist12 = CheckDistToTarget(name='CheckDist_12', threshold=0.38)
        CheckDist2Decor = pt.decorators.FailureIsSuccess(name='CheckDist_2Decorator',child=CheckDist12)
        Drive_to_idiom2 = self.DriveToGoal(tol = PLACETREE_GOAL_TOL)
        waitfortrans3 = Wait(name='waitfortrans3', duration=3)
        CheckDist_3 = CheckDistToTarget(name='CheckDist_3', threshold = 0.38)
        Place1 = Place(name='Place1')
        Place1Decorator = pt.decorators.Retry(
            name='Place1Decorator', child=Place1, num_failures=5)
        RemoveTargetFromList1 = RemoveTargetFromList(
            name='RemoveTargetFromList1',Done=True)
        Backup3 = self.BackUp(1.0)
        PlaceSeq2.add_children(
            [Backagain,waitfortrans2,CheckDist2Decor,Drive_to_idiom2, waitfortrans3,CheckDist_3,Place1Decorator, RemoveTargetFromList1, Backup3])
        do_explore = pt.behaviours.SetBlackboardVariable(name="set_flag_on_Blackboard", variable_name="explore_flag", variable_value=True, overwrite=True)
        PlaceSelector.add_children([PlaceSeq1, PlaceSeq2, do_explore])
        Place_Tree.add_children(
            [ObjectInHand1, GetTargetList111,SetCurrentTarget1, PlanAndDrive1, PlaceSelector])
        ####################################################


        ###################### EXPLORATION #######################################################

        explore_path_and_drive= self.Exploration()
        explore_path_and_drive_to  = pt.decorators.SuccessIsFailure(name='explore_path_and_drive_to',child=explore_path_and_drive)
        check_explored = CheckExplored(name='CheckExplored', threshold=55)
        publish_map_done = PublishMessage(name='PublishMapDone',turnvalue=0.0, forwardvalue=0.0, topic='/map_done')

        done_seq = pt.composites.Sequence(name='done_seq',memory=True, children=[check_explored, publish_map_done])
        initial_explore_selector = pt.composites.Selector(name="InitialExploreSeelector", memory=True, children=[done_seq,explore_path_and_drive_to])
        InitialExploration = pt.decorators.Retry(name='InitialExploration', child=initial_explore_selector, num_failures=10)
        ################################################################################################

        Exploration = self.Exploration()

        #EternalGuard = pt.decorators.EternalGuard(
        #   name="EternalGuard", child=Exploration, blackboard_keys=['explore_flag'], condition=self.check)
        #Failsafe = pt.decorators.FailureIsSuccess(name="Failsafe", child=EternalGuard)
        Explorationflagseq = pt.composites.Sequence(name='Explorationflagq', memory=True)
        checkexploreflag = CheckExploreFlag(name="checkexploreflag")
        Explorationflagseq.add_children([checkexploreflag,Exploration])

        PickPlaceSeq.add_children([Pick_Tree, Place_Tree])
        Succeses = pt.behaviours.Success(name='Success')


        PickPlaceExploreSelector.add_children([PickPlaceSeq, Explorationflagseq,Succeses])  

        #Repeat_decorator = pt.decorators.FailureIsSuccess(
        #    name='Repeat_decorator', child=PickPlaceExploreSelector)
#
        Repeat = pt.decorators.Repeat(
            name='Repeat', child=PickPlaceExploreSelector, num_success=-1)

        main_task_seq.add_children([InitialExploration, Repeat])

        timeout = pt.decorators.Timeout(
            name='timeout', child=main_task_seq, duration=1200)

        back_to_origin = pt.composites.Sequence(
            name="back_to_origin", memory=True)
        temp = PointWithRadius()
        temp.point.header.frame_id = "map"
        temp.point.header.stamp = self.get_clock().now().to_msg()
        temp.point.point.x = 0.0
        temp.point.point.y = 0.0
        set_to_origin = pt.behaviours.SetBlackboardVariable(
            name="set_to_origin", variable_name="curr_target", variable_value=temp, overwrite=True)

        PlanAndDrive2= self.PlanAndDrive(tol = 0.06)
        Speak = PublishMessage(name = 'SPEAK', turnvalue=0,forwardvalue=0,topic='/speaker')
        back_to_origin.add_children([set_to_origin, PlanAndDrive2, Speak])
        selector = pt.composites.Selector(name="selector", memory=True, children=[
                                          timeout, back_to_origin])
        root.add_children([oneshot, selector])
        #root.add_children([oneshot,PickPlaceSeq])
        #####TEST#########
        #test = pt.composites.Sequence(name="test",memory=True)
        #point = PointWithRadius()
        #point.point.header.frame_id = "map"
        #point.point.header.stamp = self.get_clock().now().to_msg()
        #point.point.point.x = 0.0 
        #point.point.point.y = 1.0
#
        #set =pt.behaviours.SetBlackboardVariable(name="set_target", variable_name="curr_target", variable_value=point, overwrite=True)
        #Go = self.PlanAndDrive()
        #test.add_children([set,Go])
        #root.add_children([oneshot,test])
################## JUST FOR TESTING ########################################
        ##### JUST FOR TESTING PICK AND PLACE TREE#####################
        #CheckDost = CheckDistToTarget(name='CheckDist')
        #root.add_children([oneshot,PickPlaceSeq])

        ##### JUST FOR TESTING EXPLORATION ############################
        #set_flag_on_Blackboard = pt.behaviours.SetBlackboardVariable(name="set_flag_on_Blackboard", variable_name="explore_flag", variable_value=True, overwrite=True)
        #set_helper_on_Blackboard = pt.behaviours.SetBlackboardVariable(name="print", variable_name="helper", variable_value="helper", overwrite=True)
        #root.add_children([set_flag_on_Blackboard,set_helper_on_Blackboard,oneshot,InitialExploration])
        #root.add_child(CheckDost)
#######################################################################



        self.tree = pt.trees.BehaviourTree(root)
        self.tree.setup(timeout=15, node=self)

        self.tick_tree_timer = self.create_timer(0.1, self.tick_tree)
        
    def check(self):
        return Blackboard().get('explore_flag')
        ################## EXPLORATION ##############################
    def Exploration(self):
        get_object_list_exp = GetObstacleList(name='get_object_list_exp')
        get_obs_map = UpdateObsMap(name='get_obs_map')
        update_viz_grid_exp1 = VizGrid(name='update_viz_grid_exp1')
        explore_point = ExplorePoint(name='explore_point')
        PlanAndDrive = self.PlanAndDrive(tol = 0.16)
        get_obs_map2 = UpdateObsMap(name='get_obs_map2')

        update_viz_grid_exp0 = VizGrid(name='update_viz_grid_exp0')
        SetFlagToFalse = pt.behaviours.SetBlackboardVariable(name="set_flag_on_Blackboard", variable_name="explore_flag", variable_value=False, overwrite=True)

        ExploreSeq = pt.composites.Sequence(name="Explore_path", memory=True, children=[get_object_list_exp, get_obs_map, update_viz_grid_exp1, explore_point,PlanAndDrive, get_obs_map2,update_viz_grid_exp0,SetFlagToFalse])
        Explore = pt.decorators.Retry(name='Explore', child=ExploreSeq, num_failures=1)
        return Explore 
        
    
        ################## DRIVETO##############################
    def DriveToGoal(self, tol):
        Local_plan_seq = pt.composites.Sequence(
            name="Local_plan_seq", memory=True)
        get_object_list0 = GetObstacleList(name='get_object_list0')
        local_plan0 = LocalPlanner(name='local_plan0')
        pp0 = PurePursuit(
            name="Pure Pursuit0",
            action_type=DriveTo,
            action_name="/drive_to",
            key="global_path",
            tolerence=tol,
            generate_feedback_message=lambda feedback: f"Current Pose: ({feedback.pose.position.x}, {feedback.pose.position.y})")
        Local_plan_seq.add_children([get_object_list0, local_plan0, pp0])
        retry_once_0 = pt.decorators.Retry(
            name='retry_once_0', child=Local_plan_seq, num_failures=2)
        #Test with new Custom decorator
        DriveToGoal =  RepeatCustom(name='DriveToGoal', child=retry_once_0, num_success=1)
        return DriveToGoal

        #######################################################
    
    def PlanAndDrive(self, tol):
        ################# PLAN AND DRIVE######################
        Plan_and_drive_seq = pt.composites.Sequence(
            name="Plan_and_drive_seq", memory=True)
        get_object_list1 = GetObstacleList(name='get_object_list1')
        Global_plan0 = GlobalPlanner(name='Global_plan0')
        RemoveGlobalObs0 = RemoveGlobalObs(name='RemoveGlobalObs0')
        Drive_to_idiom0 = self.DriveToGoal(tol)
        Plan_and_drive_seq.add_children(
            [get_object_list1, Global_plan0, RemoveGlobalObs0, Drive_to_idiom0])
        PlanAndDrive = pt.decorators.Retry(
            name='PlanAndDrive', child=Plan_and_drive_seq, num_failures=1)
        return PlanAndDrive
        ###############################################################
    def BackUp(self, float):
            ############## BACKUP##########################
        BackUp = pt.composites.Sequence(name="Backupseq", memory=True)
        Publish0 = PublishMessage(
            name='Publish0', forwardvalue=-0.5, turnvalue=0.0, topic='/cmd_vel')
        Wait1 = Wait(name='Wait', duration=float)
        Publish1 = PublishMessage(
            name='Publish1', forwardvalue=0.0, turnvalue=0.0,topic='/cmd_vel')
        BackUp.add_children([Publish0, Wait1, Publish1])
        return BackUp
    def crawl(self):
        Crawl = pt.composites.Sequence(name="Crawl", memory=True)
        Publish0 = PublishMessage(
            name='Publish0', forwardvalue=0.2, turnvalue=0.0, topic='/cmd_vel')
        Wait1 = Wait(name='Wait', duration=5)
        Publish1 = PublishMessage(
            name='Publish1', forwardvalue=0.0, turnvalue=0.0,topic='/cmd_vel')
        Crawl.add_children([Publish0, Wait1, Publish1])
        return Crawl
    def PickUpFallback(self):
        seq0 = pt.composites.Sequence(name='PickUpFallback0', memory=True)
        
        move0 = MoveArm(name='MoveArm0',closed=False, reset = False)
        usbcam0 = CallUsbCam(name='UsbCam0')
        #Crawl = self.crawl()
        #Paralell = pt.composites.Parallel(name='Paralelle', policy=pt.common.ParallelPolicy.SuccessOnSelected(children=([usbcam0])))
#
        #Paralell.add_children([usbcam0,Crawl])
        PickUP0  = PickUp(name='PickUp0')
        PickDecor0 = pt.decorators.Retry(name='PickDecor0',child=PickUP0, num_failures=5)
        CheckClaws12 = CheckClaws(name='CheckClaws12')
        Back0 = self.BackUp(1.0)
        seq0.add_children([move0,usbcam0, PickDecor0,CheckClaws12, Back0])
        
        seq1 = pt.composites.Sequence(name='PickUpFallback1', memory=True)
        #Backup = self.BackUp(2.0)
    
        move1 = MoveArm(name='MoveArm1',closed=True, reset = False)

        usbcam1 = CallUsbCam(name='UsbCam1')
        #Crawl1 = self.crawl()
        #Paralell1 = pt.composites.Parallel(name='Paralelle12',policy=pt.common.ParallelPolicy.SuccessOnSelected(children=([usbcam1])))
#
#
        #Paralell1.add_children([usbcam1,Crawl1])
        PickUP1  = PickUp(name='PickU1p')
        PickDecor1 = pt.decorators.Retry(name='PickDecor1',child=PickUP1, num_failures=5)
        CheckCLaws21 = CheckClaws(name='CheckClaws21')
        Back1 = self.BackUp(1.0)
        seq1.add_children([move1,usbcam1, PickDecor1,CheckCLaws21 ,Back1])
        reset_arm = MoveArm(name='ResetArm',closed=False, reset = True)
        reset_decor = pt.decorators.SuccessIsFailure(name='ResetDecor',child=reset_arm)
        fallback = pt.composites.Selector(name='Fallback',memory=True)
        fallback.add_children([seq0, seq1,reset_decor])
        return fallback

    def tick_tree(self):
        # print("Ticking the behavior tree...")
        self.tree.tick()

        if self.tree.root.status == pt.common.Status.SUCCESS:
            self.get_logger().info('Behavior Tree Execution Finished: Success')
            self.tick_tree_timer.cancel()


def main():
    rclpy.init()
    tree = MS3BT()
    executor = SingleThreadedExecutor()
    executor.add_node(tree)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
