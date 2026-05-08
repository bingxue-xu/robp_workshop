## Pacage name <behaviour_tree>

### Executable: bt2       
bt2 is the executable behavior tree for Milestone 2, which includes behaviors such as detection, global planning, and pure pursuit running in sequence.
Arm behaviours can be added if you run on the robot.

### How To Run

For simulation without pick up on robot
(The detection should be able to publish a est_pose):
```
ros2 launch start_robot_package start_robot_launch.xml
ros2 bag play --read-ahead-queue-size 1000 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/SLAMBAG2/SLAMBAG2_0.db3 --remap tf:=tf_null path:=path_null tf_static:=tf_static_null
ros2 run detection detection
ros2 run path_planning global_planning
ros2 run pure_pursuit_action_server pure_pursuit_action_server_exe
ros2 run behaviour_tree bt2
```

