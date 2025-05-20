## Introduction 

This repository demonstrates how to run a camera-based differential robot on ROS2.
The goal is to enable the robot to drive based on what it sees.


![Line Follower Demo](assets/line_follower_demo.gif)



## Installation 


- **Ubuntu**, tutorial here https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

- **ROS 2**, Humble page https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.htmlLinks 

- **Kobuki** for simulation

        sudo apt install ros-humble-kobuki-ros-interfaces

- ROS 2 **packages**, 

        cd ~/workshop_ws/src
        git clone --recurse-submodules https://github.com/bingxue-xu/robp_workshop.git 
        cd ~/workshop_ws
        rosdep install --from-paths src -y --ignore-src --as-root pip:false
        colcon build --symlink-install

        source /opt/ros/jazzy/setup.bash
        source ~/workshop_ws/install/local_setup.bash




## Run

**Simulation** 

    ros2 launch robp_boot_camp_launch workshop_sim_launch.xml

**Robot**

    ros2 launch robp_boot_camp_launch workshop_launch.xml

**Plotting (both simulation and robot)**

    ros2 run perception plotting


## Implement to your own solution

You can customize the system to your own needs by modifying perception and control, for example, following an object instead of a line

- **perception**:  *src/perception/perception/perception.py*

- **Controller**: *src/controller/controller/controller.py*