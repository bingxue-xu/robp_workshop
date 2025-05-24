## Introduction 

This repository demonstrates how to run a camera-based differential robot on ROS2.
The goal is to enable the robot to drive based on what it sees.



https://github.com/user-attachments/assets/4e851c0b-155f-403f-a164-380398083e3c




## Installation 

- **Ubuntu**, tutorial here https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

- **ROS 2**, Jazzy page https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html  

- **Kobuki** for simulation

        sudo apt install ros-jazzy-kobuki-ros-interfaces

- **SSHPass** for SSH

        sudo apt install sshpass

- ROS 2 **packages**, 

        cd ~/workshop_ws
        git clone https://github.com/bingxue-xu/robp_workshop.git src
        cd ~/workshop_ws
        rosdep install --from-paths src -y --ignore-src --as-root pip:false
        colcon build --symlink-install

        source /opt/ros/jazzy/setup.bash
        source ~/workshop_ws/install/local_setup.bash

## Run

**Simulation** 

    ros2 launch robp_boot_camp_launch workshop_sim_launch.xml
    
**Student code** 

    ros2 launch robp_boot_camp_launch workshop_student_launch.xml

**Robot**

    ros2 launch robp_boot_camp_launch workshop_launch.xml

**Plotting (both simulation and robot)**

    ros2 run perception plotting

**Debugging (both simulation and robot)**

    ros2 topic echo /motor_controller/twist


## Implement to your own solution

You can customize the system to your own needs by modifying perception and control, for example, following an object instead of a line

- **Perception**:  *src/perception/perception/perception.py*

- **Controller**: *src/controller/controller/controller.py*

## Documentation 
Here are some useful links:
- ROS (Robot Operating System) wiki: https://wiki.ros.org/ 
- The HSV Color Model: [the HSV Color Model](https://medium.com/@dijdomv01/a-beginners-guide-to-understand-the-color-models-rgb-and-hsv-244226e4b3e3)
