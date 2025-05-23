#!/bin/bash

# this forces bash to use the Desktop directory i.e. where this bash script is located
cd "${0%/*}"

source /opt/ros/jazzy/setup.bash
source ../install/local_setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run perception plotting
