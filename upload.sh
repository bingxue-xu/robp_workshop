#!/bin/bash

# this forces bash to use the Desktop directory i.e. where this bash script is located
cd "${0%/*}"

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

group_name=""

while [ "$group_name" = "" ]
do 
	read -p "Enter group name: " group_name
done

echo -e "${GREEN}Uploading group '${group_name}' solution to robot${NC}"

sshpass -p "ros2" ssh -o StrictHostKeyChecking=no rosuser@192.168.128.108 "mkdir -p ~/${group_name}"
sshpass -p "ros2" scp /home/robot/workshop_ws/src/perception/perception/perception.py rosuser@192.168.128.108:~/${group_name}/perception.py

if [ "$?" != "0" ]
then
	echo -e "${RED}Could not connect to the robot${NC}"
	sleep 10
	exit $?
fi

sshpass -p "ros2" scp /home/robot/workshop_ws/src/controller/controller/controller.py rosuser@192.168.128.108:~/${group_name}/controller.py

if [ "$?" != "0" ]
then
	echo -e "${RED}Could not connect to the robot${NC}"
	sleep 10
	exit $?
fi

echo -e "${GREEN}Solution has been uploaded to robot as group '${group_name}'${NC}"

sleep 3
