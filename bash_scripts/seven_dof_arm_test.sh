#!/bin/bash

RED='\033[0;31m'

if [ $# -gt 0 ]
then
	ARG1=$1
else
	ARG1='zero'
fi

if [ $ARG1 == 'execute_arm' ]
then
	roslaunch seven_dof_arm_config demo.launch
	echo -e "You can use the programmed trajectory running the command: ${RED}rosrun seven_dof_arm_test pick_place_test2"
	echo -e "The programm you wait for a detected aruco marker or a published msg at --aruco_pose-- topic "
	echo -e "You can run: ${RED}rosrun simple_aruco_detector aruco_marker_detector"
	echo -e "Or simply pubilsh an aruco msg at the topic: ${RED}rostopic pub -1 /aruco_pose ..."
elif [ $ARG1 == 'execute_my_robot_velodyne' ]
then
	roslaunch my_robot my_robot.launch
	echo -e "You can control the robot running: ${RED}roslaunch my_robot my_robot2_teleop.launch"
elif [ $ARG1 == 'execute_my_robot_aruco' ]
then
	roslaunch my_robot my_robot2.launch
	echo -e "After finishing launch the robot run: ${RED}rosrun simple_aruco_detector gazebo_aruco_detector"
	echo -e "You can control the robot running: ${RED}roslaunch my_robot my_robot2_teleop.launch"

else
	echo -e "${RED}Please enter a valid argument"
fi

