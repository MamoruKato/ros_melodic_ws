# SIMPLE ARUCO DETECTOR for ROS

A ROS node that detects ARUCO markers and publishes their poses as TFs.

Tested on ROS Kinetic, Ubuntu 16.04.

Licence MIT.

There is different nodes in this package
1 - aruco_marker_detector is a node used for detecting aruco markers and publisng its pose
on a topic named aruco_pose with msg type of simple_aruco_detector::aruco_msg, created before.
  - for running it you just launch master with roscore and then run the node:
  - rosrun simple_aruco_detector aruco_marker_detector

2 - an other node is camera calibration used for generating a config file
 
3 - simple_aruco_detector_node is the original file that uses camera image topic
for getting aruco markers detection, and publishing it as tf coordinates in an
other topic

4 - I have added now a new feature i wanted to test, gazebo_aruco_detector is
used for getting information from camera_usb simulated at gazebo, so you can
launch my_robot that has a camera on it, and can be used for identifiyng
aruco_marker on gazebo simulation, or so i hope. I need some more tests.

5 - Added a new script for making a simple interface, when identifing an aruco marker
it subscribe to /aruco_pose to get information, and displays the marker indentified,its
called aruco_interface.py


