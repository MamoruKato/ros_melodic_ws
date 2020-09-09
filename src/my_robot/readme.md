This file has the purpose to explain a little bit of what has been done here:
First of all:
	1 - We wrote the robot description file named: "my_robot.xacro"
		Inside this was added all the subjects related to its phisical
		description, like visual, collision and inertial.
		The next step after writing its characteristics and adding related
		type of joints was to attach transimissions types on each joint that
		is supposed to move, in this case wheel joints and cylinder link which
		hokuyo link is attached to.
	2 - to make it moveable through topic publish msg, can be done by two different ways
		One of them, and previously used by us, is to set a gazebo plugin for each moveable
		joint. by example you can attach br_wheel_joint and bl_wheel_joint to diff_driver_plugin
		which i do not recommend, but make it work. this way you may be able to publish geometry_msgs/Twist
		into a related topic resulting in a drivable robot by publishing msg into the topic. The problem is
		that you might not be able to use moveit setup assistant to implement path planning and other desired 
		funcionalities.
		What was made than was writing a configuration file named: "my_robot_controllers.yaml" where was declared
		our joint_state_controller/JointStateController which would publish the joint states into the the topic in
		a rate of 50 per second allowing us to remap robot position on rviz by getting the data published by
		robot_state_publisher topic which subscribes to joint_state_publisher and converts to links positioning (tf).
		The last thing done in this file was writing the controllers for back left and back rigth wheel joints, which
		are the desired to move.
	3 - Writing the launch file
		In the launch file we named: "my_robot_control.launch" is launched all the important section to make the robot
		driveable. These are the following: launch the world, spawn the controllers configuration files into Ros
		parameter server , executing a node to load the controllers, launching robot state publisher to transform incoming data
		from joint state publisher, which needs to be launched as well, into tf format, for gazebo and rviz visualization and
		remap for continuous movement, and them spawn the robot on gazebo. 

	4 - Added new feature to the robot
		You can use the camera on my_robot2 as a detector for aruco markers. Its planned to use it for automotion of the
		diff robot for moving nearest the aruco marker and then stop a few centimeters from it, allowing the arm to execute
		its planned trajectory.
		Another feature that can be added is to use the image_raw detected by the camera and use it for generating pointCloud
		perception, for gmapping and autonomous navigation.

