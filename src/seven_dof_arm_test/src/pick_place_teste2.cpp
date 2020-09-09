//C++
#include <iostream>
#include <boost/bind.hpp>
// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//aruco msgs implemented at simple_aruco_detector package
#include <simple_aruco_detector/aruco_msg.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//include struct of type planningManagementStruct which contains all variables for planning
#include "../includes/planningManagmentStruct.h"



void close_gripper(planningManagmentStruct& _planningManagementStruct)
{
  const robot_state::JointModelGroup* joint_model_group =
    _planningManagementStruct.gripper_group->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = _planningManagementStruct.gripper_group->getCurrentState();
  std::vector<double> joint_group_positions;

  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -0.009;

  _planningManagementStruct.gripper_group->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveItErrorCode success = _planningManagementStruct.gripper_group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  ros::Duration(2).sleep();

  _planningManagementStruct.gripper_group->execute(*_planningManagementStruct.myPlan);

  ros::Duration(2).sleep();

  return;
}

void open_gripper(planningManagmentStruct& _planningManagementStruct)
{

  return;
}


//Now is just needed to set a position according to aruco marker
void pick(planningManagmentStruct& _planningManagementStruct)
{
  std::string grasping_name = "grasping_object";

  moveit_msgs::MotionPlanResponse response;
  planning_interface::MotionPlanResponse res;
  moveit_msgs::DisplayTrajectory display_trajectory;

  const robot_state::JointModelGroup *joint_model_group = _planningManagementStruct.group->getCurrentState()->getJointModelGroup("arm");

  geometry_msgs::Pose _move_position;
  _move_position.orientation = _planningManagementStruct.target_pose->orientation;

  _move_position.position.x = _planningManagementStruct.target_pose->position.x - 0.1;
  _move_position.position.y = _planningManagementStruct.target_pose->position.y;
  _move_position.position.z = _planningManagementStruct.target_pose->position.z;

  _planningManagementStruct.group->setPoseTarget(_move_position);

  moveit::planning_interface::MoveItErrorCode success = _planningManagementStruct.group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

    // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();

  ros::Duration(2).sleep();

  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);

  _move_position.position.x = _planningManagementStruct.target_pose->position.x;
  _move_position.position.y = _planningManagementStruct.target_pose->position.y;
  _move_position.position.z = _planningManagementStruct.target_pose->position.z;

  _planningManagementStruct.group->setPoseTarget(_move_position);


  //Plan the trajectory and check for sucess or failure
  success = _planningManagementStruct.group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

  // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  //Publishes the calculated trajectory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();

  ros::Duration(2).sleep();

  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);

  ros::Duration(5).sleep();
  close_gripper(_planningManagementStruct);

  //Hold the object
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = _planningManagementStruct.group->getEndEffectorLink().c_str();
  //attached_object.object = planning_scene_interface.getObjects().find(grasping_name)->second;
  attached_object.object = *(_planningManagementStruct.grasping_object);

  _planningManagementStruct.planning_scene_interface->applyAttachedCollisionObject(attached_object);
  ros::Duration(2).sleep();

  //group.execute(myPlan);

  //ros::Duration(2).sleep();



}

//set a position according to aruco marker
void place(planningManagmentStruct& _planningManagementStruct)
{

  moveit_msgs::MotionPlanResponse response;
  planning_interface::MotionPlanResponse res;
  moveit_msgs::DisplayTrajectory display_trajectory;

  const robot_state::JointModelGroup *joint_model_group = _planningManagementStruct.group->getCurrentState()->getJointModelGroup("arm");

  std::string grasping_name = "grasping_object";

  geometry_msgs::Pose _move_position;

  _move_position.position.y = _planningManagementStruct.target_pose->position.y -0.1;
  _move_position.position.x = _planningManagementStruct.target_pose->position.x + 0.1;
  _move_position.position.z = _planningManagementStruct.target_pose->position.z;
  _planningManagementStruct.group->setPoseTarget(_move_position);

  //Calculate the kinematics for arriving the target position
  moveit::planning_interface::MoveItErrorCode success = _planningManagementStruct.group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

  // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  //Publishs planed trajetory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();


  ros::Duration(2).sleep();
  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);


  //Remove object from grasping frame and place it on table
  moveit_msgs::AttachedCollisionObject attached_object;

  _planningManagementStruct.grasping_object->operation = _planningManagementStruct.grasping_object->REMOVE;

  attached_object.link_name = _planningManagementStruct.group->getEndEffectorLink().c_str();
  attached_object.object = *_planningManagementStruct.grasping_object;


  _planningManagementStruct.planning_scene_interface->applyAttachedCollisionObject(attached_object);

  //sets Another goal to the arm and realize the same steps
  _move_position.position.y = _planningManagementStruct.target_pose->position.y -0.1;
  _move_position.position.x = _planningManagementStruct.target_pose->position.x - 0.1;
  _move_position.position.z = _planningManagementStruct.target_pose->position.z;

  _planningManagementStruct.group->setPoseTarget(_move_position);

  success = _planningManagementStruct.group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");

  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

  // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();

  ros::Duration(2).sleep();

  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);

  //ros::Duration(2).sleep();


}

//Just go back to original pose
void back_to_init_pose(planningManagmentStruct& _planningManagementStruct)
{
  const robot_state::JointModelGroup *joint_model_group = _planningManagementStruct.group->getCurrentState()->getJointModelGroup("arm");

  moveit_msgs::MotionPlanResponse response;
  planning_interface::MotionPlanResponse res;
  moveit_msgs::DisplayTrajectory display_trajectory;

  moveit_msgs::RobotTrajectory trajectory;
  geometry_msgs::Pose target_pose;

  _planningManagementStruct.group->setPoseTarget(*_planningManagementStruct.init_pose);
  moveit::planning_interface::MoveItErrorCode success = _planningManagementStruct.group->plan(*_planningManagementStruct.myPlan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

  // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  //Publishs planed trajetory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();

  ros::Duration(2).sleep();

  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);

  ros::Duration(2).sleep();

  // now go to the position again using a cartesiasn path
  target_pose.position.x = 0.33;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;
  _planningManagementStruct.waypoints.push_back(target_pose);

  std::string grasping_name = "grasping_object";

  //Calculate the kinematics for arriving the target position using compute cartesian path
  // It calculates a bunch of trajectory points for arriving at the goal
  // The numbers of points are calculated though the "0.01" param which is the distance between the points
  double fraction = _planningManagementStruct.group->computeCartesianPath(_planningManagementStruct.waypoints,
                                                                          0.01,
                                                                          0.0,
                                                                          trajectory);

  //Moveit::planning_interface::MoveGroup::Plan struct has an atributte named trajectory_
  // So you can use computeCartesianPath for generating a trajectory and pass it to the plan
  //after that you just execute it and you can also display it at rviz as done in this code
  _planningManagementStruct.myPlan->trajectory_ = trajectory;


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  _planningManagementStruct.display_publisher->publish(display_trajectory);

  // Visualize the plan in RViz
  _planningManagementStruct.visual_tools->deleteAllMarkers();
  //Publishs planed trajetory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  _planningManagementStruct.visual_tools->publishTrajectoryLine(_planningManagementStruct.myPlan->trajectory_, joint_model_group);
  _planningManagementStruct.visual_tools->trigger();
  ros::Duration(3).sleep();

  _planningManagementStruct.group->execute(*_planningManagementStruct.myPlan);

}

//Need to add feature for diferent position of the object according to aruco pose
void addCollisionObjects(planningManagmentStruct& _planningManagementStruct)
{


  //Creates an object with collision
  //Sets its id
  _planningManagementStruct.grasping_object->id = "grasping_object";


  //Create a primitive shape form and set it sizes, in this case a box shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.08;

  //Create a position for adding the shape
  geometry_msgs::Pose pose;
  pose.orientation.w = _planningManagementStruct.target_pose->orientation.w;
  pose.position.y =  _planningManagementStruct.target_pose->position.y;
  pose.position.x =  _planningManagementStruct.target_pose->position.x;
  pose.position.z =  _planningManagementStruct.target_pose->position.z;
  /*
  pose.orientation.w = 1.0;
  pose.position.y =  0.0;
  pose.position.x =  0.33;
  pose.position.z =  0.35;
  */
  //Add Shape and pose to the collision object
  _planningManagementStruct.grasping_object->primitives.push_back(primitive);
  _planningManagementStruct.grasping_object->primitive_poses.push_back(pose);
  _planningManagementStruct.grasping_object->operation = _planningManagementStruct.grasping_object->ADD;
  //Sets the reference frame to the collision object
  _planningManagementStruct.grasping_object->header.frame_id = _planningManagementStruct.group->getPlanningFrame().c_str();




  //Creates another shape in this case the box created previously with different dimensions
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.3;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.32;

  //create another collision object named grasping_table
  _planningManagementStruct.grasping_table->id = "grasping_table";

  //Use the message pose to set another position
  pose.position.y =  _planningManagementStruct.target_pose->position.y;
  pose.position.x =  _planningManagementStruct.target_pose->position.x + 0.13;
  pose.position.z =  _planningManagementStruct.target_pose->position.z - 0.2;

  //Add shape and pose to the collision object grasping_table
  _planningManagementStruct.grasping_table->primitives.push_back(primitive);
  _planningManagementStruct.grasping_table->primitive_poses.push_back(pose);
  _planningManagementStruct.grasping_table->operation = _planningManagementStruct.grasping_object->ADD;
  _planningManagementStruct.grasping_table->header.frame_id = _planningManagementStruct.group->getPlanningFrame().c_str();

  //Creates a vector of collision objects and add the grasping one
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(*_planningManagementStruct.grasping_object);
  collision_objects.push_back(*_planningManagementStruct.grasping_table);

  _planningManagementStruct.planning_scene_interface->addCollisionObjects(collision_objects);
  ros::Duration(2).sleep();

}

//Callback function for getting the aruco marker position all call all the other functions
void aruco_get_poseCallBack(const simple_aruco_detector::aruco_msg::ConstPtr& msg, planningManagmentStruct& _planningManagementStruct)
{
  ROS_INFO("Aruco get Pose call Back");
  //gets Aruco marker position and sets it as arm's target position
  _planningManagementStruct.target_pose->position.x = msg->pose.x;
  _planningManagementStruct.target_pose->position.y = msg->pose.y;
  _planningManagementStruct.target_pose->position.z = msg->pose.z;
  _planningManagementStruct.target_pose->orientation = msg->orientation;

  addCollisionObjects(_planningManagementStruct);
  pick(_planningManagementStruct);
  place(_planningManagementStruct);
  back_to_init_pose(_planningManagementStruct);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seven_dof_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planningManagmentStruct _planningManagementStruct;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPoseReferenceFrame("base_link");

  //Added to close and open the gripper using joint states
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  gripper_group.setPoseReferenceFrame("grasping_frame");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit::planning_interface::MoveGroupInterface::Plan myPlan;


  ros::Duration(2).sleep();

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(group.getPlanningFrame().c_str());
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);


  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* Sleep a little to allow time to startup rviz, etc..
     This ensures that visual_tools.prompt() isn't lost in a sea of logs*/
  ros::Duration(5).sleep();

  moveit::planning_interface::PlanningSceneInterface current_scene;
  moveit_msgs::CollisionObject grasping_table;
  moveit_msgs::CollisionObject grasping_object;
  geometry_msgs::Pose target_pose;
  geometry_msgs::PoseStamped init_pose = group.getCurrentPose(); //get arm initial position





  //ADD node subscriber into aruco_pose for getting pose from camera and performing movement
  //Sets all the variable to the struct
  //Can be used later for aruco_pose identification inside callback function
  _planningManagementStruct.group = &group;
  _planningManagementStruct.gripper_group = &gripper_group;
  _planningManagementStruct.planning_scene_interface = &current_scene;
  _planningManagementStruct.grasping_object = &grasping_object;
  _planningManagementStruct.grasping_table = &grasping_table;
  _planningManagementStruct.myPlan = &myPlan;
  _planningManagementStruct.visual_tools = &visual_tools;
  _planningManagementStruct.display_publisher = &display_publisher;
  _planningManagementStruct.target_pose = &target_pose;
  _planningManagementStruct.init_pose = &init_pose.pose;

  //Make a subscriber to aruco detector node
  //directory to the aruco_detector_node ~/melodic_ws/src/simple_aruco_detector/aruco_marker_detector.cpp
  //node aruco_marker_detector_node
  //Boost library is used to pass more than one argument to call back function, so can use
  //the message from aruco node and change the planningManagementStruct.
  ros::Subscriber aruco_pose_sub = nh.subscribe<simple_aruco_detector::aruco_msg>("aruco_pose",100, boost::bind(aruco_get_poseCallBack,_1,_planningManagementStruct));
  ROS_INFO("Node ready to get aruco poses");

  while (ros::ok())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  ros::waitForShutdown();
//  ros::shutdown();
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL
