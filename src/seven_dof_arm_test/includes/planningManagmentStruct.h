#ifndef PLANNINGMANAGMENTSTRUCT_H


//Struct used for all planning scene
struct planningManagmentStruct
{

  //Moveit Section
  moveit::planning_interface::MoveGroupInterface* group;
  moveit::planning_interface::MoveGroupInterface* gripper_group;
  moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
  moveit_msgs::CollisionObject* grasping_object; //needed just for pick and place
  moveit_msgs::CollisionObject* grasping_table;  //needed just for pick and place

  moveit::planning_interface::MoveGroupInterface::Plan* myPlan;
  moveit_visual_tools::MoveItVisualTools* visual_tools;

  //Variables for setting positions
  geometry_msgs::Pose* target_pose;
  geometry_msgs::Pose* init_pose;
  std::vector<geometry_msgs::Pose> waypoints;


  //Publisher for rviz display trajectory
  ros::Publisher* display_publisher;

};


#endif
