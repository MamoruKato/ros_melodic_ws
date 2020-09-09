// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//TODO add pick code
void pick(moveit::planning_interface::MoveGroupInterface& group,
          moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
          moveit_msgs::CollisionObject& grasping_object,
          moveit::planning_interface::MoveGroupInterface::Plan& myPlan,
          ros::Publisher& display_publisher,
          moveit_msgs::MotionPlanResponse& response,
          planning_interface::MotionPlanResponse& res,
          moveit_msgs::DisplayTrajectory& display_trajectory,
          moveit_visual_tools::MoveItVisualTools visual_tools)
{
  std::string grasping_name = "grasping_object";

  const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

  geometry_msgs::Pose target_pose;

  target_pose.orientation.x = 0;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 1;

  target_pose.position.x = 0.32;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;

  group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveItErrorCode success = group.plan(myPlan); 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response); 


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

    // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visual_tools.trigger();

  ros::Duration(2).sleep();

  group.execute(myPlan);

  target_pose.position.x = 0.34;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;

  group.setPoseTarget(target_pose);


  //Plan the trajectory and check for sucess or failure
  success = group.plan(myPlan); 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response); 


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  //Publishes the calculated trajectory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visual_tools.trigger();

  ros::Duration(2).sleep();

  group.execute(myPlan);

  //Hold the object
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = group.getEndEffectorLink().c_str();
  //attached_object.object = planning_scene_interface.getObjects().find(grasping_name)->second;
  attached_object.object = grasping_object;

  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  ros::Duration(2).sleep();

  //group.execute(myPlan);

  //ros::Duration(2).sleep();



}


//TODO Add place code
void place(moveit::planning_interface::MoveGroupInterface& group,
           moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
           moveit_msgs::CollisionObject& grasping_object,
           moveit::planning_interface::MoveGroupInterface::Plan& myPlan,
           ros::Publisher& display_publisher,
           moveit_msgs::MotionPlanResponse& response,
           planning_interface::MotionPlanResponse& res,
           moveit_msgs::DisplayTrajectory& display_trajectory,
           moveit_visual_tools::MoveItVisualTools visual_tools)
{

  const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

  std::string grasping_name = "grasping_object";

  geometry_msgs::Pose target_pose;

  target_pose.position.y = -0.1;
  target_pose.position.x = 0.34;
  target_pose.position.z = 0.35;
  group.setPoseTarget(target_pose);

  //Calculate the kinematics for arriving the target position
  moveit::planning_interface::MoveItErrorCode success = group.plan(myPlan); 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");


  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response); 


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  //Publishs planed trajetory
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visual_tools.trigger();


  ros::Duration(2).sleep();
  group.execute(myPlan);


  //Remove object from grasping frame and place it on table
  moveit_msgs::AttachedCollisionObject attached_object;

  grasping_object.operation = grasping_object.REMOVE;

  attached_object.link_name = group.getEndEffectorLink().c_str();
  attached_object.object = grasping_object;


  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  //sets Another goal to the arm and realize the same steps
  target_pose.position.y = -0.1;
  target_pose.position.x = 0.32;
  target_pose.position.z = 0.35;

  group.setPoseTarget(target_pose);

  success = group.plan(myPlan); 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED"); 

  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response); 


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visual_tools.trigger();
  
  ros::Duration(2).sleep();

  group.execute(myPlan);

  //ros::Duration(2).sleep();


}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                         moveit_msgs::CollisionObject& grasping_object,
                         moveit_msgs::CollisionObject& grasping_table,
                         moveit::planning_interface::MoveGroupInterface& group)
{


  //Creates an object with collision
  //Sets its id
  grasping_object.id = "grasping_object";


  //Create a primitive shape form and set it sizes, in this case a box shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.08;

  //Create a position for adding the shape
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.y =  0.0;
  pose.position.x =  0.33;
  pose.position.z =  0.35;

  //Add Shape and pose to the collision object
  grasping_object.primitives.push_back(primitive);
  grasping_object.primitive_poses.push_back(pose);
  grasping_object.operation = grasping_object.ADD;
  //Sets the reference frame to the collision object
  grasping_object.header.frame_id = group.getPlanningFrame().c_str();




  //Creates another shape in this case the box created previously with different dimensions
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.3;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.32;

  //create another collision object named grasping_table
  grasping_table.id = "grasping_table";

  //Use the message pose to set another position
  pose.position.y =  0.0;
  pose.position.x =  0.46;
  pose.position.z =  0.15;

  //Add shape and pose to the collision object grasping_table
  grasping_table.primitives.push_back(primitive);
  grasping_table.primitive_poses.push_back(pose);
  grasping_table.operation = grasping_object.ADD;
  grasping_table.header.frame_id = group.getPlanningFrame().c_str();

  //Creates a vector of collision objects and add the grasping one
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(grasping_object);
  collision_objects.push_back(grasping_table);

  planning_scene_interface.addCollisionObjects(collision_objects);
  ros::Duration(2).sleep();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seven_dof_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  moveit::planning_interface::MoveGroupInterface group("arm");
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
  moveit_msgs::MotionPlanResponse response;
  planning_interface::MotionPlanResponse res;
  moveit_msgs::DisplayTrajectory display_trajectory;




  //ADD node subscriber into aruco_pose for getting pose from camera and performing movement
  
  addCollisionObjects(current_scene,grasping_object, grasping_table, group);
  pick(group, current_scene,grasping_object, myPlan, display_publisher,response, res, display_trajectory, visual_tools);
  place(group, current_scene,grasping_object, myPlan, display_publisher,response, res, display_trajectory, visual_tools);


  ros::waitForShutdown();
  ros::shutdown();
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
