#include "ros/ros.h"
#include "std_msgs/String.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <octomap_msgs/Octomap.h>

#define LOOP_RATE 2

octomap::OcTree tree (0.1);   // declare your octomap at 0.1 resolution
octomap::point3d _sensorOrigin(0.0663452131594,0.0039590353764,-0.149999638869);
octomath::Vector3 _sensorPose(0,0,0);							//Can get from robot_state_publisher, just find out what topic it publishs in
octomath::Quaternion _sensorOrientation(0,0,0,0);				//Can get from robot_state_publisher, just find out in what topic its been  published
bool _atualizedPose = false;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& pointCloud)
{	
    octomap::Pointcloud octomap_scan;
	octomap::pointCloud2ToOctomap(*pointCloud,octomap_scan);

	octomath::Pose6D _sensor6DPose(_sensorPose,_sensorOrientation);
	tree.insertPointCloud(octomap_scan,_sensorOrigin,_sensor6DPose);

}
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "pcl2octo");
	ros::NodeHandle nh;
	tf::TransformListener listener;

	
	ros::Subscriber getPointCloud = nh.subscribe("velodyne_points",1000,PointCloudCallBack);




    //\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\//\
    // read in your data here either rosbag or pcl
    // http://wiki.ros.org/rosbag/Code%20API#cpp_api
    // http://pointclouds.org/documentation/tutorials/pcd_file_format.php
    //\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

	ros::Rate loop_rate(LOOP_RATE);
	while(ros::ok())
	{
		
		tf::StampedTransform transform;
	
		try
		{
			listener.lookupTransform("/base_link","/base_link",ros::Time::now() ,transform);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1).sleep();
		}
	
		octomath::Vector3 _tempSensorPose(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
		_sensorPose = _tempSensorPose;
		ros::spinOnce();
		loop_rate.sleep();
	}
 	tree.writeBinary("simple_tree.bt");
    return 0;

}
