#include "includes/odom_subscriber.h"

OdometrySubscriber::OdometrySubscriber()
{
  _odomSub = nh.subscribe("odom",100,&OdometrySubscriber::do_callback,this);
  _odomData.pose.pose.position.x = 0;
  _odomData.pose.pose.position.y = 0;
  _odomData.pose.pose.position.z = 0;

  _odomData.pose.pose.orientation.x = 0;
  _odomData.pose.pose.orientation.y = 0;
  _odomData.pose.pose.orientation.z = 0;
  _odomData.pose.pose.orientation.w = 0;

  return;
}

OdometrySubscriber::~OdometrySubscriber()
{
  return;
}

void OdometrySubscriber::do_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  _odomData.pose.pose.position.x = msg->pose.pose.position.x;
  _odomData.pose.pose.position.y = msg->pose.pose.position.y;
  _odomData.pose.pose.position.z = msg->pose.pose.position.z;

  _odomData.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  _odomData.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  _odomData.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  _odomData.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  ROS_INFO("[Status] Received odometry message");
}

nav_msgs::Odometry OdometrySubscriber::getOdometryData()
{
  return _odomData;
}
