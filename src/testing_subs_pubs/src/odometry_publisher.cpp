#include "includes/odometry_publisher.h"

OdometryPublisher::OdometryPublisher(int rValue)
{
  _odomPub = nh.advertise<nav_msgs::Odometry>("odom",1);
  _rateValue = rValue;
  return;
}

OdometryPublisher::~OdometryPublisher()
{
  return;
}

void OdometryPublisher::publishOdometryData()
{
  ros::Rate _r(_rateValue);
  nav_msgs::Odometry _odom_msg;
  _odom_msg.pose.pose.position.x = 1.0;
  _odom_msg.pose.pose.position.y = 2.0;
  _odom_msg.pose.pose.position.z = -1.0;

  _odom_msg.pose.pose.orientation.x = 0.5;
  _odom_msg.pose.pose.orientation.y = 0.2;
  _odom_msg.pose.pose.orientation.z = 0.1;
  _odom_msg.pose.pose.orientation.w = 0.0;

  while(ros::ok)
  {
    _odomPub.publish(_odom_msg);
    _r.sleep();
  }

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_data_publisher");
  OdometryPublisher _odomPublisher(10);

  _odomPublisher.publishOdometryData();

  return 0;
}
