#ifndef _ODOMETRY_SUBSCRIBER_H
#define _ODOMETRY_SUBSCRIBER_H

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <string>
#include "nav_msgs/Odometry.h"

class OdometrySubscriber
{
public:
  OdometrySubscriber();
  ~OdometrySubscriber();
  void do_callback(const nav_msgs::Odometry::ConstPtr& msg);
  nav_msgs::Odometry getOdometryData();

private:
  ros::NodeHandle nh;
  ros::Subscriber _odomSub;
  nav_msgs::Odometry _odomData;
};



#endif
