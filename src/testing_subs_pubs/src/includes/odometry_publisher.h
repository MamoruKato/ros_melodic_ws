#ifndef _ODOMETRY_PUBLISHER_H
#define _ODOMETRY_PUBLISHER_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

class OdometryPublisher
{

public:
  OdometryPublisher(int rValue);
  ~OdometryPublisher();
  void publishOdometryData();
private:
  ros::NodeHandle nh;
  ros::Publisher _odomPub;
  int _rateValue;
};


#endif
