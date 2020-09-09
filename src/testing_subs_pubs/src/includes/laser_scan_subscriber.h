#ifndef LASER_SCAN_SUBSCRIBER_H
#define LASER_SCAN_SUBSCRIBER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

class LaserScanSubscriber
{
  const unsigned int number_of_readings = 140;
  const unsigned int laser_frequency = 40;
public:
  LaserScanSubscriber();
  ~LaserScanSubscriber();
  void do_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  sensor_msgs::LaserScan getLaserScanData();
private:
  ros::NodeHandle nh;
  ros::Subscriber _laserScanSub;
  sensor_msgs::LaserScan _laserData;

};



#endif
