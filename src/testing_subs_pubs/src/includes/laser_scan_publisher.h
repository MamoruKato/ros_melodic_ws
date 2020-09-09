#ifndef LASER_SCAN_PUBLISHER_H
#define LASER_SCAN_PUBLISHER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

class LaserScanPublisher
{
  const unsigned int number_of_readings = 140;
  const unsigned int laser_frequency = 40;
public:
  LaserScanPublisher(int rateValue);
  ~LaserScanPublisher();
  void LaserScanPub();
private:
  ros::NodeHandle nh;
  ros::Publisher _laserScanPub;
  int _rateValue;

};





#endif
