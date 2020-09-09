#include "includes/laser_scan_subscriber.h"

LaserScanSubscriber::LaserScanSubscriber()
{
  _laserScanSub = nh.subscribe("/scan",100,&LaserScanSubscriber::do_callback, this);

  //inicialize ranges vector with zero values to avoid segmentation_fault
  for(int i = 0; i < number_of_readings; i++)
  {
    _laserData.ranges.push_back(0);
  }

  for(int i = 0; i < number_of_readings; i++)
  {
    _laserData.intensities.push_back(0);
  }

  ROS_INFO_STREAM("Subscriber Started: " << _laserData);
  return;
}

LaserScanSubscriber::~LaserScanSubscriber()
{
  return;
}

void LaserScanSubscriber::do_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  //reset vectors so they can be filled with the msg values
  _laserData.ranges.clear();
  _laserData.intensities.clear();

  for(int i = 0; i < number_of_readings; i++)
  {
    _laserData.ranges.push_back(msg->ranges[i]);
  }

  for(int i = 0; i < number_of_readings; i++)
  {
    _laserData.intensities.push_back(msg->intensities[i]);
  }

  ROS_INFO("I heard LaserScan messages");

  return;
}

sensor_msgs::LaserScan LaserScanSubscriber::getLaserScanData()
{
  return _laserData;
}


/*
int main(int argc, char** argv)
{
  ros::init(argc,argv,"laser_scan_subscriber");
  LaserScanSubscriber laserSubscriber;
  return 0;
}
*/
