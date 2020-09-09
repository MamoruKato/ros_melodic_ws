#include "includes/laser_scan_publisher.h"

LaserScanPublisher::LaserScanPublisher(int rateValue)
{
  _rateValue = rateValue;
  _laserScanPub = nh.advertise<sensor_msgs::LaserScan>("scan",1);
  return;
}


LaserScanPublisher::~LaserScanPublisher()
{
  return;
}

void LaserScanPublisher::LaserScanPub()
{
  ros::Time scan_time = ros::Time::now();
  ros::Rate _rate(_rateValue);

  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.stamp = scan_time;
  scan_msg.header.frame_id = "laser_frame";
  scan_msg.angle_min = -1.57;
  scan_msg.angle_max = 1.57;
  scan_msg.angle_increment = 3.14 / number_of_readings;
  scan_msg.time_increment = (1/laser_frequency)/number_of_readings;
  scan_msg.range_min = 0.0;
  scan_msg.range_max = 100.0;

  scan_msg.ranges.resize(number_of_readings);
  scan_msg.intensities.resize(number_of_readings);

  for(int i = 0; i < number_of_readings; i++)
  {
    scan_msg.ranges[i] = i;
    scan_msg.intensities[i] = 100 + i;
  }


  while(ros::ok())
  {
    _laserScanPub.publish(scan_msg);
    _rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"laser_scan_publisher");
  LaserScanPublisher _laserScanPub(10);
  _laserScanPub.LaserScanPub();

  return 0;
}
