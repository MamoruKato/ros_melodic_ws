#include "includes/laser_scan_subscriber.h"
#include "includes/odom_subscriber.h"
#include "includes/move_robot.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"main_node");
  LaserScanSubscriber _laserSub;
  OdometrySubscriber _odomSub;
  MoveRobot _moveRobotPub;

  ros::Rate _rate(10);

  while(ros::ok())
  {
    //You can use the data from laser scan to publish or move the robot around
    ROS_INFO_STREAM("Laser Data received: \n" << _laserSub.getLaserScanData());
    ROS_INFO_STREAM("Odom Data received: \n" << _odomSub.getOdometryData());
    _moveRobotPub.MoveOrientation("forward",0.3);
    ros::spinOnce();
    _rate.sleep();
  }

  return 0;;
}
