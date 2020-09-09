#ifndef _MOVE_ROBOT_H
#define _MOVE_ROBOT_H

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class MoveRobot
{
public:
  MoveRobot();
  ~MoveRobot();
  void MoveStraight(float vel);
  void TurnLeft(float vel);
  void TurnRight(float vel);
  void MoveBack(float vel);
  void MoveOrientation(std::string choose_orientation, float vel);
  void Stop();

private:
  ros::NodeHandle _nh;
  ros::Publisher _velPub;
  ros::Rate _rate;
};


#endif
