#include "includes/move_robot.h"


MoveRobot::MoveRobot(): _rate(1)
{
  _velPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  return;
}

MoveRobot::~MoveRobot()
{
  return;
}

void MoveRobot::MoveStraight(float vel)
{
  geometry_msgs::Twist _velCommand;
  _velCommand.linear.x = vel;

  Stop();
  _rate.sleep();

  _velPub.publish(_velCommand);
}

void MoveRobot::TurnLeft(float vel)
{
  geometry_msgs::Twist _velCommand;
  _velCommand.angular.z = -vel;

  Stop();
  _rate.sleep();

  _velPub.publish(_velCommand);
}

void MoveRobot::TurnRight(float vel)
{
  geometry_msgs::Twist _velCommand;
  _velCommand.angular.z = vel;

  Stop();
  _rate.sleep();

  _velPub.publish(_velCommand);
}

void MoveRobot::MoveBack(float vel)
{
  geometry_msgs::Twist _velCommand;
  _velCommand.linear.x = -vel;

  Stop();
  _rate.sleep();

  _velPub.publish(_velCommand);
}

void MoveRobot::Stop()
{
  geometry_msgs::Twist _velCommand;
  _velCommand.linear.x = 0;
  _velCommand.angular.z = 0;

  _velPub.publish(_velCommand);
}

void MoveRobot::MoveOrientation(std::string choose_orientation, float vel)
{
  if(choose_orientation == "forward")
    MoveStraight(vel);
  else if(choose_orientation == "left")
    TurnLeft(vel);
  else if(choose_orientation == "right")
    TurnRight(vel);
  else
    MoveBack(vel);

  return;

}
