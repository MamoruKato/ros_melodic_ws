#ifndef _DISTANCE_ACTION_SERVER
#define _DISTANCE_ACTION_SERVER

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include "testing_subs_pubs/ActionTestAction.h"

class DistanceActionServer
{
public:
  DistanceActionServer();
  ~DistanceActionServer();
  void DistanceCallBack(const testing_subs_pubs::ActionTestGoal::ConstPtr& goal);
  void exec_callback_odometry(const nav_msgs::Odometry::ConstPtr& msg);
private:
  void calculateDistance(nav_msgs::Odometry last_data);

private:
  ros::NodeHandle _nh;
  actionlib::SimpleActionServer<testing_subs_pubs::ActionTestAction> _as;
  ros::Rate _rate;
  ros::Time _startTime;
  ros::Subscriber _odomSub;
  nav_msgs::Odometry _currentValue;
  testing_subs_pubs::ActionTestFeedback _feedback;
  testing_subs_pubs::ActionTestResult _result;
  float _walkedDistance;
  bool _firstOdomReading;

};



#endif
