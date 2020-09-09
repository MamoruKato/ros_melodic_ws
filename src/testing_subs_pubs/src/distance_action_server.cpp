#include "includes/distance_action_server.h"

DistanceActionServer::DistanceActionServer():
  _as(_nh,"distance_action_server",boost::bind(&DistanceActionServer::DistanceCallBack,this,_1),false),
  _rate(10)
{
  _odomSub = _nh.subscribe("/odom",100,&DistanceActionServer::exec_callback_odometry,this);
  _firstOdomReading = true;
  _walkedDistance = 0;
  _as.start();

  return;
}


DistanceActionServer::~DistanceActionServer()
{
  return;
}

//Call back function to the action server
/*
* Used for reading the Action goal
* The action goal has a float32 required_time and float32 distance
* the action server will run till the timeout requirement is met
* or the goal distance is reached when it happens
* the result message variable will be used to send a message to
* the client, telling him which was the end of the call.
* succeeded or not, and why.
*/
void DistanceActionServer::DistanceCallBack(const testing_subs_pubs::ActionTestGoal::ConstPtr& goal)
{
  bool success = false;
  std::string message = "Not completed Due to timeout error";
  ros::Time actualTime = ros::Time::now();
  _startTime = actualTime;

  ros::Duration rosTimeout(goal->required_time.data);
  _feedback.distance.data = 0;

  ROS_INFO("Started Server");

  if(rosTimeout.toSec() > 0)
  {
      ros::Duration executedTime = actualTime - _startTime;

      while(executedTime.toSec() < rosTimeout.toSec())
      {
        if(_as.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("distance_action_server: Preempted");
          _as.setPreempted();
          break;
        }

        if(_walkedDistance >= goal->goalDistance.data)
        {
          success = true;
          _feedback.distance.data = _walkedDistance;
          message = "Maze completed";
          ROS_INFO("Maze completed");
          break;
        }
        else
        {
          _feedback.distance.data = _walkedDistance;
          _as.publishFeedback(_feedback);
        }

        actualTime = ros::Time::now();
        executedTime = (actualTime - _startTime);
        _rate.sleep();
      }

      if(success)
      {
        _result.message.data = message;
        _as.setSucceeded(_result);
      }
      else
      {
        _result.message.data = message;
        ROS_INFO("distance_action_server: Aborted");
        _as.setAborted(_result);
      }

    }
    else
    {
      ROS_INFO("distance_action_server: Aborted for invalid timeout value");
      _as.setAborted(_result);
    }

  return;
}

/* function callback to the odometry subscriber
 * Will update the value from odom current value
 * which is used to measure the travelled distance by the robot
*/
void DistanceActionServer::exec_callback_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(_firstOdomReading)
  {
    _currentValue = *msg;
    calculateDistance(*msg);
    _firstOdomReading = false;
  }
  else
  {
    calculateDistance(*msg);
    _currentValue = *msg;
  }

  return;
}

//Private functions

/*
 *calculate distance based on odometry readings
 *the last_data used as argument to the function is
 *the last data readed by the odometry
 *the current value is the last one saved by the callback function
 *use vector math: |(final_point - start_point)| to calculate walked
 *distance and save it to _walkedDistance
*/
void DistanceActionServer::calculateDistance(nav_msgs::Odometry last_data)
{
  float distance_x = 0;
  float distance_y = 0;
  float last_x = last_data.pose.pose.position.x;
  float last_y = last_data.pose.pose.position.y;
  float current_x = _currentValue.pose.pose.position.x;
  float current_y = _currentValue.pose.pose.position.y;

  distance_y = abs(last_y - current_y);
  distance_x = abs(last_x - current_x);

  //no caso da funcao abs nao funcionar
  /*
  if(distance_y < 0)
    distance_y = -distance_y;
  if(distance_x < 0)
    distance_x = -distance_x;
  */

  _walkedDistance = _walkedDistance + (distance_y + distance_x);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distance_server");

  DistanceActionServer _obServer;
  ros::spin();

  return 0;
}
