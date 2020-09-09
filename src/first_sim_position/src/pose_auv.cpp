#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


#include <sstream>

#define LOOP_RATE 2
#define PI 3.1415926

geometry_msgs::Twist msg;
int gira;
int iLowestRange;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	int i = 0;
	float lowest_distance = 0;

	lowest_distance = data->ranges[0];

	for (i = 0;i < sizeof(data->ranges); i++)
	{
		if(data->ranges[i] < lowest_distance && lowest_distance < data->range_max)
		{
			if(lowest_distance <= 2.2)
			{
				iLowestRange = - 2;
			}
			else
			{
				iLowestRange = i;
				lowest_distance = data->ranges[i];
			}
		} 
	}

	ROS_INFO("Data get from Laser Scan: [%f] : ", data->ranges[iLowestRange]);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	gira = 1;
	iLowestRange = -1;
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber scan_sub = n.subscribe("scan",1000,scanCallback);


  ros::Rate loop_rate(LOOP_RATE);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

		if(gira == 1)
		{
			msg.linear.x = 0;
			msg.angular.z = 0.01;
			gira = 0;
		}
		else
		{
			if(iLowestRange != -1)
			{
				
				if(iLowestRange != -2)
				{
					if(count == 0)
					{
						msg.linear.x = 0;
						msg.angular.z =  (PI/2 -iLowestRange* 0.00436940183863);
					}
					else
					{
						msg.linear.x = 1.0;
						msg.angular.z = 0;
					}
					count ++;
				}
				else
				{
						msg.linear.x = 0;
						msg.angular.z = 0;
				}
			}
			else
			{
				msg.linear.x = 0;
				msg.angular.z = PI/LOOP_RATE;
				gira = 1;
			}
		}
	
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
		if(count == LOOP_RATE + 1)
		{
			count = 0;
			iLowestRange = - 1;
		}

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
