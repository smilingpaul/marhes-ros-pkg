#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Txt1Teleop");
	ros::NodeHandle n;

	ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber joySub = n.subscribe<joy::Joy>("joy", 10, &Txt1Teleop::joyCB, this);
	
  while

	return 0;
}
