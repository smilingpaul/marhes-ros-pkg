/*
 * txt_teleop.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Titus Appel
 */

#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"

class Txt1Teleop{
public:
	Txt1Teleop(ros::NodeHandle nh);
	virtual ~Txt1Teleop();
private:
	void joyCB(const joy::Joy::ConstPtr& joy);

	ros::NodeHandle n;
	ros::Subscriber joySub;
	ros::Publisher velPub;

	int linAxis, angAxis;
};

Txt1Teleop::Txt1Teleop(ros::NodeHandle nh):n(nh)
{
	ros::NodeHandle n_private("~");

	n_private.param("linAxis", linAxis, 1);
	n_private.param("angAxis", angAxis, 0);

	velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	joySub = n.subscribe<joy::Joy>("joy", 10, &Txt1Teleop::joyCB, this);
}


Txt1Teleop::~Txt1Teleop()
{

}

void Txt1Teleop::joyCB(const joy::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
    vel.linear.x = (double)1600 * joy->axes[linAxis];
    vel.angular.z = (double)1600 * joy->axes[angAxis];

    velPub.publish(vel);
    ROS_INFO("Linear: %f; Angular: %f", vel.linear.x, vel.angular.z);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Txt1Teleop");
	ros::NodeHandle n;

	Txt1Teleop *p = new Txt1Teleop(n);

  	ros::spin();

	return 0;
}
