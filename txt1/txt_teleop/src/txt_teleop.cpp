#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

class Txt1Teleop{
public:
	Txt1Teleop(ros::NodeHandle nh);
	virtual ~Txt1Teleop();
private:
	void joyCB(joy::Joy msg);
  void cmdVelTmrCB(const ros::TimerEvent& e);

	ros::NodeHandle n;
	ros::Subscriber joySub;
	ros::Publisher velPub;
	ros::Timer cmdVelTmr;
	joy::Joy joyMsg;

	int linAxis, angAxis;
	double lin_vel_max, ang_vel_max, radius_min;
	bool joyRxFlag;
};

Txt1Teleop::Txt1Teleop(ros::NodeHandle nh):n(nh)
{
	ros::NodeHandle n_private("~");

	n_private.param("linAxis", linAxis, 1);
	n_private.param("angAxis", angAxis, 0);
	n_private.param("linVelMax", lin_vel_max, 1.5);
	n_private.param("radiusMin", radius_min, 0.555);
	
	joyRxFlag = false;
	ang_vel_max = 0.0;

	velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	joySub = n.subscribe<joy::Joy>("joy", 10, &Txt1Teleop::joyCB, this);
	cmdVelTmr = n.createTimer(ros::Duration(0.1), &Txt1Teleop::cmdVelTmrCB, this);
}


Txt1Teleop::~Txt1Teleop()
{

}

void Txt1Teleop::cmdVelTmrCB(const ros::TimerEvent& e)
{
  geometry_msgs::Twist vel;
	if (joyRxFlag && joyMsg.buttons[0] == 1)
  {
    vel.linear.x = lin_vel_max * joyMsg.axes[linAxis];
    ang_vel_max = std::abs(vel.linear.x) / radius_min;
    vel.angular.z = ang_vel_max * joyMsg.axes[angAxis];
  }
  else
  {
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
  }
  velPub.publish(vel);
  ROS_INFO("Linear: %f; Angular: %f", vel.linear.x, vel.angular.z);
}

void Txt1Teleop::joyCB(joy::Joy msg)
{
	joyMsg = msg;
	joyRxFlag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Txt1Teleop");
	ros::NodeHandle n;

	Txt1Teleop *p = new Txt1Teleop(n);

  ros::spin();

	return 0;
}


/*
 * txt_teleop.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Titus Appel
 *

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
*/
