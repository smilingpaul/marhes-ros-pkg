#ifndef TXT_H
#define TXT_H

#include "ros/ros.h"
#include "txt_driver/packet.h"
#include "txt_driver/serial.h"
#include "txt_driver/Battery.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

class TXT1
{
public:
	geometry_msgs::Twist cmdVelMsg;
	nav_msgs::Odometry odomMsg;
	nav_msgs::Odometry combOdomMsg;
	geometry_msgs::PoseWithCovarianceStamped combPoseMsg;
	geometry_msgs::TransformStamped odomTransMsg;
	geometry_msgs::Quaternion odomQuat;
	txt_driver::Battery batteryMsg;
	tf::TransformBroadcaster odomBroadcaster;

	Serial::Serial * mySerial;

	TXT1(ros::NodeHandle nh);
	virtual ~TXT1();
	void pubOdom(double x, double y, double theta, double vx,
			double vtheta);
	void pubBattery(double cell1, double cell2, double cell3);
private:
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber comb_odom_sub;
	ros::Publisher odom_pub;
	ros::Publisher battery_pub;
	ros::Timer cmd_vel_tmr;
	ros::Timer comb_odom_tmr;

	ros::NodeHandle n;
	std::string port;

	void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);
	void cmdVelTmrCB(const ros::TimerEvent& e);
//	void combOdomCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void combOdomCB(const nav_msgs::OdometryConstPtr &msg);
	void combOdomTmrCB(const ros::TimerEvent& e);
};
#endif
