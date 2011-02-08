#ifndef TXT_H
#define TXT_H

#include "ros/ros.h"
#include "txt_driver/packet.h"
#include "txt_driver/serial.h"

#include "sensor_msgs/Imu.h"
//#include "gps_common/GPSFix.h"
//#include "gps_common/GPSStatus.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

class TXT1
{
public:
	sensor_msgs::Imu imuMsg;
//	gps_common::GPSFix gpsFixMsg;
//	gps_common::GPSStatus gpsStatusMsg;
	geometry_msgs::Twist cmdVelMsg;
	nav_msgs::Odometry odomMsg;
    geometry_msgs::TransformStamped odomTransMsg;
    geometry_msgs::Quaternion odomQuat;
    tf::TransformBroadcaster odomBroadcaster;

	Serial::Serial * mySerial;

	TXT1(ros::NodeHandle nh);
	virtual ~TXT1();
	void pubOdom(double x, double y, double theta, double vx,
			double vtheta);
private:
	ros::Subscriber imu_sub;
//	ros::Subscriber gpsFix_sub;
//    ros::Subscriber gpsStatus_sub;
	ros::Subscriber cmd_vel_sub;

	ros::Publisher odom_pub;

	ros::Timer cmd_vel_tmr;

	ros::NodeHandle n;
	std::string port;

	void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
//	void gpsFixCB(const gps_common::GPSFix::ConstPtr& msg);
//	void gpsStatusCB(const gps_common::GPSStatus::ConstPtr& msg);
	void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);
	void cmdVelTmrCB(const ros::TimerEvent& e);
};

#endif
