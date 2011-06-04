#ifndef TXT_H
#define TXT_H

#include "ros/ros.h"
#include "txt_driver/packet.h"
#include "txt_driver/serial.h"
#include "txt_driver/Battery.h"
#include "txt_driver/pid.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

class TXT1
{
public:
	geometry_msgs::Twist cmd_vel_msg_;
	nav_msgs::Odometry odom_msg_;
	nav_msgs::Odometry comb_odom_msg_;
	geometry_msgs::PoseWithCovarianceStamped comb_pose_msg_;
	geometry_msgs::TransformStamped odom_trans_msg_;
	geometry_msgs::Quaternion odom_quat_;
	txt_driver::Battery battery_msg_;
	tf::TransformBroadcaster odom_broadcaster_;
	txt_driver::pid::Request pid_req_;

	Serial::Serial * my_serial_;

	TXT1(ros::NodeHandle nh);
	virtual ~TXT1();
	void pubOdom(double x, double y, double theta, double vx,
			double vtheta);
	void pubBattery(double batt1, double batt2);
private:
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber comb_odom_sub_;
	ros::Publisher odom_pub_;
	ros::Publisher battery_pub_;
	ros::Timer cmd_vel_tmr_;
	ros::Timer comb_odom_tmr_;
	ros::ServiceServer pid_srv_;

	ros::NodeHandle n_;
	std::string port_;
	int comb_odom_cnt_;
	int COMB_ODOM_CNT_LIMIT_;

	//int kp_lv_, ki_lv_, kd_lv_, kp_av_, ki_av_, kd_av_;

	void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);
	void cmdVelTmrCB(const ros::TimerEvent& e);
//	void combOdomCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void combOdomCB(const nav_msgs::OdometryConstPtr &msg);
	void combOdomTmrCB(const ros::TimerEvent& e);
	bool pidSrvCB(txt_driver::pid::Request& request, txt_driver::pid::Response& response);
};
#endif
