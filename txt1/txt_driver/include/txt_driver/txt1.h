#ifndef TXT_H
#define TXT_H

#include "ros/ros.h"
#include "txt_driver/packet.h"
#include "txt_driver/serial.h"
#include "txt_driver/Battery.h"
#include "txt_driver/PidTerms.h"
#include "txt_driver/Pwm.h"
#include "txt_driver/Pid.h"
#include "txt_driver/PwmTest.h"
#include "txt_driver/Shutdown.h"
#include "txt_driver/SwitchedPwr.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"

#include <stdlib.h>
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

  bool shutdown_;
	Serial * my_serial_;

	TXT1(ros::NodeHandle nh);
	virtual ~TXT1();
	void pubOdom(double x, double y, double theta, double vx,
			double vtheta);
	void pubBattery(double batt1, double batt2);
	void pubPidTerms(double pterm, double iterm, double dterm, double signal);
private:
	ros::Subscriber cmd_vel_sub_, comb_odom_sub_, pwm_sub_;
	ros::Publisher odom_pub_, battery_pub_, pid_terms_pub_;
	ros::Timer cmd_vel_tmr_, comb_odom_tmr_;
	ros::ServiceServer pid_srv_, shutdown_srv_, pwm_test_srv_, switch_pwr_srv_;

	ros::NodeHandle n_;
	std::string port_;
	bool pwr_auto_, use_comb_odom_;
	int comb_odom_cnt_;
	static const int COMB_ODOM_CNT_LIMIT_ = 5;
	
	void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);
	void cmdVelTmrCB(const ros::TimerEvent& e);
	void combOdomCB(nav_msgs::Odometry msg);
	void combOdomTmrCB(const ros::TimerEvent& e);
	void pwmMsgCB(txt_driver::Pwm msg);
	bool pidSrvCB(txt_driver::Pid::Request& request, txt_driver::Pid::Response& response);
	bool pwmSetValsCB(txt_driver::PwmTest::Request& request, txt_driver::PwmTest::Response& response);
	bool shutdownSrvCB(txt_driver::Shutdown::Request& request, txt_driver::Shutdown::Response& response);
	bool switchPwrCB(txt_driver::SwitchedPwr::Request& request, txt_driver::SwitchedPwr::Response& response);
};
#endif
