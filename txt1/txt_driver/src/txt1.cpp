/*! \brief Brief description.
 *         Brief description continued.
 *
 *  Detailed description starts here.
 */

#include "ros/ros.h"
#include "txt_driver/txt1.h"

TXT1::TXT1(ros::NodeHandle nh)
{
	n_ = nh;
	ros::NodeHandle n_private("~");

	std::string def = "/dev/ttyUSB0";
	n_private.param("port", port_, def);
	n_private.param("pwr_auto", pwr_auto_, false);
	// TODO: Add PID Gains Parameter table
	
	// Initialize variables
	shutdown_ = false;

  // Initialize the subscribers
	cmd_vel_sub_ = n_.subscribe("/cmd_vel", 1, &TXT1::cmdVelCB, this);
	
  // Added 'no delay' option because it was buffering every two data packets and 
  // sending old data
	comb_odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/vo", 100, 
	    &TXT1::combOdomCB, this, ros::TransportHints().tcpNoDelay());
	pwm_sub_ = n_.subscribe<txt_driver::Pwm>("/pwm", 10, &TXT1::pwmMsgCB, this);

	cmd_vel_tmr_ = n_.createTimer(ros::Duration(0.1), &TXT1::cmdVelTmrCB, this);
	comb_odom_tmr_ = n_.createTimer(ros::Duration(0.020), &TXT1::combOdomTmrCB, 
	    this);

	odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 50);
	battery_pub_ = n_.advertise<txt_driver::Battery>("/battery", 50);
	pid_terms_pub_ = n_.advertise<txt_driver::PidTerms>("/pid_terms", 1);

	pid_srv_ = n_.advertiseService("/pid_change", &TXT1::pidSrvCB, this);
  shutdown_srv_ = n_.advertiseService("/shutdown_computer", &TXT1::shutdownSrvCB, this);
  pwm_test_srv_ = n_.advertiseService("/pwm_change", &TXT1::pwmSetValsCB, this);
  switch_pwr_srv_ = n_.advertiseService("/switched_pwr", &TXT1::switchPwrCB, this);

	// Initialize combined odometry message
	comb_odom_msg_.header.stamp = ros::Time::now();
	comb_odom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Open serial port @ 57600 baud 8N1
	my_serial_ = new Serial(port_, Serial::BAUD_57600, Serial::SIZE_8,
			Serial::NONE, Serial::ONE);

 	if (my_serial_->isOpen)
 		ROS_INFO("Serial Port: %s is Opened.", port_.c_str());
 	else
 		ROS_ERROR("Failed to open port: %s.", port_.c_str());
}

TXT1::~TXT1()
{
  	my_serial_->Close();
  	delete my_serial_;
}

/******************************************************************************
 * Name: 	cmdVelCB
 * Inputs:	msg - The cmdVel message from the subscriber
 * Outputs: none
 * 
 * Description: This is the callback for the cmd_vel subscriber.  It stores
 *              the received message in the cmdVelMsg variable for later use.
 *              It also builds an velocity command data packet and sends it to 
 *              the TXT1 for use in the control loop.
 ******************************************************************************/

void TXT1::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
	cmd_vel_msg_ = *msg;
}

void TXT1::cmdVelTmrCB(const ros::TimerEvent& e)
{
	Packet packet;
	packet.BuildCmdVel(cmd_vel_msg_);
	packet.Send(my_serial_);
}

void TXT1::combOdomTmrCB(const ros::TimerEvent& e)
{
	if (comb_odom_cnt_ > 0)//COMB_ODOM_CNT_LIMIT_)
	{
    use_comb_odom_ = true;
	}
	else
	{
	  use_comb_odom_ = false;
	}
	comb_odom_cnt_ = 0;
}

void TXT1::combOdomCB(nav_msgs::Odometry msg)
{
  comb_odom_cnt_++;
  comb_odom_msg_ = msg;
  if (use_comb_odom_ == true)
  {
	  Packet packet;
		packet.BuildCombOdom(comb_odom_msg_);
		packet.Send(my_serial_);
	}
}

void TXT1::pwmMsgCB(txt_driver::Pwm msg)
{
  Packet packet;
  packet.BuildPwm(msg);
  packet.Send(my_serial_);
}

void TXT1::pubOdom(double x, double y, double theta, double vx, double vtheta)
{
	ros::Time currentTime = ros::Time::now();

    //first, we'll publish the transform over tf
	odom_trans_msg_.header.stamp = currentTime;
	odom_trans_msg_.header.frame_id = "/odom";
	odom_trans_msg_.child_frame_id = "/base_footprint";

  odom_trans_msg_.transform.translation.x = x;
  odom_trans_msg_.transform.translation.y = y;
  odom_trans_msg_.transform.translation.z = 0.0;
  odom_quat_ = tf::createQuaternionMsgFromYaw(theta);
  odom_trans_msg_.transform.rotation = odom_quat_;

  //send the transform
  odom_broadcaster_.sendTransform(odom_trans_msg_);

  //next, we'll publish the odometry message over ROS
  odom_msg_.header.stamp = currentTime;
  odom_msg_.header.frame_id = "/odom";

  //set the position
  odom_msg_.pose.pose.position.x = x;
  odom_msg_.pose.pose.position.y = y;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation = odom_quat_;

  /// set the position covariance
  odom_msg_.pose.covariance[0] = 1.0;
  odom_msg_.pose.covariance[7] = 1.0;
  odom_msg_.pose.covariance[14] = 99999;
  odom_msg_.pose.covariance[21] = 99999;
  odom_msg_.pose.covariance[28] = 99999;
  odom_msg_.pose.covariance[35] = 0.5;

  //set the velocity
  odom_msg_.child_frame_id = "/base_footprint";
  odom_msg_.twist.twist.linear.x = vx;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.linear.z = 0.0;
  odom_msg_.twist.twist.angular.z = vtheta;

  //publish the message
  odom_pub_.publish(odom_msg_);
}

void TXT1::pubBattery(double batt1, double batt2)
{
	battery_msg_.header.stamp = ros::Time::now();
	battery_msg_.header.frame_id = "battery";
	battery_msg_.batt1 = batt1;
	battery_msg_.batt2 = batt2;
	battery_pub_.publish(battery_msg_);
}

void TXT1::pubPidTerms(double pterm, double iterm, double dterm, double signal)
{
  txt_driver::PidTerms msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "pid_terms";
  msg.pterm = pterm;
  msg.iterm = iterm;
  msg.dterm = dterm;
  msg.signal = signal;
  pid_terms_pub_.publish(msg);  
}

bool TXT1::pidSrvCB(txt_driver::Pid::Request& request, txt_driver::Pid::Response& response)
{
	Packet packet;
	packet.BuildPidTx(request);
	packet.Send(my_serial_);

	response.result = true;
	return true;
}

bool TXT1::pwmSetValsCB(txt_driver::PwmTest::Request& request, txt_driver::PwmTest::Response& response)
{
  Packet packet;
  packet.BuildPWMTestTx(request);
  packet.Send(my_serial_);
  
  response.result = true;
  return true;
}

bool TXT1::shutdownSrvCB(txt_driver::Shutdown::Request& request, txt_driver::Shutdown::Response& response)
{
  shutdown_ = true;
  ros::shutdown();
  response.result = true;
  return true;
}

bool TXT1::switchPwrCB(txt_driver::SwitchedPwr::Request& request, txt_driver::SwitchedPwr::Response& response)
{
  Packet packet;
  packet.BuildSwitchPwr(request);
  packet.Send(my_serial_);
  
  response.result = true;
  return true;
}

TXT1 *p;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TXT1");
  ros::NodeHandle n;

  p = new TXT1(n);
  Packet rxPacket;
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    //rxPacket.Receive(p->my_serial_);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  //spinner.spin(); // spin() will not return until the node has been shutdown
  
  //ros::spin();
  
  // Do this to make sure the computer shuts down when batteries are bad
  // Modify your /etc/sudoers file by adding a line like this:-
  // %admin ALL = NOPASSWD: /sbin/shutdown
  if (p->shutdown_)
    system("sudo shutdown -h now");

  return 0;
}
