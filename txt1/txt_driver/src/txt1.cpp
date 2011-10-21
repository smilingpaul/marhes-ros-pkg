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
	n_private_ = ros::NodeHandle("~");

	std::string def = "/dev/ttyUSB0";
	n_private_.param("port", port_, def);
	n_private_.param("pwr_auto", pwr_auto_, false);
	n_private_.param("batt1", batt1_, true);
	n_private_.param("batt2", batt2_, false);
		
	// Initialize variables
	shutdown_ = false;

  // Initialize the subscribers
	cmd_vel_sub_ = n_.subscribe("/cmd_vel", 1, &TXT1::cmdVelCB, this);
	
  // Added 'no delay' option because it was buffering every two data packets and 
  // sending old data
	comb_odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom_comb", 100, 
	    &TXT1::combOdomCB, this, ros::TransportHints().tcpNoDelay());
	pwm_sub_ = n_.subscribe<txt_driver::Pwm>("/pwm", 10, &TXT1::pwmMsgCB, this);

	cmd_vel_tmr_ = n_.createTimer(ros::Duration(0.1), &TXT1::cmdVelTmrCB, this);
	comb_odom_tmr_ = n_.createTimer(ros::Duration(0.020), &TXT1::combOdomTmrCB, 
	    this);
	rx_process_tmr_ = n_.createTimer(ros::Duration(0.010), &TXT1::rxProcess, this);
	pid_resp_tmr_ = n_.createTimer(ros::Duration(1.0), &TXT1::pidRespTmr, this);
  pid_resp_tmr_.stop();

	odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 50);
	battery_pub_ = n_.advertise<txt_driver::Battery>("/battery", 50);
	//pid_terms_pub_ = n_.advertise<txt_driver::PidTerms>("/pid_terms", 1);

	//pid_srv_ = n_.advertiseService("/pid_change", &TXT1::pidSrvCB, this);
	pid_load_srv_ = n_.advertiseService("/pid_load", &TXT1::pidLoadSrvCB, this);
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
 		
 	loadPids(true);
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
  odom_msg_.pose.covariance[14] = 99999.0;
  odom_msg_.pose.covariance[21] = 99999.0;
  odom_msg_.pose.covariance[28] = 99999.0;
  odom_msg_.pose.covariance[35] = 2.0;

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
	
	// Calculate the expected time left of the batteries
	// the curve used is V = -0.033691t + 12.165769
	double v, t;
	if (batt1_ && !batt2_)
	  v = batt1;
  else if (!batt1_ && batt2_)
    v = batt2;
  else
    v = (batt1 + batt2) / 2;
  
  t = ((v - 12.165769) / -0.033691) * 60.0;
  t = -1 * (t - 50 * 60);
	battery_msg_.expected_time = ros::Duration(t);
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

bool TXT1::pidLoadSrvCB(txt_driver::PidLoad::Request& request, txt_driver::PidLoad::Response& response)
{
  loadPids(true);
  response.result = true;
  return true;
}

void TXT1::rxProcess(const ros::TimerEvent& e)
{
  msg_u * msg = rxPacket_.Receive(my_serial_);
  while (msg != NULL)
  {
    processData(msg);
    msg = rxPacket_.Receive(my_serial_);
  }
}

void TXT1::loadPids(bool wait)
{
  ros::NodeHandle n_private("~");
  lin_pids_.clear();
  ang_pids_.clear();
  
  if (n_private.hasParam("linear_pid"))
	{
	  // Get the array
	  XmlRpc::XmlRpcValue lin_pids;
	  n_private.getParamCached("linear_pid", lin_pids);
	  if (lin_pids.getType() != XmlRpc::XmlRpcValue::TypeArray)
	  {  
	    ROS_ERROR("Error reading linear_pid list parameter.");
	    exit(0);
	  }
	  
	  int size = lin_pids.size();
	  if (size != LIN_PID_VALS_)
	  {
	    ROS_ERROR("The size of the linear pid array is not 3, exiting.");
	    exit(0);
	  }
	  
	  for (int i = 0; i < size; i++)
	  {
	    lin_pids_.push_back((double)lin_pids[i]);
	  }
	}
	else
	{
	  ROS_ERROR("Linear PID values not set, exiting.");
	  exit(0);
	}
	
	if (n_private.hasParam("angular_pid"))
	{
	  // Get the array
	  XmlRpc::XmlRpcValue ang_pids;
	  n_private.getParamCached("angular_pid", ang_pids);
	  if (ang_pids.getType() != XmlRpc::XmlRpcValue::TypeArray)
	  {  
	    ROS_ERROR("Error reading angular_pid list parameter.");
	    exit(0);
	  }
	  
	  int size = ang_pids.size();
	  //ROS_INFO("SIZE: %d", size);
	  if (size % ANG_PID_VALS_ != 0)
	  {
	    ROS_ERROR("The size of the angular pid array is not a multiple of 4, exiting.");
	    exit(0);
	  }
	  //ROS_INFO("Gains: %f, %f, %f, %f", (double)ang_pids[0], (double)ang_pids[1], (double)ang_pids[2], (double)ang_pids[3]);
	  for (int i = 0; i < size; i++)
	  {
	    ang_pids_.push_back((double)ang_pids[i]);
	  }
	  
	  ang_pids.clear();
	}
	else
	{
	  ROS_ERROR("Angular PID values not set, exiting.");
	  exit(0);
	}
	
	
	// Send PID gains to TXT1 robot.  Then wait for a response timeout.
  Packet packet;
  packet.BuildPid(lin_pids_, ang_pids_);
  packet.Send(my_serial_);
  ROS_INFO("Sending TXT1 parameters ...");
  
  if (wait)
    pid_resp_tmr_.start();
}

void TXT1::pidRespTmr(const ros::TimerEvent& e)
{
  Packet packet;
  packet.BuildPid(lin_pids_, ang_pids_);
  packet.Send(my_serial_);
  ROS_INFO("Sending TXT1 parameters ...");
}

void TXT1::processData(msg_u * msg)
{
	double xpos, ypos, theta, linvel, angvel, batt1, batt2, pterm, iterm, dterm, signal;

	switch(msg->var.header.var.command)
	{
		case CMD_ODOM_ENC:
			if (msg->var.header.var.length != SIZE_ODOM_ENC)
				break;

			xpos = (double)((msg->var.data[0] << 24) + (msg->var.data[1] << 16) + (msg->var.data[2] << 8) + msg->var.data[3]) / 1000;
			ypos = (double)((msg->var.data[4] << 24) + (msg->var.data[5] << 16) + (msg->var.data[6] << 8) + msg->var.data[7]) / 1000;
			theta = (double)((msg->var.data[8] << 24) + (msg->var.data[9] << 16) + (msg->var.data[10] << 8) + msg->var.data[11]) / 1000;
			linvel = (double)((msg->var.data[12] << 24) + (msg->var.data[13] << 16) + (msg->var.data[14] << 8) + msg->var.data[15]) / 1000;
			angvel = (double)((msg->var.data[16] << 24) + (msg->var.data[17] << 16) + (msg->var.data[18] << 8) + msg->var.data[19]) / 1000;
			pubOdom(xpos, ypos, theta, linvel, angvel);
			break;
		case CMD_BATTERY:
			if (msg->var.header.var.length != SIZE_BATTERY)
				break;

			batt1 = (double)((msg->var.data[0] << 8) + msg->var.data[1]) / 1000;
			batt2 = (double)((msg->var.data[2] << 8) + msg->var.data[3]) / 1000;

			pubBattery(batt1, batt2);
			break;
		case CMD_PID_TERMS:
		  if (msg->var.header.var.length != SIZE_PID_TERMS)
		    break;
		    
		    pterm = (double)((msg->var.data[0] << 24) + (msg->var.data[1] << 16) + 
		                     (msg->var.data[2] << 8) + msg->var.data[3]);
 		    iterm = (double)((msg->var.data[4] << 24) + (msg->var.data[5] << 16) + 
		                     (msg->var.data[6] << 8) + msg->var.data[7]);
 		    dterm = (double)((msg->var.data[8] << 24) + (msg->var.data[9] << 16) + 
		                     (msg->var.data[10] << 8) + msg->var.data[11]);
 		    signal = (double)((msg->var.data[12] << 24) + (msg->var.data[13] << 16) + 
		                     (msg->var.data[14] << 8) + msg->var.data[15]);
		    pubPidTerms(pterm, iterm, dterm, signal);
		  break;
		case CMD_PID:
		  if (msg->var.header.var.length != 1)
		    break;
		  if (msg->var.data[0] == MSG_DATA_GOOD)
		  {
		    pid_resp_tmr_.stop();	  
		    ROS_INFO("Received parameter response.");
	    }
	    else
	    {
	      ROS_INFO("Error receiving parameter response.");
	    }
		  break;		    
		default:
			break;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TXT1");
  ros::NodeHandle n;

  TXT1 * p = new TXT1(n);
  ros::Rate loop_rate(100);

/*  while (ros::ok())
  {
    p->rxProcess();

    ros::spinOnce();
    loop_rate.sleep();
  }*/
  
  ros::spin();
  
  // Do this to make sure the computer shuts down when batteries are bad
  // Modify your /etc/sudoers file by adding a line like this:-
  // %admin ALL = NOPASSWD: /sbin/shutdown
  if (p->shutdown_)
    system("sudo shutdown -h now");

  return 0;
}
