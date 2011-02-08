#include "ros/ros.h"
#include "txt_driver/txt1.h"

TXT1::TXT1(ros::NodeHandle nh)
{
	n = nh;
	ros::NodeHandle n_private("~");

	std::string def = "/dev/ttyUSB0";
	n_private.param("port", port, def);

	imu_sub = n.subscribe("imu/data", 1000, &TXT1::imuCB, this);
//	gpsFix_sub = n.subscribe("fix", 1000, &TXT1::gpsFixCB, this);
//    gpsStatus_sub = n.subscribe("status", 1000, &TXT1::gpsStatusCB, this);
	cmd_vel_sub = n.subscribe("cmd_vel", 1000, &TXT1::cmdVelCB, this);

	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	cmd_vel_tmr = n.createTimer(ros::Duration(0.05), &TXT1::cmdVelTmrCB, this);

	mySerial = new Serial(port, Serial::BAUD_115200, Serial::SIZE_8,
			Serial::NONE, Serial::ONE);

 	if (mySerial->isOpen)
 		ROS_INFO("Serial Port: %s is Opened.", port.c_str());
 	else
 		ROS_ERROR("Failed to open port: %s.", port.c_str());
}

TXT1::~TXT1()
{
  	mySerial->Close();
  	delete mySerial;
}

/******************************************************************************
 * Name: 	imuCB
 * Inputs:	msg - The imu message from the imu subscriber
 * Outputs: none
 * 
 * Description: This is the callback for the imu/data subscriber.  It stores
 *              the received message in the imuMsg variable for later use.
 *              It also builds an imu data packet and sends it to the TXT1 for
 *              use in the control loop.
 ******************************************************************************/

void TXT1::imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    imuMsg = *msg;
}

/******************************************************************************
 * Name: 	gpsFixCB
 * Inputs:	msg - The gps fix message from the gps fix subscriber
 * Outputs: none
 * 
 * Description: This is the callback for the gps fix subscriber.  It stores
 *              the received message in the gpsFixMsg variable for later use.
 *              It also builds a gps fix data packet and sends it to the TXT1 
 *              for use in the control loop.
 ******************************************************************************/

//void TXT1::gpsFixCB(const gps_common::GPSFix::ConstPtr& msg)
//{//	double x = 0, y = 0, th = 0, vx = 0.1, vt = 0.1;
	//	ros::Time current_time, last_time;
	//	last_time = ros::Time::now();
	//	current_time = ros::Time::now();
//	gpsFixMsg = *msg;
//}

/******************************************************************************
 * Name: 	gpsStatusCB
 * Inputs:	msg - The gps status message from the gps status subscriber
 * Outputs: none
 * 
 * Description: This is the callback for the gps status subscriber.  It stores
 *              the received message in the gpsStatusMsg variable for later use.
 *              It also builds an gps status data packet and sends it to the 
 *              TXT1 for use in the control loop.
 ******************************************************************************/

//void TXT1::gpsStatusCB(const gps_common::GPSStatus::ConstPtr& msg)
//{
//	gpsStatusMsg = *msg;
//}

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
	cmdVelMsg = *msg;
}

void TXT1::cmdVelTmrCB(const ros::TimerEvent& e)
{
	Packet packet;
	packet.BuildCmdVel(cmdVelMsg);
	packet.Send(mySerial);
}

void TXT1::pubOdom(double x, double y, double theta, double vx, double vtheta)
{
	ros::Time currentTime = ros::Time::now();

    //first, we'll publish the transform over tf
	odomTransMsg.header.stamp = currentTime;
	odomTransMsg.header.frame_id = "/odom";
	odomTransMsg.child_frame_id = "/base_footprint";

    odomTransMsg.transform.translation.x = x;
    odomTransMsg.transform.translation.y = y;
    odomTransMsg.transform.translation.z = 0.0;
    odomQuat = tf::createQuaternionMsgFromYaw(theta);
    odomTransMsg.transform.rotation = odomQuat;

    //send the transform
    odomBroadcaster.sendTransform(odomTransMsg);

    //next, we'll publish the odometry message over ROS
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = "/odom";

    //set the position
    odomMsg.pose.pose.position.x = x;
    odomMsg.pose.pose.position.y = y;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = odomQuat;

    //set the velocity
    odomMsg.child_frame_id = "/base_footprint";
    odomMsg.twist.twist.linear.x = vx;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.z = vtheta;

    //publish the message
    odom_pub.publish(odomMsg);
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
  		rxPacket.Receive(p->mySerial);

      	ros::spinOnce();
    	loop_rate.sleep();
  	}

  	return 0;
}
