#include "ros/ros.h"
#include "txt_driver/txt1.h"

TXT1::TXT1(ros::NodeHandle nh)
{
	n = nh;
	ros::NodeHandle n_private("~");

	std::string def = "/dev/ttyUSB0";
	n_private.param("port", port, def);

	cmd_vel_sub = n.subscribe("/cmd_vel", 1000, &TXT1::cmdVelCB, this);
//	comb_odom_sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, &TXT1::combOdomCB, this);
	comb_odom_sub = n.subscribe("/vo", 1000, &TXT1::combOdomCB, this);
	odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	battery_pub = n.advertise<txt_driver::Battery>("/battery", 50);
	cmd_vel_tmr = n.createTimer(ros::Duration(0.1), &TXT1::cmdVelTmrCB, this);
	comb_odom_tmr = n.createTimer(ros::Duration(0.05), &TXT1::combOdomTmrCB, this);

	// Added for testing
	combOdomMsg.header.stamp = ros::Time::now();
	combOdomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//	combOdomMsg.twist.twist.linear.x = 0.0;
//	combOdomMsg.twist.twist.angular.z = 0.0;

	mySerial = new Serial(port, Serial::BAUD_57600, Serial::SIZE_8,
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

void TXT1::combOdomTmrCB(const ros::TimerEvent& e)
{
	Packet packet;
	packet.BuildCombOdom(combOdomMsg);
	packet.Send(mySerial);
}
//void TXT1::combOdomCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
void TXT1::combOdomCB(const nav_msgs::OdometryConstPtr &msg)
{
	combOdomMsg = *msg;
//	ros::Duration dt = msg->header.stamp - combPoseMsg.header.stamp;
//	double vx = sqrt(pow(msg->pose.pose.position.x -
//		  combPoseMsg.pose.pose.position.x, 2) +
//	      pow(msg->pose.pose.position.y - combPoseMsg.pose.pose.position.y, 2))
//	      / dt.toSec();
//	double vt = (tf::getYaw(msg->pose.pose.orientation) -
//			tf::getYaw(combPoseMsg.pose.pose.orientation)) / dt.toSec();
//
//	combPoseMsg = *msg;
//	combOdomMsg.header.stamp = ros::Time::now();
//	combOdomMsg.pose.pose = combPoseMsg.pose.pose;
//	combOdomMsg.twist.twist.linear.x = vx;
//	combOdomMsg.twist.twist.angular.z = vt;
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

    /// set the position covariance
    odomMsg.pose.covariance[0] = 1.0;
    odomMsg.pose.covariance[7] = 1.0;
    odomMsg.pose.covariance[14] = 99999;
    odomMsg.pose.covariance[21] = 99999;
    odomMsg.pose.covariance[28] = 99999;
    odomMsg.pose.covariance[35] = 0.5;

    //set the velocity
    odomMsg.child_frame_id = "/base_footprint";
    odomMsg.twist.twist.linear.x = vx;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.z = vtheta;

    //publish the message
    odom_pub.publish(odomMsg);
}

void TXT1::pubBattery(double cell1, double cell2, double cell3)
{
	batteryMsg.header.stamp = ros::Time::now();
	batteryMsg.header.frame_id = "battery";
	batteryMsg.cell1 = cell1;
	batteryMsg.cell2 = cell2;
	batteryMsg.cell3 = cell3;

	battery_pub.publish(batteryMsg);
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
