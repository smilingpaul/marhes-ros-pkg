/*
 * main.c
 *
 *  Created on: Jan 11, 2011
 *      Author: root
 */
#include "ros/ros.h"
#include "xbee_test/serial.h"
#include "xbee_test/xbee.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xbee_node");
	ros::NodeHandle n;

	Serial::Serial * mySerial = new Serial("/dev/ttyUSB0", Serial::BAUD_115200,
			Serial::SIZE_8, Serial::NONE, Serial::ONE);
	ros::Rate loop_rate(0.2);	// Loop once every 5 secs

	// Send pluses and receive OK
	while (!xbeePacket::StartATMode(mySerial));

	ROS_INFO("OK");

	while(n.ok())
	{
		//xbeePacket::getDB(mySerial);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
