/*
 * xbee.cpp
 *
 *  Created on: Jan 11, 2011
 *      Author: root
 */

#include "xbee_test/xbee.h"

bool xbeePacket::StartATMode( Serial::Serial * port )
{
	unsigned char cmd_atMode[] = "+++";
	unsigned char readBytes[10] = {0};
	ros::Time currentTime, lastTime;
	ros::Duration elapsedTime;
	int cnt;
	unsigned char tempByte;

	port->Write(cmd_atMode, 3);
	lastTime = ros::Time::now();

	while (!((readBytes[0] == 'O') && (readBytes[1] == 'K') &&
			(readBytes[2] == '\r')) && (elapsedTime.toSec() < 10.0))
	{
		cnt = port->Read(&tempByte, 1);

		if (cnt > 0)
		{
			readBytes[0] = readBytes[1];
			readBytes[1] = readBytes[2];
			readBytes[2] = tempByte;
		}

		currentTime = ros::Time::now();
		elapsedTime = currentTime - lastTime;
		lastTime = currentTime;
	}

	if ( elapsedTime.toSec() < 10.0 )
		return true;
	else
		return false;
}

int xbeePacket::SendATND( Serial::Serial * port )
{
	unsigned char cmd_ATND[] = "ATND\r";
	unsigned char readBytes[250] = {0};
	unsigned char tempByte;
	int cnt = 0, temp = 0, index = 0;
	struct msg_ATND myMsg;

	memset(readBytes, 0, 250);

	cnt = 0;
	port->Write(cmd_ATND, 5);

	while ( cnt < 3 )
		cnt += port->Read(&readBytes[cnt], 1);

	while (!(readBytes[cnt - 1] == '\r' && readBytes[cnt - 2] == '\r' && readBytes[cnt - 3] == '\r'))
	{
		cnt += port->Read(&readBytes[cnt], 1);

//		MY<CR>
//		SH<CR>
//		SL<CR>
//		NI<CR> (Variable length)
//		PARENT_NETWORK ADDRESS (2 Bytes)<CR>
//		DEVICE_TYPE<CR> (1 Byte: 0=Coord, 1=Router, 2=End Device)
//		STATUS<CR> (1 Byte: Reserved)
//		PROFILE_ID<CR> (2 Bytes)
//		MANUFACTURER_ID<CR> (2 Bytes)
//		<CR>
	}

	while (index < cnt)
	{
		temp = (temp << 8) + readBytes[index];
		index++;

		if (readBytes[index] == '\r')
		{
			ROS_INFO("RX: %d", temp);
			index++;
			temp = 0;
		}
	}

	ROS_INFO("RX: ");

	return 0;
}
