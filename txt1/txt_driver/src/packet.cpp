#include "ros/ros.h"
#include "txt_driver/packet.h"

extern TXT1* p;

Packet::Packet()
{
	dataNum = 0;
	droppedPkts = 0;
	totalPkts = 0;
	memset(packet,0,sizeof(packet));
}

Packet::~Packet()
{

}

/******************************************************************************
 * Name: 	Build
 * Inputs:	data - The char array of data bytes
 *          dataSize - The size the the char array of data bytes
 * Outputs: int - 0 for completed ok, 1 for error
 * 
 * Description: This method builds a data packet to be sent to the TXT1.  It
 *              attaches a header of two bytes to the beginning and then a byte 
 *              indicating the length of the data section including the two 
 *              checksum bytes.  Finally the checksum is calculated and attached
 *              to the end with the bytes in reverse order like the Pioneers.
 ******************************************************************************/

int Packet::Build(unsigned char *data, unsigned char dataSize)
{
	short checksum;

	packetSize = dataSize + 5;

	packet[0] = 0xFA;				// Write 2 header bytes
    packet[1] = 0xFB;

	packet[2] = dataSize + 2;		// Write the length of packet, which is
									// everything from the command byte to the
                                    // checksum

  	memcpy( &packet[3], data, dataSize );  	// Copy all the data bytes to packet

	checksum = CalcChkSum();				// Calculate the checksum
	packet[3+dataSize] = checksum >> 8;		// Put the checksum bytes in
	packet[3+dataSize+1] = checksum & 0xFF;	// reverse order

	if (!Check()) {
		ROS_ERROR("Checksum malfunction.");
		return(1);
	}

	return(0);
}

/******************************************************************************
 * Name: 	BuildCmdVel
 * Inputs:	msg - The Cmd Vel msg from the subscriber
 * Outputs: int - 0 is success, 1 is failure
 *
 * Description: This method assembles the CmdVel msg into a packet to send to
 *              the TXT1.
 *
 * 5 Byte Msg - CMD = 103 = 0x67
 * CMD | FOR_VEL | ROT_VEL
 *----------------------------
 * 1B  |    2B   |   2B
 ******************************************************************************/

int Packet::BuildCmdVel(const geometry_msgs::Twist &msg)
{
	unsigned char data[SIZE_VEL] = {0};
	short vel = 0;

	data[0] = CMD_VEL;			        	// Write the VEL command

	vel = (short)(msg.linear.x * 1000);		// Write the linear velocity
	data[1] = (unsigned char)(vel >> 8);
	data[2] = (unsigned char)(vel & 0x00FF);

	vel = (short)(msg.angular.z * 1000);	// Write the angular velocity
	data[3] = (unsigned char)(vel >> 8);
	data[4] = (unsigned char)(vel & 0x00FF);

	Build(data, SIZE_VEL);				// Add the header and checksum
	return(0);
}

int Packet::BuildCombOdom(const nav_msgs::Odometry &msg)
{
	unsigned char data[SIZE_ODOM_COMB] = {0};
	int temp;

	data[0] = CMD_ODOM_COMB;

	temp = (int)(msg.pose.pose.position.x * 1000);
	data[1] = (unsigned char)(temp >> 24);
	data[2] = (unsigned char)(temp >> 16);
	data[3] = (unsigned char)(temp >> 8);
	data[4] = (unsigned char)(temp & 0x000000FF);

	temp = (int)(msg.pose.pose.position.y * 1000);
	data[5] = (unsigned char)(temp >> 24);
	data[6] = (unsigned char)(temp >> 16);
	data[7] = (unsigned char)(temp >> 8);
	data[8] = (unsigned char)(temp & 0x000000FF);

	temp = (int)(tf::getYaw(msg.pose.pose.orientation) * 1000);
	data[9] = (unsigned char)(temp >> 24);
	data[10] = (unsigned char)(temp >> 16);
	data[11] = (unsigned char)(temp >> 8);
	data[12] = (unsigned char)(temp & 0x000000FF);

	temp = (int)(msg.twist.twist.linear.x * 1000);
	data[13] = (unsigned char)(temp >> 24);
	data[14] = (unsigned char)(temp >> 16);
	data[15] = (unsigned char)(temp >> 8);
	data[16] = (unsigned char)(temp & 0x000000FF);

	temp = (int)(msg.twist.twist.angular.z * 1000);
	data[17] = (unsigned char)(temp >> 24);
	data[18] = (unsigned char)(temp >> 16);
	data[19] = (unsigned char)(temp >> 8);
	data[20] = (unsigned char)(temp & 0x000000FF);

	Build(data, SIZE_ODOM_COMB);

	return(0);
}

/******************************************************************************
 * Name: 	CalcChkSum()
 * Inputs:	None
 * Outputs: int - The calculated checksum
 * 
 * Description: This method calculates the checksum according to the Pioneer 
 *              documentation.
 ******************************************************************************/

int Packet::CalcChkSum()
{
	unsigned char *buffer = &packet[3];
  	int c = 0;
  	int n;

  	n = packetSize - 5;							// Get the number of data bytes
	
	// For the even number of bytes, successively adding data byte pairs (high 
	// byte first) to a running checksum (initially zero), disregarding sign and 
	// overflow.
  	while (n > 1) 
	{
    	c+= (*(buffer)<<8) | *(buffer+1);	// Add byte pair to checksum
    	c = c & 0xffff;						// Disregard overflow
    	n -= 2;								// Calc next pointer
    	buffer += 2;
  	}
	
	// If there are an odd number of data bytes, the last byte is XORed to the 
	// low-order byte of the checksum.
  	if (n>0) c = c ^ (int)*(buffer++);

  	return(c);
}

/******************************************************************************
 * Name: 	Check()
 * Inputs:	None
 * Outputs: bool - If the checksum is correct
 * 
 * Description: This method calculates the checksum and compares it to the 
 *              packets checksum.
 ******************************************************************************/

bool Packet::Check()
{
  	unsigned short chksum;

	// Compare the calculated checksum with the packet's checksum
  	chksum = CalcChkSum();
  	if ( chksum == ((packet[packetSize-2] << 8) | packet[packetSize-1]))
   		return(true);

  	return(false);
}

/******************************************************************************
 * Name: 	Send()
 * Inputs:	port - The serial port handle to send on
 * Outputs: int - 0 is success, 1 is failure
 * 
 * Description: This method sends the assembled packet on the given serial port.
 ******************************************************************************/

void Packet::Send( Serial::Serial* port )
{
	int cnt = 0;

	try
	{
		cnt = port->Write(packet, packetSize);
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR("Failure to send. Error: %s", e.what());
	}

	if (cnt != packetSize)
	{
		ROS_ERROR("Not all bytes sent.");
	}
}

void Packet::Receive( Serial::Serial * port )
{
	unsigned char tempByte;

	while ((dataNum < MAX_PACKET_SIZE) && (port->Read(&tempByte, 1) > 0))
	{
		if (dataNum < 3)						// Read the packet header
		{
			if (packet[0] == 0xFA && packet[1] == 0xFB)
			{
				dataNum = 3;
				packet[2] = tempByte;
				packetSize = packet[2] + 3;
			}
			else
			{
				packet[0] = packet[1];
				packet[1] = tempByte;
			}
		}
		else									// Get the rest of the packet
		{
			packet[dataNum] = tempByte;
			dataNum++;

			if (dataNum >= (packet[2] + 3))
			{
				if (Check())
				{
					ProcessData();
					totalPkts++;
				}
				else
				{
					droppedPkts++;
					totalPkts++;
					ROS_INFO("Dropped Packets = %d / %d", droppedPkts, totalPkts);
					ROS_INFO("Packet: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
							packet[0],packet[1],packet[2],packet[3],packet[4],packet[5],
							packet[6],packet[7],packet[8],packet[9],packet[10],packet[11],packet[12],
							packet[13],packet[14],packet[15],packet[16],packet[17],packet[18],packet[19],packet[20],
							packet[21],packet[22],packet[23],packet[24],packet[25]);
				}

				dataNum = 0;
				memset(packet,0,sizeof(packet));
			}
		}
	}

	if (dataNum >= MAX_PACKET_SIZE)
		dataNum = 0;
}

void Packet::ProcessData()
{
	double xpos, ypos, theta, linvel, angvel, cell1, cell2, cell3;

	switch(packet[3])
	{
		case CMD_ODOM_ENC:
			if (packetSize - 5 != SIZE_ODOM_ENC)
				break;

			xpos = (double)((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]) / 1000;
			ypos = (double)((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]) / 1000;
			theta = (double)((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]) / 1000;
			linvel = (double)((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]) / 1000;
			angvel = (double)((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]) / 1000;
			//			ROS_INFO("Linvel: %d, %d, %d, %d", packet[16], packet[17], packet[18], packet[19]);
			ROS_INFO("Packet: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
					packet[0],packet[1],packet[2],packet[3],packet[4],packet[5],
					packet[6],packet[7],packet[8],packet[9],packet[10],packet[11],packet[12],
					packet[13],packet[14],packet[15],packet[16],packet[17],packet[18],packet[19],packet[20],
					packet[21],packet[22],packet[23],packet[24],packet[25]);
			p->pubOdom(xpos, ypos, theta, linvel, angvel);
			break;
		case CMD_BATTERY:
			ROS_INFO("%d,%d,%d,%d,%d,%d",packet[4],packet[5],packet[6],packet[7],packet[8],packet[9]);

			if (packetSize - 5 != SIZE_BATTERY)
				break;

			cell1 = (double)((packet[4] << 8) + packet[5]) / 1000;
			cell2 = (double)((packet[6] << 8) + packet[7]) / 1000;
			cell3 = (double)((packet[8] << 8) + packet[9]) / 1000;



			p->pubBattery(cell1, cell2, cell3);
			break;
		default:

			break;
	}
}

void Packet::Print()
{
  if (packet) {
    ROS_INFO("\"");
    for(int i=0;i<packetSize;i++) {
      ROS_INFO("%u ", packet[i]);
    }
    ROS_INFO("\"");
  }
}


void Packet::PrintHex()
{
  if (packet) {
    ROS_INFO("\"");
    for(int i=0;i<packetSize;i++) {
      ROS_INFO("0x%.2x ", packet[i]);
    }
    ROS_INFO("\"");
  }
}
