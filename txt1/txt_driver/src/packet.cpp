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
 * Name: 	BuildGpsFix
 * Inputs:	msg - The GPSFix msg from the subscriber
 * Outputs: int - 0 is success, 1 is failure
 * 
 * Description: This method assembles the GpsFix msg into a packet to send to 
 *              the TXT1.
 *        
 * 19 Byte Msg - CMD = 100 = 0x64
 * CMD | LATITUDE | LONGITUDE | HEMISPHERE | ALTITUDE | ALTPOS | DIRECTION
 *--------------------------------------------------------------------------
 * 1B  |    4B    |     4B    |    1B      |    4B    |    1B  |     4B
 ******************************************************************************/

//int Packet::BuildGpsFix(const gps_common::GPSFix &msg)
//{
//
//#ifdef USE_STRINGS
//
//	std::stringstream ss;
//
//	ss << CMD_GPS_FIX << msg.latitude << "," << msg.longitude << "," << \
//			msg.altitude << "," << msg.track;
//
//	Build((unsigned char*)ss.str().c_str(), ss.str().length());
//
//	return 0;
//
//#else
//
//	char data[SIZE_GPS_FIX] = {0};
//	char latNorth = 1;
//	char longWest = 1;
//	char altPos = 1;
//	char* value;
//
//	data[0] = CMD_GPS_FIX;			        // Write the GPS_FIX command
//
//	if (msg.latitude < 0)                   // Pick hemisphere, Pos = North
//	    latNorth = 0;
//	value = FloatToBytes(msg.latitude,4);   // Convert lat float to 4 byte array
//	memcpy( &data[1], value, 4 );
//
//	if (msg.longitude >= 0)                 // Pick hemisphere, Pos = East
//	    longWest = 0;
//	value = FloatToBytes(msg.longitude,4);  // Convert lat float to 4 byte array
//	memcpy( &data[5], value, 4 );
//
//	data[9] = (latNorth << 4) | longWest;	// Write hemisphere byte
//
//	if(msg.altitude < 0)					// Write the altitude to data
//		altPos = 0;
//	value = FloatToBytes(msg.altitude, 4);
//	memcpy( &data[10], value, 4);
//
//	data[14] = altPos;						// Write altitude sign
//
//	value = FloatToBytes(msg.track, 4); 	// Write the direction to data
//	memcpy( &data[15], value, 4);
//
//	Build(data, SIZE_GPS_FIX);				// Add the header and checksum
//	return(0);
//
//#endif
//}

/******************************************************************************
 * Name: 	BuildGpsStatus
 * Inputs:	msg - The GPSStatus msg from the subscriber
 * Outputs: int - 0 is success, 1 is failure
 *
 * Description: This method assembles the GpsStatus msg into a packet to send to
 *              the TXT1.
 *
 * 7 Byte Msg - CMD = 101 = 0x65
 * CMD | STATUS | SATS_VIS | SATS_USED
 *--------------------------------------
 * 1B  |   2B   |    2B    |    2B
 *
 * STATUS - NO_FIX=0 | FIX=1 | DGPS_FIX=3 | WAAS_FIX=5
 ******************************************************************************/

//int Packet::BuildGpsStatus(const gps_common::GPSStatus &msg)
//{
//
//#ifdef USE_STRINGS
//
//	std::stringstream ss;
//
//	ss << CMD_GPS_STATUS << msg.status << "," << msg.satellites_visible << \
//			"," << msg.satellites_used;
//
//	Build((unsigned char*)ss.str().c_str(), ss.str().length());
//
//	return 0;
//
//#else
//
//	char data[SIZE_GPS_STATUS] = {0};
//
//	data[0] = CMD_GPS_STATUS;			    // Write the GPS_FIX command
//
//	data[1] = msg.status >> 8;				// Write the GPS Status
//	data[2] = msg.status & 0xFF;
//
//	data[3] = msg.satellites_visible >> 8;	// Write the GPS Sats Visible
//	data[4] = msg.satellites_visible & 0xFF;
//
//	data[5] = msg.satellites_used >> 8;		// Write the GPS Sats Used
//	data[6] = msg.satellites_used & 0xFF;
//
//	Build(data, SIZE_GPS_STATUS);				// Add the header and checksum
//	return(0);
//
//#endif
//}

/******************************************************************************
 * Name: 	BuildImuData
 * Inputs:	msg - The ImuData msg from the subscriber
 * Outputs: int - 0 is success, 1 is failure
 *
 * Description: This method assembles the ImuData msg into a packet to send to
 *              the TXT1.
 *
 * 31 Byte Msg - CMD = 102 = 0x66
 * CMD | ORI_X | ORI_Y | ORI_Z | ORI_W | ORI_SIGNS | LA_X | LA_Y | LA_Z | LA_POS
 *------------------------------------------------------------------------------
 * 1B  |  4B   |  4B   |  4B   |  4B   |    1B     |  4B  |  4B  |  4B  |   1B
 *
 * SIGNS Bytes are 1 if positive
 ******************************************************************************/

int Packet::BuildImuData(const sensor_msgs::Imu &msg)
{

#ifdef USE_STRINGS
	std::stringstream ss;
    std::string str;
    const char cmd[2] = {CMD_IMU_DATA, 0};
    
	ss << msg.orientation.x << "," << msg.orientation.y << "," \
			<< msg.orientation.z << "," <<	msg.orientation.w << "," << \
			msg.linear_acceleration.x <<  "," << msg.linear_acceleration.y << \
			"," << msg.linear_acceleration.z;
    
    str.append(cmd, 1);
    str.append(ss.str());
    
	Build((unsigned char*)str.c_str(), str.length());

	return 0;

#else

	char data[SIZE_IMU_DATA] = {0};
	char tempSign = 0xFF;
	char* value;

	data[0] = CMD_IMU_DATA;			        // Write the IMU DATA command

	// Write the X Orientation bytes
	if (msg.orientation.x < 0)              // Store the sign
	    tempSign &= ~(1<<3);
	value = FloatToBytes(msg.orientation.x,4);
	memcpy( &data[1], value, 4 );

	// Write the Y Orientation bytes
	if (msg.orientation.y < 0)              // Store the sign
	    tempSign &= ~(1<<2);
	value = FloatToBytes(msg.orientation.y,4);
	memcpy( &data[5], value, 4 );

	// Write the Z Orientation bytes
	if (msg.orientation.z < 0)              // Store the sign
	    tempSign &= ~(1<<1);
	value = FloatToBytes(msg.orientation.z,4);
	memcpy( &data[9], value, 4 );

	// Write the W Orientation bytes
	if (msg.orientation.w < 0)              // Store the sign
	    tempSign &= ~(1<<0);
	value = FloatToBytes(msg.orientation.w,4);
	memcpy( &data[13], value, 4 );

	data[17] = tempSign;					// Write orientation sign
	tempSign = 0xFF;

	// Write the X Linear Acceleration bytes
	if (msg.linear_acceleration.x < 0)   	// Store the sign
	    tempSign &= ~(1<<2);
	value = FloatToBytes(msg.linear_acceleration.x,4);
	memcpy( &data[18], value, 4 );
	Uart2TxArr(uint8_t *data, uint8_t numBytes)
	// Write the Y Linear Acceleration bytes
	if (msg.linear_acceleration.y < 0)   	// Store the sign
		tempSign &= ~(1<<1);
	value = FloatToBytes(msg.linear_acceleration.y,4);
	memcpy( &data[22], value, 4 );

	// Write the Z Linear Acceleration bytes
	if (msg.linear_acceleration.z < 0)   	// Store the sign
		tempSign &= ~(1<<0);
	value = FloatToBytes(msg.linear_acceleration.z,4);
	memcpy( &data[26], value, 4 );

	data[30] = tempSign;

	Build(data, SIZE_IMU_DATA);		// Add the header and checksum
	return(0);

#endif
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

	vel = (short)msg.linear.x;		// Write the linear velocity
	data[1] = (unsigned char)(vel >> 8);
	data[2] = (unsigned char)(vel & 0x00FF);

	vel = (short)msg.angular.z;	// Write the angular velocity
	data[3] = (unsigned char)(vel >> 8);
	data[4] = (unsigned char)(vel & 0x00FF);

	Build(data, SIZE_VEL);				// Add the header and checksum
	return(0);
}

unsigned char* Packet::FloatToBytes(float value, unsigned char chars)
{
	unsigned long val;
    float temp = value;
    unsigned char *ret = new unsigned char[chars];
    unsigned char loop;

    if (temp < 0)                           // make positive
	    temp = -temp;

    val = (unsigned long)(temp * DIV);      // convert to 4 byte long

    for ( loop = 0; loop < chars; loop++)  // Parse out bytes
        ret[loop] = (unsigned char)(val >> (8*(chars-loop-1)));

    return ret;
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
	double xpos, ypos, theta, linvel, angvel;

	switch(packet[3])
	{
		case CMD_ENC_ODOM:
			if (packetSize - 5 != SIZE_ENC_ODOM)
				break;

			xpos = (double)((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]) / 1000;
			ypos = (double)((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]) / 1000;
			theta = (double)((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]) / 1000;
			linvel = (double)((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]) / 1000;
			angvel = (double)((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]) / 1000;

			p->pubOdom(xpos, ypos, theta, linvel, angvel);

//			ROS_INFO("Packet: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
//					packet[0],packet[1],packet[2],packet[3],packet[4],packet[5],
//					packet[6],packet[7],packet[8],packet[9],packet[10],packet[11],packet[12],
//					packet[13],packet[14],packet[15],packet[16],packet[17],packet[18],packet[19],packet[20],
//					packet[21],packet[22],packet[23],packet[24],packet[25]);
//			ROS_INFO("xpos: %f", xpos);
//			ROS_INFO("ypos: %f", ypos);
//			ROS_INFO("theta: %f", theta);
//			ROS_INFO("linvel: %f", linvel);
//			ROS_INFO("angvel: %f", angvel);
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
