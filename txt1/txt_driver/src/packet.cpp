#include "ros/ros.h"
#include "txt_driver/packet.h"

extern TXT1* p;

Packet::Packet()
{
	dataNum_ = 0;
	droppedPkts_ = 0;
	totalPkts_ = 0;
}

Packet::~Packet()
{

}

/******************************************************************************
 * Name: 	Build
 * Inputs:	data - The char array of data bytes
 *          dataSize - The size the the char array of data bytes
 * Outputs: int32_t - 0 for completed ok, 1 for error
 * 
 * Description: This method builds a data packet to be sent to the TXT1.  It
 *              attaches a header of two bytes to the beginning and then a byte 
 *              indicating the length of the data section including the two 
 *              checksum bytes.  Finally the checksum is calculated and attached
 *              to the end with the bytes in reverse order like the Pioneers.
 ******************************************************************************/

uint8_t Packet::Build(uint8_t *data, uint8_t dataSize, uint8_t command)
{
	uint16_t checksum;

	msg_.var.header.var.start_byte_1 = 0xFA;   // Write 2 header bytes
  msg_.var.header.var.start_byte_2 = 0xFB;
	msg_.var.header.var.length = dataSize;     // Write the length of packet
  msg_.var.header.var.command = command;

  memcpy( &msg_.var.data[0], data, dataSize ); // Copy all the data bytes to packet

	checksum = CalcChkSum();				             // Calculate the checksum
	msg_.var.data[dataSize] = checksum >> 8;		 // Put the checksum bytes in
	msg_.var.data[dataSize+1] = checksum & 0xFF; // reverse order

	if (!Check()) {
		ROS_ERROR("Checksum malfunction.");
		return(1);
	}

	return(0);
}

/******************************************************************************
 * Name: 	BuildCmdVel
 * Inputs:	msg_ - The Cmd Vel msg_ from the subscriber
 * Outputs: int32_t - 0 is success, 1 is failure
 *
 * Description: This method assembles the CmdVel msg_ int32_to a packet to send to
 *              the TXT1.
 *
 * 5 Byte msg_ - CMD = 103 = 0x67
 * CMD | FOR_VEL | ROT_VEL
 *----------------------------
 * 1B  |    2B   |   2B
 ******************************************************************************/

uint8_t Packet::BuildCmdVel(const geometry_msgs::Twist &msg)
{
	uint8_t data[SIZE_VEL] = {0};
	int16_t vel = 0;

	vel = (int16_t)(msg.linear.x * 1000);		// Write the linear velocity
	data[0] = (uint8_t)(vel >> 8);
	data[1] = (uint8_t)(vel & 0x00FF);

	vel = (int16_t)(msg.angular.z * 1000);	// Write the angular velocity
	data[2] = (uint8_t)(vel >> 8);
	data[3] = (uint8_t)(vel & 0x00FF);

	Build(data, SIZE_VEL, CMD_VEL);         // Add the header and checksum
	return(0);
}

uint8_t Packet::BuildCombOdom(const nav_msgs::Odometry &msg)
{
	uint8_t data[SIZE_ODOM_COMB] = {0};
	int32_t temp;

	temp = (int32_t)(msg.pose.pose.position.x * 1000);
	data[0] = (uint8_t)(temp >> 24);
	data[1] = (uint8_t)(temp >> 16);
	data[2] = (uint8_t)(temp >> 8);
	data[3] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(msg.pose.pose.position.y * 1000);
	data[4] = (uint8_t)(temp >> 24);
	data[5] = (uint8_t)(temp >> 16);
	data[6] = (uint8_t)(temp >> 8);
	data[7] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(tf::getYaw(msg.pose.pose.orientation) * 1000);
	data[8] = (uint8_t)(temp >> 24);
	data[9] = (uint8_t)(temp >> 16);
	data[10] = (uint8_t)(temp >> 8);
	data[11] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(msg.twist.twist.linear.x * 1000);
	data[12] = (uint8_t)(temp >> 24);
	data[13] = (uint8_t)(temp >> 16);
	data[14] = (uint8_t)(temp >> 8);
	data[15] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(msg.twist.twist.angular.z * 1000);
	data[16] = (uint8_t)(temp >> 24);
	data[17] = (uint8_t)(temp >> 16);
	data[18] = (uint8_t)(temp >> 8);
	data[19] = (uint8_t)(temp & 0x000000FF);

	Build(data, SIZE_ODOM_COMB, CMD_ODOM_COMB);

	return(0);
}

uint8_t Packet::BuildPidTx(const txt_driver::pid::Request &req)
{
	uint8_t data[SIZE_PID_TX] = {0};
	int32_t temp;

	temp = (int32_t)(req.kp_lv);
	data[0] = (uint8_t)(temp >> 24);
	data[1] = (uint8_t)(temp >> 16);
	data[2] = (uint8_t)(temp >> 8);
	data[3] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(req.ki_lv);
	data[4] = (uint8_t)(temp >> 24);
	data[5] = (uint8_t)(temp >> 16);
	data[6] = (uint8_t)(temp >> 8);
	data[7] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(req.kd_lv);
	data[8] = (uint8_t)(temp >> 24);
	data[9] = (uint8_t)(temp >> 16);
	data[10] = (uint8_t)(temp >> 8);
	data[11] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(req.kp_av);
	data[12] = (uint8_t)(temp >> 24);
	data[13] = (uint8_t)(temp >> 16);
	data[14] = (uint8_t)(temp >> 8);
	data[15] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(req.ki_av);
	data[16] = (uint8_t)(temp >> 24);
	data[17] = (uint8_t)(temp >> 16);
	data[18] = (uint8_t)(temp >> 8);
	data[19] = (uint8_t)(temp & 0x000000FF);

	temp = (int32_t)(req.kd_av);
	data[20] = (uint8_t)(temp >> 24);
	data[21] = (uint8_t)(temp >> 16);
	data[22] = (uint8_t)(temp >> 8);
	data[23] = (uint8_t)(temp & 0x000000FF);

	Build(data, SIZE_PID_TX, CMD_PID_TX);

	return(0);
}

uint8_t Packet::BuildPWMTx(const txt_driver::pwm::Request &req)
{
  uint8_t data[SIZE_PWM_TX] = {0};
	int32_t temp;
	
	temp = (int32_t)(req.esc);
	data[0] = (uint8_t)(temp >> 24);
	data[1] = (uint8_t)(temp >> 16);
	data[2] = (uint8_t)(temp >> 8);
	data[3] = (uint8_t)(temp & 0x000000FF);
	
  temp = (int32_t)(req.front);
	data[4] = (uint8_t)(temp >> 24);
	data[5] = (uint8_t)(temp >> 16);
	data[6] = (uint8_t)(temp >> 8);
	data[7] = (uint8_t)(temp & 0x000000FF);
	
	temp = (int32_t)(req.rear);
	data[8] = (uint8_t)(temp >> 24);
	data[9] = (uint8_t)(temp >> 16);
	data[10] = (uint8_t)(temp >> 8);
	data[11] = (uint8_t)(temp & 0x000000FF);
	
	Build(data, SIZE_PWM_TX, CMD_PWM_TX);
	
  return(0);
}

/******************************************************************************
 * Name: 	CalcChkSum()
 * Inputs:	None
 * Outputs: int32_t - The calculated checksum
 * 
 * Description: This method calculates the checksum according to the Pioneer 
 *              documentation.
 ******************************************************************************/

int32_t Packet::CalcChkSum()
{
  uint8_t *buffer = &msg_.var.header.var.command;
  int32_t c = 0;
  int32_t n;

  n = msg_.var.header.var.length + 1;				// Get the number of data bytes
	
	// For the even number of bytes, successively adding data byte pairs (high 
	// byte first) to a running checksum (initially zero), disregarding sign and 
	// overflow.
  while (n > 1) 
	{
    	c+= (*(buffer)<<8) | *(buffer+1);	// Add byte pair to checksum
    	c = c & 0xffff;						// Disregard overflow
    	n -= 2;								// Calc next point32_ter
    	buffer += 2;
  	}
	
	// If there are an odd number of data bytes, the last byte is XORed to the 
	// low-order byte of the checksum.
  	if (n>0) c = c ^ (int32_t)*(buffer++);

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
  	if ( chksum == ((msg_.var.data[msg_.var.header.var.length] << 8) | (msg_.var.data[msg_.var.header.var.length + 1])))
   		return(true);

  	return(false);
}

/******************************************************************************
 * Name: 	Send()
 * Inputs:	port - The serial port handle to send on
 * Outputs: int32_t - 0 is success, 1 is failure
 * 
 * Description: This method sends the assembled packet on the given serial port.
 ******************************************************************************/

void Packet::Send( Serial::Serial* port )
{
	int32_t cnt = 0;

	try
	{
		cnt = port->Write(msg_.bytes, msg_.var.header.var.length + HEADER_SIZE	+ CHKSUM_SIZE);
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR("Failure to send. Error: %s", e.what());
	}

	if (cnt != msg_.var.header.var.length + HEADER_SIZE	+ CHKSUM_SIZE)
	{
		ROS_ERROR("Not all bytes sent.");
	}
}

void Packet::Receive( Serial::Serial * port )
{
	while ((dataNum_ < MAX_PACKET_SIZE) && (port->Read(&msg_.bytes[dataNum_], 1) > 0))
	{
		if (dataNum_ == 0)						// Read the packet header
		{
		  if (msg_.var.header.var.start_byte_1 = 0xFA)
		  {
		    dataNum_++;
		  }
		}
		else if (dataNum_ == 1)
		{
		  if (msg_.var.header.var.start_byte_2 = 0xFB)
		  {
		    dataNum_++;
		  }
		  else
		  {
		    dataNum_ = 0;
		  }
		}
		else									// Get the rest of the packet
		{
        dataNum_++;
        
      // If last byte, process cksum               
      if(dataNum_ >= (HEADER_SIZE + msg_.var.header.var.length + CHKSUM_SIZE))
      {                                 // then data.
        if(Check())
        {
          ProcessData();
          totalPkts_++;
          dataNum_ = 0;
        }
        else
        {
          droppedPkts_++;
          totalPkts_++;
          ROS_INFO("Dropped Packets = %d / %d", droppedPkts_, totalPkts_);
          dataNum_ = 0;
        }
      }
    }		 
  }

	if (dataNum_ >= MAX_PACKET_SIZE)
		dataNum_ = 0;
}

void Packet::ProcessData()
{
	double xpos, ypos, theta, linvel, angvel, batt1, batt2;

	switch(msg_.var.header.var.command)
	{
		case CMD_ODOM_ENC:
			if (msg_.var.header.var.length != SIZE_ODOM_ENC)
				break;

			xpos = (double)((msg_.var.data[0] << 24) + (msg_.var.data[1] << 16) + (msg_.var.data[2] << 8) + msg_.var.data[3]) / 1000;
			ypos = (double)((msg_.var.data[4] << 24) + (msg_.var.data[5] << 16) + (msg_.var.data[6] << 8) + msg_.var.data[7]) / 1000;
			theta = (double)((msg_.var.data[8] << 24) + (msg_.var.data[9] << 16) + (msg_.var.data[10] << 8) + msg_.var.data[11]) / 1000;
			linvel = (double)((msg_.var.data[12] << 24) + (msg_.var.data[13] << 16) + (msg_.var.data[14] << 8) + msg_.var.data[15]) / 1000;
			angvel = (double)((msg_.var.data[16] << 24) + (msg_.var.data[17] << 16) + (msg_.var.data[18] << 8) + msg_.var.data[19]) / 1000;
			p->pubOdom(xpos, ypos, theta, linvel, angvel);
			break;
		case CMD_BATTERY:
			if (msg_.var.header.var.length != SIZE_BATTERY)
				break;

			batt1 = (double)((msg_.var.data[0] << 8) + msg_.var.data[1]) / 1000;
			batt2 = (double)((msg_.var.data[2] << 8) + msg_.var.data[3]) / 1000;

			p->pubBattery(batt1, batt2);
			break;
		default:

			break;
	}
}
