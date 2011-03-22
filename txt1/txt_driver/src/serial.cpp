/*
 * serial.cpp
 *
 *  Created on: Jan 2, 2011
 *      Author: root
 */

#include "ros/ros.h"
#include "txt_driver/serial.h"

Serial::Serial(string port, BAUD_RATE br, DATA_BITS db, PARITY p, STOP_BITS sb)
{
	close(fd);

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0)
	{
		isOpen = false;
		ROS_ERROR("Couldn't open serial port %s", port.c_str());
	}
	else
	{
		if (tcgetattr(fd, &my_termios) < 0)
		{
			isOpen = false;
			close(fd);
			ROS_ERROR("Couldn't read attributes of serial port %s", port.c_str());
		}
		else
		{
			tcflush(fd, TCIFLUSH);

			my_termios.c_iflag = 0;
			//my_termios.c_iflag = IGNBRK | IGNCR;
			my_termios.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
			my_termios.c_oflag = 0;
			my_termios.c_cflag = 0;
			my_termios.c_cflag = db | p | sb | CREAD | CLOCAL;
			//my_termios.c_cflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			my_termios.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
			my_termios.c_cc[VMIN]  = 0;
			my_termios.c_cc[VTIME] = 0;

			cfsetospeed(&my_termios, br);
			cfsetispeed(&my_termios, br);

			if (tcsetattr(fd, TCSANOW, &my_termios) < 0)
			{
				isOpen = false;
				close(fd);
				ROS_ERROR("Couldn't set attributes of serial port %s", port.c_str());
			}
			else
			{
				isOpen = true;
			}
		}
	}
}

Serial::~Serial() {
	this->Close();
}

void Serial::Close(void)
{
	close(fd);
	isOpen = false;
}

int Serial::Write(unsigned char * bytes, int n) throw (ros::Exception)
{
	int numBytes;

	if (isOpen)
	{
		numBytes = write(fd, bytes, n);

		if (numBytes < 0)
		{
			ROS_ERROR("Serial port error sending bytes!");
			throw new ros::Exception("Serial port error sending bytes!");
		}
	}
	else
	{
		ROS_ERROR("Serial port is not open!");
		throw new ros::Exception("Serial port is not open!");
	}
	return numBytes;
}

int Serial::Read(unsigned char *bytes, int n) throw (ros::Exception)
{
	int numBytes;

	if (isOpen)
	{
		numBytes = read(fd, bytes, n);

//		if (numBytes < 0)
//		{
//			ROS_ERROR("Serial port error reading bytes!");
//			throw new ros::Exception("Serial port error reading bytes!");
//		}
	}
	else
	{
		ROS_ERROR("Serial port is not open!");
		throw new ros::Exception("Serial port is not open!");
	}
	return numBytes;
}
