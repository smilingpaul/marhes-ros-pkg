/*
 * serial.h
 *
 *  Created on: Jan 2, 2011
 *      Author: root
 */

#ifndef SERIAL_H_
#define SERIAL_H_

using namespace std;

#include "ros/ros.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

class Serial {
public:
	bool isOpen;

	typedef enum {BAUD_300 = B300, BAUD_600 = B600, BAUD_1200 = B1200, BAUD_1800 = B1800,
		BAUD_2400 = B2400, BAUD_4800 = B4800, BAUD_9600 = B9600, BAUD_19200 = B19200,
		BAUD_38400 = B38400, BAUD_57600 = B57600, BAUD_115200 = B115200,
		BAUD_230400 = B230400} BAUD_RATE;
	typedef enum {SIZE_5 = CS5, SIZE_6 = CS6, SIZE_7 = CS7, SIZE_8 = CS8} DATA_BITS;
	typedef enum {NONE = 0, EVEN = PARENB, ODD = PARENB | PARODD} PARITY;
	typedef enum {ONE = 0, TWO = CSTOPB} STOP_BITS;

	Serial(string port, BAUD_RATE br, DATA_BITS db, PARITY p, STOP_BITS sb);
	virtual ~Serial();
	void Close(void);
	int Write(unsigned char * bytes, int n) throw (ros::Exception);
	int Read(unsigned char *bytes, int n) throw (ros::Exception);

private:
	int fd;
	struct termios my_termios;
};
#endif /* SERIAL_H_ */
