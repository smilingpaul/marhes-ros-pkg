#ifndef PACKET_H
#define PACKET_H

#include "ros/ros.h"
#include "txt_driver/serial.h"
#include "txt_driver/txt1.h"
#include "sensor_msgs/Imu.h"
//#include "gps_common/GPSFix.h"
//#include "gps_common/GPSStatus.h"
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>

#define MAX_PACKET_SIZE		255

// Divisor for making floats a hexadecimal representation
#define DIV_NUM		        32768000		
#define DIV_DEN				1000
#define DIV					(DIV_NUM / DIV_DEN)

#define USE_STRINGS

class Packet
{
	private:
		unsigned char packet[MAX_PACKET_SIZE];
		unsigned char packetSize, dataNum;
		ros::Time rxTime;
		int droppedPkts, totalPkts;

		typedef enum {CMD_GPS_FIX = 100, CMD_GPS_STATUS = 101,
			CMD_IMU_DATA = 102, CMD_VEL = 103, CMD_ENC_ODOM = 104} COMMANDS;
		typedef enum {SIZE_GPS_FIX = 19, SIZE_GPS_STATUS = 7,
			SIZE_IMU_DATA = 31, SIZE_VEL = 5, SIZE_ENC_ODOM = 21} SIZES;

		unsigned char* FloatToBytes(float value, unsigned char chars);
		int CalcChkSum();
		bool Check();
		void ProcessData();
	public:
		Packet();
		virtual ~Packet();
		int Build(unsigned char *data, unsigned char dataSize);
//		int BuildGpsFix(const gps_common::GPSFix &msg);
//        int BuildGpsStatus(const gps_common::GPSStatus &msg);
        int BuildImuData(const sensor_msgs::Imu &msg);
        int BuildCmdVel(const geometry_msgs::Twist &msg);
        void Send( Serial::Serial* port );
        void Receive( Serial::Serial * port );
        void Print();
		void PrintHex();
};

#endif
