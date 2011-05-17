#ifndef PACKET_H
#define PACKET_H

#include "ros/ros.h"
#include "txt_driver/serial.h"
#include "txt_driver/txt1.h"
#include "txt_driver/pid.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>

#define MAX_PACKET_SIZE		255

class Packet
{
	private:
		unsigned char packet[MAX_PACKET_SIZE];
		unsigned char packetSize, dataNum;
		ros::Time rxTime;
		int droppedPkts, totalPkts;

		typedef enum {CMD_VEL = 103, CMD_ODOM_ENC = 104, CMD_ODOM_COMB = 105,
			CMD_BATTERY = 106, CMD_PID_TX = 107} COMMANDS;
		typedef enum {SIZE_VEL = 5, SIZE_ODOM_ENC = 21, SIZE_ODOM_COMB = 21,
			SIZE_BATTERY = 7, SIZE_PID_TX = 25} SIZES;

		int CalcChkSum();
		bool Check();
		void ProcessData();
	public:
		Packet();
		virtual ~Packet();
		int Build(unsigned char *data, unsigned char dataSize);
        int BuildCmdVel(const geometry_msgs::Twist &msg);
        int BuildCombOdom(const nav_msgs::Odometry &msg);
        int BuildPidTx(const txt_driver::pid::Request &req);
        void Send( Serial::Serial* port );
        void Receive( Serial::Serial * port );
        void Print();
		void PrintHex();
};

#endif
