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
#define MAX_DATA_SIZE		  (MAX_PACKET_SIZE - HEADER_SIZE - CHKSUM_SIZE )
#define COMMAND_SIZE      1
#define HEADER_SIZE			  4   // FA, FB, SIZE, CMD
#define CHKSUM_SIZE       2

class Packet
{
	private:
		typedef enum {CMD_VEL = 103, CMD_ODOM_ENC = 104, CMD_ODOM_COMB = 105,
			CMD_BATTERY = 106, CMD_PID_TX = 107} COMMANDS;
		typedef enum {SIZE_VEL = 4, SIZE_ODOM_ENC = 20, SIZE_ODOM_COMB = 20,
			SIZE_BATTERY = 4, SIZE_PID_TX = 24} SIZES;
			
    typedef struct {
      uint8_t start_byte_1;
      uint8_t start_byte_2;
      uint8_t length;
      uint8_t command;
    } header_t;
    
    typedef union {
      header_t var;
      uint8_t bytes[HEADER_SIZE];
    } header_u;

    typedef struct {
      header_u header;
      uint8_t data[MAX_DATA_SIZE];
      uint16_t chksum;
    } msg_t;
    
    typedef union {
      msg_t var;
      uint8_t bytes[MAX_PACKET_SIZE];
    } msg_u;

		msg_u msg_;
		uint8_t dataNum_;
		uint32_t droppedPkts_, totalPkts_;

		int CalcChkSum();
		bool Check();
		void ProcessData();
	public:
		Packet();
		virtual ~Packet();
		uint8_t Build(uint8_t *data, uint8_t dataSize, uint8_t command);
    uint8_t BuildCmdVel(const geometry_msgs::Twist &msg);
    uint8_t BuildCombOdom(const nav_msgs::Odometry &msg);
    uint8_t BuildPidTx(const txt_driver::pid::Request &req);
    void Send( Serial::Serial* port );
    void Receive( Serial::Serial * port );
};

#endif
