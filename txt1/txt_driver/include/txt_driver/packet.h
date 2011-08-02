#ifndef PACKET_H
#define PACKET_H

#include "ros/ros.h"
#include "txt_driver/serial.h"
//#include "txt_driver/txt1.h"
#include "txt_driver/Pid.h"
#include "txt_driver/PwmTest.h"
#include "txt_driver/Pwm.h"
#include "txt_driver/SwitchedPwr.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>

#define MAX_PACKET_SIZE		255
#define MAX_DATA_SIZE		  (MAX_PACKET_SIZE - HEADER_SIZE - CHKSUM_SIZE )
#define COMMAND_SIZE      1
#define HEADER_SIZE			  4   // FA, FB, SIZE, CMD
#define CHKSUM_SIZE       2
#define MSG_DATA_GOOD     0x3C
#define MSG_DATA_ERROR    0x55

typedef enum {CMD_VEL = 103, CMD_ODOM_ENC = 104, CMD_ODOM_COMB = 105,
	CMD_BATTERY = 106, CMD_PID_TX = 107, CMD_PWM_TEST_TX = 108, CMD_PID_TERMS = 109, 
	CMD_SWITCH_PWR = 110, CMD_PWM = 111, CMD_PID = 112} COMMANDS;
typedef enum {SIZE_VEL = 4, SIZE_ODOM_ENC = 20, SIZE_ODOM_COMB = 20,
	SIZE_BATTERY = 4, SIZE_PID_TX = 24, SIZE_PWM_TEST_TX = 20, SIZE_PID_TERMS = 16, 
	SIZE_SWITCH_PWR = 2, SIZE_PWM = 8, SIZE_PID_MAX = 108} SIZES;
	
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

class Packet
{
	public:		
		Packet();
		virtual ~Packet();
		uint8_t Build(uint8_t *data, uint8_t dataSize, uint8_t command);
    uint8_t BuildCmdVel(const geometry_msgs::Twist &msg);
    uint8_t BuildCombOdom(const nav_msgs::Odometry &msg);
    bool BuildPid(std::vector<double> lin, std::vector<double> ang);
    uint8_t BuildPidTx(const txt_driver::Pid::Request &req);
    uint8_t BuildPWMTestTx(const txt_driver::PwmTest::Request &req);
    uint8_t BuildSwitchPwr(const txt_driver::SwitchedPwr::Request &req);
    uint8_t BuildPwm(const txt_driver::Pwm &msg);
    void Send( Serial * port );
    msg_u * Receive( Serial * port );
    
  private:
		msg_u msg_;
		uint8_t dataNum_;
		uint32_t droppedPkts_, totalPkts_;

		int CalcChkSum();
		bool Check();
		void ProcessData();
};

#endif
