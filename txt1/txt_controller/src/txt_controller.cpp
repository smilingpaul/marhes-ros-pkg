#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "txt_controller/Gains.h"
#include "txt_controller/PidTerms.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "joy/Joy.h"

class TxtController{
public:
	TxtController(ros::NodeHandle nh);
	virtual ~TxtController();
private:
	ros::NodeHandle n_;
	ros::Subscriber vel_sub_, odom_sub_, gains_sub_, joy_sub_;
	ros::Publisher pid_term_pub_;
	ros::Timer pid_loop_tmr_;
	
  geometry_msgs::Twist cmd_vel_msg_;
  nav_msgs::Odometry odom_msg_;
  uint32_t kp_lv_, ki_lv_, kd_lv_;
  uint32_t kp_av_, ki_av_, kd_av_;
  double loop_freq_;
  int fd_, VELOCITY_PWM_MAX, VELOCITY_PWM_MIN;
  struct termios my_termios_;
  std::string port_;
  bool serial_open_;
	joy::Joy joyMsg;
  bool joyRxFlag;
  
  void cmdVelCB(geometry_msgs::Twist msg);
  void odomCB(nav_msgs::Odometry msg);
  void gainsCB(txt_controller::Gains msg);
  void pidTmr(const ros::TimerEvent& e);
  void sendCommands(int32_t linear, int32_t angular);
  void joyCB(joy::Joy msg);
};

TxtController::TxtController(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  port_ = "/dev/ttyUSB0";

  n_private.param("freq", loop_freq_, 50.0);
  n_private.param("port", port_, port_);
  kp_lv_ = 0; ki_lv_ = 0; kd_lv_ = 0;
  kp_av_ = 0; ki_av_ = 0; kd_av_ = 0;
  VELOCITY_PWM_MAX = 24000;
  VELOCITY_PWM_MIN = -24000;
  
  close(fd_);
	fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd_ < 0)
	{
		serial_open_ = false;
		ROS_ERROR("Couldn't open serial port %s", port_.c_str());
	}
	else
	{
		if (tcgetattr(fd_, &my_termios_) < 0)
		{
			serial_open_ = false;
			close(fd_);
			ROS_ERROR("Couldn't read attributes of serial port %s", port_.c_str());
		}
		else
		{
			tcflush(fd_, TCIFLUSH);

			my_termios_.c_iflag = 0;
			//my_termios_.c_iflag = IGNBRK | IGNCR;
			my_termios_.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
			my_termios_.c_oflag = 0;
			my_termios_.c_cflag = 0;
			my_termios_.c_cflag = CS8 | 0 | 0 | CREAD | CLOCAL;
			//my_termios_.c_cflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			my_termios_.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
			my_termios_.c_cc[VMIN]  = 0;
			my_termios_.c_cc[VTIME] = 0;

			cfsetospeed(&my_termios_, B57600);
			cfsetispeed(&my_termios_, B57600);

			if (tcsetattr(fd_, TCSANOW, &my_termios_) < 0)
			{
				serial_open_ = false;
				close(fd_);
				ROS_ERROR("Couldn't set attributes of serial port %s", port_.c_str());
			}
			else
			{
				serial_open_ = true;
				ROS_INFO("Opened serial port: %s", port_.c_str());
			}
		}
	}
	
	joyRxFlag = false;
  
  vel_sub_ = n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &TxtController::cmdVelCB, this);
  odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom", 1, &TxtController::odomCB, this);
  gains_sub_ = n_.subscribe<txt_controller::Gains>("/gains", 1, &TxtController::gainsCB, this);
  pid_term_pub_ = n_.advertise<txt_controller::PidTerms>("/pid_terms", 1);
  pid_loop_tmr_ = n_.createTimer(ros::Duration(1 / loop_freq_), &TxtController::pidTmr, this);
  
  joy_sub_ = n_.subscribe<joy::Joy>("joy", 10, &TxtController::joyCB, this);
}

TxtController::~TxtController()
{

}

void TxtController::cmdVelCB(geometry_msgs::Twist msg)
{
  cmd_vel_msg_ = msg;
}

void TxtController::odomCB(nav_msgs::Odometry msg)
{
  odom_msg_ = msg;
}

void TxtController::gainsCB(txt_controller::Gains msg)
{
  kp_lv_ = msg.kp_lv; ki_lv_ = msg.ki_lv; kd_lv_ = msg.kd_lv;
  kp_av_ = msg.kp_av; ki_av_ = msg.ki_av; kd_av_ = msg.kd_av;

}

void TxtController::joyCB(joy::Joy msg)
{
	joyMsg = msg;
	joyRxFlag = true;
}

void TxtController::pidTmr(const ros::TimerEvent& e)
{
  static float e_lv = 0, e_av = 0;
  static float e_lv_last = 0, e_av_last = 0;
  static float e_lv_last2 = 0, e_av_last2 = 0;
  static float lpterm, literm, ldterm;
  static float apterm, aiterm, adterm;
  static int32_t u_lv = 0, u_av = 0;
  static txt_controller::PidTerms termsMsg;
  
  e_lv = cmd_vel_msg_.linear.x - odom_msg_.twist.twist.linear.x;
  e_av = cmd_vel_msg_.angular.z - odom_msg_.twist.twist.angular.z;
  
  lpterm = kp_lv_ * (e_lv - e_lv_last);
  literm = ki_lv_ * (e_lv + e_lv_last) / (2 * loop_freq_);
  ldterm = kd_lv_ * (e_lv - 2 * e_lv_last + e_lv_last2) * loop_freq_;
  
  u_lv += (int32_t)(lpterm + literm + ldterm);
  
  if (u_lv > VELOCITY_PWM_MAX)
    u_lv = VELOCITY_PWM_MAX;
    		    
  if (u_lv < VELOCITY_PWM_MIN)
    u_lv = VELOCITY_PWM_MIN;
    
  apterm = kp_av_ * (e_av - e_av_last);
  aiterm = ki_av_ * (e_av + e_av_last) / (2 * loop_freq_);
  adterm = kd_av_ * (e_av - 2 * e_av_last + e_av_last2) * loop_freq_;
  
  u_av += (int32_t)(apterm + aiterm + adterm);
  
  if (u_av > VELOCITY_PWM_MAX)
    u_av = VELOCITY_PWM_MAX;
    		    
  if (u_av < VELOCITY_PWM_MIN)
    u_av = VELOCITY_PWM_MIN;

  // Set the PWM duty cycles for the motor and the steering servos
  if (joyRxFlag && joyMsg.buttons[2] == 1)
    sendCommands((int32_t)u_lv, (int32_t)(cmd_vel_msg_.angular.z * 9000));
  else
  {
    sendCommands(0, 0);
    u_lv = 0; u_av = 0; e_lv_last = 0; e_av_last = 0; e_lv_last2 = 0; e_av_last2 = 0;
  }

  termsMsg.pterm = lpterm;
  termsMsg.iterm = literm;
  termsMsg.dterm = ldterm;
  termsMsg.signal = u_lv;
  pid_term_pub_.publish(termsMsg);

  // Store last velocity errors
  e_lv_last2 = e_lv_last;
  e_av_last2 = e_av_last;
  e_lv_last = e_lv;
  e_av_last = e_av;
}

void TxtController::sendCommands(int32_t linear, int32_t angular)
{
  static uint8_t data[10];
  ROS_INFO("Linear: %i, Angular: %i", linear, angular);
    
  data[0] = 0xFA;
  data[1] = 0xFB;
  data[2] = 0x04;
  data[3] = 0x55;
  data[4] = (uint8_t)(linear >> 8);
  data[5] = (uint8_t)(linear & 0xFF);
  data[6] = (uint8_t)(angular >> 8);
  data[7] = (uint8_t)(angular & 0xFF); 
  
  ROS_INFO("data6: %i, data7: %i", data[6], data[7]);

  write(fd_, data, 8);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TxtController");
	ros::NodeHandle n;

	TxtController *p = new TxtController(n);

  ros::spin();

	return 0;
}
