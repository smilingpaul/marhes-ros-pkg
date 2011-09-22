#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class VelFilter
{
private:
  ros::NodeHandle n_;
  ros::Subscriber vel_sub_;
  ros::Publisher vel_pub_;
  ros::Timer tmr_;
  geometry_msgs::Twist vel_msg_, vel_msg_out_;
  ros::Time last_time_;
  double ang_slope_, ang_delay_, total_delay_, sign_, period_;
  int vel_state_;
  
  enum {STATE_DELAY, STATE_RAMP, STATE_CONSTANT};
     
  void vel_cb(geometry_msgs::Twist msg);
  void tmr_cb(const ros::TimerEvent& event);
public:
  VelFilter(ros::NodeHandle nh);
};

VelFilter::VelFilter(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  n_private.param("ang_slope", ang_slope_, 1.54); // rad/s/s
  n_private.param("ang_delay", ang_delay_, 0.35); // s
  
  
  vel_msg_.linear.x = 0.0;
  vel_msg_.angular.z = 0.0;
  vel_msg_out_ = vel_msg_;
  vel_state_ = STATE_CONSTANT;
  last_time_ = ros::Time::now();
  sign_ = 1;
  period_ = 0.01;
  
  vel_sub_ = n_.subscribe<geometry_msgs::Twist>("vel_in", 1, &VelFilter::vel_cb, this);
  vel_pub_ = n_.advertise<geometry_msgs::Twist>("vel_out", 1);
  tmr_ = n_.createTimer(ros::Duration(period_), &VelFilter::tmr_cb, this);
  
  ROS_INFO("Leaving setup");
}

void VelFilter::vel_cb(geometry_msgs::Twist msg)
{
  if (vel_msg_.linear.x != msg.linear.x || vel_msg_.angular.z != msg.angular.z)
  {
    vel_state_ = STATE_RAMP;
    total_delay_ = 0;
      
    vel_msg_ = msg;
  }
}

void VelFilter::tmr_cb(const ros::TimerEvent& event)
{
  ros::Time now = ros::Time::now();
  ros::Duration elapsed = now - last_time_;
  //ROS_INFO("L: %f, N: %f", last_time_.toSec(), now.toSec());
  last_time_ = now;
  //ROS_INFO("Elapsed: %f", elapsed.toSec());
  
  switch(vel_state_)
  {
    case STATE_DELAY:
        total_delay_ += elapsed.toSec();
        if (total_delay_ >= ang_delay_)
        {
          vel_state_ = STATE_RAMP;
        }
        //ROS_INFO("STATE_DELAY");
      break;
    case STATE_RAMP:
      if (vel_msg_out_.angular.z <= vel_msg_.angular.z + ang_slope_ * period_ &&
          vel_msg_out_.angular.z >= vel_msg_.angular.z - ang_slope_ * period_)
        vel_state_ = STATE_CONSTANT;      
      else
      {
        // Determine the sign
        if (vel_msg_.angular.z > vel_msg_out_.angular.z)
          sign_ = 1;
        else
          sign_ = -1;
        vel_msg_out_.linear.x = vel_msg_.linear.x;
        vel_msg_out_.angular.z = vel_msg_out_.angular.z + sign_ * ang_slope_ * elapsed.toSec();  
      }
      //ROS_INFO("STATE_RAMP");
      break;
    case STATE_CONSTANT:
      //ROS_INFO("STATE_CONSTANT");
      vel_msg_out_ = vel_msg_;
      break;
    default:
      //ROS_INFO("DEFAULT");
      break;
  }
  
  vel_pub_.publish(vel_msg_out_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_filter");
	ros::NodeHandle n;
	
	VelFilter* v = new VelFilter(n);
	
	ros::spin();
	
	return 0;
}
