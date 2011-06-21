#ifndef STATES_LIGHT_H
#define STATES_LIGHT_H

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"

class States
{
public:
  typedef enum{LeftForward, StraightForward, RightForward,
	           LeftBackward, StraightBackward, RightBackward} states;
               
  States(ros::NodeHandle nh);
  states GetState(void);
  double GetReward(void);
  double GetDistance(void);
  int GetNumStates(void);
  
private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_msg_;
  states state_;
  int num_states_;
  double prev_light_, current_light_, current_distance_;

  void odom_cb(const nav_msgs::Odometry msg);
};

#endif
