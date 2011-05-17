#ifndef STATES_H
#define STATES_H

#include "ros/ros.h"
#include "phidgets_ros/Float64Stamped.h"
#include "nav_msgs/Odometry.h"

class States
{
public:
  States(ros::NodeHandle nh);
  int GetState(void);
  double GetReward(void);
  double GetDistance(void);
  int GetNumStates(void);

  enum {FLLS, FRLS, RLLS, RRLS};
  
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_odom_, sub_flls_, sub_frls_, sub_rlls_, sub_rrls_;
  ros::Timer tmr_state_;
  int state_, num_states_;
  double ls_vals_[4];

  void cb_flls(phidgets_ros::Float64Stamped msg);
  void cb_frls(phidgets_ros::Float64Stamped msg);
  void cb_rlls(phidgets_ros::Float64Stamped msg);
  void cb_rrls(phidgets_ros::Float64Stamped msg);
};

#endif
