#ifndef ACTIONS_H
#define ACTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class Actions
{
public:
  Actions(ros::NodeHandle nh);
//  void Move(int action);
  int GetNumActions(void);
  
private:  
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  int num_actions_;
  double ang_vel_lim_, ang_inc_;
  std::vector<double> ang_vels_;
};

#endif
