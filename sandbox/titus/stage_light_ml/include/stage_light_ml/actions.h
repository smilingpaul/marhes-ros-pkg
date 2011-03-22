#ifndef ACTIONS_H
#define ACTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class Actions
{
public:
  typedef enum {LeftForward, StraightForward, RightForward, LeftBackward, StraightBackward, RightBackward} moveType;

  Actions(ros::NodeHandle nh);
  void Move(moveType type);
  
private:  
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
};

#endif
