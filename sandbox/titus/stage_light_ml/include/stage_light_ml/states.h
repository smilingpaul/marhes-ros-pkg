#ifndef STATES_H
#define STATES_H

#include "ros/ros.h"
#include "stage_msgs/StageBlobScan.h"

class States
{
public:
//  typedef enum{LeftClose, StraightClose, RightClose,
//               LeftMid, StraightMid, RightMid,
//               LeftFar, StraightFar, RightFar, NotFound} states;
	typedef enum{Left, Straight, Right, LeftToNotFound, RightToNotFound} states;
  static const int INTERVALS = 3;
               
  States(ros::NodeHandle nh);
  states GetState(void);
  double GetReward(void);
  double GetDistance(void);
  int GetNumStates(void);
  
private:
  ros::NodeHandle n_;
  ros::Subscriber blob_sub_;
  stage_msgs::StageBlobScan blob_msg_;
  states state_;
  int num_states_;
  double prev_distance_, current_distance_;
  
  void blob_cb(const stage_msgs::StageBlobScan msg);
};

#endif
