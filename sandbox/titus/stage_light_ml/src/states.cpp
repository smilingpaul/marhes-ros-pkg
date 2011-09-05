#include "stage_light_ml/states.h"
               
States::States(ros::NodeHandle nh)
{
  n_ = nh;
  blob_sub_ = n_.subscribe<stage_msgs::StageBlobScan>("/scan_blobfinder", 1000, &States::blob_cb, this);
  num_states_ = 5;//10;
  prev_distance_ = 0.0;
  current_distance_ = 1000000;
  state_ = States::RightToNotFound;//States::NotFound;
}

States::states States::GetState(void)
{
  return state_; 
}

double States::GetReward(void)
{
	double reward;
//	if (state_ == States::NotFound)
//		reward = -1;
	if (state_ == States::RightToNotFound)
		reward = -1.5;
	else if (state_ == States::LeftToNotFound)
		reward = -1.5;
	else
	{
		reward = 2.5 * (prev_distance_ - current_distance_);
		prev_distance_ = current_distance_;
	}

	return reward;
}

double States::GetDistance(void)
{
	return current_distance_;
}

int States::GetNumStates(void)
{
	return num_states_;
}
  
void States::blob_cb(const stage_msgs::StageBlobScan msg)
{
  float xpos, pos_ratio, dist_ratio;
  int state_sum;
  
  blob_msg_ = msg;
  
  if(blob_msg_.blobs.size() > 0)
  {
    xpos = blob_msg_.blobs[0].left + (blob_msg_.blobs[0].right - blob_msg_.blobs[0].left) / 2;
    pos_ratio = xpos / blob_msg_.scan_width;
    current_distance_ = blob_msg_.blobs[0].range;
    dist_ratio = current_distance_ / blob_msg_.range;
    
    state_sum = (int)(pos_ratio * INTERVALS);
    state_sum += (int)(dist_ratio * INTERVALS) * INTERVALS;
    state_ = (States::states)state_sum;
  }
  else
  {
	  if (state_ == States::Right)
		  state_ = States::RightToNotFound;
	  if (state_ == States::Left)
		  state_ = States::LeftToNotFound;
  }
  
  ROS_INFO("State: %d", (int)state_);
}  

//int main(int argc, char **argv)
//{
//	ros::init(argc, argv, "states_test");
//	ros::NodeHandle n;
//
//  States* s = new States(n);
//  ros::spin();
//
//  return 0;
//}
