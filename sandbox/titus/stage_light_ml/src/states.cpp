#include "stage_light_ml/states.h"
               
States::States(ros::NodeHandle nh)
{
  n_ = nh;
  blob_sub_ = n_.subscribe<stage_msgs::StageBlobScan>("/scan_blobfinder", 1000, &States::blob_cb, this);
}

States::states States::GetState(void)
{
  return state_; 
}

int States::GetReward(void)
{
  return(0);
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
    dist_ratio = blob_msg_.blobs[0].range / blob_msg_.range;
    
    state_sum = (int)(pos_ratio * INTERVALS);
    state_sum += (int)(dist_ratio * INTERVALS) * INTERVALS;
    state_ = (States::states)state_sum;
  }
  else
  {
    state_ = States::NotFound;
  }
  
  ROS_INFO("State: %d", (int)state_);
}  

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "states_test");
	ros::NodeHandle n;
	
  States* s = new States(n);  
  ros::spin();
}
