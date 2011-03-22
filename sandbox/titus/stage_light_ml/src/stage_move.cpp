#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <stage.hh>

class StageMove
{
public:
  StageMove(ros::NodeHandle nh, double period)
  {
    n_ = nh;
    tmr_ = n_.createTimer(ros::Duration(period), &StageMove::tmr_cb, this);
    pose_sub_ = n_.subscribe("stage_set_pose", 10, &StageMove::pose_cb, this);
  }

private:
  ros::NodeHandle n_;
  ros::Timer tmr_;
  ros::Subscriber pose_sub_;
  geometry_msgs::Pose pose_msg_;
  
  void tmr_cb(const ros::TimerEvent& e)
  {
  
  }
  
  void pose_cb(const geometry_msgs::PoseConstPtr &msg)
  {
    pose_msg_ = *msg;
  }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stage_move");
	ros::NodeHandle n;
	
	StageMove s* = new StageMove(n, 0.05);
	ros::spin();
}
