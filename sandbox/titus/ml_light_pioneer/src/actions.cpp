#include "ml_light_pioneer/actions.h"

Actions::Actions(ros::NodeHandle nh)
{
  n_ = nh;

  ros::NodeHandle n_private("~");
  n_private.param("num_actions", num_actions_, 3);
  n_private.param("ang_vel_lim", ang_vel_lim_, 1.0);

  vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  ang_vels_ = std::vector<double>(num_actions_);
  ang_inc_ = 2 * ang_vel_lim_ / (num_actions_ - 1);
  for (int i = 0; i < num_actions_; i++)
    ang_vels_[i] = -ang_vel_lim_ + i * ang_inc_;
}

void Actions::Move(int action)
{
  geometry_msgs::Twist vel_msg;

  if (action >= 0 && action < num_actions_)
  {
    vel_msg.linear.x = 0.3;
    vel_msg.angular.z = ang_vels_[action];
    vel_pub_.publish(vel_msg);
  }
}

int Actions::GetNumActions(void)
{
	return num_actions_;
}

/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "actions_test");
  ros::NodeHandle n;
  int i = 0;

  Actions* r = new Actions(n);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
    r->Move((Actions::moveType)i);
    i++;
    if (i > 5)
      i = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
*/
