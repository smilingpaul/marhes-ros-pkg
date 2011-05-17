#include "ml_light_pioneer/states.h"
               
States::States(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");

  n_private.param("num_states", num_states_, 8);
  n_private.param("xdist", xdist_, 8 / 0.0254);
  n_private.param("ydist", ydist_, 6 / 0.0254);

  sub_odom_ = n_.subscribe("odom", 1, &States::cb_odom, this);
  sub_flls_ = n_.subscribe("flls", 1, &States::cb_flls, this);
  sub_frls_ = n_.subscribe("frls", 1, &States::cb_frls, this);
  sub_rlls_ = n_.subscribe("rlls", 1, &States::cb_rlls, this);
  sub_rrls_ = n_.subscribe("rrls", 1, &States::cb_rrls, this);
  tmr_state_ = n_.createTimer(ros::Duration(0.1), &States::cb_tmr_state, this);

  hyp_ = sqrt(xdist_ * xdist_ + ydist_ * ydist_);
  cos_ang_ = xdist_ / hyp_;
  sin_ang_ = ydist_ / hyp_;
  light_dir_ = 0;
  light_dir_last_ = 0;

  ang_inc_ = 2 * M_PI / num_states_;
  ang_start_ = ang_inc_ - M_PI / num_states_ - M_PI;

  state_ = 0;
}

int States::GetState(void)
{
  return state_; 
}

double States::GetReward(void)
{
	double reward = light_dir_last_ - light_dir_;
  light_dir_last_ = light_dir_;
	return reward;
}

int States::GetNumStates(void)
{
	return num_states_;
}

void States::cb_tmr_state(const ros::TimerEvent& event)
{
  double x, y;
  int i;
  
  x = cos_ang_ * (ls_vals_[FLLS] + ls_vals_[FRLS] - ls_vals_[RLLS] - ls_vals_[RRLS]);
  y = sin_ang_ * (ls_vals_[FLLS] - ls_vals_[FRLS] + ls_vals_[RLLS] - ls_vals_[RRLS]);

  light_dir_ = atan2(y, x);

  for (i = 0; i < num_states_; i++)
  {
    if (light_dir_ < ang_start_ + i * ang_inc_)
      break;
  }
  
  if (i == num_states_)
    state_ = 0;
  else
    state_ = i;

  ROS_INFO("Direction: %f, State: %d", light_dir_, state_);
}
  
void States::cb_odom(nav_msgs::Odometry msg)
{
  ROS_INFO("Data odom: ");
}

void States::cb_flls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[FLLS] = msg.data;
  //ROS_INFO("FLLS: %f", ls_vals_[FLLS]);
}

void States::cb_frls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[FRLS] = msg.data;
  //ROS_INFO("FRLS: %f", ls_vals_[FRLS]);
}

void States::cb_rlls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[RLLS] = msg.data;
  //ROS_INFO("RLLS: %f", ls_vals_[RLLS]);
}

void States::cb_rrls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[RRLS] = msg.data;
  //ROS_INFO("RRLS: %f", ls_vals_[RRLS]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "states_test");
	ros::NodeHandle n;

  States* s = new States(n);
  ros::spin();

  return 0;
}
