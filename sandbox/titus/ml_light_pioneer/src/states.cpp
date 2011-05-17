#include "ml_light_pioneer/states.h"
               
States::States(ros::NodeHandle nh, unsigned int num_states, unsigned int xdist, unsigned int ydist)
{
  n_ = nh;

  sub_odom_ = n_.subscribe("odom", 1, &States::cb_odom, this);
  sub_flls_ = n_.subscribe("flls", 1, &States::cb_flls, this);
  sub_frls_ = n_.subscribe("frls", 1, &States::cb_frls, this);
  sub_rlls_ = n_.subscribe("rlls", 1, &States::cb_rlls, this);
  sub_rrls_ = n_.subscribe("rrls", 1, &States::cb_rrls, this);
  tmr_state_ = n_.createTimer(ros::Duration(0.1), &States::cbtmr_state, this);

  hyp_ = sqrt(xdist * xdist + ydist * ydist);
  cos_ang_ = xdist / hyp_;
  sin_ang_ = ydist / hyp_;
  light_dir_ = 0;

  num_states_ = num_states;
  state_ = 0;
}

int States::GetState(void)
{
  return state_; 
}

double States::GetReward(void)
{
	double reward = 1;
	return reward;
}

double States::GetDistance(void)
{
	return 0;
}

int States::GetNumStates(void)
{
	return num_states_;
}

void States::cbtmr_state(const ros::TimerEvent& event)
{
  double x, y;
  
  x = cos_ang_ * (ls_vals_[FLLS] + ls_vals_[FRLS] - ls_vals_[RLLS] - ls_vals_[RRLS]);
  y = sin_ang_ * (ls_vals_[FLLS] - ls_vals_[FRLS] + ls_vals_[RLLS] - ls_vals_[RRLS]);

  light_dir_ = atan2(y, x);
  if A.dir <= 3 * pi / 8 && A.dir > 1 * pi / 8
    A.state = 1;
  elseif A.dir <= 1 * pi / 8 && A.dir > -1 * pi / 8
    A.state = 2;
  elseif A.dir <= -1 * pi / 8 && A.dir > -3 * pi / 8
    A.state = 3;
  elseif A.dir <= 5 * pi / 8 && A.dir > 3 * pi / 8
    A.state = 4;
  elseif A.dir <= -3 * pi / 8 && A.dir > -5 * pi / 8
    A.state = 5;
  elseif A.dir <= 7 * pi / 8 && A.dir > 5 * pi / 8
    A.state = 6;
  elseif A.dir <= -5 * pi / 8 && A.dir > -7 * pi / 8
    A.state = 8;
  elseif A.dir <= -7 * pi / 8 || A.dir > 7 * pi / 8
    A.state = 7;
  end
}
  
void States::cb_odom(nav_msgs::Odometry msg)
{
  ROS_INFO("Data odom: ");
}

void States::cb_flls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[FLLS] = msg.data;
}

void States::cb_frls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[FRLS] = msg.data;
}

void States::cb_rlls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[RLLS] = msg.data;
}

void States::cb_rrls(phidgets_ros::Float64Stamped msg)
{
  ls_vals_[RRLS] = msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "states_test");
	ros::NodeHandle n;

  States* s = new States(n, 8);
  ros::spin();

  return 0;
}
