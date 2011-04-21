#include "stage_light_ml/states_light.h"
               
States::States(ros::NodeHandle nh)
{
  n_ = nh;
  odom_sub_ = n_.subscribe("/odom", 10, &States::odom_cb, this);
  num_states_ = 6;
  prev_light_ = 0.0;
  current_light_ = 0.0;
  current_distance_ = 20;
  state_ = States::StraightForward;
}

States::states States::GetState(void)
{
  return state_; 
}

double States::GetReward(void)
{
	double reward;
	reward = current_light_ - prev_light_;
	prev_light_ = current_light_;

	return reward;
}

double States::GetDistance(void)
{
	return sqrt(pow(-10.0 - odom_msg_.pose.pose.position.x, 2) + pow(10.0 - odom_msg_.pose.pose.position.y, 2));//current_distance_;
}

int States::GetNumStates(void)
{
	return num_states_;
}
  
void States::odom_cb(const nav_msgs::Odometry msg)
{
  odom_msg_ = msg;
  float x = odom_msg_.pose.pose.position.x;
  float y = odom_msg_.pose.pose.position.y;
  float t = tf::getYaw(odom_msg_.pose.pose.orientation);
  current_distance_ = sqrt(pow(-10.0 - x, 2) + pow(10.0 - y, 2));
  current_light_ = 100*(1 - pow(current_distance_ / 14.0, 1));
  float theta = atan2(10 - y, -10 - x); // returns pi -> -pi
  float bearing = theta - t;
  
  if (bearing < -M_PI)
	  bearing += 2 * M_PI;
  if (bearing > M_PI)
	  bearing -= 2 * M_PI;

//  ROS_INFO("Current Light: %f, Current Distance: %f, Current Bearing:%f", current_light_, current_distance_, bearing);

  if (bearing <= M_PI / 2 && bearing > M_PI / 6)
  	  state_ = States::LeftForward;
  else if (bearing <= M_PI / 6 && bearing > -M_PI / 6)
  	  state_ = States::StraightForward;
  else if (bearing <= -M_PI / 6 && bearing > -M_PI / 2)
  	  state_ = States::RightForward;
  else if (bearing <= -M_PI / 2 && bearing > -5 * M_PI / 6)
   	  state_ = States::RightBackward;
  else if (bearing <= -5 * M_PI / 6 || bearing > 5 * M_PI / 6)
   	  state_ = States::StraightBackward;
  else if (bearing <= 5 * M_PI / 6 && bearing > M_PI / 2)
   	  state_ = States::LeftBackward;
  else
	  ROS_INFO("No State.");

//  ROS_INFO("State: %d", (int)state_);
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
