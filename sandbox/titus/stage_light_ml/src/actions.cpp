#include "stage_light_ml/actions.h"

Actions::Actions(ros::NodeHandle nh)
{
  n_ = nh;
  vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  num_actions_ = 6;
}
  
void Actions::Move(moveType type)
{
  geometry_msgs::Twist vel_msg;

  switch(type)
  {
    case LeftForward:
      vel_msg.linear.x = 0.5;
      vel_msg.angular.z = 0.5;
      break;
    case StraightForward:
      vel_msg.linear.x = 0.5;
      vel_msg.angular.z = 0;
      break;
    case RightForward:
      vel_msg.linear.x = 0.5;
      vel_msg.angular.z = -0.5;
      break;
    case LeftBackward:
      vel_msg.linear.x = -0.5;
      vel_msg.angular.z = 0.5;
      break;
    case StraightBackward:
      vel_msg.linear.x = -0.5;
      vel_msg.angular.z = 0;
      break;
    case RightBackward:
      vel_msg.linear.x = -0.5;
      vel_msg.angular.z = -0.5;
      break;
    default:
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0;
      break;
  }
  
  vel_pub_.publish(vel_msg);
}

int Actions::GetNumActions(void)
{
	return num_actions_;
}

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "actions_test");
//  ros::NodeHandle n;
//  int i = 0;
//
//  Actions* r = new Actions(n);
//  ros::Rate loop_rate(1);
//
//  while(ros::ok())
//  {
//    r->Move((Actions::moveType)i);
//    i++;
//    if (i > 5)
//      i = 0;
//
//    ros::spinOnce();
//    loop_rate.sleep();
//  }
//
//  return 0;
//}
	
