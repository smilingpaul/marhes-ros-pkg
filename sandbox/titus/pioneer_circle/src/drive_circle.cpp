#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "p2os_driver/MotorState.h"
#include "nav_msgs/Odometry.h"

void odom_cb(nav_msgs::Odometry odom_msg)
{
  ROS_INFO("PosX: %f, PosY: %f, LinVel: %f, AngVel: %f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
    odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "pioneer_circle");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  std::string odom_topic, vel_topic, motor_state_topic;

  n_private.param("vel_topic", vel_topic, std::string("cmd_vel"));
  n_private.param("odom_topic", odom_topic, std::string("pose"));
  n_private.param("motor_state_topic", motor_state_topic, std::string("cmd_motor_state"));

	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(vel_topic.c_str(),1);
	ros::Publisher mot_pub = n.advertise<p2os_driver::MotorState>(motor_state_topic.c_str(),1);
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.c_str(), 10, odom_cb);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    p2os_driver::MotorState mot_state_msg;
    mot_state_msg.state = true;
    mot_pub.publish(mot_state_msg);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 1.0;
    vel_pub.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}
