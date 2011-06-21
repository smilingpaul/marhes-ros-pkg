#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Publisher velPub;

void joyCB(const joy::Joy::ConstPtr& joy) {
	geometry_msgs::Twist vel;
	vel.linear.x = (double)1.6 * joy->axes[0];
	vel.angular.z = (double)0.5 * joy->axes[1];
	velPub.publish(vel);
	ROS_INFO("Linear: %f; Angular: %f", vel.linear.x, vel.angular.z);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "teleop_robot");
	ros::NodeHandle n;
	velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber joySub = n.subscribe<joy::Joy>("joy", 10, joyCB);
	ros::spin();
	return 0;
}
