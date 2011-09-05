#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
		
//void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
visualization_msgs::Marker updateMarker;
nav_msgs::Odometry odom;

void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
	updateMarker.pose.position.x = 0;
	updateMarker.pose.position.y = 0;
	updateMarker.pose.position.z = 0;
	updateMarker.pose.orientation.x = msg->orientation.x;
	updateMarker.pose.orientation.y = msg->orientation.y;
	updateMarker.pose.orientation.z = msg->orientation.z;
	updateMarker.pose.orientation.w = msg->orientation.w;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imuDisp");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imuCB);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("vizualization_marker", 1);

	ROS_INFO("Enter While Loop\n\r");

  	while (ros::ok())
  	{
		visualization_msgs::Marker marker;		

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = "/my_frame";
		marker.ns = "imuDisp";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.pose.position.x = updateMarker.pose.position.x;
		marker.pose.position.y = updateMarker.pose.position.y;
		marker.pose.position.z = updateMarker.pose.position.z;
		marker.pose.orientation.x = updateMarker.pose.orientation.x;
		marker.pose.orientation.y = updateMarker.pose.orientation.y;
		marker.pose.orientation.z = updateMarker.pose.orientation.z;
		marker.pose.orientation.w = updateMarker.pose.orientation.w;

		marker.lifetime = ros::Duration();
		marker_pub.publish(marker);

		ros::spinOnce();
		loop_rate.sleep();
  	}

  	return 0;
}
