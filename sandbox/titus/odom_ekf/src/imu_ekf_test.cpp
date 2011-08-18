#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"

class ImuEkf
{
private:
  ros::NodeHandle n_;
  
  ros::Subscriber imu_sub_;
  
public:
  ImuEkf(ros::NodeHandle nh);
  void imuCallback(sensor_msgs::Imu msg);
};

ImuEkf::ImuEkf(ros::NodeHandle nh)
{
  n_ = nh;
  
  imu_sub_ = n_.subscribe<sensor_msgs::Imu>("imu/data", 1, &ImuEkf::imuCallback, this);
}

void ImuEkf::imuCallback(sensor_msgs::Imu msg)
{
  ROS_INFO("lin acc: %f, %f, %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_ekf");
	ros::NodeHandle n;

	ImuEkf* e = new ImuEkf(n);
	ros::spin();

	return 0;
}
