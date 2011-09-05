#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include "LinearMath/btMatrix3x3.h"
#include "tf/tf.h"

using namespace MatrixWrapper;

class ImuEkf
{
private:
  ros::NodeHandle n_;  
  ros::Subscriber imu_sub_;
  ros::Publisher marker_pub_;
  
  ColumnVector G;
  
  Matrix getDCM(double yaw, double pitch, double roll);
public:
  ImuEkf(ros::NodeHandle nh);
  void imuCallback(sensor_msgs::Imu msg);
};

ImuEkf::ImuEkf(ros::NodeHandle nh):G(3)
{
  n_ = nh;
  G(1) = 0; G(2) = 0; G(3) = 9.80665;
  
  imu_sub_ = n_.subscribe<sensor_msgs::Imu>("imu/data", 1, &ImuEkf::imuCallback, this);
  marker_pub_ = n_.advertise<visualization_msgs::Marker>("imu", 1);
}

void ImuEkf::imuCallback(sensor_msgs::Imu msg)
{
	visualization_msgs::Marker marker;	
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = "/odom";
	marker.ns = "imu";
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

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = msg.orientation.x;
	marker.pose.orientation.y = msg.orientation.y;
	marker.pose.orientation.z = msg.orientation.z;
	marker.pose.orientation.w = msg.orientation.w;
	
	marker_pub_.publish(marker);
	
	double rx, ry, rz;
	btQuaternion q;
  tf::quaternionMsgToTF(msg.orientation, q);
	btMatrix3x3(q).getRPY(rx, ry, rz);
	ROS_INFO("Euler Angles: %f, %f, %f", rx, ry, rz);
	ColumnVector Ab(3);
	Ab(1) = msg.linear_acceleration.x;
	Ab(2) = msg.linear_acceleration.y;
	Ab(3) = msg.linear_acceleration.z;
	ColumnVector An(3);
	An = getDCM(rz, ry, rx) * Ab - G; 

  geometry_msgs::Point p1, p2;
  std::vector <geometry_msgs::Point> pts;
  p1.x = 0; p1.y = 0; p1.z = 0;
  p2.x = An(1); p2.y = An(2); p2.z = An(3);
  pts.push_back(p1);
  pts.push_back(p2);
		
  marker.id = 1;
	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.0;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
  marker.points = pts;
		
	marker_pub_.publish(marker);
}

Matrix ImuEkf::getDCM(double yaw, double pitch, double roll)
{
  Matrix C_n_b(3,3);
  C_n_b(1,1) = cos(pitch)*cos(yaw);
  C_n_b(1,2) = -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
  C_n_b(1,3) = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
  C_n_b(2,1) = cos(pitch)*sin(yaw);
  C_n_b(2,2) = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
  C_n_b(2,3) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
  C_n_b(3,1) = -sin(pitch);
  C_n_b(3,2) = sin(roll) * cos(pitch);
  C_n_b(3,3) = cos(roll)*cos(pitch);    
  
  return C_n_b;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_ekf");
	ros::NodeHandle n;

	ImuEkf* e = new ImuEkf(n);
	ros::spin();

	return 0;
}
