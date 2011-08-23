#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include "tf/tf.h"
#include "odom_ekf/eif.h"

using namespace MatrixWrapper;

class EifNode
{
private:
  ros::NodeHandle n_;  
  ros::Subscriber imu_sub_, gps_sub_, encoder_sub_, vicon_sub_;
  ros::Publisher odom_pub_;
  ros::Timer pub_tmr_;
  sensor_msgs::Imu imu_msg_;
  
  double freq_, angular_vel_;
  Eif * filter_;
  
  void imuCallback(sensor_msgs::Imu msg);
  void gpsCallback(nav_msgs::Odometry msg);
  void encoderCallback(nav_msgs::Odometry msg);
  void viconCallback(geometry_msgs::PoseStamped msg);
  void tmrCallback(const ros::TimerEvent& event);
public:
  EifNode(ros::NodeHandle nh);
};

EifNode::EifNode(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  n_private.param("freq", freq_, 50.0);
  
  if (freq_ > 100)
  {
    ROS_WARN("Using maximum frequency of 100 Hz instead of %f.", freq_);
    freq_ = 100;
  }
  else if (freq_ < 10)
  {
    ROS_WARN("Using minimum frequency of 10 Hz instead of %f.", freq_);
    freq_ = 10;
  }
  
  filter_ = new Eif();
  
  imu_sub_ = n_.subscribe<sensor_msgs::Imu>("imu/data", 1, &EifNode::imuCallback, this);
  gps_sub_ = n_.subscribe<nav_msgs::Odometry>("odom_gps", 1, &EifNode::gpsCallback, this);
  encoder_sub_ = n_.subscribe<nav_msgs::Odometry>("odom_encoder", 1, &EifNode::encoderCallback, this);
  vicon_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("vicon_pose", 1, &EifNode::viconCallback, this);
  odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom_est", 1);
  pub_tmr_ = n_.createTimer(ros::Duration(1 / freq_), &EifNode::tmrCallback, this);
}

void EifNode::imuCallback(sensor_msgs::Imu msg)
{
  // Need to translate imu angular velocities and accelerations to base link
  
  // Store imu message
  imu_msg_ = msg;
  
  // Create the BFL ColumnVector Measurement
  Matrix cov(4, 4);
  cov = 0;
  cov(1, 1) = 0.010;//msg.linear_acceleration_covariance[0];
  cov(2, 2) = 0.010;//msg.linear_acceleration_covariance[0];
  cov(3, 3) = 0.020;//msg.linear_acceleration_covariance[0];
  cov(4, 4) = 0.1;//msg.linear_acceleration_covariance[4];
  
  // Predict EIF
  filter_->Predict(msg);//msg.linear_acceleration.x, angular_vel_, cov);
}

void EifNode::gpsCallback(nav_msgs::Odometry msg)
{
  // Translate gps pose to base link
  // Create the BFL ColumnVector Measurement
  // Update EIF
}

void EifNode::encoderCallback(nav_msgs::Odometry msg)
{
  // Translate encoder velocities to base link
  // Create the BFL ColumnVector Measurement
  // Update EIF
}

void EifNode::viconCallback(geometry_msgs::PoseStamped msg)
{
  // Translate vicon pose to base link
  // Create the BFL ColumnVector Measurement
  // Update EIF
}

void EifNode::tmrCallback(const ros::TimerEvent& event)
{
  // Get the state vector
  ColumnVector state_est = filter_->GetEstimate();
  
  // Create odom_est message
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = state_est(1);
  odom_msg.pose.pose.position.y = state_est(2);
  odom_msg.pose.pose.position.z = state_est(3);
  odom_msg.twist.twist.linear.x = state_est(4);
  odom_msg.twist.twist.linear.y = state_est(5);
  odom_msg.twist.twist.linear.z = state_est(6);
  odom_msg.twist.twist.angular.x = imu_msg_.angular_velocity.x;
  odom_msg.twist.twist.angular.y = imu_msg_.angular_velocity.y;
  odom_msg.twist.twist.angular.z = imu_msg_.angular_velocity.z;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(state_est(9), state_est(8), state_est(7));
  
  // Publish the estimated odometry
  // Angular velocity is directly from IMU? (Try to incorporate w into EIF)  
  odom_pub_.publish(odom_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_ekf");
	ros::NodeHandle n;

	EifNode* e = new EifNode(n);
	ros::spin();

	return 0;
}
