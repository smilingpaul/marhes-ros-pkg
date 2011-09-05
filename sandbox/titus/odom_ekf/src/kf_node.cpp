#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include "tf/tf.h"
#include "odom_ekf/kf.h"
#include "odom_ekf/ins.h"

using namespace MatrixWrapper;

class KfNode
{
private:
  ros::NodeHandle n_;  
  ros::Subscriber imu_sub_, gps_sub_, encoder_sub_, vicon_sub_;
  ros::Publisher odom_pub_;
  ros::Timer pub_tmr_;
  sensor_msgs::Imu imu_msg_;
  nav_msgs::Odometry first_gps_msg_;
  
  double freq_, angular_vel_;
  bool first_gps_rx_, ins_initialized_;
  Kf * filter_;
  Ins * ins_;
       
  void imuCallback(sensor_msgs::Imu msg);
  void gpsCallback(nav_msgs::Odometry msg);
  void encoderCallback(nav_msgs::Odometry msg);
  void viconCallback(geometry_msgs::PoseStamped msg);
  void tmrCallback(const ros::TimerEvent& event);
public:
  KfNode(ros::NodeHandle nh);
};

KfNode::KfNode(ros::NodeHandle nh)
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
  
  filter_ = new Kf();
  ins_ = new Ins();
  
  first_gps_rx_ = true;
  ins_initialized_ = false;
  
  imu_sub_ = n_.subscribe<sensor_msgs::Imu>("imu/data", 1, &KfNode::imuCallback, this);
  gps_sub_ = n_.subscribe<nav_msgs::Odometry>("odom_gps", 1, &KfNode::gpsCallback, this);
  encoder_sub_ = n_.subscribe<nav_msgs::Odometry>("odom_encoder", 1, &KfNode::encoderCallback, this);
  vicon_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("vicon_pose", 1, &KfNode::viconCallback, this);
  odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom_est", 1);
  pub_tmr_ = n_.createTimer(ros::Duration(1 / freq_), &KfNode::tmrCallback, this);
}

void KfNode::imuCallback(sensor_msgs::Imu msg)
{
  // Store imu message
  imu_msg_ = msg;
  imu_msg_.angular_velocity.y = -imu_msg_.angular_velocity.y;
  imu_msg_.angular_velocity.z = -imu_msg_.angular_velocity.z;
  imu_msg_.linear_acceleration.y = -imu_msg_.linear_acceleration.y;
  imu_msg_.linear_acceleration.z = -imu_msg_.linear_acceleration.z;
  
  btQuaternion q;
  double rx, ry, rz;
  tf::quaternionMsgToTF(msg.orientation, q);
	btMatrix3x3(q).getRPY(rx, ry, rz);
	rx += M_PI;
	imu_msg_.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);
  
  /*
  ROS_INFO("%f, %f, %f, %f, %f, %f", imu_msg_.angular_velocity.x,
                                     imu_msg_.angular_velocity.y,
                                     imu_msg_.angular_velocity.z,
                                     imu_msg_.linear_acceleration.x,
                                     imu_msg_.linear_acceleration.y,
                                     imu_msg_.linear_acceleration.z);
  */
  
  if(ins_initialized_)
  {
    // Update INS
    ins_->Integrate(imu_msg_);
    
    // Predict KF
    filter_->Predict(ins_->GetLastFn(), ins_->GetCnb(), ins_->GetDt());
  }
  else
  {
    ins_initialized_ = ins_->Initialize(imu_msg_);
  }
}

void KfNode::gpsCallback(nav_msgs::Odometry msg)
{
  if (first_gps_rx_)
  {
    first_gps_msg_ = msg;
    first_gps_rx_ = false;
  }
  else
  {
    // Translate gps pose to base link
    // Create the BFL ColumnVector Measurement
    ColumnVector meas(3);
    meas(1) = msg.pose.pose.position.x - first_gps_msg_.pose.pose.position.x;
    meas(2) = msg.pose.pose.position.y - first_gps_msg_.pose.pose.position.y;
    meas(3) = msg.pose.pose.position.z - first_gps_msg_.pose.pose.position.z;
    
    Matrix Rgps(3, 3);
    Rgps = 0;
    Rgps(1, 1) = 0.01; Rgps(2, 2) = 0.01; Rgps(3, 3) = 0.01;
       
    // Update KIF
    filter_->UpdateGPS(ins_->GetState().sub(1, 3), meas, Rgps);
    //ins_->Correct(filter_->GetState());
  }
}

void KfNode::encoderCallback(nav_msgs::Odometry msg)
{
  // Translate encoder velocities to base link
  // Create the BFL ColumnVector Measurement
  // Update EIF
}

void KfNode::viconCallback(geometry_msgs::PoseStamped msg)
{
  // Translate vicon pose to base link
  // Create the BFL ColumnVector Measurement
  // Update EIF
}

void KfNode::tmrCallback(const ros::TimerEvent& event)
{
  // Get the state vector
  ColumnVector state_est = ins_->GetState();
  
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
  
  ColumnVector errors = filter_->GetState();
  //ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f", errors(1), errors(2), errors(3), errors(4), errors(5), errors(6), errors(7), errors(8), errors(9));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eif");
	ros::NodeHandle n;

	KfNode* e = new KfNode(n);
	ros::spin();

	return 0;
}
