#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <math.h>

using namespace std;

class TFToOdom
{
public:
  TFToOdom(void)
  {
	  ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    n_private.param("odom_frame", odom_frame_, string("/odom"));
    n_private.param("odom_topic", odom_topic_, string("/vo"));
    n_private.param("odom_filter_topic", odom_filter_topic_, string("/vo_filter"));
    n_private.param("freq", freq_, 50.0);
//    n_private.param("bool_tf", bool_tf_, true);

    if (!n_private.hasParam("frame"))
      ROS_FATAL("Error: Parameter \"frame\" was not provided");
    else
      n_private.getParam("frame", frame_);

    ROS_INFO(frame_.c_str());

    odom_pub_ = n.advertise<nav_msgs::Odometry>(odom_topic_.c_str(), 50);
    odom_filter_pub_ = n.advertise<nav_msgs::Odometry>(odom_filter_topic_.c_str(), 50); 
    timer_ = n.createTimer(ros::Duration(1/freq_), &TFToOdom::UpdateOdom, this);    
  }

  void UpdateOdom(const ros::TimerEvent& event)
  {
    tf::StampedTransform transform;
    double vx, vtheta, dt;

    try
    {
      listener_.lookupTransform(odom_frame_, frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    nav_msgs::Odometry odom_msg;
    //next, we'll publish the odometry message over ROS
    odom_msg.header.stamp = transform.stamp_;
    odom_msg.header.frame_id = transform.frame_id_;
    odom_msg.child_frame_id = transform.child_frame_id_;

    //set the position
    odom_msg.pose.pose.position.x = transform.getOrigin().x();
    odom_msg.pose.pose.position.y = transform.getOrigin().y();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = transform.getRotation().x();
    odom_msg.pose.pose.orientation.y = transform.getRotation().y();
    odom_msg.pose.pose.orientation.z = transform.getRotation().z();
    odom_msg.pose.pose.orientation.w = transform.getRotation().w();

    odom_msg.pose.covariance[0] = 4.5;
    odom_msg.pose.covariance[7] = 4.5;
    odom_msg.pose.covariance[14] = 99999;
    odom_msg.pose.covariance[21] = 99999;
    odom_msg.pose.covariance[28] = 99999;
    odom_msg.pose.covariance[35] = 0.75;

    dt = (transform.stamp_ - last_transform_.stamp_).toSec();
    vx = sqrt(pow(transform.getOrigin().x() - last_transform_.getOrigin().x(), 2) + 
      pow(transform.getOrigin().y() - last_transform_.getOrigin().y(), 2)) / dt;
    vtheta = (tf::getYaw(transform.getRotation()) - tf::getYaw(last_transform_.getRotation())) / dt;

    //set the velocity
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = vtheta;

    //publish the message
    odom_pub_.publish(odom_msg);

    last_transform_ = transform;    
  }
private:
  ros::NodeHandle n_;
  ros::Publisher odom_pub_, odom_filter_pub_;
  ros::Timer timer_;
  tf::TransformListener listener_;
  tf::StampedTransform last_transform_;
  string frame_, odom_frame_, odom_topic_, odom_filter_topic_;
  double freq_;
//  bool bool_tf_;
};

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_tf_to_odom");

  TFToOdom* translater = new TFToOdom();

  ros::spin();
}
