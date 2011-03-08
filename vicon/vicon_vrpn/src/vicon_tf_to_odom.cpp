#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

using namespace std;

class TFToOdom
{
public:
  TFToOdom(ros::NodeHandle n, string frame, string odom_frame, string odom_topic):
    n_(n), frame_(frame), odom_frame_(odom_frame), odom_topic_(odom_topic)
  {
    odom_pub_ = n_.advertise<nav_msgs::Odometry>(odom_topic_.c_str(), 50); 
  }

  void UpdateOdom(void)
  {
    tf::StampedTransform transform;

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

    //set the velocity
//    odom_msg.twist.twist.linear.x = vx;
//    odom_msg.twist.twist.linear.y = 0.0;
//    odom_msg.twist.twist.linear.z = 0.0;
//    odom_msg.twist.twist.angular.z = vtheta;

    //publish the message
    odom_pub_.publish(odom_msg);

    last_transform_ = transform;    
  }
private:
  ros::NodeHandle n_;
  ros::Publisher odom_pub_;
  tf::TransformListener listener_;
  tf::StampedTransform last_transform_;
  string frame_, odom_frame_, odom_topic_;
};

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_tf_to_odom");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  string frame, odom_frame, odom_topic;
  double freq;

  n_private.param("frame", frame, string("/Pioneer01"));
  n_private.param("odom_frame", odom_frame, string("/odom"));
  n_private.param("odom_topic", odom_topic, string("/vo"));
  n_private.param("freq", freq, 200.0);

  TFToOdom* translater = new TFToOdom(n, frame, odom_frame, odom_topic);
  ros::Rate loop_rate(freq);

  while(ros::ok())
  {
    translater->UpdateOdom();

    ros::spinOnce();
    loop_rate.sleep();
  }  
}
