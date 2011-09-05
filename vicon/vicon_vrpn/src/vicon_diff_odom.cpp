#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include <cmath>

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
    n_private.param("freq", freq_, 100.0);
//    n_private.param("bool_tf", bool_tf_, true);

    if (!n_private.hasParam("frame"))
      ROS_FATAL("Error: Parameter \"frame\" was not provided");
    else
      n_private.getParam("frame", frame_);

    ROS_INFO("%s", frame_.c_str());
    dt_ = 1 / freq_;
    last_theta_ = 0.0;
    
    v1 = v2 = v3 = v4 = w1 = w2 = w3 = w4 = 0.0;

    odom_pub_ = n.advertise<nav_msgs::Odometry>(odom_topic_.c_str(), 50);
    odom_filter_pub_ = n.advertise<nav_msgs::Odometry>(odom_filter_topic_.c_str(), 50); 
    timer_ = n.createTimer(ros::Duration(1/freq_), &TFToOdom::UpdateOdom, this);
  }

  void UpdateOdom(const ros::TimerEvent& event)
  {
    tf::StampedTransform transform;
    double d_theta;
    bool test = false;

    try
    {
      listener_.lookupTransform(odom_frame_, frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    dt_ = transform.stamp_.toSec() - last_transform_.stamp_.toSec();

    if (dt_ > 0)
    {
	    nav_msgs::Odometry odom_msg;
	    //next, we'll publish the odometry message over ROS
	    odom_msg.header.stamp = transform.stamp_;
	    odom_msg.header.frame_id = transform.frame_id_;
	    odom_msg.child_frame_id = transform.child_frame_id_;

	    //set the position
	    odom_msg.pose.pose.position.x = transform.getOrigin().getX();
	    odom_msg.pose.pose.position.y = transform.getOrigin().getY();
	    odom_msg.pose.pose.position.z = transform.getOrigin().getZ();
	    odom_msg.pose.pose.orientation.x = transform.getRotation().getX();
 	    odom_msg.pose.pose.orientation.y = transform.getRotation().getY();
 	    odom_msg.pose.pose.orientation.z = transform.getRotation().getZ();
 	    odom_msg.pose.pose.orientation.w = transform.getRotation().getW();

      // Figure out theta
      d_theta = tf::getYaw(transform.getRotation()) - tf::getYaw(last_transform_.getRotation());
	    
		  //set the velocity
		  odom_msg.twist.twist.linear.x = sqrt(
				  pow(transform.getOrigin().getX() - last_transform_.getOrigin().getX(), 2) +
				  pow(transform.getOrigin().getY() - last_transform_.getOrigin().getY(), 2)) / dt_;
		  odom_msg.twist.twist.linear.y = 0.0;
		  odom_msg.twist.twist.linear.z = 0.0;
		  odom_msg.twist.twist.angular.z = (tf::getYaw(transform.getRotation()) - 
		      tf::getYaw(last_transform_.getRotation())) / dt_;
		      
 		  odom_msg.twist.twist.linear.x = (odom_msg.twist.twist.linear.x + v1 + v2 + v3 + v4) / 5;
		  odom_msg.twist.twist.angular.z = (odom_msg.twist.twist.angular.z + w1 + w2 + w3 + w4) / 5;

		  //publish the message
		  odom_pub_.publish(odom_msg);
		  last_transform_ = transform;
		  
		  v4 = v3; v3 = v2; v2 = v1; v1 = odom_msg.twist.twist.linear.x;
      w4 = w3; w3 = w2; w2 = w1; w1 = odom_msg.twist.twist.angular.z;
    }
  }
private:
  ros::NodeHandle n_;
  ros::Publisher odom_pub_, odom_filter_pub_;
  ros::Timer timer_;
  tf::TransformListener listener_;
  tf::StampedTransform last_transform_;
  string frame_, odom_frame_, odom_topic_, odom_filter_topic_;
  float dt_;
  double freq_, last_theta_;
//  bool bool_tf_;
  CvKalman* kalman_;
  CvMat* z_;
  double v1, v2, v3, v4, w1, w2, w3, w4;
};

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_diff_odom");

  TFToOdom* translater = new TFToOdom();

  ros::spin();

  return 0;
}
