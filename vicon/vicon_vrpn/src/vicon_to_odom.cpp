#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include "nav_msgs/Odometry.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class t_user_callback {
  public:
    string t_name, t_ref_frame, t_subject_frame;
};


/*****************************************************************************
 *
   Callback handlers
 *
 *****************************************************************************/

void	VRPN_CALLBACK handle_tracker_pos_quat (void *userdata, const vrpn_TRACKERCB t)
{
	t_user_callback	*t_data = (t_user_callback *)userdata;

  nav_msgs::Odometry odom_msg;
  
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = t_data -> t_ref_frame;
  odom_msg.pose.pose.position.x = t.pos[0] / 1000;
  odom_msg.pose.pose.position.y = t.pos[1] / 1000;
  odom_msg.pose.pose.position.z = t.pos[2] / 1000;
  odom_msg.pose.pose.orientation.x = t.quat[0];
  odom_msg.pose.pose.orientation.y = t.quat[1];
  odom_msg.pose.pose.orientation.z = t.quat[2];
  odom_msg.pose.pose.orientation.w = t.quat[3];
      
//  transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
//  transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
//  tf::StampedTransform stampTransform(transform, time_now, 
//        t_data->t_ref_frame, t_data->t_subject_frame + "/vicon");
//  tf_broadcaster.sendTransform(stampTransform);
}

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_vrpn_client");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  // Variable declaration
  string host_addr, tf_ref_frame, subject, address;
  ros::Rate loop_rate(100);
  
  // Get parameters
  n_private.param("host_addr", host_addr, string("Vicon"));
  n_private.param("tf_ref_frame", tf_ref_frame, string("/vicon_world"));     
  
  // Check and get the subject list
  if (!n_private.hasParam("subject"))
    ROS_FATAL("Error: subject_list was not provided");

  n_private.getParam("subject", subject);
  address = subject + "@" + host_addr;
    
  // Name the device and open it as a tracker
  vrpn_Tracker_Remote *tkr = new vrpn_Tracker_Remote(address.c_str());
  
  t_user_callback *tc = new t_user_callback;

  // Fill in the user-data callback information
  tc->t_name = address;
  tc->t_ref_frame = tf_ref_frame;
  tc->t_subject_frame = subject;

  // Set up the tracker callback handlers
  tkr->register_change_handler(tc, handle_tracker_pos_quat);

  ROS_INFO("Tracking subject at: %s", ((string)addresses[i]).c_str());

/* 
 * main interactive loop
 */
  while ( ros::ok() ) {

    // Let all the devices do their things
    for (int i = 0; i < num_subjects; i++) {
      tkr->mainloop();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

 return 0;
}   /* main */
