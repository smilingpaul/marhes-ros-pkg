#include "ros/ros.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

ros::Publisher pose_pub;

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

  geometry_msgs::PoseStamped pose_msg;
  
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = t_data -> t_ref_frame;
  pose_msg.pose.position.x = t.pos[0] / 1000;
  pose_msg.pose.position.y = t.pos[1] / 1000;
  pose_msg.pose.position.z = t.pos[2] / 1000;
  pose_msg.pose.orientation.x = t.quat[0];
  pose_msg.pose.orientation.y = t.quat[1];
  pose_msg.pose.orientation.z = t.quat[2];
  pose_msg.pose.orientation.w = t.quat[3];
  
  pose_pub.publish(pose_msg); 
}

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_vrpn_client");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  // Variable declaration
  string host_addr, tf_ref_frame, subject, address;
  ros::Rate loop_rate(50);
  vrpn_Tracker_Remote *tkr;
  
  // Get parameters
  n_private.param("host_addr", host_addr, string("Vicon"));
  n_private.param("tf_ref_frame", tf_ref_frame, string("/vicon_world"));     
  
  // Check and get the subject list
  if (!n_private.hasParam("subject"))
    ROS_FATAL("Error: subject_list was not provided");

  n_private.getParam("subject", subject);
  address = subject + "@" + host_addr;
    
  // Name the device and open it as a tracker
  tkr = new vrpn_Tracker_Remote(address.c_str());
  
  t_user_callback *tc = new t_user_callback;

  // Fill in the user-data callback information
  tc->t_name = address;
  tc->t_ref_frame = tf_ref_frame;
  tc->t_subject_frame = subject;

  // Set up the tracker callback handlers
  tkr->register_change_handler(tc, handle_tracker_pos_quat);

  ROS_INFO("Tracking subject at: %s", address.c_str());
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);

/* 
 * main interactive loop
 */
  while ( ros::ok() ) {

    // Let all the devices do their things
    tkr->mainloop();

    ros::spinOnce();
    loop_rate.sleep();
  }

 return 0;
}   /* main */
