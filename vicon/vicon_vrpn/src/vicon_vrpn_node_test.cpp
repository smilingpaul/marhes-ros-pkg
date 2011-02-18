#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include <stdlib.h>
#include <stdio.h>

using namespace std;

//-------------------------------------
// This section contains the data structure that holds information on
// the devices that are created.  For each named device, a remote of 
// type tracker is created.

class device_info {
  public:
	  string name;
	  vrpn_Tracker_Remote *tkr;
};

const int MAX_DEVICES = 50;

//-------------------------------------
// This section contains the data structure that is used to determine how
// often to print a report for each sensor of each tracker.  Each element
// contains a counter that is used by the callback routine to keep track
// of how many it has skipped.  There is an element for each possible sensor.
// A new array of elements is created for each new tracker object, and a
// pointer to it is passed as the userdata pointer to the callback handlers.
// A separate array is kept for the position, velocity, and acceleration for each
// tracker.  The structure passed to the callback handler also has a
// string that is the name of the tracker.

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
  static tf::TransformBroadcaster tf_broadcaster;
	t_user_callback	*t_data = (t_user_callback *)userdata;

  ros::Time time_now = ros::Time::now();
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
  transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
  tf::StampedTransform stampTransform(transform, time_now, 
        t_data->t_ref_frame, t_data->t_subject_frame);
  tf_broadcaster.sendTransform(stampTransform);
}

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_vrpn_client");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  // Variable declaration
  string host_addr, tf_ref_frame, subjects[MAX_DEVICES], addresses[MAX_DEVICES];
  device_info device_list[MAX_DEVICES];
  int num_subjects;
  ros::Rate loop_rate(300);
  
  // Get parameters
  n_private.param("host_addr", host_addr, string("Vicon"));
  n_private.param("tf_ref_frame", tf_ref_frame, string("/vicon_world"));     

  // Check and get the subject list
  if (!n_private.hasParam("subject_list"))
    ROS_FATAL("Error: subject_list was not provided");

  XmlRpc::XmlRpcValue subject_list;
  n_private.getParam("subject_list", subject_list);
  ROS_ASSERT(subject_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  num_subjects = subject_list.size();     // Get size of list

  // If list is greater than MAX then quit
  if (num_subjects > MAX_DEVICES)
    ROS_FATAL("Tracking limit is %d subjects.", MAX_DEVICES);

  // Setup each subject tracker
  for (int i = 0; i < num_subjects; ++i) 
  {
    // Get the address
    ROS_ASSERT(subject_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    subjects[i] = (string)subject_list[i];       
    addresses[i] = subjects[i] + "@" + host_addr;
    
	  device_info *dev;                     // Create the device

	  // Name the device and open it as a tracker
	  dev = &device_list[i];
	  dev->name = addresses[i];
	  dev->tkr = new vrpn_Tracker_Remote(dev->name.c_str());

	  // If we are printing the tracker reports, prepare the
	  // user-data callbacks and hook them up to be called with
	  // the correct data for this device.
    vrpn_Tracker_Remote *tkr = dev->tkr;
    t_user_callback *tc = new t_user_callback;

    // Fill in the user-data callback information
    tc->t_name = dev->name;
    tc->t_ref_frame = tf_ref_frame;
    tc->t_subject_frame = subjects[i];

    // Set up the tracker callback handlers
    tkr->register_change_handler(tc, handle_tracker_pos_quat);

    ROS_INFO("Tracking subject at: %s", ((string)addresses[i]).c_str());
  }

/* 
 * main interactive loop
 */
  while ( ros::ok() ) {

    // Let all the devices do their things
    for (int i = 0; i < num_subjects; i++) {
      device_list[i].tkr->mainloop();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

 return 0;
}   /* main */
