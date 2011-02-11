/*			vrpn_print_devices.C

	This is a VRPN client program that will connect to one or more VRPN
	server devices and print out descriptions of any of a number of
	message types.  These include at least tracker, button, dial and analog
	messages.  It is useful for testing whether a server is up and running,
	and perhaps for simple debugging.
*/

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <stdlib.h>
#include <stdio.h>
#ifndef	_WIN32_WCE
#include <signal.h>
#endif
#include <string.h>

#ifndef _WIN32
#ifndef	_WIN32_WCE
#include <strings.h>
#endif
#endif

#include "vicon_vrpn/vrpn_Shared.h"
#include "vicon_vrpn/vrpn_Tracker.h"
#include <vector>
using namespace std;

//-------------------------------------
// This section contains the data structure that holds information on
// the devices that are created.  For each named device, a remote of each
// type (tracker, button, analog, dial, text) is created.  A particular device
// may only report a subset of these values, but that doesn't hurt anything --
// there simply will be no messages of the other types to receive and so
// nothing is printed.

class device_info {
  public:
	  const char		    *name;
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
	  char t_name[vrpn_MAX_TEXT_LEN];
    const char *t_ref_frame, *t_subject_frame;
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

  // Make sure we have a count value for this sensor
//  while (t_data->t_counts.size() <= static_cast<unsigned>(t.sensor)) {
//    t_data->t_counts.push_back(0);
//  }

	// See if we have gotten enough reports from this sensor that we should
	// print this one.  If so, print and reset the count.
//	if ( ++t_data->t_counts[t.sensor] >= tracker_stride ) {
//		t_data->t_counts[t.sensor] = 0;
//		printf("Tracker %s, sensor %d:\n        pos (%5.2f, %5.2f, %5.2f); quat (%5.2f, %5.2f, %5.2f, %5.2f)\n",
//			t_data->t_name,
//			t.sensor,
//			t.pos[0], t.pos[1], t.pos[2],
//			t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
//	}
  ros::Time time_now = ros::Time::now();
  //ROS_INFO("handle_tracker\tSensor %d is now at (%g,%g,%g)\n", t.sensor, t.pos[0], t.pos[1], t.pos[2]);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(t.pos[0] / 1000, t.pos[1] / 1000, t.pos[2] / 1000));
  transform.setRotation(tf::Quaternion(t.quat[0], t.quat[1], t.quat[2], t.quat[3]));
  tf::StampedTransform stampTransform(transform, time_now, t_data->t_ref_frame, t_data->t_subject_frame);
  tf_broadcaster.sendTransform(stampTransform);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_vrpn_client");
	ros::NodeHandle n;
  ros::NodeHandle n_private("~"); 

  string host_addr, tf_ref_frame, subjects[MAX_DEVICES], addresses[MAX_DEVICES];
  device_info device_list[MAX_DEVICES];
  int num_subjects, num_devices;
  //tf::TransformBroadcaster tf_broadcaster;

  n_private.param("host_addr", host_addr, string("Vicon"));
  n_private.param("tf_ref_frame", tf_ref_frame, string("/vicon_world"));     

  if (!n.hasParam("subject_list"))
    ROS_FATAL("Error: subject_list was not provided");

  XmlRpc::XmlRpcValue subject_list;
  n.getParam("subject_list", subject_list);
  ROS_ASSERT(subject_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  num_subjects = subject_list.size();
  num_devices = 0;

  if (num_subjects > MAX_DEVICES)
    ROS_FATAL("Tracking limit is %d subjects.", MAX_DEVICES);

  for (int i = 0; i < num_subjects; ++i) 
  {
    ROS_ASSERT(subject_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    subjects[i] = (string)subject_list[i];       
    addresses[i] = subjects[i] + "@" + host_addr;
    
	  device_info *dev;

	  // Name the device and open it as everything
	  dev = &device_list[num_devices];
	  dev->name = addresses[i].c_str();
	  dev->tkr = new vrpn_Tracker_Remote(dev->name);

	  // If we are printing the tracker reports, prepare the
	  // user-data callbacks and hook them up to be called with
	  // the correct data for this device.
    vrpn_Tracker_Remote *tkr = dev->tkr;
    t_user_callback *tc1 = new t_user_callback;

    // Fill in the user-data callback information
    strncpy(tc1->t_name, dev->name, sizeof(tc1->t_name));
    strncpy(tc1->t_ref_frame, tf_ref_frame.c_str(), sizeof(tc1->t_ref_frame));
    strncpy(tc1->t_subject_frame, subjects[i].c_str(), sizeof(tc1->t_subject_frame));

    // Set up the tracker callback handlers
    tkr->register_change_handler(tc1, handle_tracker_pos_quat);

	  num_devices++;

    ROS_INFO("Tracking subject at: %s", ((string)addresses[i]).c_str());
  }

/* 
 * main interactive loop
 */
  while ( ros::ok() ) {

    // Let all the devices do their things
    for (int i = 0; i < num_devices; i++) {
      device_list[i].tkr->mainloop();
    }

      // Sleep for 1ms so we don't eat the CPU
    vrpn_SleepMsecs(1);
  }

 return 0;
}   /* main */
